"""
failsafe.py
===========
Safety monitor. Deliberately separates the safety logic from the mission logic.
On a problem the monitor returns a reason string; the state machine then decides to
switch to ABORT. ABORT stops outright where commanding a mode would be pointless or
dangerous (never armed, or the FC has left our mode) and otherwise hands off to
RECOVER - which LANDS by default. RTL only when config.recovery_action says so; it
climbs to RTL_ALT first, which indoors is the ceiling.

Note: this is it's own abort logic on the companion side. It is independent of
ArduPilot's internal failsafes (BATT_LOW_VOLT, FS_*). Both can - and should -
coexist.
"""

import json
import logging
import math
import time
from pathlib import Path
from typing import Optional

from config import Config
from drone import Drone

log = logging.getLogger(__name__)


class FailsafeMonitor:
    def __init__(self, drone: Drone, config: Config):
        self.drone = drone
        self.config = config
        self._phase_start: Optional[float] = None
        self._phase_name: str = ""
        self._phase_budget: float = config.phase_timeout_s
        self._telemetry_misses: int = 0  # consecutive reads with no telemetry
        self._battery_lows: int = 0      # consecutive readings below the threshold
        self._alt_disagrees: int = 0     # consecutive EKF-vs-rangefinder mismatches
        self._mode_monitored: bool = False  # armed by watch_mode() once we set GUIDED
        # FC values as they were BEFORE this run changed them, for restore_params().
        # Mirrored to a file on disk: parameters live in the flight controller and
        # outlive this process, so a kill/battery-pull must not lose the baseline.
        self._params_before: dict = {}
        self._backup_path = (Path(config.param_backup_file) if config.param_backup_file
                             else Path(config.log_dir) / "fc_params_backup.json")

    # ------------------------------------------------------------------
    # FC parameter bookkeeping: remember -> set -> restore, crash-safe
    # ------------------------------------------------------------------
    def recover_stale_params(self) -> None:
        """Restore FC parameters a PREVIOUS run changed but never put back.

        restore_params() runs on every exit path of this process - but a battery
        pull, a kill -9 or a Pi brownout ends the process without it, and the FC
        then keeps the mission's fence/limits forever (parameters are persistent).
        That is exactly how the fence that caused the 2026-08-21 crash came to be
        armed during a MANUAL flight: a companion run wrote it and died before the
        restore. So every run starts by checking the on-disk backup and putting
        any leftover values back first - BEFORE snapshotting its own baseline.
        """
        if not self._backup_path.exists():
            return
        try:
            stale = json.loads(self._backup_path.read_text())
        except (OSError, ValueError) as exc:
            log.warning(f"[FAILSAFE] Unreadable param backup {self._backup_path}: {exc}")
            return
        if not stale:
            self._unlink_backup()
            return
        log.warning(f"[FAILSAFE] A previous run left changed FC parameters behind - "
                    f"restoring: {stale}")
        failed = self._write_params(stale)
        if failed:
            self._persist(failed)   # keep what we could not restore for the next try
        else:
            self._unlink_backup()

    def _remember_and_set(self, name: str, value: float) -> bool:
        """Read + record the FC's current value, then write the new one.

        Returns True when the write was confirmed. A parameter the firmware does
        not know (read and write both fail) is a warning, not an abort - the
        firmware then simply keeps its default. The baseline is only recorded
        after a successful write, so restore never touches a parameter this run
        did not actually change."""
        known = name in self._params_before
        before = None if known else self.drone.read_param(name)
        try:
            self.drone.set_param(name, value)
        except TimeoutError:
            log.warning(f"[FAILSAFE] {name} not accepted - it keeps the firmware default")
            return False
        if not known:
            if before is not None:
                self._params_before[name] = before
                self._persist(self._params_before)
            else:
                log.warning(f"[FAILSAFE] {name} was set, but its previous value could "
                            f"not be read - it will NOT be restored on exit")
        return True

    def _remember_and_set_strict(self, name: str, value: float) -> None:
        """Like _remember_and_set, but refuses to proceed without a restorable
        baseline, and lets an unconfirmed write propagate. Used for the fence:
        an altitude fence with a mode-change action is a behaviour change for the
        NEXT (possibly manual) flight, so 'set but cannot restore' is exactly the
        booby trap that caused the 2026-08-21 crash. Failing here happens on the
        ground, before arming, which is where this failure belongs."""
        if name not in self._params_before:
            before = self.drone.read_param(name)
            if before is None:
                raise RuntimeError(
                    f"Could not read {name} from the FC - refusing to change a fence "
                    f"parameter whose previous value cannot be restored")
            self._params_before[name] = before
            self._persist(self._params_before)
        self.drone.set_param(name, value)   # TimeoutError propagates on purpose

    def _write_params(self, values: dict) -> dict:
        """Write a set of parameters; returns the ones that failed.
        FENCE_ENABLE=0 goes first so a fence is disarmed before its shape changes."""
        failed = {}
        names = sorted(values, key=lambda n: 0 if (n == "FENCE_ENABLE" and not values[n]) else 1)
        for name in names:
            try:
                self.drone.set_param(name, values[name])
            except TimeoutError:
                failed[name] = values[name]
                log.warning(f"[FAILSAFE] Could not restore {name} to {values[name]}. "
                            f"The FC may still carry this run's value - check before "
                            f"flying manually.")
        return failed

    def _persist(self, values: dict) -> None:
        try:
            self._backup_path.parent.mkdir(parents=True, exist_ok=True)
            self._backup_path.write_text(json.dumps(values))
        except OSError as exc:
            log.warning(f"[FAILSAFE] Could not write the param backup "
                        f"{self._backup_path}: {exc}")

    def _unlink_backup(self) -> None:
        try:
            self._backup_path.unlink(missing_ok=True)
        except OSError:
            pass

    # ------------------------------------------------------------------
    # FC-side safety envelope (set once before the mission)
    # ------------------------------------------------------------------
    def setup_safety_envelope(self) -> None:
        """Force the flight-controller limits that make an indoor abort survivable.

        This is NOT our own failsafe logic - it is what the AUTOPILOT does on its own
        when something goes wrong, and its defaults are built for open sky:

          * FENCE_ACTION defaults to 1 ("RTL or Land"), and RTL first CLIMBS to
            RTL_ALT (default 1500 cm = 15 m) before returning. In a hall that means
            flying into the ceiling. We set 2 ("Always Land") plus a low RTL_ALT.
          * WPNAV_SPEED_UP defaults to 250 cm/s, which overshoots a 2 m takeoff by
            more than 2 m - enough to breach a low altitude fence on its own.

        We set these before every flight rather than trusting whatever happens to be
        stored in the FC, because a wiped or freshly flashed board silently reverts to
        the outdoor defaults.
        """
        if not self.config.enforce_safety_envelope:
            log.warning("[FAILSAFE] Safety envelope DISABLED by config")
            return
        # A parameter the firmware does not know must not throw us out of a mission on
        # the ground, but it MUST be visible: whatever we failed to set is still at the
        # autopilot's outdoor default. Each write records the FC's previous value first,
        # so restore_params() can put the envelope back on exit.
        self._remember_and_set("WPNAV_SPEED_UP", self.config.climb_rate_cms)
        # Horizontal speed. The firmware default of 1000 cm/s crosses a hall in under a
        # second and is what turns a diverging position estimate into a flyaway rather
        # than a slow drift you can watch and take over from.
        self._remember_and_set("WPNAV_SPEED", self.config.cruise_speed_cms)
        self._set_rtl_altitude(self.config.rtl_alt_m)
        # FENCE_ACTION deliberately NOT set here. It is meaningless with the fence off,
        # and writing it anyway left an unrestored change on the flight controller.
        # Everything fence-related lives in setup_geofence()/restore_geofence(), so that
        # what we change is exactly what we put back.
        log.info(
            f"[FAILSAFE] Safety envelope: climb<={self.config.climb_rate_cms} cm/s, "
            f"cruise<={self.config.cruise_speed_cms} cm/s, "
            f"RTL_ALT={self.config.rtl_alt_m} m"
        )

    # ------------------------------------------------------------------
    # Position-sensor gate (GPS-denied) - run once, before arming
    # ------------------------------------------------------------------
    def verify_position_sensors(self) -> Optional[str]:
        """Prove the rangefinder and optical flow actually produce data. Returns a
        reason string to abort with, or None when they look usable.

        This is a flyaway guard, not a nicety. A rangefinder stuck at 0.00 m does not
        stop the mission anywhere else: EKF_POS_HORIZ_REL can still come up, arming can
        still succeed, and the failure only shows once the position estimate has drifted
        far enough for the controller to chase it at full throttle. Our own
        params/README.md documents 366 m of drift measured while the vehicle stood
        still, from exactly this cause.

        The check is deliberately crude - it asks "is there any signal at all", not "is
        it accurate". Accuracy needs a flight; presence does not, and presence is what
        was missing every time this went wrong.
        """
        if not self.config.verify_position_sensors or not self.config.gps_denied:
            return None

        log.info(f"[PREARM] Checking position sensors for {self.config.sensor_check_s:.0f} s ...")
        r = self.drone.read_position_sensors(self.config.sensor_check_s)

        rng_max = r["rangefinder_max"]
        log.info(
            f"[PREARM] rangefinder: {r['rangefinder_samples']} samples, "
            f"max={rng_max if rng_max is None else round(rng_max, 2)} m | "
            f"optical flow: {r['flow_samples']} messages, "
            f"max quality={r['flow_quality_max']}"
        )

        if r["rangefinder_samples"] == 0:
            return "NO_RANGEFINDER_DATA"
        if r["flow_samples"] == 0:
            return "NO_OPTICAL_FLOW_DATA"

        # Deliberately NOT an abort: sitting on the floor, a perfectly healthy sensor
        # reads ~0 m, so a zero here proves nothing either way. The distinction between
        # "broken" and "on the ground" can only be made in the air, which is what
        # verify_rangefinder_tracks_altitude() does after the climb.
        if rng_max is None or rng_max < self.config.rangefinder_min_valid_m:
            log.warning(
                "[PREARM] The rangefinder reads 0.00 m. On the ground that is normal - "
                "it will be re-checked against the actual height after takeoff."
            )
        log.info("[PREARM] Position sensors are streaming")
        return None

    def verify_rangefinder_tracks_altitude(self, expected_alt_m: float) -> Optional[str]:
        """After the climb, the rangefinder must report roughly the height we are at.

        THIS is the check that catches the flyaway precondition, and it has to happen in
        the air: on the ground a working sensor and a dead one both read 0.00 m. Here
        they differ - at 1 m the working one says ~1 m and the dead one still says 0.00.

        Doing it right after takeoff means the abort happens at takeoff height, where
        LAND is a short descent, rather than after the position estimate has had a whole
        search pattern to drift. Our params/README.md records 366 m of drift from
        exactly this cause.

        The tolerance is deliberately loose (a fraction of the commanded altitude, not a
        band around it): this asks "does the sensor respond to height at all", not "is it
        accurate". Accuracy is a tuning question; presence is a safety one.
        """
        if not self.config.verify_position_sensors or not self.config.gps_denied:
            return None
        if expected_alt_m <= 0:
            return None

        r = self.drone.read_position_sensors(self.config.sensor_check_s)
        rng_max = r["rangefinder_max"]
        floor = expected_alt_m * self.config.rangefinder_track_fraction
        log.info(
            f"[SENSORS] In the air at ~{expected_alt_m:.2f} m: rangefinder max="
            f"{rng_max if rng_max is None else round(rng_max, 2)} m "
            f"(needs > {floor:.2f} m), flow messages={r['flow_samples']}"
        )

        if rng_max is None or rng_max < floor:
            log.warning(
                f"[SENSORS] The rangefinder still reads "
                f"{0.0 if rng_max is None else rng_max:.2f} m while the aircraft is at "
                f"~{expected_alt_m:.2f} m. Without a height the EKF cannot scale optical "
                f"flow into a velocity, so the position estimate will drift and the "
                f"controller will chase it. Landing now, on purpose."
            )
            return "RANGEFINDER_NOT_TRACKING"
        if r["flow_samples"] == 0:
            return "OPTICAL_FLOW_LOST"
        log.info("[SENSORS] Rangefinder tracks altitude - position estimate has a height reference")
        return None

    def _set_rtl_altitude(self, metres: float) -> None:
        """Set the RTL altitude, coping with the 4.6 -> 4.7 parameter rename.

        ArduPilot 4.5/4.6 has RTL_ALT in CENTIMETRES; 4.7 renamed it to RTL_ALT_M in
        METRES. Setting the wrong one is not an error - the autopilot simply ignores an
        unknown parameter, leaving the 15 m default in place. So try the name our
        flight controller uses first and fall back to the newer one.
        """
        if self._remember_and_set("RTL_ALT", metres * 100.0):   # 4.5 / 4.6: centimetres
            return
        if not self._remember_and_set("RTL_ALT_M", metres):     # 4.7+: metres
            log.warning(
                "[FAILSAFE] Could not set the RTL altitude (neither RTL_ALT nor "
                "RTL_ALT_M exists) - an RTL may climb to the firmware default"
            )

    # ------------------------------------------------------------------
    # Geofence (set once before the mission)
    # ------------------------------------------------------------------
    def setup_geofence(self) -> None:
        if not self.config.geofence_enable:
            self._check_stale_fence()
            return
        # Remember what the fence was BEFORE we touched it, so restore_params() can put
        # it back. Learned the hard way: these parameters live in the flight controller
        # and outlive the script. A 4 m "Always Land" fence left enabled turns the next
        # MANUAL flight into an automatic landing the moment the pilot climbs past it -
        # from the pilot's seat that looks like a random failsafe, with no connection to
        # a companion run that ended minutes ago. That is not hypothetical: it is the
        # mechanism of the 2026-08-21 crash. The strict variant therefore refuses to
        # write any fence parameter whose previous value it could not read, and an
        # unconfirmed write ends the mission - on the ground, before arming.
        #
        # Set the fence TYPE first (default 1 = max-altitude only, which works without a
        # horizontal position estimate - indoor-safe), then the altitude, then enable.
        self._remember_and_set_strict("FENCE_TYPE", self.config.fence_type)
        self._remember_and_set_strict("FENCE_ALT_MAX", self.config.fence_alt_max_m)
        self._remember_and_set_strict("FENCE_ACTION", self.config.fence_action)
        self._remember_and_set_strict("FENCE_ENABLE", 1)
        log.warning(
            f"[FAILSAFE] Geofence ENABLED (type={self.config.fence_type}, "
            f"alt_max={self.config.fence_alt_max_m} m, action={self.config.fence_action}). "
            f"Note: propeller downwash spikes the BAROMETRIC altitude near the ground - "
            f"up to 6.7 m in our flight logs - so a low altitude fence will breach on "
            f"takeoff and FENCE_ACTION will yank the aircraft out of the pilot's mode."
        )
        log.info(f"[FAILSAFE] Previous fence saved for restore: "
                 f"{ {k: v for k, v in self._params_before.items() if k.startswith('FENCE_')} }")

    def _check_stale_fence(self) -> None:
        """geofence_enable=False means hands off the fence - but not eyes off.

        The 2026-08-21 crash fence was not set by the flight it crashed: it was a
        leftover from an earlier companion run. With the fence now off by default,
        nothing would ever notice such a leftover again - so when we do NOT manage
        the fence ourselves, we at least check the FC is not flying with one that
        some earlier run forgot. Only the ENABLE bit is touched; the other FENCE_*
        values are left for the owner to inspect."""
        enable = self.drone.read_param("FENCE_ENABLE")
        if enable is None:
            log.warning("[FAILSAFE] Could not read FENCE_ENABLE - unable to verify "
                        "that no stale fence is armed on the FC")
            return
        if int(enable) == 0:
            return
        details = {name: self.drone.read_param(name)
                   for name in ("FENCE_TYPE", "FENCE_ALT_MAX", "FENCE_ACTION")}
        log.warning(f"[FAILSAFE] The FC arrived with a fence ENABLED that this run did "
                    f"not ask for ({details}) - most likely left behind by an earlier "
                    f"run that never restored it. Disabling it (FENCE_ENABLE=0); the "
                    f"other FENCE_* parameters are left unchanged.")
        try:
            self.drone.set_param("FENCE_ENABLE", 0)
        except TimeoutError:
            log.warning("[FAILSAFE] Could not disable the stale fence - do NOT fly "
                        "until FENCE_ENABLE has been checked by hand")

    def restore_params(self) -> None:
        """Put back every FC parameter this run changed. Called on EVERY exit path.

        Not cosmetic: parameters live in the flight controller and outlive this
        process. A 4 m FENCE_ACTION=2 fence left enabled forces the next MANUAL
        flight into LAND the moment the barometer spikes past it - that is the
        verified mechanism of the 2026-08-21 ceiling crash. The on-disk backup is
        deleted only when everything was restored; whatever failed stays in the
        file so the next run's recover_stale_params() can try again.
        """
        if not self._params_before:
            return
        log.info(f"[FAILSAFE] Restoring the FC parameters this run changed: "
                 f"{self._params_before}")
        failed = self._write_params(self._params_before)
        self._params_before = {}
        if failed:
            self._persist(failed)
        else:
            self._unlink_backup()

    # ------------------------------------------------------------------
    # Phase timeout
    # ------------------------------------------------------------------
    def start_phase(self, name: str, budget_s: Optional[float] = None) -> None:
        """Mark the start of a mission phase for the timeout check.

        `budget_s` lets a phase carry its own budget. SEARCH needs this: the default
        spiral has 25 waypoints, so charging the whole phase against phase_timeout_s
        (60 s) aborts the mission long before the pattern has been flown.
        """
        self._phase_name = name
        self._phase_start = time.time()
        self._phase_budget = self.config.phase_timeout_s if budget_s is None else budget_s

    def phase_timed_out(self) -> bool:
        if self._phase_start is None:
            return False
        return (time.time() - self._phase_start) > self._phase_budget

    # ------------------------------------------------------------------
    # Link health
    # ------------------------------------------------------------------
    def link_lost(self) -> bool:
        """True if no FC heartbeat arrives within the configured timeout."""
        return not self.drone.link_alive(self.config.heartbeat_timeout_s)

    # ------------------------------------------------------------------
    # Battery check
    # ------------------------------------------------------------------
    def battery_critical(self, batt: dict) -> bool:
        """True if voltage or remaining capacity is at/below the abort threshold.
        Expects a valid battery dict (the None / missing-data case is handled in
        check() via the telemetry-miss counter)."""
        if batt["voltage"] <= self.config.battery_min_voltage:
            return True
        if 0 <= batt["remaining"] <= self.config.battery_min_percent:
            return True
        return False

    # ------------------------------------------------------------------
    # Combined check: call once per loop iteration
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # Mode monitoring
    # ------------------------------------------------------------------
    def watch_mode(self) -> None:
        """Arm the mode check. Call this once the mission has actually put the FC into
        its working mode.

        Before that the vehicle is legitimately in whatever mode it booted into
        (STABILIZE), so checking from the first loop iteration aborts every mission
        before it starts - which is exactly what happened the first time this check
        went in.
        """
        self._mode_monitored = True
        log.info(f"[FAILSAFE] Watching for a mode change away from {self.config.expect_mode}")

    def mode_lost(self) -> Optional[str]:
        """Reason string if the FC is no longer in the mode we command from.

        The autopilot can leave GUIDED without asking us: the pilot flips a switch, a
        fence is breached, an EKF failsafe fires. From that moment every setpoint we
        send is silently discarded - and this is the dangerous part - our own abort
        path would then command a flight mode and OVERRIDE the human who took control. So
        losing the mode is an abort reason in its own right, handled by stopping
        rather than by commanding anything.

        We saw exactly this in SITL: a fence breach switched the vehicle to RTL while
        the mission kept sending waypoints for 60 s to a vehicle that had already
        crashed.
        """
        expected = self.config.expect_mode
        if not expected or not self._mode_monitored:
            return None
        mode = self.drone.get_mode()
        if mode and mode != expected:
            return f"MODE_CHANGED_{mode}"
        return None

    # ------------------------------------------------------------------
    # Combined check: call once per loop iteration
    # ------------------------------------------------------------------
    def check(self) -> Optional[str]:
        """
        Returns a reason string if an abort is required, otherwise None.

        Order: link first (cheapest signal of a dead connection), then mode, then
        telemetry health, then battery, then the phase timeout. A SINGLE missing
        telemetry read is tolerated (streams hiccup); only a run of them aborts - this
        closes the old blind spot where missing data silently meant "all good". The
        same applies to battery voltage, which sags under load.
        """
        self.drone.tick()   # heartbeat out, autopilot messages in

        if self.link_lost():
            return "LINK_LOSS"

        mode_reason = self.mode_lost()
        if mode_reason:
            return mode_reason

        batt = self.drone.get_battery()
        if batt is None:
            self._telemetry_misses += 1
            if self._telemetry_misses >= self.config.telemetry_max_misses:
                return "NO_TELEMETRY"
        else:
            self._telemetry_misses = 0
            if self.battery_critical(batt):
                self._battery_lows += 1
                if self._battery_lows >= self.config.battery_low_samples:
                    return "LOW_BATTERY"
            else:
                self._battery_lows = 0

        runaway = self.position_implausible()
        if runaway:
            return runaway

        diverged = self.altitude_implausible()
        if diverged:
            return diverged

        if self.phase_timed_out():
            return f"TIMEOUT_{self._phase_name}"
        return None

    # ------------------------------------------------------------------
    # Flyaway guard (in flight)
    # ------------------------------------------------------------------
    def position_implausible(self) -> Optional[str]:
        """Abort if the reported local position leaves a plausible envelope.

        This is the horizontal counterpart to the altitude geofence, done in software
        because the fence we can set indoors is altitude-only (`FENCE_TYPE = 1`; a
        circle fence needs a horizontal position the autopilot may not trust). It costs
        one LOCAL_POSITION_NED read we are taking anyway.

        The threshold is sized from the search pattern rather than the hall: the spiral
        reaches `search_max_radius_m` plus one `search_step_m`, so a position well beyond
        that is the filter running away, not the aircraft flying. Whichever it is, the
        answer is the same - land - because a position estimate that cannot be trusted
        makes every subsequent navigation command meaningless.

        Deliberately does NOT fire on a missing reading: no data is the telemetry
        failsafe's job, and treating it as a runaway would abort on a hiccup.
        """
        if self.config.max_position_radius_m <= 0 or not self.config.gps_denied:
            return None
        pos = self.drone.get_local_position()
        if not pos:
            return None
        distance = math.hypot(pos["north"], pos["east"])
        if distance > self.config.max_position_radius_m:
            log.warning(
                f"[FAILSAFE] Position estimate is {distance:.1f} m from the origin, "
                f"beyond the {self.config.max_position_radius_m:.0f} m envelope. Either "
                f"the aircraft is running away or the estimate is - landing either way."
            )
            return "POSITION_IMPLAUSIBLE"
        return None

    def altitude_implausible(self) -> Optional[str]:
        """The vertical twin of position_implausible(): EKF altitude vs the raw
        rangefinder, checked on every failsafe tick.

        This is the exact signature of the 2026-08-21 crash: the EKF believed
        -1070 m while the rangefinder truthfully read 0.02 m on the floor. The
        one-time check after takeoff (verify_rangefinder_tracks_altitude) proves the
        SENSOR responds to height; this check watches the ESTIMATE for the rest of
        the flight, because a filter that stops fusing its height source diverges
        silently while every sensor still looks healthy.

        The rangefinder measures distance to whatever is BELOW the aircraft, so tall
        clutter under the flight path can disagree legitimately - hence the loose
        threshold and the consecutive-sample counter, mirroring the battery check.
        Deliberately does NOT fire on a missing reading (no data is the telemetry
        failsafe's job), nor beyond the sensor's trustworthy range.
        """
        if not self.config.gps_denied or self.config.alt_disagree_max_m <= 0:
            return None
        rng = self.drone.get_rangefinder()
        if rng is None or rng > 7.5:   # no reading / near the MTF-01P's 8 m limit
            return None
        pos = self.drone.get_position()
        if not pos:
            return None
        if abs(pos["rel_alt"] - rng) > self.config.alt_disagree_max_m:
            self._alt_disagrees += 1
            log.warning(
                f"[FAILSAFE] EKF altitude {pos['rel_alt']:.1f} m vs rangefinder "
                f"{rng:.2f} m ({self._alt_disagrees}/{self.config.alt_disagree_samples})"
            )
            if self._alt_disagrees >= self.config.alt_disagree_samples:
                return "EKF_ALT_DIVERGED"
        else:
            self._alt_disagrees = 0
        return None