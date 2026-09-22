"""
failsafe.py
===========
Safety monitor. Deliberately separates the safety logic from the mission logic.
On a problem the monitor returns a reason string; the state machine then decides to
switch to ABORT. ABORT stops outright where commanding a mode would be pointless or
dangerous (never armed, or the FC has left our mode) and otherwise hands off to
RECOVER - which LANDS by default. RTL only when config.recovery_action says so; it
climbs to RTL_ALT first, which indoors is the ceiling.

Note: this is its own abort logic on the companion side. It is independent of
ArduPilot's internal failsafes (BATT_LOW_VOLT, FS_*). Both can - and should -
coexist.

FC PARAMETER OWNERSHIP (team decision 2026-08-24): the companion no longer WRITES
any FC parameter - Mission Planner plus the published params/flight_v2.param are the
one source of truth, and this monitor VERIFIES them read-only. The fence, the
envelope limits (WPNAV_*, RTL_ALT) and their values all live in
params/flight_v2.param now. verify_fence_disabled() is what remains of the old
set->restore fence machinery: a read-only check that refuses to fly when a fence
this run did not ask for is armed.
"""

import logging
import math
import os
import time
from typing import Optional

import paramcheck
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
            self._hint_fresh_sitl()
            return "NO_RANGEFINDER_DATA"
        if r["flow_samples"] == 0:
            self._hint_fresh_sitl()
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

    def _hint_fresh_sitl(self) -> None:
        """Explain the one abort the guard cannot tell apart from a broken sensor.

        NO_RANGEFINDER_DATA / NO_OPTICAL_FLOW_DATA is CORRECT behaviour: without a
        height and a flow signal a GPS-denied flight would drift away (params/README.md
        records 366 m of it). But on the SIMULATOR "no samples at all" almost never
        means a broken sensor - it means a fresh or wiped SITL still at firmware
        defaults (RNGFND1_TYPE=0, FLOW_TYPE=0, EKF sources still on GPS) that never had
        the indoor profile loaded. The fix is then how the simulator was STARTED, not a
        change in this code - and that is exactly the hint the bare abort reason does
        not give. Emitted only for the simulation profile: on the real aircraft a dead
        stream is a genuine hardware fault, and pointing at a parameter file would be
        misleading.
        """
        if not self.config.is_simulation:
            return
        log.warning(
            "[PREARM] SITL is streaming no rangefinder/optical-flow data at all - on the "
            "simulator that is almost always a fresh or wiped SITL still at firmware "
            "defaults (RNGFND1_TYPE=0, FLOW_TYPE=0), not a broken sensor. Start the "
            "simulator with the flight parameters instead of a bare sim_vehicle.py:\n"
            "    python sitl.py\n"
            "which passes the generated mirror as a startup defaults file, so the "
            "rangefinder backend and its sub-parameters come up in the same boot (see "
            "params/README.md, which also documents the SIM_TERRAIN trap that makes the "
            "rangefinder read a constant 0.00 m)."
        )

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

    # ------------------------------------------------------------------
    # Flight-parameter verification (read-only, before the mission arms)
    # ------------------------------------------------------------------
    def verify_flight_parameters(self) -> Optional[str]:
        """Compare the live FC against the published flight set. Returns a reason
        string to abort with, or None.

        This is the other half of the 2026-08-24 ownership decision. Handing the FC
        parameters to Mission Planner removed the surprise-overwrite failure mode that
        caused the 2026-08-21 crash, but it opened a quieter one: nothing then noticed
        when the aircraft in front of you stopped being the aircraft the code was
        reasoned about. A parameter changed for one experiment and left behind is
        invisible in the air and obvious in a diff - so the mission takes the diff
        before it arms.

        Still strictly read-only: it reports, it never writes. Per the ownership rule
        the fix is made in Mission Planner, or the aircraft is captured and a new
        versioned file published (`dumpparams.py`).

        Which parameters, and which of them are worth refusing a flight over, lives in
        paramcheck.py. `config.param_check` chooses what a CRITICAL difference does:
        "abort" (default), "warn" or "off".
        """
        mode = (self.config.param_check or "off").lower()
        if mode == "off":
            return None

        path = (self.config.expected_params_path
                or paramcheck.newest_flight_set(simulated=self.config.is_simulation))
        expected = paramcheck.load_param_file(path)
        if expected is None:
            # A missing file downgrades to "no verification", never to "all good" and
            # never to a crash: the check is a guard, and a guard that blocks a flight
            # because its own reference file moved would just get switched off.
            log.warning(
                "[PREARM] No published parameter file found "
                f"({path or paramcheck.PARAMS_DIR}) - flying WITHOUT the parameter "
                "check. Publish params/flight_v<N>.param (aircraft) or generate "
                "params/sitl_flight_v<N>.parm (simulator)."
            )
            return None

        if not expected:
            log.warning(
                f"[PREARM] {os.path.basename(path)} exists but contains no parameters - "
                "likely an interrupted `dumpparams.py` run, not an empty publish. "
                "Flying WITHOUT the parameter check. Re-run `python dumpparams.py` and "
                "confirm the file is non-empty before trusting this gate again."
            )
            return None

        names = [name for name in paramcheck.VERIFIED_PARAMS if name in expected]
        log.info(f"[PREARM] Verifying {len(names)} parameters against "
                 f"{os.path.basename(path)} (read-only) ...")
        # Batched: one request burst per round, bounded by the round timeout. A loop of
        # single reads would cost tries x timeout per name and could stall the pre-arm
        # for minutes on a quiet link.
        live = self.drone.read_params(names)

        unreadable = [name for name, value in live.items() if value is None]
        critical = paramcheck.compare(live, expected, paramcheck.CRITICAL_PARAMS)
        informational = paramcheck.compare(live, expected, paramcheck.INFORMATIONAL_PARAMS)

        for name, value, want in informational:
            log.warning(f"[PREARM] {name} = {value} on the FC, published {want} "
                        f"(informational, not blocking)")
        for name, value, want in critical:
            log.warning(f"[PREARM] {name} = {value} on the FC, published {want}  <-- MISMATCH")
        if unreadable:
            # An unreadable safety parameter is a finding, not an absence of one - but
            # not one to ground a flight over on its own, because a busy link drops
            # PARAM_VALUE replies and that is indistinguishable from a missing name.
            log.warning(f"[PREARM] Could not read: {', '.join(unreadable)} - not verified")

        if not critical:
            log.info(f"[PREARM] Flight parameters match {os.path.basename(path)}")
            return None

        log.warning(
            f"[PREARM] {len(critical)} flight-critical parameter(s) differ from "
            f"{os.path.basename(path)}. The companion does not write FC parameters "
            f"(team decision 2026-08-24): change them in Mission Planner, or capture "
            f"the aircraft and publish a new versioned file (python dumpparams.py)."
        )
        if mode == "warn":
            log.warning("[PREARM] param_check is 'warn' - flying anyway")
            return None
        return "FC_PARAMS_MISMATCH"

    # ------------------------------------------------------------------
    # Geofence (read-only verification before the mission)
    # ------------------------------------------------------------------
    def verify_fence_disabled(self) -> Optional[str]:
        """Refuse to fly when a fence this run did not ask for is armed on the FC.

        Read-only by design (team decision 2026-08-24, parameter ownership): the
        companion no longer WRITES any fence parameter - configuring or clearing a
        fence is the operator's job in Mission Planner. What remains is the DETECTION,
        and it earns its keep twice over: the 2026-08-21 crash fence was a leftover
        from an EARLIER companion run (not the flight it crashed), and on 2026-08-24
        the FC arrived carrying a Mission-Planner-suggested fence too. Either way an
        armed fence indoors is a refusal - a barometric altitude fence sits inside its
        own sensor's noise band near the ground (propeller downwash spikes BAlt to
        4-6.7 m in our flight logs, centimetres off the floor), and a fence whose
        action is a mode change turns that bad measurement into a manoeuvre nobody
        commanded. That is the mechanism of the 2026-08-21 ceiling crash.

        Returns "UNEXPECTED_FENCE_ENABLED" when an unasked-for fence is armed, otherwise
        None. An unreadable FENCE_ENABLE stays a warning (not an abort): we cannot prove
        a fence is armed, so we do not block on the guess.
        """
        enable = self.drone.read_param("FENCE_ENABLE")
        if enable is None:
            log.warning("[FAILSAFE] Could not read FENCE_ENABLE - unable to verify "
                        "that no stale fence is armed on the FC")
            return None
        if int(enable) == 0:
            return None
        details = {name: self.drone.read_param(name)
                   for name in ("FENCE_TYPE", "FENCE_ALT_MAX", "FENCE_ACTION")}
        log.warning(f"[FAILSAFE] The FC arrived with a fence ENABLED that this run did "
                    f"not ask for ({details}) - most likely left behind by an earlier "
                    f"run, or suggested by Mission Planner. REFUSING TO FLY. The "
                    f"companion no longer writes FC parameters (team decision "
                    f"2026-08-24): set FENCE_ENABLE=0 in Mission Planner before flying.")
        return "UNEXPECTED_FENCE_ENABLED"

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