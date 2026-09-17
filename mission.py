"""
mission.py
==========
The delivery state machine. Two paths share the same machine, chosen by
config.gps_denied:

    GPS / outdoor (Phase 1):
        IDLE -> TAKEOFF -> ENROUTE -> OVER_TARGET -> DROP -> RECOVER -> DONE
    Indoor / GPS-denied (Phase 2):
        IDLE -> TAKEOFF -> SEARCH -> APPROACH -> DROP -> RECOVER -> DONE
                              ^_________|  (target lost)
                                                      (ABORT on failsafe)
    Bring-up hover (config.hover_test_s > 0), short-circuits both:
        IDLE -> TAKEOFF -> HOVER -> RECOVER -> DONE

Each state does exactly one thing and returns the next state. The failsafe is
checked before every step - if it fires, the machine jumps to ABORT.

RECOVER is where the aircraft comes down. It LANDS by default rather than flying
RTL, because RTL first climbs to RTL_ALT and indoors that is the ceiling; see
config.recovery_action. ABORT itself is deliberately not a single path:

    never armed        -> DONE       (nothing to recover; do not command a mode
                                      at a vehicle standing on the ground)
    mode changed       -> DONE       (the pilot or an FC failsafe has control -
                                      commanding anything now would fight them)
    otherwise          -> RECOVER    (land / return, then disarm)
"""

import logging
import math
import time
from enum import Enum, auto

from camera import Camera
from config import Config
from drone import Drone
from failsafe import FailsafeMonitor
from release import ReleaseMechanism
from search import make_search_pattern

log = logging.getLogger(__name__)


class State(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    HOVER = auto()        # bring-up: climb, hold still, come down (no search, no drop)
    SEARCH = auto()       # indoor: fly a pattern, look for the target
    APPROACH = auto()     # indoor: visual servoing onto a detected target
    ENROUTE = auto()      # GPS: fly to known coordinates
    OVER_TARGET = auto()  # GPS: fine centring over the target
    DROP = auto()
    RECOVER = auto()      # come down safely: LAND indoors, RTL outdoors
    ABORT = auto()
    DONE = auto()


class DeliveryMission:
    def __init__(self, drone: Drone, camera: Camera, failsafe: FailsafeMonitor,
                 config: Config, release: ReleaseMechanism):
        self.drone = drone
        self.camera = camera
        self.failsafe = failsafe
        self.config = config
        self.release = release
        self.state = State.IDLE
        self.abort_reason = None
        # Did we ever get the motors running? An abort before arming must NOT command
        # a flight mode at a vehicle sitting on the ground.
        self._airborne = False

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self) -> None:
        log.info("\n=== MISSION START ===")
        dispatch = {
            State.IDLE: self._idle,
            State.TAKEOFF: self._takeoff,
            State.HOVER: self._hover,
            State.SEARCH: self._search,
            State.APPROACH: self._approach,
            State.ENROUTE: self._enroute,
            State.OVER_TARGET: self._over_target,
            State.DROP: self._drop,
            State.RECOVER: self._recover,
            State.ABORT: self._abort,
        }

        try:
            while self.state != State.DONE:
                # Failsafe check before each state (unless we are already recovering)
                if self.state not in (State.ABORT, State.RECOVER):
                    reason = self.failsafe.check()
                    if reason:
                        self.abort_reason = reason
                        log.info(f"\n[FAILSAFE] ABORT due to: {reason}")
                        self.state = State.ABORT
                        continue

                log.info(f"\n--- State: {self.state.name} ---")
                self.state = dispatch[self.state]()
        except BaseException as exc:
            log.error(f"[MISSION] Aborting on unhandled error: {exc!r}")
            self._emergency_land()
            raise
        finally:
            # Nothing to restore any more by design: since the 2026-08-24 ownership
            # decision the companion writes no FC parameter, so there is no baseline to
            # put back on exit (the old fence/envelope restore lived here).
            log.info("\n=== MISSION END ===")

    def _emergency_land(self) -> None:
        """Last-ditch attempt to get the aircraft down. Never raises."""
        if not self._airborne:
            return
        try:
            # Same rule as everywhere else: if the FC already left our mode, the pilot
            # or an FC failsafe is flying - commanding LAND now would override them.
            if self.failsafe.mode_lost():
                log.warning("[MISSION] FC is no longer in our mode - the pilot or an "
                            "FC failsafe has control, NOT commanding LAND")
                return
            self.drone.land()
            log.warning("[MISSION] Emergency LAND commanded")
        except Exception:
            log.exception("[MISSION] Could not command LAND - aircraft may still be flying")

    # ------------------------------------------------------------------
    # States (each returns the next state)
    # ------------------------------------------------------------------
    def _idle(self) -> State:
        """Preparation: (origin), fence verification, drop servo, GUIDED, arm. Per the
        2026-08-24 ownership decision the companion no longer WRITES FC parameters here;
        it verifies the fence and refuses to fly if one it did not ask for is armed."""
        log.info("[IDLE] FC parameters are Mission-Planner-owned (team decision "
                 "2026-08-24): the companion verifies them but does not write any. "
                 "See params/README.md.")

        # Indoors without GPS the EKF needs an origin before it can report a position.
        if self.config.gps_denied and self.config.set_origin_on_start:
            if not self.drone.set_origin(
                self.config.origin_lat, self.config.origin_lon, self.config.origin_alt
            ):
                # Not fatal: the FC may legitimately already have an origin (SITL with
                # GPS on). But it means local NED is anchored somewhere we did not
                # choose, so the flight log must say so.
                log.warning("[IDLE] Continuing with the origin the FC already had")

        fence_problem = self.failsafe.verify_fence_disabled()
        if fence_problem:
            return self._fail(fence_problem)
        self.release.setup()
        self.release.reset()

        # Prove the sensors that FEED the position estimate before trusting the estimate
        # itself. EKF_POS_HORIZ_REL can come up on a rangefinder stuck at 0.00 m, and
        # then nothing stops the mission until the drifting estimate is being chased at
        # full throttle. This gate runs on the ground, where a refusal is free.
        sensor_problem = self.failsafe.verify_position_sensors()
        if sensor_problem:
            return self._fail(sensor_problem)

        # Pilot-assisted start: the human flies it off the ground, the companion takes
        # over in the air. See _wait_for_pilot() for why that is sometimes the only way.
        if self.config.takeover_mode:
            return self._wait_for_pilot()

        # Indoor (GPS-denied) needs only the RELATIVE EKF position (optical flow);
        # outdoor needs the ABSOLUTE one (GPS).
        if not self.drone.wait_ready_to_arm(require_abs=not self.config.gps_denied):
            return self._fail("NO_POSITION_ESTIMATE")
        if not self.drone.set_mode("GUIDED"):
            return self._fail("MODE_GUIDED_REFUSED")
        # Only now may the mode check run: until this point the FC is legitimately in
        # whatever mode it booted into.
        self.failsafe.watch_mode()
        if not self.drone.arm():
            return self._fail("ARMING_FAILED")

        self._airborne = True
        self.failsafe.start_phase("TAKEOFF")
        return State.TAKEOFF

    def _wait_for_pilot(self) -> State:
        """Wait for the PILOT to arm and fly the aircraft up, then take over in GUIDED.

        Two reasons this exists, and the second is the important one:

        1. A flow-only vehicle may not offer `EKF_POS_HORIZ_REL` while it sits on the
           floor. Optical flow needs height before the filter will trust it, so a
           companion that insists on a position estimate BEFORE arming can deadlock:
           no height without a takeoff, no takeoff without a position estimate. Flying
           the first metre by hand breaks that circle.
        2. It puts the human in charge of the riskiest moment. The companion never arms
           and never leaves the ground - it inherits an aircraft that is already stable
           in the air, with a pilot whose hands are already on the sticks.

        We do NOT arm, do NOT command a takeoff, and do NOT touch the throttle. We wait
        until the vehicle is armed, above `takeover_min_alt_m` and reporting a usable
        position estimate, and only then ask for GUIDED.
        """
        log.warning("=" * 62)
        log.warning("[TAKEOVER] Waiting for the PILOT to arm and climb to "
                    f"{self.config.takeover_min_alt_m:.1f} m.")
        log.warning("[TAKEOVER] The companion will NOT arm and will NOT take off.")
        log.warning("[TAKEOVER] Keep the throttle stick near centre: if you switch back "
                    "to a manual mode, that stick position takes effect immediately.")
        log.warning("=" * 62)

        need_flag = 0x08 if self.config.gps_denied else 0x10
        deadline = time.time() + self.config.takeover_timeout_s
        announced = False
        while time.time() < deadline:
            self.drone.tick()

            if not self.drone.is_armed():
                time.sleep(0.5)
                continue
            if not announced:
                log.info("[TAKEOVER] Pilot has armed. Waiting for altitude ...")
                announced = True

            # Gate the handover on the RANGEFINDER, not on the EKF altitude. In the
            # 2026-08-21 crash logs the EKF altitude read +1070 m while the aircraft
            # stood on the floor (no height source fused), while the rangefinder read
            # a truthful 0.02 m throughout - an EKF-based gate would have passed the
            # instant the pilot armed. The raw sensor is the one we can lean on here.
            rng_alt = self.drone.get_rangefinder()
            if rng_alt is None or rng_alt < self.config.takeover_min_alt_m:
                time.sleep(0.5)
                continue

            # Cross-check the EKF against the sensor. A large disagreement IS the
            # diverged-vertical-estimate signature that caused the crash: handing a
            # GUIDED aircraft to a filter in that state is the flyaway setup.
            pos = self.drone.get_position()
            rel_alt = pos["rel_alt"] if pos else None
            if (rel_alt is not None
                    and abs(rel_alt - rng_alt) > self.config.takeover_alt_disagree_m):
                log.warning(f"[TAKEOVER] EKF altitude ({rel_alt:.1f} m) and rangefinder "
                            f"({rng_alt:.2f} m) disagree - the height estimate cannot "
                            f"be trusted. NOT taking over. Land manually.")
                return self._fail("EKF_ALT_DIVERGED")

            # Only hand over onto a position estimate we would have accepted on the
            # ground. Taking over without one is exactly the flyaway setup.
            msg = self.drone.wait_ekf_flag(need_flag, timeout=self.config.takeover_ekf_wait_s)
            if not msg:
                log.warning("[TAKEOVER] Airborne at "
                            f"{rng_alt:.2f} m but still no position estimate - NOT "
                            "taking over. Land manually.")
                return self._fail("NO_POSITION_ESTIMATE_IN_AIR")

            log.info(f"[TAKEOVER] Airborne at {rng_alt:.2f} m with a position estimate "
                     f"- taking over")
            if not self.drone.set_mode("GUIDED"):
                return self._fail("MODE_GUIDED_REFUSED")
            self.failsafe.watch_mode()
            self._airborne = True
            # The pilot already did the climb, so skip TAKEOFF entirely and go straight
            # to what this run is actually testing.
            if self.config.hover_test_s > 0:
                self.failsafe.start_phase("HOVER", budget_s=self.config.hover_test_s + 30.0)
                return State.HOVER
            if self.config.gps_denied:
                self.failsafe.start_phase("SEARCH", budget_s=self.config.search_timeout_s)
                return State.SEARCH
            self.failsafe.start_phase("ENROUTE")
            return State.ENROUTE

        return self._fail("PILOT_NEVER_TOOK_OFF")

    def _fail(self, reason: str) -> State:
        """Record why we are aborting, then go to ABORT.

        Every abort path must name itself: a flight log that only says
        "[ABORT] Reason: None" is useless for post-flight analysis, and that is
        exactly what the old code produced for arming and takeoff failures.
        """
        self.abort_reason = reason
        log.warning(f"[ABORT] {reason}")
        return State.ABORT

    def _takeoff(self) -> State:
        altitude = self.takeoff_altitude()
        if not self.drone.takeoff(altitude):
            return self._fail("TAKEOFF_FAILED")

        # Now that we are off the ground, the rangefinder has to prove itself: on the
        # floor a dead sensor and a healthy one both read 0.00 m, at altitude they do
        # not. Checking here means a failure aborts at takeoff height, before the
        # position estimate has a whole search pattern in which to drift.
        sensor_problem = self.failsafe.verify_rangefinder_tracks_altitude(altitude)
        if sensor_problem:
            return self._fail(sensor_problem)
        # Bring-up mode: climb, hold, come down. Nothing else is exercised, which is the
        # point - it isolates "can the aircraft hold a position under companion control"
        # from every later unknown (search pattern, camera, drop).
        if self.config.hover_test_s > 0:
            self.failsafe.start_phase("HOVER", budget_s=self.config.hover_test_s + 30.0)
            return State.HOVER
        if self.config.gps_denied:
            self.failsafe.start_phase("SEARCH", budget_s=self.config.search_timeout_s)
            return State.SEARCH
        self.failsafe.start_phase("ENROUTE")
        return State.ENROUTE

    def takeoff_altitude(self) -> float:
        """Altitude the mission climbs to. Indoor uses search_altitude (a hall ceiling
        is metres, not tens of metres), the GPS path uses cruise_alt.
        hover_test_alt overrides both when set, so a bring-up hover can be flown lower
        than the search altitude without touching the rest of the configuration."""
        if self.config.hover_test_s > 0 and self.config.hover_test_alt > 0:
            return self.config.hover_test_alt
        return self.config.search_altitude if self.config.gps_denied else self.config.cruise_alt

    def _hover(self) -> State:
        """Hold the takeoff position for `hover_test_s`, then land.

        This is milestone 1 of hardware bring-up: it answers exactly one question — can
        the aircraft hold height and position on companion commands? — and answers it
        without a search pattern, a camera or a payload release in the way.

        It also logs the horizontal DRIFT from the position it started at, once per
        second. That number is the actual result of the test: a stable optical-flow hold
        stays within a few tens of centimetres, while a drifting one walks away steadily
        and tells you the flow or the rangefinder is not really working, even though the
        aircraft is technically flying.

        The camera is polled too but never acted upon, so the same run doubles as
        milestone 2 (does the detector see a pad directly below?) as soon as
        camera_source is set to "real".
        """
        start = self.drone.get_local_position()
        origin_n = start["north"] if start else 0.0
        origin_e = start["east"] if start else 0.0
        log.info(f"[HOVER] Holding for {self.config.hover_test_s:.0f} s "
                 f"at ({origin_n:.2f}, {origin_e:.2f}) ...")

        deadline = time.time() + self.config.hover_test_s
        worst_drift = 0.0
        while time.time() < deadline:
            reason = self.failsafe.check()
            if reason:
                self.abort_reason = reason
                log.info(f"[HOVER] Failsafe during hover: {reason}")
                return State.ABORT

            pos = self.drone.get_local_position()
            if pos:
                drift = math.hypot(pos["north"] - origin_n, pos["east"] - origin_e)
                worst_drift = max(worst_drift, drift)
                log.info(f"[HOVER] alt={-pos['down']:.2f} m  drift={drift:.2f} m  "
                         f"(worst {worst_drift:.2f} m)")

            offset = self.camera.get_target_offset()
            if offset["detected"]:
                log.info(f"[HOVER] Camera sees the target: dx={offset['dx']:+.2f} "
                         f"dy={offset['dy']:+.2f} m (not acting on it)")

            time.sleep(1.0)

        log.info(f"[HOVER] Done. Worst horizontal drift: {worst_drift:.2f} m")
        self.failsafe.start_phase("RECOVER")
        return State.RECOVER

    def _enroute(self) -> State:
        self.drone.goto(self.config.target_lat, self.config.target_lon, self.config.cruise_alt)
        arrived = self.drone.wait_arrival(
            self.config.target_lat,
            self.config.target_lon,
            self.config.arrival_radius_m,
            timeout=self.config.phase_timeout_s,
        )
        if arrived:
            self.failsafe.start_phase("OVER_TARGET")
            return State.OVER_TARGET
        return self._fail("ENROUTE_TIMEOUT")

    def _over_target(self) -> State:
        """
        Fine alignment over the target by visual servoing. Reads the camera's ground
        offset to the target (dx, dy, in METRES) and nudges the drone in the body frame
        until it is centred, then drops.

        Axis mapping (downward-facing camera): dx -> body 'right', dy -> body
        'forward'. Each step is scaled by approach_gain and clamped to max_nudge_m,
        so corrections stay small and safe. With the MockCamera (dx=dy=0) the first
        frame is already centred and we go straight to the drop.
        """
        tol = self.config.centre_tolerance
        deadline = time.time() + self.config.phase_timeout_s

        while time.time() < deadline:
            # Keep the GCS heartbeat alive: this loop can run for the whole phase
            # timeout, and a silence longer than FS_GCS_TIMEOUT (5 s) makes the FC fire
            # its own GCS failsafe on a companion that is merely centring.
            self.drone.tick()
            if self._pilot_took_over("CAM"):
                return State.ABORT
            offset = self.camera.get_target_offset()
            if not offset["detected"]:
                log.info("[CAM] No target detected, waiting ...")
                time.sleep(0.5)
                continue

            dx, dy = offset["dx"], offset["dy"]
            if abs(dx) <= tol and abs(dy) <= tol:
                log.info(f"[CAM] Centred (dx={dx:.2f}, dy={dy:.2f})")
                return State.DROP

            log.info(f"[CAM] Correcting (dx={dx:.2f}, dy={dy:.2f})")
            self._nudge_from_offset(dx, dy)
            time.sleep(self.config.nudge_settle_s)

        return self._fail("ALIGNMENT_TIMEOUT")

    # ------------------------------------------------------------------
    # Indoor / GPS-denied states (Phase 2)
    # ------------------------------------------------------------------
    def _search(self) -> State:
        """
        Fly a search pattern in local NED and look for the target with the camera.
        On detection -> APPROACH; if the whole pattern is exhausted without a detection
        -> ABORT -> RECOVER, which LANDS where the drone is (config.recovery_action,
        default "land"). Only recovery_action="rtl" returns to launch, and that is an
        outdoor choice: RTL climbs to RTL_ALT first, which indoors is the ceiling.
        """
        pattern = make_search_pattern(self.config)
        waypoints = pattern.waypoints()
        log.info(f"[SEARCH] {self.config.search_pattern} pattern, {len(waypoints)} waypoints, "
                 f"cadence={self.config.detection_cadence}")
        down = -self.config.search_altitude
        misses = 0   # consecutive waypoints we failed to reach

        for (north, east) in waypoints:
            # SEARCH is long-running, so re-check the failsafe on every leg.
            reason = self.failsafe.check()
            if reason:
                self.abort_reason = reason
                log.info(f"[SEARCH] Failsafe during search: {reason}")
                return State.ABORT

            self.drone.goto_local(north, east, down)

            if self.config.detection_cadence == "continuous":
                result = self._poll_until_arrival_or_detection(north, east)
                if result == "detected":
                    return State.APPROACH
                if result == "mode_lost":
                    return State.ABORT
                if result == "timeout":
                    # Same rule as stop_and_look below: never pretend we got there.
                    misses += 1
                    log.warning(f"[SEARCH] Waypoint ({north:.1f}, {east:.1f}) not reached "
                                f"({misses}/{self.config.search_max_misses})")
                    if misses >= self.config.search_max_misses:
                        return self._fail("WAYPOINTS_UNREACHABLE")
                    continue
                misses = 0   # "arrived"
            else:  # stop_and_look
                arrived = self.drone.wait_local_arrival(
                    north, east, self.config.local_arrival_radius_m,
                    timeout=self.config.waypoint_timeout_s,
                )
                if not arrived:
                    # Never pretend we got there. Not reaching a waypoint means either
                    # the vehicle is not following us (mode changed) or the position
                    # estimate has diverged - looking for the target from an unknown
                    # place, and flying the rest of the pattern from it, is worse than
                    # stopping.
                    misses += 1
                    log.warning(f"[SEARCH] Waypoint ({north:.1f}, {east:.1f}) not reached "
                                f"({misses}/{self.config.search_max_misses})")
                    if misses >= self.config.search_max_misses:
                        return self._fail("WAYPOINTS_UNREACHABLE")
                    continue
                misses = 0
                time.sleep(self.config.look_settle_s)
                if self.camera.get_target_offset()["detected"]:
                    log.info(f"[SEARCH] Target detected near ({north:.1f}, {east:.1f})")
                    return State.APPROACH

        log.warning("[SEARCH] Pattern exhausted, no target found")
        return self._fail("TARGET_NOT_FOUND")

    def _poll_until_arrival_or_detection(self, north: float, east: float) -> str:
        """
        Continuous cadence: poll the camera while flying toward (north, east).
        Returns one of "detected" / "arrived" / "timeout" / "mode_lost".

        A timeout is deliberately NOT reported as arrival: a leg that never converges
        means the vehicle is not following us (mode changed) or the position estimate
        has diverged, and the caller must count it as a miss - exactly like the
        stop_and_look cadence does - instead of "flying" the rest of the pattern on
        paper. The per-leg budget is waypoint_timeout_s, the same one stop_and_look
        uses for a single leg (config.py: "waypoint_timeout_s bounds a SINGLE leg").
        """
        deadline = time.time() + self.config.waypoint_timeout_s
        while time.time() < deadline:
            self.drone.tick()   # see _over_target: keeps FS_GCS_* from firing on us
            if self._pilot_took_over("SEARCH"):
                return "mode_lost"
            if self.camera.get_target_offset()["detected"]:
                log.info("[SEARCH] Target detected en route")
                return "detected"
            pos = self.drone.get_local_position()
            if pos and math.hypot(north - pos["north"], east - pos["east"]) <= self.config.local_arrival_radius_m:
                return "arrived"
            time.sleep(0.2)
        return "timeout"

    def _approach(self) -> State:
        """
        Visual servoing toward a detected target until centred, then DROP. If the
        target is lost for too many frames in a row, fall back to SEARCH.
        """
        tol = self.config.centre_tolerance
        deadline = time.time() + self.config.phase_timeout_s
        lost = 0

        while time.time() < deadline:
            self.drone.tick()   # see _over_target: keeps FS_GCS_* from firing on us
            if self._pilot_took_over("APPROACH"):
                return State.ABORT
            offset = self.camera.get_target_offset()
            if not offset["detected"]:
                lost += 1
                if lost >= self.config.approach_lost_max:
                    log.warning("[APPROACH] Target lost -> back to SEARCH")
                    return State.SEARCH
                log.info(f"[APPROACH] Target lost ({lost}), waiting ...")
                time.sleep(0.3)
                continue

            lost = 0
            dx, dy = offset["dx"], offset["dy"]
            if abs(dx) <= tol and abs(dy) <= tol:
                log.info(f"[APPROACH] Centred over target (dx={dx:.2f}, dy={dy:.2f})")
                return State.DROP
            self._nudge_from_offset(dx, dy)
            time.sleep(self.config.nudge_settle_s)

        return self._fail("APPROACH_TIMEOUT")

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------
    def _pilot_took_over(self, tag: str) -> bool:
        """Cheap per-iteration takeover check, shared by the servoing/search loops.

        The loops that call this (OVER_TARGET, the continuous-cadence SEARCH leg, and
        APPROACH) can each run for a whole phase timeout, and cannot afford the full
        failsafe.check() every iteration - it blocks on a battery read. But noticing that
        the PILOT (or an FC failsafe) has taken over must NOT wait for the phase to end:
        from the moment the FC leaves GUIDED every setpoint we send is discarded anyway,
        and the flight log needs the instant the human took control, not the moment we got
        around to looking. So each loop does this cheap mode check on every pass instead.

        On takeover it records the reason and logs the shared warning under the caller's
        tag, then returns True; the caller keeps its own return path (State.ABORT vs the
        "mode_lost" sentinel). Returns False while the FC is still in our mode.
        """
        mode_reason = self.failsafe.mode_lost()
        if not mode_reason:
            return False
        self.abort_reason = mode_reason
        log.warning(f"[{tag}] {mode_reason} - the pilot or an FC failsafe has "
                    f"control, stopping")
        return True

    def _nudge_from_offset(self, dx: float, dy: float) -> None:
        """Turn a ground offset in METRES (dx -> body right, dy -> body forward) into a
        clamped body-frame nudge. Shared by OVER_TARGET and APPROACH.

        Both are metres, which is why approach_gain is a dimensionless P gain (body
        metres moved per metre of ground error) and centre_tolerance is 15 cm - not
        15 % of the image. Every Camera implementation reports metres; RealCamera
        converts the image angle with the pinhole relation to keep it that way."""
        right = self._clamp(self.config.approach_gain * dx, self.config.max_nudge_m)
        forward = self._clamp(self.config.approach_gain * dy, self.config.max_nudge_m)
        log.info(f"[ALIGN] nudge fwd={forward:+.2f} right={right:+.2f}")
        self.drone.move_body_offset(forward, right)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        """Clamp 'value' into [-limit, +limit]."""
        return max(-limit, min(limit, value))

    def _drop(self) -> State:
        """Perform the release and verify it (FC: servo read-back; Pi: open-loop).

        `config.skip_drop` runs the whole approach without actually releasing. That is
        milestone 4 of bring-up: prove the drone finds the pad and centres over it,
        while nothing can fall out of the aircraft and nothing needs the drop mechanism
        to be calibrated yet. The state machine is otherwise identical, so a green
        skip_drop run means only the release itself is still untested.
        """
        if self.config.skip_drop:
            log.warning("[DROP] skip_drop is set - centred over the target, "
                        "releasing NOTHING (bring-up mode)")
            self.failsafe.start_phase("RECOVER")
            return State.RECOVER

        self.release.drop()
        time.sleep(0.5)  # brief wait so the new value reaches the telemetry
        if self.release.confirm():
            log.info("[DROP] Release confirmed")
        else:
            log.warning("[DROP] release not confirmed")
        time.sleep(1.0)
        self.release.reset()
        self.failsafe.start_phase("RECOVER")
        return State.RECOVER

    def _recover(self) -> State:
        """Bring the aircraft down safely: LAND indoors, RTL outdoors.

        LAND is the indoor default (config.recovery_action) because it needs no
        position estimate, no home and - crucially - no altitude headroom. RTL first
        CLIMBS to RTL_ALT before returning, which in a hall means flying into the
        ceiling. Outdoors RTL is the better choice and can be selected in the config.
        """
        if self.config.recovery_action == "rtl":
            log.info("[RECOVER] Returning to launch")
            accepted = self.drone.return_to_launch()
        else:
            log.info("[RECOVER] Landing here")
            accepted = self.drone.land()
        if accepted is False:
            # Do not claim a landing that was never commanded. The FC refusing the
            # mode (or a dead link) means the aircraft is still doing whatever it was
            # doing - only the transmitter can help now.
            log.warning("[RECOVER] The FC did NOT confirm the recovery mode - take "
                        "over on the transmitter NOW if the aircraft is still flying.")

        if self.drone.wait_disarmed(self.config.phase_timeout_s):
            log.info("[RECOVER] Landed and disarmed")
            self._airborne = False
        else:
            # Deliberately NOT disarming here. The old code sent a disarm on timeout,
            # which at best is refused by the autopilot and at worst cuts the motors
            # of a vehicle that is still in the air. If it has not come down, the
            # right answer is to keep the LAND command standing and let the pilot take
            # over - our kill switch is on the transmitter, not in this script.
            log.warning(
                "[RECOVER] Still armed after the timeout - LAND stays commanded. "
                "Take over on the transmitter if the aircraft is still flying."
            )
        return State.DONE

    def _abort(self) -> State:
        """Controlled abort.

        Two cases, and telling them apart matters: if we never armed (a failed pre-arm
        or a refused GUIDED), the aircraft is sitting on the ground and commanding a
        flight mode at it is pointless noise that the old code nevertheless produced -
        together with a log line claiming it had "landed and disarmed".

        If the mode changed under us, the pilot (or the FC) is in control. Commanding
        anything now would fight them, so we only stop.
        """
        log.info(f"[ABORT] Reason: {self.abort_reason}")

        if not self._airborne:
            log.info("[ABORT] Never armed - nothing to recover")
            return State.DONE

        if self.abort_reason and self.abort_reason.startswith("MODE_CHANGED"):
            log.warning(
                "[ABORT] The flight controller left our mode - the pilot or an FC "
                "failsafe is in control. Sending nothing further."
            )
            return State.DONE

        # The recorded reason is not the whole truth: a failure whose CAUSE was a mode
        # change can arrive here under another name (a takeoff that timed out because
        # the pilot flipped to STABILIZE mid-climb reads TAKEOFF_FAILED; a fence-forced
        # LAND during the climb reads the same). So ask the FC directly before
        # commanding anything - if it is no longer in our mode, someone else is flying.
        if self.failsafe.mode_lost():
            log.warning(
                "[ABORT] The FC is no longer in our mode - the pilot or an FC "
                "failsafe is in control. Sending nothing further."
            )
            return State.DONE

        self.failsafe.start_phase("RECOVER")
        return State.RECOVER