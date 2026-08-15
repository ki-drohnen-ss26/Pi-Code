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
            # Any crash here used to kill the process outright, leaving the aircraft
            # ARMED in GUIDED holding its last position target. GUID_TIMEOUT does not
            # apply to position targets, so it would hover there until the battery ran
            # out - with no companion left to rescue it. Bring it down first.
            log.error(f"[MISSION] Aborting on unhandled error: {exc!r}")
            self._emergency_land()
            raise
        finally:
            log.info("\n=== MISSION END ===")

    def _emergency_land(self) -> None:
        """Last-ditch attempt to get the aircraft down. Never raises."""
        if not self._airborne:
            return
        try:
            self.drone.land()
            log.warning("[MISSION] Emergency LAND commanded")
        except Exception:
            log.exception("[MISSION] Could not command LAND - aircraft may still be flying")

    # ------------------------------------------------------------------
    # States (each returns the next state)
    # ------------------------------------------------------------------
    def _idle(self) -> State:
        """Preparation: (origin), safety envelope, geofence, drop servo, GUIDED, arm."""
        # Indoors without GPS the EKF needs an origin before it can report a position.
        if self.config.gps_denied and self.config.set_origin_on_start:
            if not self.drone.set_origin(
                self.config.origin_lat, self.config.origin_lon, self.config.origin_alt
            ):
                # Not fatal: the FC may legitimately already have an origin (SITL with
                # GPS on). But it means local NED is anchored somewhere we did not
                # choose, so the flight log must say so.
                log.warning("[IDLE] Continuing with the origin the FC already had")

        self.failsafe.setup_safety_envelope()
        self.failsafe.setup_geofence()
        self.release.setup()
        self.release.reset()

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
        altitude = self.config.search_altitude if self.config.gps_denied else self.config.cruise_alt
        if not self.drone.takeoff(altitude):
            return self._fail("TAKEOFF_FAILED")
        if self.config.gps_denied:
            self.failsafe.start_phase("SEARCH", budget_s=self.config.search_timeout_s)
            return State.SEARCH
        self.failsafe.start_phase("ENROUTE")
        return State.ENROUTE

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
        Fine alignment over the target by visual servoing. Reads the camera's
        image offset (dx, dy) and nudges the drone in the body frame until it is
        centred, then drops.

        Axis mapping (downward-facing camera): dx -> body 'right', dy -> body
        'forward'. Each step is scaled by approach_gain and clamped to max_nudge_m,
        so corrections stay small and safe. With the MockCamera (dx=dy=0) the first
        frame is already centred and we go straight to the drop.
        """
        tol = self.config.centre_tolerance
        deadline = time.time() + self.config.phase_timeout_s

        while time.time() < deadline:
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
        On detection -> APPROACH; if the whole pattern is exhausted without a
        detection -> ABORT (return home).
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
                if self._poll_until_arrival_or_detection(north, east):
                    return State.APPROACH
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

    def _poll_until_arrival_or_detection(self, north: float, east: float) -> bool:
        """
        Continuous cadence: poll the camera while flying toward (north, east).
        Returns True as soon as the target is detected, False once the waypoint is
        reached without a detection.
        """
        deadline = time.time() + self.config.phase_timeout_s
        while time.time() < deadline:
            if self.camera.get_target_offset()["detected"]:
                log.info("[SEARCH] Target detected en route")
                return True
            pos = self.drone.get_local_position()
            if pos and math.hypot(north - pos["north"], east - pos["east"]) <= self.config.local_arrival_radius_m:
                return False
            time.sleep(0.2)
        return False

    def _approach(self) -> State:
        """
        Visual servoing toward a detected target until centred, then DROP. If the
        target is lost for too many frames in a row, fall back to SEARCH.
        """
        tol = self.config.centre_tolerance
        deadline = time.time() + self.config.phase_timeout_s
        lost = 0

        while time.time() < deadline:
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
    def _nudge_from_offset(self, dx: float, dy: float) -> None:
        """Turn an image offset (dx -> body right, dy -> body forward) into a clamped
        body-frame nudge. Shared by OVER_TARGET and APPROACH."""
        right = self._clamp(self.config.approach_gain * dx, self.config.max_nudge_m)
        forward = self._clamp(self.config.approach_gain * dy, self.config.max_nudge_m)
        log.info(f"[ALIGN] nudge fwd={forward:+.2f} right={right:+.2f}")
        self.drone.move_body_offset(forward, right)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        """Clamp 'value' into [-limit, +limit]."""
        return max(-limit, min(limit, value))

    def _drop(self) -> State:
        """Perform the release and verify it (FC: servo read-back; Pi: open-loop)."""
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
            self.drone.return_to_launch()
        else:
            log.info("[RECOVER] Landing here")
            self.drone.land()

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

        self.failsafe.start_phase("RECOVER")
        return State.RECOVER