"""
mission.py
==========
The delivery state machine. Two paths share the same machine, chosen by
config.gps_denied:

    GPS / outdoor (Phase 1):
        IDLE -> TAKEOFF -> ENROUTE -> OVER_TARGET -> DROP -> RTL -> DONE
    Indoor / GPS-denied (Phase 2):
        IDLE -> TAKEOFF -> SEARCH -> APPROACH -> DROP -> RTL -> DONE
                              ^_________|  (target lost)
                                                      (ABORT on failsafe)

Each state does exactly one thing and returns the next state. The failsafe is
checked before every step - if it fires, the machine jumps to ABORT (and from
there in a controlled way to RTL/DONE).
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
    RTL = auto()
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
            State.RTL: self._rtl,
            State.ABORT: self._abort,
        }

        while self.state != State.DONE:
            # Failsafe check before each state (unless we are already aborting/landing)
            if self.state not in (State.ABORT, State.RTL):
                reason = self.failsafe.check()
                if reason:
                    self.abort_reason = reason
                    log.info(f"\n[FAILSAFE] ABORT due to: {reason}")
                    self.state = State.ABORT
                    continue

            log.info(f"\n--- State: {self.state.name} ---")
            self.state = dispatch[self.state]()

        log.info("\n=== MISSION END ===")

    # ------------------------------------------------------------------
    # States (each returns the next state)
    # ------------------------------------------------------------------
    def _idle(self) -> State:
        """Preparation: (origin), geofence, configure drop servo, GUIDED, arm."""
        # Indoors without GPS the EKF needs an origin before it can report a position.
        if self.config.gps_denied and self.config.set_origin_on_start:
            self.drone.set_origin(
                self.config.origin_lat, self.config.origin_lon, self.config.origin_alt
            )
        self.failsafe.setup_geofence()
        self.release.setup()
        self.release.reset()

        # Indoor (GPS-denied) needs only the RELATIVE EKF position (optical flow);
        # outdoor needs the ABSOLUTE one (GPS).
        if not self.drone.wait_ready_to_arm(require_abs=not self.config.gps_denied):
            return State.ABORT
        if not self.drone.set_mode("GUIDED"):
            return State.ABORT
        if not self.drone.arm():
            return State.ABORT

        self.failsafe.start_phase("TAKEOFF")
        return State.TAKEOFF

    def _takeoff(self) -> State:
        altitude = self.config.search_altitude if self.config.gps_denied else self.config.cruise_alt
        if not self.drone.takeoff(altitude):
            return State.ABORT
        if self.config.gps_denied:
            self.failsafe.start_phase("SEARCH")
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
        return State.ABORT

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

        log.warning("[CAM] Timeout during target alignment")
        return State.ABORT

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
                self.drone.wait_local_arrival(
                    north, east, self.config.local_arrival_radius_m,
                    timeout=self.config.phase_timeout_s,
                )
                time.sleep(self.config.look_settle_s)
                if self.camera.get_target_offset()["detected"]:
                    log.info(f"[SEARCH] Target detected near ({north:.1f}, {east:.1f})")
                    return State.APPROACH

        log.warning("[SEARCH] Pattern exhausted, no target found")
        self.abort_reason = "TARGET_NOT_FOUND"
        return State.ABORT

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

        log.warning("[APPROACH] Timeout")
        return State.ABORT

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
        self.failsafe.start_phase("RTL")
        return State.RTL

    def _rtl(self) -> State:
        """Return to launch. Waits until the drone has landed and disarmed."""
        self.drone.return_to_launch()
        if self.drone.wait_disarmed(self.config.phase_timeout_s):
            log.info("[RTL] Landed and disarmed")
        else:
            log.warning("[RTL] Timeout - forcing disarm")
            self.drone.disarm()
        return State.DONE

    def _abort(self) -> State:
        """Controlled abort: return to launch (RTL)."""
        log.info(f"[ABORT] Reason: {self.abort_reason} -> RTL")
        self.failsafe.start_phase("RTL")
        return State.RTL