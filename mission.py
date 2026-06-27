"""
mission.py
==========
The delivery state machine. Casts the loose command sequence into a clean,
extensible state machine:

    IDLE -> TAKEOFF -> ENROUTE -> OVER_TARGET -> DROP -> RTL -> DONE
                                                          (ABORT on failsafe)

Each state does exactly one thing and returns the next state. The failsafe is
checked before every step - if it fires, the machine jumps to ABORT (and from
there in a controlled way to RTL/DONE).
"""

import logging
import time
from enum import Enum, auto

from camera import Camera
from config import Config
from drone import Drone
from failsafe import FailsafeMonitor

log = logging.getLogger(__name__)


class State(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    ENROUTE = auto()
    OVER_TARGET = auto()
    DROP = auto()
    RTL = auto()
    ABORT = auto()
    DONE = auto()


class DeliveryMission:
    def __init__(self, drone: Drone, camera: Camera, failsafe: FailsafeMonitor, config: Config):
        self.drone = drone
        self.camera = camera
        self.failsafe = failsafe
        self.config = config
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
        """Preparation: geofence, configure drop servo, GUIDED, arm."""
        self.failsafe.setup_geofence()
        self.drone.configure_drop_servo()
        self.drone.reset_servo()

        if not self.drone.wait_ready_to_arm():
            return State.ABORT
        if not self.drone.set_mode("GUIDED"):
            return State.ABORT
        if not self.drone.arm():
            return State.ABORT

        self.failsafe.start_phase("TAKEOFF")
        return State.TAKEOFF

    def _takeoff(self) -> State:
        if self.drone.takeoff(self.config.cruise_alt):
            self.failsafe.start_phase("ENROUTE")
            return State.ENROUTE
        return State.ABORT

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

            right = self._clamp(self.config.approach_gain * dx, self.config.max_nudge_m)
            forward = self._clamp(self.config.approach_gain * dy, self.config.max_nudge_m)
            log.info(f"[CAM] Correcting (dx={dx:.2f}, dy={dy:.2f}) -> fwd={forward:+.2f} right={right:+.2f}")
            self.drone.move_body_offset(forward, right)
            time.sleep(self.config.nudge_settle_s)

        log.warning("[CAM] Timeout during target alignment")
        return State.ABORT

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        """Clamp 'value' into [-limit, +limit]."""
        return max(-limit, min(limit, value))

    def _drop(self) -> State:
        """Perform the release and verify it via the read-back servo value."""
        self.drone.drop()
        time.sleep(0.5)  # brief wait so the new value reaches the telemetry
        value = self.drone.read_servo(expected=self.config.drop_pwm)
        if value is not None and abs(value - self.config.drop_pwm) <= 50:
            log.info("[DROP] Release confirmed")
        else:
            log.warning("[DROP] servo value not as expected")
        time.sleep(1.0)
        self.drone.reset_servo()
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