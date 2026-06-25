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

import time
from enum import Enum, auto

from camera import Camera
from config import Config
from drone import Drone
from failsafe import FailsafeMonitor


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
        print("\n=== MISSION START ===")
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
                    print(f"\n[FAILSAFE] ABORT due to: {reason}")
                    self.state = State.ABORT
                    continue

            print(f"\n--- State: {self.state.name} ---")
            self.state = dispatch[self.state]()

        print("\n=== MISSION END ===")

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
        Fine alignment over the target using the camera. With the MockCamera
        (dx=dy=0) the drone is centred immediately and goes straight to the drop.
        Later the real camera provides real dx/dy -> the correction loop goes
        here (small position commands).
        """
        tol = 0.15  # allowed offset below which we count as "centred"
        deadline = time.time() + self.config.phase_timeout_s

        while time.time() < deadline:
            offset = self.camera.get_target_offset()
            if not offset["detected"]:
                print("[CAM] No target detected, waiting ...")
                time.sleep(0.5)
                continue

            if abs(offset["dx"]) <= tol and abs(offset["dy"]) <= tol:
                print(f"[CAM] Centred (dx={offset['dx']}, dy={offset['dy']})")
                return State.DROP

            # --- placeholder for the real correction ---
            # Here one would nudge the drone based on dx/dy, e.g. via
            # SET_POSITION_TARGET_LOCAL_NED in the body frame. Never hit with mock.
            print(f"[CAM] Correcting position (dx={offset['dx']}, dy={offset['dy']})")
            time.sleep(0.5)

        print("[CAM] Timeout during target alignment")
        return State.ABORT

    def _drop(self) -> State:
        """Perform the release and verify it via the read-back servo value."""
        self.drone.drop()
        time.sleep(0.5)  # brief wait so the new value reaches the telemetry
        value = self.drone.read_servo(expected=self.config.drop_pwm)
        if value is not None and abs(value - self.config.drop_pwm) <= 50:
            print("[DROP] Release confirmed")
        else:
            print("[DROP] WARNING: servo value not as expected")
        time.sleep(1.0)
        self.drone.reset_servo()
        self.failsafe.start_phase("RTL")
        return State.RTL

    def _rtl(self) -> State:
        """Return to launch. Waits until the drone has landed and disarmed."""
        self.drone.return_to_launch()
        deadline = time.time() + self.config.phase_timeout_s
        while time.time() < deadline:
            self.drone.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
            if not self.drone.is_armed():
                print("[RTL] Landed and disarmed")
                return State.DONE
        print("[RTL] Timeout - forcing disarm")
        self.drone.disarm()
        return State.DONE

    def _abort(self) -> State:
        """Controlled abort: return to launch (RTL)."""
        print(f"[ABORT] Reason: {self.abort_reason} -> RTL")
        self.failsafe.start_phase("RTL")
        return State.RTL