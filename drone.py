"""
drone.py
========
The central Drone class. It wraps the MAVLink connection (self.master) and
provides all low-level actions. Organised into four sections that map onto the
project tasks:

    1. Connection & heartbeat
    2. Reading telemetry
    3. Basic commands (mode, arm, takeoff, goto, land, RTL)
    4. Payload (servo release)

Important: this class does NOT know whether SITL or a real flight controller is
on the other end. That is by design.
"""

import logging
import math
import time
from typing import Optional

from pymavlink import mavutil

from config import Config

log = logging.getLogger(__name__)


class Drone:
    def __init__(self, config: Config):
        self.config = config
        self.master: Optional[mavutil.mavfile] = None

    # ==================================================================
    # 1. Connection & heartbeat
    # ==================================================================
    def connect(self) -> None:
        """Open the MAVLink connection and wait for the first heartbeat."""
        log.info(f"[CONNECT] Connecting to {self.config.connection_string} ...")
        self.master = mavutil.mavlink_connection(
            self.config.connection_string,
            baud=self.config.baud,
            source_system=self.config.gcs_system_id,
        )
        self.wait_heartbeat()
        self.request_data_streams()

    def wait_heartbeat(self) -> None:
        """Block until a HEARTBEAT is received. Proves the link is up."""
        self.master.wait_heartbeat()
        log.info(
            f"[HEARTBEAT] Connected to system {self.master.target_system}, "
            f"component {self.master.target_component}"
        )

    def request_data_streams(self, rate_hz: int = 4) -> None:
        """
        Ask the FC to send telemetry at a fixed rate. With SITL this usually
        flows already; over a serial link to a real FC it is needed, otherwise
        position data arrives only sporadically.
        """
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1,  # 1 = start
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _command_long(self, command: int, *params) -> None:
        """Send a COMMAND_LONG (up to 7 params, missing ones default to 0)."""
        args = list(params) + [0] * (7 - len(params))
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            command,
            0,  # confirmation
            *args,
        )

    def _wait_ack(self, command: int, timeout: float = 5.0) -> Optional[int]:
        """
        Wait for the COMMAND_ACK matching the given command.
        Returns: MAV_RESULT (0 = MAV_RESULT_ACCEPTED) or None on timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            ack = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
            if ack and ack.command == command:
                return ack.result
        return None

    def set_param(self, name: str, value: float, timeout: float = 3.0) -> float:
        """
        Set a parameter and read it back for confirmation.
        ArduPilot transfers all parameters as float (REAL32) over MAVLink,
        regardless of their internal type.
        """
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            name.encode("utf-8"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg and msg.param_id == name:
                log.info(f"[PARAM] {name} = {msg.param_value}")
                return msg.param_value
        raise TimeoutError(f"No confirmation for parameter {name}")

    # ==================================================================
    # 2. Reading telemetry
    # ==================================================================
    def get_mode(self) -> str:
        """Current flight mode as a string, e.g. 'GUIDED'."""
        return self.master.flightmode

    def is_armed(self) -> bool:
        """True if the motors are armed."""
        return self.master.motors_armed()

    def get_position(self, timeout: float = 2.0) -> Optional[dict]:
        """Read GLOBAL_POSITION_INT: lat/lon in degrees, alt/rel_alt in metres."""
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
        if not msg:
            return None
        return {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.alt / 1000.0,                # above mean sea level
            "rel_alt": msg.relative_alt / 1000.0,   # above launch point
        }

    def get_battery(self, timeout: float = 2.0) -> Optional[dict]:
        """Read SYS_STATUS: voltage [V], current [A], remaining capacity [%]."""
        msg = self.master.recv_match(type="SYS_STATUS", blocking=True, timeout=timeout)
        if not msg:
            return None
        return {
            "voltage": msg.voltage_battery / 1000.0,   # mV -> V
            "current": msg.current_battery / 100.0,    # cA -> A
            "remaining": msg.battery_remaining,        # in percent
        }

    def print_telemetry(self, duration_s: float = 10.0, interval_s: float = 1.0) -> None:
        """
        Print mode, armed state, position and battery cyclically for
        'duration_s' seconds. Test: values change when you fly manually in QGC.
        """
        end = time.time() + duration_s
        while time.time() < end:
            pos = self.get_position()
            batt = self.get_battery()
            mode = self.get_mode()
            armed = "ARMED" if self.is_armed() else "DISARMED"
            if pos and batt:
                log.info(
                    f"[TELE] {mode:<8} {armed:<8} "
                    f"alt={pos['rel_alt']:5.1f}m  "
                    f"lat={pos['lat']:.6f} lon={pos['lon']:.6f}  "
                    f"batt={batt['voltage']:.1f}V/{batt['remaining']}%"
                )
            time.sleep(interval_s)

    # ==================================================================
    # 3. Basic commands
    # ==================================================================
    def set_mode(self, mode_name: str, timeout: float = 5.0) -> bool:
        """Set a flight mode (e.g. 'GUIDED', 'RTL', 'LAND') and verify it."""
        mode_map = self.master.mode_mapping()
        if mode_name not in mode_map:
            raise ValueError(f"Unknown mode '{mode_name}'. Available: {list(mode_map)}")
        self.master.set_mode(mode_map[mode_name])

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
            if msg and self.master.flightmode == mode_name:
                log.info(f"[MODE] Mode is now {mode_name}")
                return True
        log.warning(f"[MODE] mode {mode_name} not confirmed")
        return False

    def wait_ready_to_arm(self, timeout: float = 60.0) -> bool:
        """
        Wait until the EKF has a valid absolute horizontal position estimate BEFORE
        arming. A GPS fix alone is NOT enough - the EKF needs a few more seconds to
        converge, and arming earlier is rejected with 'Need position estimate'
        (made worse by an active geofence). We therefore wait for the
        EKF_POS_HORIZ_ABS flag in EKF_STATUS_REPORT, which is exactly what the
        autopilot means by 'position estimate'.

        Indoors without GPS the relevant flag is EKF_POS_HORIZ_REL instead (the
        position then comes from optical flow); swap 0x10 for 0x08 there.
        """
        EKF_POS_HORIZ_ABS = 0x10  # bit 4 of EKF_STATUS_FLAGS = absolute horizontal position
        log.info("[PREARM] Waiting for EKF position estimate ...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=1.0)
            if msg and (msg.flags & EKF_POS_HORIZ_ABS):
                log.info("[PREARM] EKF position estimate ready")
                return True
        log.warning("[PREARM] Timeout: no EKF position estimate")
        return False

    def arm(self, timeout: float = 10.0, attempts: int = 5) -> bool:
        """
        Arm the motors. On real hardware, pre-arm checks (GPS, EKF, compass) can
        delay arming - hence several attempts with a pause, instead of giving up
        immediately or blocking forever.
        """
        for attempt in range(1, attempts + 1):
            self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            result = self._wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)

            if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                deadline = time.time() + timeout
                while time.time() < deadline:
                    self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
                    if self.is_armed():
                        log.info("[ARM] Motors are armed")
                        return True
            else:
                log.warning(f"[ARM] Attempt {attempt}/{attempts} rejected (result={result}); waiting ...")
                time.sleep(2.0)
        log.warning("[ARM] Arming failed (pre-arm check?)")
        return False

    def disarm(self) -> None:
        """Disarm the motors."""
        self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        log.info("[ARM] Disarm sent")

    def takeoff(self, altitude: float, timeout: float = 30.0) -> bool:
        """
        GUIDED takeoff to 'altitude' metres (relative to launch).
        Requires GUIDED + armed. Waits until ~95 % of the altitude is reached.
        """
        self._command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)
        log.info(f"[TAKEOFF] Climbing to {altitude} m ...")

        deadline = time.time() + timeout
        while time.time() < deadline:
            pos = self.get_position()
            if pos and pos["rel_alt"] >= altitude * 0.95:
                log.info(f"[TAKEOFF] Altitude reached ({pos['rel_alt']:.1f} m)")
                return True
        log.warning("[TAKEOFF] Timeout while climbing")
        return False

    def goto(self, lat: float, lon: float, alt: float) -> None:
        """
        Fly to (lat, lon, alt) in GUIDED. Only sends the target; arrival is
        checked separately via wait_arrival().

        Note: uses global coordinates (GPS). For indoor use without GPS see the
        README -> there SET_POSITION_TARGET_LOCAL_NED + optical flow take over.
        """
        # type_mask: use position only, ignore velocity/acceleration/yaw
        type_mask = 0b0000111111111000
        self.master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,   # vx, vy, vz
            0, 0, 0,   # afx, afy, afz
            0, 0,      # yaw, yaw_rate
        )
        log.info(f"[GOTO] Target set: lat={lat:.6f} lon={lon:.6f} alt={alt} m")

    def move_body_offset(self, forward: float, right: float, down: float = 0.0) -> None:
        """
        Nudge the vehicle by a small offset relative to its CURRENT position, in
        the body frame (x=forward, y=right, z=down [NED, so down is positive]),
        via SET_POSITION_TARGET_LOCAL_NED with MAV_FRAME_BODY_OFFSET_NED.

        This is the building block for visual servoing over the target in GUIDED:
        the mission converts the camera's image offset into a step and lets the FC
        fly it. Only the position fields are used; velocity/accel/yaw are ignored.
        """
        type_mask = 0b0000111111111000  # position only
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            type_mask,
            forward, right, down,
            0, 0, 0,   # vx, vy, vz
            0, 0, 0,   # afx, afy, afz
            0, 0,      # yaw, yaw_rate
        )
        log.info(f"[ALIGN] Body nudge forward={forward:+.2f} right={right:+.2f} m")

    def link_alive(self, timeout: float = 3.0) -> bool:
        """
        True if a HEARTBEAT from the FC arrives within 'timeout'. Returns quickly
        when the link is healthy (heartbeats stream at a few Hz); it only blocks up
        to 'timeout' when the link is actually down. Used by the failsafe to detect
        a lost companion<->FC link.
        """
        return self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout) is not None

    def wait_arrival(self, lat: float, lon: float, radius_m: float, timeout: float = 60.0) -> bool:
        """Wait until the drone is within 'radius_m' of the target."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            pos = self.get_position()
            if pos:
                dist = self._haversine(pos["lat"], pos["lon"], lat, lon)
                if dist <= radius_m:
                    log.info(f"[GOTO] Target reached (distance {dist:.1f} m)")
                    return True
        log.warning("[GOTO] Timeout before arrival")
        return False

    def land(self) -> None:
        self.set_mode("LAND")

    def return_to_launch(self) -> None:
        self.set_mode("RTL")

    def wait_disarmed(self, timeout: float = 60.0) -> bool:
        """
        Block until the motors report disarmed, pumping HEARTBEATs so the armed
        state stays current. Returns True if disarmed within 'timeout', else
        False (the caller then forces a disarm). Kept on Drone - rather than
        reading master directly in the mission - so the mission logic can be
        tested against a fake drone without a real MAVLink link.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
            if not self.is_armed():
                return True
        return False

    @staticmethod
    def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Distance in metres between two GPS points."""
        r = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlmb = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
        return r * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # ==================================================================
    # 4. Payload (servo release)
    # ==================================================================
    def configure_drop_servo(self) -> None:
        """
        Set SERVOx_FUNCTION = 0 ('Disabled'). In ArduPilot 'Disabled' effectively
        means 'controlled via MAVLink/mission' - only then does the FC not
        override your DO_SET_SERVO value. Needed once in SITL.
        """
        self.set_param(f"SERVO{self.config.drop_servo}_FUNCTION", 0)

    def drop(self) -> None:
        """Perform the release: set the servo to drop_pwm via DO_SET_SERVO."""
        self._command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            self.config.drop_servo,
            self.config.drop_pwm,
        )
        log.info(f"[DROP] Servo {self.config.drop_servo} -> {self.config.drop_pwm} PWM")

    def reset_servo(self) -> None:
        """Return the servo to its neutral position (hatch closed)."""
        self._command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            self.config.drop_servo,
            self.config.neutral_pwm,
        )

    def read_servo(self, timeout: float = 3.0, expected: Optional[int] = None) -> Optional[int]:
        """
        Read the current PWM value of the drop servo from SERVO_OUTPUT_RAW.
        SERVO_OUTPUT_RAW streams continuously, so stale messages may sit in the
        buffer. We therefore read until the timeout and keep the latest value;
        if 'expected' is set, we stop as soon as that value is present.
        """
        deadline = time.time() + timeout
        latest = None
        while time.time() < deadline:
            msg = self.master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.5)
            if msg:
                latest = getattr(msg, f"servo{self.config.drop_servo}_raw", None)
                if expected is not None and latest is not None and abs(latest - expected) <= 50:
                    break
        log.info(f"[DROP] Read-back servo value: {latest} PWM")
        return latest