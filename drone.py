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
    _SEVERITY = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
                 4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}

    def __init__(self, config: Config):
        self.config = config
        self.master: Optional[mavutil.mavfile] = None
        self._last_heartbeat_sent: float = 0.0

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
        self.send_heartbeat()
        self.request_data_streams()
        self.log_autopilot_version()

    def wait_heartbeat(self, timeout: float = 30.0) -> None:
        """Block until a HEARTBEAT *from the autopilot* is received.

        Not every heartbeat on the link comes from the flight controller. Over
        mavlink-router a ground station announces itself as MAV_TYPE_GCS, and a
        MAVLink sensor (the MTF-01P speaks MAVLink too) has its own system id.
        pymavlink only locks `target_system` onto a heartbeat it recognises as a
        vehicle, but plain wait_heartbeat() returns on the *first* packet of any
        kind - and if that was the GCS, target_system stays 0 and every later
        get_mode()/is_armed() reads an empty state bucket. So we wait until
        pymavlink actually latched onto a vehicle.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.master.wait_heartbeat(timeout=1.0)
            if self.master.target_system != 0:
                log.info(
                    f"[HEARTBEAT] Connected to system {self.master.target_system}, "
                    f"component {self.master.target_component}"
                )
                return
        raise TimeoutError("No autopilot heartbeat (only GCS/sensor traffic?)")

    def send_heartbeat(self) -> None:
        """Announce ourselves as a ground station.

        Required for the FC-side GCS failsafe: ArduPilot only starts monitoring
        FS_GCS_* after it has seen at least one heartbeat from the configured GCS
        system id (SYSID_MYGCS, default 255). Setpoint traffic does NOT count. Without
        this, FS_GCS_ENABLE=1 is silently dead and the aircraft has no FC-side rescue
        if the companion dies.
        """
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
        self._last_heartbeat_sent = time.time()

    def tick(self) -> None:
        """Housekeeping to call from every polling loop. Cheap and non-blocking.

        Two jobs:
        1. Keep our GCS heartbeat alive (see send_heartbeat).
        2. Drain and LOG pending STATUSTEXT messages. This is the single most
           valuable diagnostic in the whole file: every pre-arm rejection, fence
           breach and EKF failsafe reason the autopilot produces arrives as
           STATUSTEXT, and a plain recv_match(type="SYS_STATUS") throws it away. Not
           logging it is why a rejected arming used to appear as a bare "result=4"
           with the actual reason ("PreArm: Check mag field") visible only in a
           MAVProxy console we do not have in flight.

        Draining also refreshes pymavlink's cached mode/armed state as a side effect,
        which is what makes the mode-change failsafe work.
        """
        now = time.time()
        if now - self._last_heartbeat_sent >= 1.0:
            self.send_heartbeat()
        while True:
            msg = self.master.recv_match(type="STATUSTEXT", blocking=False)
            if msg is None:
                return
            self._log_statustext(msg)

    def _log_statustext(self, msg) -> None:
        """Mirror one autopilot STATUSTEXT into our log, keeping its severity."""
        text = msg.text
        if isinstance(text, bytes):
            text = text.decode("utf-8", errors="replace")
        text = text.strip()
        label = self._SEVERITY.get(msg.severity, str(msg.severity))
        if msg.severity <= 4:          # EMERGENCY..WARNING
            log.warning(f"[FC/{label}] {text}")
        else:
            log.info(f"[FC/{label}] {text}")

    def log_autopilot_version(self, timeout: float = 3.0) -> Optional[str]:
        """Ask the FC which firmware it runs and record it.

        Worth a dedicated call because parameter names move between ArduPilot
        releases (RTL_ALT in cm on 4.5/4.6 vs RTL_ALT_M in m from 4.7,
        RNGFND1_MIN_CM vs RNGFND1_MIN). Every flight log should therefore state what
        it was actually flown against, instead of us guessing later.
        """
        self._command_long(
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        )
        msg = self.master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=timeout)
        if not msg:
            log.warning("[FC] No AUTOPILOT_VERSION received - firmware version unknown")
            return None
        raw = msg.flight_sw_version
        version = f"{(raw >> 24) & 0xFF}.{(raw >> 16) & 0xFF}.{(raw >> 8) & 0xFF}"
        log.info(f"[FC] ArduPilot flight software {version}")
        return version

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

    def set_origin(self, lat: float, lon: float, alt: float, timeout: float = 5.0) -> bool:
        """
        Tell the EKF where it is WITHOUT GPS, via SET_GPS_GLOBAL_ORIGIN. Indoors there is
        no GPS to seed the EKF origin/home, so the companion provides a reference. Any
        sensible lat/lon works - it only anchors the local NED frame and home. After this,
        LOCAL_POSITION_NED and home have a reference.

        lat/lon in degrees, alt in metres (AMSL). Returns True only if the origin we
        asked for is the one the autopilot ended up using.

        VERIFY, do not assume: ArduPilot drops SET_GPS_GLOBAL_ORIGIN silently when an
        origin is already set (and when the coordinates are invalid) - no ACK, no error
        message. We learned this the hard way: a SITL run that looked like it validated
        this feature had in fact been flying on the simulator's own origin while our
        log cheerfully claimed "EKF origin set".
        """
        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,
            int(lat * 1e7),     # degE7
            int(lon * 1e7),     # degE7
            int(alt * 1000.0),  # mm
        )
        self._command_long(
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,
        )
        msg = self.master.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=True, timeout=timeout)
        if not msg:
            log.warning("[ORIGIN] No GPS_GLOBAL_ORIGIN read-back - origin unverified")
            return False

        actual_lat, actual_lon = msg.latitude / 1e7, msg.longitude / 1e7
        # ~1e-4 deg is about 11 m: we only care that the FC uses OUR reference, not the
        # simulator's default half a world away.
        if abs(actual_lat - lat) < 1e-4 and abs(actual_lon - lon) < 1e-4:
            log.info(f"[ORIGIN] EKF origin confirmed: lat={actual_lat:.6f} lon={actual_lon:.6f}")
            return True
        log.warning(
            f"[ORIGIN] Rejected - FC uses lat={actual_lat:.6f} lon={actual_lon:.6f}, "
            f"we asked for lat={lat:.6f} lon={lon:.6f} (origin already set?)"
        )
        return False

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

    def read_param(self, name: str, tries: int = 3, timeout: float = 2.0):
        """Read one parameter without writing it. Returns the value or None.

        Read-only verification, nothing more: the companion inspects FC parameters -
        the fence check before arming, and the preflight sweep - but never writes one.
        Since the 2026-08-24 ownership decision Mission Planner and params/flight_v2.param
        are the single source of truth, so there is nothing to remember and put back;
        this reads and reports, it never restores a value because nothing here changes one.
        """
        for _ in range(tries):
            self.master.mav.param_request_read_send(
                self.master.target_system, self.master.target_component,
                name.encode("utf-8"), -1)
            deadline = time.time() + timeout
            while time.time() < deadline:
                msg = self.master.recv_match(type="PARAM_VALUE", blocking=True,
                                             timeout=timeout)
                if msg and msg.param_id == name:
                    return msg.param_value
        log.warning(f"[PARAM] Could not read {name}")
        return None

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

    def get_local_position(self, timeout: float = 2.0) -> Optional[dict]:
        """
        Read LOCAL_POSITION_NED: position in metres relative to the EKF origin
        (≈ the launch point), in the NED frame. Used for indoor navigation where
        there is no GPS - the position comes from optical flow.
        """
        msg = self.master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
        if not msg:
            return None
        return {"north": msg.x, "east": msg.y, "down": msg.z}

    def read_position_sensors(self, duration: float = 5.0) -> dict:
        """Listen for the sensors the GPS-denied position estimate is built on.

        Returns {"rangefinder_samples", "rangefinder_min", "rangefinder_max",
                 "flow_samples", "flow_quality_max"}.

        Why this exists: a broken optical-flow setup does not announce itself. The
        autopilot keeps streaming RANGEFINDER messages whose distance is simply always
        0.00, the EKF cannot scale the flow into a velocity, and the position estimate
        drifts - our params/README.md records 366 m of drift measured while the vehicle
        stood still. Everything downstream then looks healthy right up to the point where
        the position controller flies full-throttle after an imaginary error. Reading the
        raw sensor values before arming is the cheapest way to catch that on the ground.

        Note ArduPilot emits RANGEFINDER (its own message, with distance in metres) as
        well as DISTANCE_SENSOR (centimetres); we count both so a differently configured
        backend still registers.
        """
        rng: list = []
        flow_q: list = []
        deadline = time.time() + duration
        while time.time() < deadline:
            # Keep the GCS heartbeat alive: this runs for sensor_check_s (5 s) and is
            # called IN THE AIR right after takeoff - a silence that long is at the
            # FS_GCS_TIMEOUT (5 s) edge and would let the FC fire its GCS failsafe on
            # every climb. Deliberately NOT tick(): its STATUSTEXT drain would consume
            # the very RANGEFINDER/OPTICAL_FLOW messages this function is counting.
            if time.time() - self._last_heartbeat_sent >= 1.0:
                self.send_heartbeat()
            msg = self.master.recv_match(
                type=["RANGEFINDER", "DISTANCE_SENSOR", "OPTICAL_FLOW", "OPTICAL_FLOW_RAD"],
                blocking=True, timeout=1.0)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "RANGEFINDER":
                rng.append(msg.distance)                  # metres
            elif kind == "DISTANCE_SENSOR":
                rng.append(msg.current_distance / 100.0)  # centimetres -> metres
            else:
                flow_q.append(getattr(msg, "quality", 0))
        return {
            "rangefinder_samples": len(rng),
            "rangefinder_min": min(rng) if rng else None,
            "rangefinder_max": max(rng) if rng else None,
            "flow_samples": len(flow_q),
            "flow_quality_max": max(flow_q) if flow_q else None,
        }

    def get_rangefinder(self, timeout: float = 1.5) -> Optional[float]:
        """Latest raw rangefinder distance in METRES, or None if nothing arrives.

        Deliberately the SENSOR, not the EKF: in the 2026-08-21 crash logs the EKF
        altitude read +1070 m while the aircraft stood on the floor, while the
        rangefinder read a truthful 0.02 m throughout. Where the two disagree, the
        raw sensor is the one a takeover decision can lean on.
        """
        msg = self.master.recv_match(type=["RANGEFINDER", "DISTANCE_SENSOR"],
                                     blocking=True, timeout=timeout)
        if msg is None:
            return None
        if msg.get_type() == "RANGEFINDER":
            return msg.distance                    # metres
        return msg.current_distance / 100.0        # DISTANCE_SENSOR: centimetres

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

    def wait_ready_to_arm(self, timeout: float = 60.0, require_abs: bool = True) -> bool:
        """
        Wait until the EKF has a valid horizontal position estimate BEFORE arming. A
        GPS fix alone is NOT enough - the EKF needs a few seconds to converge, and
        arming earlier is rejected with 'Need position estimate' (worse with an active
        geofence). We wait for the relevant flag in EKF_STATUS_REPORT, which is exactly
        what the autopilot means by 'position estimate'.

        require_abs=True  -> EKF_POS_HORIZ_ABS (0x10): ABSOLUTE position, from GPS
                             (outdoor / Phase 1).
        require_abs=False -> EKF_POS_HORIZ_REL (0x08): RELATIVE position, from optical
                             flow (indoor / GPS-denied, Phase 2).
        """
        flag = 0x10 if require_abs else 0x08
        kind = "absolute (GPS)" if require_abs else "relative (optical flow)"
        log.info(f"[PREARM] Waiting for {kind} EKF position estimate ...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.tick()   # surfaces "PreArm: ..." messages while we wait
            msg = self.master.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=1.0)
            if msg and (msg.flags & flag):
                log.info("[PREARM] EKF position estimate ready")
                return True
        log.warning("[PREARM] Timeout: no EKF position estimate")
        return False

    def wait_ekf_flag(self, flag: int, timeout: float = 10.0) -> bool:
        """Wait for one EKF_STATUS_REPORT bit. True if it appears within `timeout`.

        Separate from wait_ready_to_arm() because the takeover path asks the same
        question at a different moment: not "may we arm" but "is the estimate good
        enough to accept an aircraft that is ALREADY flying".
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.tick()
            msg = self.master.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=1.0)
            if msg and (msg.flags & flag):
                return True
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
            # The ACK only carries a numeric result; the REASON ("PreArm: Check mag
            # field", "Arm: Gyros inconsistent") comes as STATUSTEXT, so drain it now.
            self.tick()

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
                self.tick()
        log.warning("[ARM] Arming failed - see the [FC/...] messages above for the reason")
        return False

    def takeoff(self, altitude: float, timeout: float = 30.0,
                settle_s: float = 3.0, tolerance: float = 0.5) -> bool:
        """
        GUIDED takeoff to 'altitude' metres (relative to launch).
        Requires GUIDED + armed. Returns True once the altitude is not just reached
        but HELD for `settle_s` within `tolerance` metres.

        Why the settling window: returning on the first sample above 95 % of the target
        reports success while the aircraft is still climbing. With the stock climb rate
        (WPNAV_SPEED_UP 250 cm/s) a 2 m takeoff sailed on to 4.5 m in our SITL runs,
        breached a 4 m altitude fence, and the FC switched out of GUIDED - while the
        mission happily continued sending waypoints to a vehicle that was no longer
        listening. Overshoot is a hall-ceiling problem, so we watch for it here.
        """
        # Scale the acceptance band to the target height. A fixed 0.5 m band
        # contains the GROUND for low bring-up altitudes: for a 0.5 m takeoff the
        # band |alt-0.5| <= 0.5 accepts 0.0 m, so a vehicle that never lifted would
        # "reach" the target and could pass the settle window still sitting on the
        # floor (found before the first 0.5 m milestone-1 flight on 2026-08-25).
        # eff_tol shrinks with altitude but never below 0.15 m (sensor noise floor)
        # and never above the passed tolerance: 0.5 m -> 0.2 m band, 1 m -> 0.4 m,
        # >= 1.25 m -> capped at `tolerance`.
        eff_tol = min(tolerance, max(0.15, 0.4 * altitude))
        self._command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)
        log.info(f"[TAKEOFF] Climbing to {altitude} m ...")

        deadline = time.time() + timeout
        stable_since: Optional[float] = None
        peak = 0.0
        while time.time() < deadline:
            self.tick()
            # If the FC leaves GUIDED mid-climb (pilot takeover, fence breach, EKF
            # failsafe), waiting out the rest of the timeout only delays the abort and
            # mislabels its cause. Stop immediately; the mission's abort path then asks
            # the FC who is in control before commanding anything.
            mode = self.master.flightmode
            if mode and mode != "GUIDED":
                log.warning(f"[TAKEOFF] FC left GUIDED during the climb (now {mode}) - "
                            f"stopping the wait")
                return False
            pos = self.get_position()
            if not pos:
                continue
            alt = pos["rel_alt"]
            peak = max(peak, alt)

            if abs(alt - altitude) <= eff_tol:
                if stable_since is None:
                    stable_since = time.time()
                    log.info(f"[TAKEOFF] Altitude reached ({alt:.1f} m), settling ...")
                elif time.time() - stable_since >= settle_s:
                    if peak > altitude + eff_tol:
                        log.warning(
                            f"[TAKEOFF] Overshot to {peak:.1f} m before settling at "
                            f"{alt:.1f} m - consider lowering WPNAV_SPEED_UP"
                        )
                    log.info(f"[TAKEOFF] Altitude stable at {alt:.1f} m")
                    return True
            elif stable_since is not None:
                # Drifted back out of the band - start the settling window again.
                log.info(f"[TAKEOFF] Altitude {alt:.1f} m left the band, re-settling ...")
                stable_since = None
        log.warning(f"[TAKEOFF] Timeout while climbing (peak {peak:.1f} m)")
        return False

    def goto(self, lat: float, lon: float, alt: float) -> None:
        """
        Fly to (lat, lon, alt) in GUIDED. Only sends the target; arrival is
        checked separately via wait_arrival().

        Note: uses global coordinates (GPS). For indoor use without GPS see
        goto_local() below and the frames table in docs/ARCHITECTURE.md -> there
        SET_POSITION_TARGET_LOCAL_NED + optical flow take over.
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
        the mission converts the camera's ground offset (dx/dy, in METRES - every
        Camera implementation reports metres, not image fractions) into a step and lets
        the FC fly it. Only the position fields are used; velocity/accel/yaw are ignored.
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
        True if a HEARTBEAT *from the flight controller* arrives within 'timeout'.
        Returns quickly when the link is healthy (heartbeats stream at a few Hz); it
        only blocks up to 'timeout' when the link is actually down. Used by the
        failsafe to detect a lost companion<->FC link.

        The source check is the whole point and used to be missing. On the aircraft the
        script does not talk to the FC directly: mavlink-router owns /dev/serial0 and
        fans the stream out to us, to a ground station and to a log. Accepting ANY
        heartbeat on that endpoint means a QGroundControl instance sitting on the same
        router keeps this returning True while the FC UART is dead - which is exactly
        the failure LINK_LOSS exists to catch. wait_heartbeat() above already documents
        the same trap for the same reason.

        Note we cannot use pymavlink's `condition=` argument for this: it is evaluated
        against pymavlink's last-message-per-type cache, which can still hold an old FC
        heartbeat and would let a fresh GCS packet through.
        """
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return False
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=remaining)
            if msg is None:
                return False
            # target_system is latched onto the vehicle by wait_heartbeat().
            if msg.get_srcSystem() == self.master.target_system:
                return True

    def wait_arrival(self, lat: float, lon: float, radius_m: float, timeout: float = 60.0) -> bool:
        """Wait until the drone is within 'radius_m' of the target.

        tick() belongs in here, not just in the local-NED twin below: this loop can run
        for the whole phase timeout, and without it our GCS heartbeat would fall silent
        for longer than FS_GCS_TIMEOUT (5 s). The autopilot would then fire its own GCS
        failsafe on a companion that is merely busy flying the leg it was told to fly.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.tick()
            pos = self.get_position()
            if pos:
                dist = self._haversine(pos["lat"], pos["lon"], lat, lon)
                if dist <= radius_m:
                    log.info(f"[GOTO] Target reached (distance {dist:.1f} m)")
                    return True
        log.warning("[GOTO] Timeout before arrival")
        return False

    def goto_local(self, north: float, east: float, down: float) -> None:
        """
        Fly to a position in the local NED frame (metres relative to the EKF origin),
        via SET_POSITION_TARGET_LOCAL_NED with MAV_FRAME_LOCAL_NED. This is the
        indoor / GPS-denied counterpart of goto(): no lat/lon needed.

        Note NED sign: 'down' is positive downward, so a height of h above launch is
        down = -h.
        """
        type_mask = 0b0000111111111000  # position only
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north, east, down,
            0, 0, 0,   # vx, vy, vz
            0, 0, 0,   # afx, afy, afz
            0, 0,      # yaw, yaw_rate
        )
        log.info(f"[GOTO] Local target: north={north:.1f} east={east:.1f} down={down:.1f} m")

    def wait_local_arrival(self, north: float, east: float, radius_m: float, timeout: float = 60.0) -> bool:
        """Wait until the drone is within 'radius_m' (horizontal) of a local-NED point."""
        deadline = time.time() + timeout
        worst = float("inf")
        while time.time() < deadline:
            self.tick()
            pos = self.get_local_position()
            if pos:
                dist = math.hypot(north - pos["north"], east - pos["east"])
                worst = min(worst, dist)
                if dist <= radius_m:
                    log.info(f"[GOTO] Local target reached (distance {dist:.1f} m)")
                    return True
        # Log how close we actually got - a huge residual points at a diverging
        # position estimate (optical flow without a valid rangefinder height), not at
        # a vehicle that is merely slow.
        log.warning(f"[GOTO] Timeout before local arrival (closest {worst:.1f} m)")
        return False

    def land(self) -> bool:
        """Command LAND. Returns False when the FC did not confirm the mode - the
        caller must not report a landing that was never accepted."""
        return self.set_mode("LAND")

    def return_to_launch(self) -> bool:
        return self.set_mode("RTL")

    def wait_disarmed(self, timeout: float = 60.0) -> bool:
        """
        Block until the motors report disarmed, pumping HEARTBEATs so the armed
        state stays current. Returns True if disarmed within 'timeout', else False.

        On False the caller deliberately does NOT force a disarm (see
        mission._recover): cutting the motors of a vehicle that may still be airborne
        is worse than leaving the LAND command standing. The kill switch lives on the
        transmitter, not in this script - which is why there is no companion-side disarm
        command at all; the FC disarms itself once the LAND touches down.

        Kept on Drone - rather than reading master directly in the mission - so the
        mission logic can be tested against a fake drone without a real MAVLink link.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.tick()
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