"""
preflight.py
============
Read-only flight-controller inspection. Changes NOTHING - it reads parameters, listens
to what the autopilot says about itself, and prints a verdict.

    python preflight.py            # against the real FC (via mavlink-router)
    python preflight.py --sim      # against SITL

Why this exists as its own tool: when a flight ends in an unexplained landing, the
reason is almost always in the autopilot's own STATUSTEXT messages ("Battery failsafe",
"Fence breach", "EKF variance") - and those are only visible to something that is
listening at the time. `main.py` mirrors them during a mission, but by the time you are
standing there wondering why the aircraft refuses to arm, the mission is long over.

Run it, then try to arm with the transmitter. Whatever the autopilot objects to will
appear here, verbatim.
"""

import sys
import time

from pymavlink import mavutil

from config import Config

# Grouped so the output reads like a checklist rather than a parameter dump.
GROUPS = {
    "Arming": ["ARMING_CHECK"],
    "Geofence (the companion sets these - are they still ours?)": [
        "FENCE_ENABLE", "FENCE_TYPE", "FENCE_ALT_MAX", "FENCE_ACTION",
    ],
    "Battery failsafe (latches until the FC is power-cycled)": [
        "BATT_MONITOR", "BATT_LOW_VOLT", "BATT_CRT_VOLT", "BATT_FS_LOW_ACT",
        "BATT_FS_CRT_ACT", "BATT_CAPACITY",
    ],
    "Other failsafes": ["FS_GCS_ENABLE", "FS_GCS_TIMEOUT", "FS_THR_ENABLE", "FS_EKF_ACTION"],
    "Navigation limits (the companion sets these too)": [
        "WPNAV_SPEED", "WPNAV_SPEED_UP", "RTL_ALT",
    ],
    "Position sensors": [
        "SERIAL5_PROTOCOL", "SERIAL5_BAUD", "FLOW_TYPE", "FLOW_ORIENT_YAW",
        "RNGFND1_TYPE", "RNGFND1_MIN_CM", "RNGFND1_MAX_CM", "RNGFND1_ORIENT",
    ],
    "EKF sources": [
        "AHRS_EKF_TYPE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_POSZ",
        "EK3_SRC1_VELZ", "EK3_SRC1_YAW",
    ],
}

EKF_FLAGS = [
    (0x01, "ATTITUDE"), (0x02, "VELOCITY_HORIZ"), (0x04, "VELOCITY_VERT"),
    (0x08, "POS_HORIZ_REL"), (0x10, "POS_HORIZ_ABS"), (0x20, "POS_VERT_ABS"),
    (0x40, "POS_VERT_AGL"), (0x80, "CONST_POS_MODE"),
    (0x100, "PRED_POS_HORIZ_REL"), (0x200, "PRED_POS_HORIZ_ABS"), (0x400, "UNINITIALISED"),
]


def read_param(master, name, tries=3, timeout=1.5):
    for _ in range(tries):
        master.mav.param_request_read_send(
            master.target_system, master.target_component, name.encode(), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg and msg.param_id == name:
                return msg.param_value
    return None


def main():
    config = Config.sitl() if ("--sim" in sys.argv or "--sitl" in sys.argv) else Config.pi()
    print(f"Connecting to {config.connection_string} ...")
    master = mavutil.mavlink_connection(config.connection_string, source_system=252)
    master.wait_heartbeat(timeout=20)
    print(f"Connected to system {master.target_system}\n")

    # Throttle telemetry so PARAM_VALUE replies are not lost in the stream. Learned the
    # hard way: 38 parameter requests against a 4 Hz all-streams feed lose most replies,
    # and the missing ones look exactly like parameters that do not exist.
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 0)
    time.sleep(1.5)
    while master.recv_match(blocking=False):
        pass

    for title, names in GROUPS.items():
        print(f"--- {title} ---")
        for name in names:
            value = read_param(master, name)
            print(f"  {name:20s} = {'--- not present ---' if value is None else round(value, 4)}")
        print()

    # --- live state -----------------------------------------------------
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    print("--- Listening for 20 s ---")
    print("    Try to ARM with the transmitter now. Whatever the autopilot objects to")
    print("    appears below, in its own words.\n")
    severity = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
                4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}
    ekf_flags = None
    battery = None
    rng = []
    flow_q = []
    end = time.time() + 20
    while time.time() < end:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "STATUSTEXT":
            text = msg.text.decode("utf-8", "replace") if isinstance(msg.text, bytes) else msg.text
            print(f"  [FC/{severity.get(msg.severity, msg.severity)}] {text.strip()}")
        elif kind == "EKF_STATUS_REPORT":
            ekf_flags = msg.flags
        elif kind == "SYS_STATUS":
            battery = (msg.voltage_battery / 1000.0, msg.battery_remaining)
        elif kind == "RANGEFINDER":
            rng.append(msg.distance)
        elif kind in ("OPTICAL_FLOW", "OPTICAL_FLOW_RAD"):
            flow_q.append(getattr(msg, "quality", -1))

    print("\n--- Result ---")
    if battery:
        print(f"  Battery: {battery[0]:.2f} V, {battery[1]} % remaining")
    if rng:
        print(f"  Rangefinder: {len(rng)} samples, min={min(rng):.2f} max={max(rng):.2f} m")
    if flow_q:
        print(f"  Optical flow: {len(flow_q)} messages, quality min={min(flow_q)} max={max(flow_q)}")
    else:
        print("  Optical flow: no messages")
    if ekf_flags is not None:
        print(f"  EKF flags = 0x{ekf_flags:04x}")
        for bit, label in EKF_FLAGS:
            print(f"    {'x' if ekf_flags & bit else ' '} {label}")
        if not ekf_flags & 0x08:
            print("\n  POS_HORIZ_REL is NOT set - the mission's pre-arm gate waits for")
            print("  exactly this bit, so `main.py` would hang here. On a flow-only")
            print("  vehicle sitting on the floor that can be normal: optical flow needs")
            print("  height (~8 cm for the MTF-01P) before the EKF will trust it.")


if __name__ == "__main__":
    main()
