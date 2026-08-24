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
    "Pre-arm checks (reading only - this tool never arms anything)": ["ARMING_CHECK"],
    "Geofence (the companion sets these - are they still ours?)": [
        "FENCE_ENABLE", "FENCE_TYPE", "FENCE_ALT_MAX", "FENCE_ACTION",
    ],
    "Battery failsafe (latches until the FC is power-cycled)": [
        "BATT_MONITOR", "BATT_LOW_VOLT", "BATT_CRT_VOLT", "BATT_FS_LOW_ACT",
        "BATT_FS_CRT_ACT", "BATT_CAPACITY",
    ],
    "Other failsafes": ["FS_GCS_ENABLE", "FS_GCS_TIMEOUT", "FS_THR_ENABLE", "FS_EKF_ACTION"],
    "Navigation limits (the companion sets these too)": [
        # RTL_ALT (cm) is the 4.5/4.6 name, RTL_ALT_M (m) the 4.7+ name. This variant is
        # 4.8.0-dev so RTL_ALT_M is the live one and RTL_ALT will read "not present" - we
        # list both so the output proves which firmware the FC is actually running.
        "WPNAV_SPEED", "WPNAV_SPEED_UP", "RTL_ALT", "RTL_ALT_M",
    ],
    "Position sensors": [
        "SERIAL5_PROTOCOL", "SERIAL5_BAUD", "FLOW_TYPE", "FLOW_ORIENT_YAW",
        "RNGFND1_TYPE", "RNGFND1_ORIENT",
        # RNGFND1_MIN_CM/_MAX_CM (cm) are the 4.6 names, RNGFND1_MIN/_MAX (m) the 4.7+
        # names. On this 4.8.0-dev FC the _CM pair reads "not present" and the metre pair
        # is live; on the main-repo 4.6.3 FC it is the other way round. One set showing
        # "not present" is the expected, informative result, not a fault.
        "RNGFND1_MIN_CM", "RNGFND1_MAX_CM", "RNGFND1_MIN", "RNGFND1_MAX",
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



def wait_for_vehicle(master, timeout=30):
    """Wait for a heartbeat FROM THE AUTOPILOT, not from whatever speaks first.

    pymavlink's wait_heartbeat() returns on the first heartbeat of any kind. Over
    mavlink-router that can be a ground station or another tool, and then
    target_system stays 0 - every later parameter request goes to nobody and the
    script simply hangs. drone.py has guarded against this for a while; these
    diagnostic tools had not.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        master.wait_heartbeat(timeout=2)
        if master.target_system != 0:
            return True
    print("\nNo autopilot heartbeat - target_system stayed 0.")
    print("Something is on the link, but nothing that identifies itself as a vehicle.")
    print("Check that:")
    print("  * the flight controller is powered (USB or battery),")
    print("  * mavlink-router is running:  systemctl status mavlink-router")
    print("  * it actually sees the FC:    journalctl -u mavlink-router -n 20")
    return False

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
    wait_for_vehicle(master) or sys.exit(1)
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
    altitudes = []          # EKF altitude - must NOT drift while the vehicle sits still
    sensors = None          # SYS_STATUS health bitmask
    armed_seen = False
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
            sensors = (msg.onboard_control_sensors_present,
                       msg.onboard_control_sensors_enabled,
                       msg.onboard_control_sensors_health)
        elif kind == "HEARTBEAT":
            armed_seen = armed_seen or master.motors_armed()
        elif kind == "GLOBAL_POSITION_INT":
            altitudes.append((time.time(), msg.relative_alt / 1000.0))
        elif kind == "RANGEFINDER":
            rng.append(msg.distance)
        elif kind in ("OPTICAL_FLOW", "OPTICAL_FLOW_RAD"):
            flow_q.append(getattr(msg, "quality", -1))

    print("\n--- Result ---")
    if battery:
        print(f"  Battery: {battery[0]:.2f} V, {battery[1]} % remaining")

    # --- sensor health, straight from the autopilot -------------------------
    # SYS_STATUS carries a present/enabled/health bit per sensor. This answers
    # "is the barometer alive" without touching the flight controller - which matters,
    # because a dead barometer removes the only height source that cannot be blocked
    # by something lying under the airframe.
    if sensors:
        present, enabled, health = sensors
        print("\n  Sensor health (from SYS_STATUS):")
        for bit, label in ((0x08, "barometer"), (0x100, "rangefinder"),
                           (0x40, "optical flow"), (0x04, "compass"),
                           (0x01, "gyro"), (0x02, "accelerometer")):
            if not present & bit:
                # On THIS no-baro build a missing barometer is the point of the build,
                # not a fault - say so, so nobody chases a "problem" that is by design.
                state = ("not present - EXPECTED on this no-baro build"
                         if bit == 0x08 else "not present")
            elif not enabled & bit:
                state = "present, DISABLED"
            elif not health & bit:
                state = "*** UNHEALTHY ***"
            else:
                state = "ok"
            print(f"    {label:14s} {state}")
        # Only alarm if a barometer IS present but unhealthy. A no-baro build reports it
        # "not present", handled above - that must not trip the do-not-fly warning.
        if (present & 0x08) and not (health & 0x08):
            print("\n  !! THE BAROMETER IS UNHEALTHY. ArduPilot needs it for altitude in")
            print("  !! every mode. Do not fly. And note that EK3_SRC1_POSZ = 1 (baro) is")
            print("  !! then NOT an available fallback for the rangefinder either.")

    # --- is the height estimate stable while standing still? ----------------
    # In the 2026-08-21 crash log the EKF altitude did not drift gently - it diverged
    # QUADRATICALLY, exactly like an inertial system with no height measurement at
    # all: -268 m ninety seconds after boot, -1070 m at the moment of arming three
    # minutes in, with the EKF "climb rate" at -12.6 m/s while the aircraft stood
    # motionless on the floor. EK3_SRC1_POSZ pointed at the rangefinder, EKF3 never
    # fused a single height measurement from it, and the barometer was not a
    # configured source - so the filter integrated accelerometer bias unchecked.
    # Even the first seconds of that are visible from the ground, which is what
    # this check is for - and on this no-baro build it is the ONLY thing that catches
    # the divergence, since there is no barometer to hold the height.
    if len(altitudes) >= 5 and not armed_seen:
        span = max(a for _, a in altitudes) - min(a for _, a in altitudes)
        seconds = altitudes[-1][0] - altitudes[0][0]
        drift = (altitudes[-1][1] - altitudes[0][1]) / seconds if seconds > 0 else 0.0
        print(f"\n  Reported altitude while disarmed: span {span:.2f} m over "
              f"{seconds:.0f} s, trend {drift * 100:+.1f} cm/s")
        if abs(drift) > 0.02 or span > 0.5:
            print("  !! THE HEIGHT ESTIMATE IS RUNNING AWAY while the aircraft is not")
            print("  !! moving. Every altitude-controlled mode (AltHold, Loiter, Land,")
            print("  !! Guided) will chase this. Do not fly. Check which source")
            print("  !! EK3_SRC1_POSZ selects and whether that sensor is usable.")
        else:
            print("  -> stable, as it should be when the aircraft is standing still")
    if rng:
        print(f"  Rangefinder: {len(rng)} samples, min={min(rng):.2f} max={max(rng):.2f} m")

        # A rangefinder that reads a constant ~0.02 m ON THE GROUND is healthy: the
        # MTF-01P sits a couple of centimetres above the floor and that IS the
        # distance. (We once mis-read this as a frozen/defective sensor - the crash
        # logs later showed it tracking the fatal climb 0.02 -> 4.94 m perfectly.)
        # The dangerous thing is the PAIRING with EK3_SRC1_POSZ = 2: EKF3 then has the
        # rangefinder as its ONLY height source, and in all five of our flight logs it
        # never fused a single height measurement from it - the vertical estimate ran
        # away quadratically (see the drift check above) until a fence-forced LAND flew
        # the aircraft into the ceiling at full throttle on 2026-08-21.
        posz = read_param(master, "EK3_SRC1_POSZ")
        if posz is not None and abs(posz - 2.0) < 0.1:
            # On the main aircraft POSZ=2 is the mistake and "switch to baro" is the fix.
            # HERE there is no baro to switch to - POSZ=2 (rangefinder) is the ONLY height
            # source this airframe has, so it is the INTENDED configuration. The danger is
            # unchanged (this is still exactly the 2026-08-21 setup), so instead of a fix
            # that does not exist we print the strict protocol that keeps it flyable.
            print("\n  ** EK3_SRC1_POSZ = 2: the rangefinder is the EKF's ONLY height source.")
            print("  ** On this no-baro airframe that is the INTENDED configuration - there")
            print("  ** is no barometer to fall back to. But it is also the exact setup of")
            print("  ** the 2026-08-21 crash (EKF fused no height, the vertical estimate")
            print("  ** diverged on the ground, a fence-forced LAND then went to full")
            print("  ** throttle), so the strict no-baro protocol is mandatory here:")
            print("  **   1. The altitude-drift line above IS the gate: if the reported")
            print("  **      height is running away while the aircraft stands still, DO NOT")
            print("  **      FLY - there is no second sensor to catch it.")
            print("  **   2. Bench hand-lift test first: lift the airframe by hand and")
            print("  **      confirm the EKF altitude FOLLOWS the real lift before any flight.")
            print("  **   3. Start flights only via --takeover: the pilot flies the first")
            print("  **      metre, the companion inherits an aircraft already stable in air.")
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
