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

import os
import sys
import time

from pymavlink import mavutil

import paramcheck
from config import Config
from fctools import read_param, wait_for_vehicle

# Grouped so the output reads like a checklist rather than a parameter dump.
GROUPS = {
    "Pre-arm checks (reading only - this tool never arms anything)": ["ARMING_CHECK"],
    "Geofence (read-only - the companion never sets these; it refuses to fly if one is armed)": [
        "FENCE_ENABLE", "FENCE_TYPE", "FENCE_ALT_MAX", "FENCE_ACTION",
    ],
    "Battery failsafe (latches until the FC is power-cycled)": [
        "BATT_MONITOR", "BATT_LOW_VOLT", "BATT_CRT_VOLT", "BATT_FS_LOW_ACT",
        "BATT_FS_CRT_ACT", "BATT_CAPACITY",
    ],
    "Other failsafes": ["FS_GCS_ENABLE", "FS_GCS_TIMEOUT", "FS_THR_ENABLE", "FS_EKF_ACTION"],
    "Navigation limits (read-only - verified against the documented flight set)": [
        "WPNAV_SPEED", "WPNAV_SPEED_UP", "RTL_ALT",
    ],
    "Position sensors": [
        "SERIAL5_PROTOCOL", "SERIAL5_BAUD", "FLOW_TYPE", "FLOW_ORIENT_YAW",
        "RNGFND1_TYPE", "RNGFND1_MIN_CM", "RNGFND1_MAX_CM", "RNGFND1_ORIENT",
        "RNGFND1_GNDCLEAR",
    ],
    "EKF sources": [
        "AHRS_EKF_TYPE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_POSZ",
        "EK3_SRC1_VELZ", "EK3_SRC1_YAW",
    ],
    "Flight modes (switch mapping)": [
        "FLTMODE_CH", "FLTMODE1", "FLTMODE2", "FLTMODE3", "FLTMODE4", "FLTMODE5",
        "FLTMODE6",
    ],
}

EKF_FLAGS = [
    (0x01, "ATTITUDE"), (0x02, "VELOCITY_HORIZ"), (0x04, "VELOCITY_VERT"),
    (0x08, "POS_HORIZ_REL"), (0x10, "POS_HORIZ_ABS"), (0x20, "POS_VERT_ABS"),
    (0x40, "POS_VERT_AGL"), (0x80, "CONST_POS_MODE"),
    (0x100, "PRED_POS_HORIZ_REL"), (0x200, "PRED_POS_HORIZ_ABS"), (0x400, "UNINITIALISED"),
]



def _expected_path_from_argv(argv, simulated):
    """--expected PATH, otherwise the highest-numbered published set.

    Resolved by version rather than by a hard-coded name so publishing a v3 needs no
    source edit here (see paramcheck.newest_flight_set). A simulated run is compared
    against the generated mirror, because the mirror deviates from the aircraft on
    purpose and measuring SITL against the flight set would report those intended
    deviations as faults."""
    if "--expected" in argv:
        i = argv.index("--expected")
        if i + 1 < len(argv):
            return argv[i + 1]
    return paramcheck.newest_flight_set(simulated=simulated)


def main():
    sim = "--sim" in sys.argv or "--sitl" in sys.argv
    config = Config.sitl() if sim else Config.pi()

    # Read-only verification against the documented flight parameter set (team decision
    # 2026-08-24: Mission Planner + params/flight_v2.param are the single source of
    # truth). A missing file downgrades to "no verification", never a crash.
    expected_path = _expected_path_from_argv(sys.argv, sim)
    expected = paramcheck.load_param_file(expected_path)
    if expected is None:
        print(f"Note: no expected parameter file at "
              f"{expected_path or paramcheck.PARAMS_DIR} - skipping the read-only "
              f"comparison against the documented flight set.\n")
    else:
        print(f"Verifying against {expected_path} ({len(expected)} documented "
              f"parameters).\n")
    mismatches = []   # (name, live_value, expected_value)
    checked = 0
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
            # tries/timeout kept at preflight's historical 3/1.5 (fctools defaults to 4/2.0).
            value = read_param(master, name, tries=3, timeout=1.5)
            shown = "--- not present ---" if value is None else round(value, 4)
            suffix = ""
            # Compare against the documented set when we have both a live reading and an
            # expected value. FLTMODE is deliberately NOT special-cased: v2 still carries
            # 0 there, and a MISMATCH against a transmitter that has been remapped is
            # exactly the point of the feature (v3 will carry the real mapping).
            if expected is not None and name in expected and value is not None:
                checked += 1
                if abs(value - expected[name]) <= 1e-4:
                    suffix = "   ok"
                else:
                    suffix = f"   EXPECTED {round(expected[name], 4)} <-- MISMATCH"
                    mismatches.append((name, value, expected[name]))
            print(f"  {name:20s} = {shown}{suffix}")
        print()

    # --- read-only verdict against the documented flight set --------------------
    if expected is not None:
        print("--- Parameter verification (read-only) ---")
        print(f"  Checked {checked} documented parameter(s); {len(mismatches)} mismatch(es).")
        if mismatches:
            print("  !! The FC differs from the documented flight set "
                  f"({os.path.basename(expected_path)}). Per the 2026-08-24 ownership")
            print("  !! decision the companion does NOT write these. Either change the FC")
            print("  !! back via Mission Planner, or capture the aircraft and publish a")
            print("  !! new versioned param file. Differing parameters:")
            for name, live, want in mismatches:
                print(f"       {name:20s} FC={round(live, 4)}  documented={round(want, 4)}")
        else:
            print("  -> the FC matches the documented flight set.")
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
                state = "not present"
            elif not enabled & bit:
                state = "present, DISABLED"
            elif not health & bit:
                state = "*** UNHEALTHY ***"
            else:
                state = "ok"
            print(f"    {label:14s} {state}")
        if (present & 0x08) and not (health & 0x08):
            print("\n  !! THE BAROMETER IS UNHEALTHY. It is not the EKF height source here")
            print("  !! (the rangefinder is, by our own configuration choice), but it is")
            print("  !! our independent altitude witness/reference, and stock 4.6.3 will")
            print("  !! not even boot without it. Do not fly.")

    # --- is the height estimate stable while standing still? ----------------
    # This drift line is the ground GO/NO-GO gate, and the crash chain behind it is
    # unchanged: in the 2026-08-21 crash log the EKF altitude did not drift gently - it
    # diverged QUADRATICALLY, exactly like an inertial system with no height measurement
    # at all: -268 m ninety seconds after boot, -1070 m at the moment of arming three
    # minutes in, with the EKF "climb rate" at -12.6 m/s while the aircraft stood
    # motionless on the floor. EK3_SRC1_POSZ pointed at the rangefinder and EKF3 fused
    # not a single height measurement from it, so the filter integrated accelerometer
    # bias unchecked. What the 2026-08-25 SITL work refined is only WHY nothing was
    # fused: the RNGFND1_MIN_CM validity floor. A landed reading below that floor is
    # flagged out-of-range-low, the driver hands the EKF nothing, and with the
    # rangefinder as the only height source there is nothing to correct the vertical
    # estimate. In SITL that is provably the blocker (the landed reading is 0.00 m, below
    # any non-zero MIN_CM); on the real aircraft the landed reading of 0.02 m sits one
    # centimetre above a 0.01 m (MIN_CM 1) floor, so the same mechanism is the leading
    # hypothesis pending the colleague diff. Even the first seconds of that divergence
    # are visible from the ground, which is what this check is for.
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
        # With EK3_SRC1_POSZ = 2 the rangefinder is the EKF's ONLY height source, and in
        # all five of our flight logs EKF3 never fused a single height measurement from
        # it - the vertical estimate ran away quadratically (see the drift check above)
        # until a fence-forced LAND flew the aircraft into the ceiling at full throttle on
        # 2026-08-21. The 2026-08-25 SITL work showed the non-fusion is NOT the POSZ=2
        # source choice itself but the RNGFND1_MIN_CM validity floor rejecting the landed
        # reading: below the floor the driver reports out-of-range-low and the EKF gets
        # nothing. Proven in SITL; the leading hypothesis on the real aircraft, whose
        # 0.02 m landed reading clears a 0.01 m (MIN_CM 1) floor by a single centimetre.
        posz = read_param(master, "EK3_SRC1_POSZ", tries=3, timeout=1.5)
        if posz is not None and abs(posz - 2.0) < 0.1:
            print("\n  !! EK3_SRC1_POSZ = 2: the rangefinder is the EKF's ONLY height source.")
            print("  !! The task asks for altitude hold using the LiDAR; which EKF source")
            print("  !! carries the height is ours to pick, and the barometer (POSZ = 1)")
            print("  !! stays available if we decide the evidence points that way. It")
            print("  !! WORKS as configured: milestone 2 flew fully green under POSZ=2 in")
            print("  !! SITL (2026-08-25). It was ALSO the configuration of the 2026-08-21")
            print("  !! crash (the EKF fused no height, the vertical estimate diverged on")
            print("  !! the ground, a fence-forced LAND went to full throttle) - but the")
            print("  !! 2026-08-25 SITL work showed the blocker was the RNGFND1_MIN_CM")
            print("  !! validity floor rejecting the landed reading, NOT POSZ=2 itself. So")
            print("  !! above is the GO/NO-GO gate: drifting on the ground = the EKF is")
            print("  !! fusing no height = DO NOT FLY. If it drifts, check RNGFND1_MIN_CM")
            print("  !! (the validity floor - the landed reading must clear it; SITL proved")
            print("  !! this, and ours clears a 0.01 m floor by only 1 cm) and")
            print("  !! RNGFND1_GNDCLEAR (the expected on-ground reading in cm - ours is 5,")
            print("  !! the parameter's own minimum; the true mounting is ~2 cm),")
            print("  !! and run the parameter diff against the colleague team's working")
            print("  !! POSZ=2 aircraft. Before the first flight of a session, do the")
            print("  !! hand-lift test: the EKF altitude must follow a real lift (fclog.py")
            print("  !! shows it).")
    if flow_q:
        print(f"  Optical flow: {len(flow_q)} messages, quality min={min(flow_q)} max={max(flow_q)}")
    else:
        print("  Optical flow: no messages")

    # No rangefinder AND no flow, on the simulator, is almost never a broken sensor: it
    # is a fresh or wiped SITL still at firmware defaults (RNGFND1_TYPE=0, FLOW_TYPE=0)
    # that never had the indoor profile loaded - the exact state that aborts a mission
    # with NO_RANGEFINDER_DATA and gives no hint the fix is one line in this console.
    # (Only for --sim/--sitl: on the real FC a dead stream is a hardware fault, not a
    # forgotten param file.)
    if sim and not rng and not flow_q:
        print("\n  Neither the rangefinder nor optical flow produced a single message. On")
        print("  SITL that is almost always a fresh or wiped simulator still at firmware")
        print("  defaults (RNGFND1_TYPE=0, FLOW_TYPE=0), not a broken sensor. Load the")
        print("  SITL mirror of the flight set in the MAVProxy console. Load it")
        print("  TWICE: the RNGFND1_* sub-parameters only exist after the first pass sets")
        print("  RNGFND1_TYPE and the FC reboots, so on pass one the alphabetically-earlier")
        print("  RNGFND1_GNDCLEAR/MAX_CM/MIN_CM lines are discarded as unknown:")
        print("      param load params/sitl_flight_v2.parm")
        print("      reboot")
        print("      param load params/sitl_flight_v2.parm")
        print("      reboot")
        print("  (see params/README.md, incl. the SIM_TERRAIN trap that pins the")
        print("  rangefinder at a constant 0.00 m.)")
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
