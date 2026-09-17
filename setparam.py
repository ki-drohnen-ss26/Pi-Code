"""
setparam.py
===========
Set flight-controller parameters from the Pi, with read-back verification.

    python setparam.py LOG_DISARMED 1                      # e.g. capture a debugging session
    python setparam.py RNGFND1_GNDCLEAR 2 LOG_DISARMED 1  # several at once
    python setparam.py --show FENCE_ENABLE EK3_SRC1_POSZ  # read only, change nothing
    python setparam.py --reboot                            # reboot the FC, change nothing
    python setparam.py RNGFND1_GNDCLEAR 2 --reboot         # set, verify, then reboot
    python setparam.py --sim ...                           # against SITL

ROLE: this is a MANUAL OPERATOR tool - a human runs it from a shell as the GCS-equivalent,
and the mission/companion code never imports or calls it. It is NOT the owner of the flight
parameters either. Since the 2026-08-24 ownership decision Mission Planner and
params/flight_v2.param are the single source of truth for what the aircraft flies, and the
companion writes no FC parameter (the mission stack in drone.py/mission.py/failsafe.py issues
no PARAM_SET at all). Use setparam by hand for SITL experiments, one-off diagnostics (e.g.
LOG_DISARMED for a debugging session) and rebooting the FC - not to configure the flight set.

Every write is READ BACK and confirmed. A silently ignored parameter is the failure mode
that matters here: ArduPilot accepts a PARAM_SET for a name it does not know and simply
does nothing, so "I set it" and "it is set" are different statements. `param load` has
the same trap, which is how a parameter file can look like it applied while leaving the
value at its default.

REFUSES TO RUN WHILE ARMED. Changing navigation or failsafe parameters underneath a
flying aircraft is not something a convenience script should make easy.
"""

import sys
import time

from pymavlink import mavutil

from config import Config
from fctools import read_param, set_param, wait_for_vehicle

# Parameters that only take effect after a reboot. Warned about rather than blocked,
# because setting them and rebooting later is a legitimate workflow.
NEEDS_REBOOT = (
    "SERIAL", "RNGFND1_TYPE", "FLOW_TYPE", "GPS1_TYPE", "EK3_ENABLE", "AHRS_EKF_TYPE",
    "LOG_BACKEND_TYPE", "BRD_",
)



def main() -> None:
    argv = [a for a in sys.argv[1:]]
    simulate = "--sim" in argv or "--sitl" in argv
    reboot = "--reboot" in argv
    show_only = "--show" in argv
    for flag in ("--sim", "--sitl", "--reboot", "--show"):
        while flag in argv:
            argv.remove(flag)

    config = Config.sitl() if simulate else Config.pi()
    print(f"Connecting to {config.connection_string} ...")
    master = mavutil.mavlink_connection(config.connection_string, source_system=251)
    wait_for_vehicle(master) or sys.exit(1)
    print(f"Connected to system {master.target_system}")

    if master.motors_armed():
        print("\nREFUSING: the vehicle is ARMED. Disarm first.")
        raise SystemExit(1)

    # Quiet the telemetry so PARAM_VALUE replies are not lost in the stream.
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 0)
    time.sleep(1.5)
    while master.recv_match(blocking=False):
        pass

    if show_only:
        print()
        for name in argv:
            print(f"  {name:20s} = {read_param(master, name)}")
        return

    if len(argv) % 2 != 0:
        print("\nUsage: python setparam.py NAME VALUE [NAME VALUE ...]")
        raise SystemExit(2)

    failures = []
    pending_reboot = []
    print()
    for i in range(0, len(argv), 2):
        name, raw = argv[i], argv[i + 1]
        try:
            wanted = float(raw)
        except ValueError:
            print(f"  {name}: '{raw}' is not a number")
            failures.append(name)
            continue

        before = read_param(master, name)
        if before is None:
            print(f"  {name:20s} DOES NOT EXIST on this firmware - not set")
            failures.append(name)
            continue

        confirmed = set_param(master, name, wanted)
        if confirmed is None:
            print(f"  {name:20s} {before} -> ??? (no read-back, NOT confirmed)")
            failures.append(name)
        elif abs(confirmed - wanted) > 1e-4:
            print(f"  {name:20s} {before} -> {confirmed}  REJECTED (asked for {wanted})")
            failures.append(name)
        else:
            print(f"  {name:20s} {before} -> {confirmed}  ok")
            if any(name.startswith(p) for p in NEEDS_REBOOT):
                pending_reboot.append(name)

    if pending_reboot:
        print(f"\n  NOTE: {', '.join(pending_reboot)} only take effect after a reboot.")

    if failures:
        print(f"\n  {len(failures)} parameter(s) NOT set: {', '.join(failures)}")

    if reboot:
        print("\nRebooting the flight controller ...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        # Free the UDP port BEFORE rebinding it: two sockets bound to the same udpin
        # endpoint race for packets, and the old one winning looks exactly like
        # "FC did not come back" while the FC is in fact fine.
        master.close()
        time.sleep(3)
        came_back = False
        for attempt in range(1, 13):
            time.sleep(4)
            try:
                again = mavutil.mavlink_connection(config.connection_string,
                                                   source_system=251)
                if again.wait_heartbeat(timeout=6):
                    print(f"  FC back after ~{attempt * 4} s")
                    came_back = True
                    again.close()
                    break
                again.close()
            except Exception:
                pass
        if not came_back:
            print("  FC did not come back - check it")

    # After the reboot handling on purpose: a reboot must not turn failed writes into
    # exit code 0 (scripts and humans both read the exit code as "all values applied").
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
