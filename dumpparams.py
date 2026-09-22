"""
dumpparams.py
=============
Capture the live flight controller's complete parameter set into a versioned file.

    python dumpparams.py                    # -> params/flight_v<next>.param
    python dumpparams.py --out /tmp/x.param # write somewhere else
    python dumpparams.py --sim              # capture SITL instead of the aircraft
    python dumpparams.py --stdout           # print, write nothing

ROLE: this is the "publish" half of the parameter workflow, and a MANUAL OPERATOR tool
like preflight.py and setparam.py. The mission code never imports it.

WHY IT EXISTS
-------------
Since the 2026-08-24 ownership decision the FC parameters have exactly one owner:
Mission Planner plus the published, versioned flight set in params/. The companion
verifies the live aircraft against that file before every mission
(`failsafe.verify_flight_parameters()`) and writes nothing. That loop only closes if
publishing a new version is EASY - otherwise the honest answer to "the aircraft changed"
becomes "switch the check off".

So the whole update procedure is two commands:

    python dumpparams.py                                   # publish flight_v<next>
    python params/generate_sitl_flight_params.py           # regenerate the SITL mirror

Nothing else changes: both the mission check and preflight.py resolve the published set
by VERSION (highest `flight_v<N>.param`), so a v3 is picked up with no source edit.
Commit both files - the point of a versioned set is that a flight log can be matched to
the exact configuration it was flown under.

FORMAT
------
`NAME,VALUE`, sorted, which is what Mission Planner's "Save to file" writes and what
`params/flight_v2.param` already is. Integer-valued parameters are written without a
decimal point so a diff against a Mission Planner export stays readable.

This tool is READ-ONLY on the aircraft: it requests parameters and writes a local file.
It never sends a PARAM_SET.
"""

import os
import sys
import time

from pymavlink import mavutil

import paramcheck
from config import Config
from fctools import wait_for_vehicle


def next_version_path(simulated: bool) -> str:
    """params/flight_v<highest+1>.param, or v1 when nothing is published yet."""
    current = paramcheck.newest_flight_set(simulated=simulated)
    version = 1
    if current:
        digits = "".join(c for c in os.path.basename(current) if c.isdigit())
        version = int(digits) + 1 if digits else 1
    name = f"sitl_flight_v{version}.parm" if simulated else f"flight_v{version}.param"
    return os.path.join(paramcheck.PARAMS_DIR, name)


def fetch_all(master, timeout: float = 90.0) -> dict:
    """Download the full parameter list. Returns {NAME: value}.

    PARAM_REQUEST_LIST is answered with one PARAM_VALUE per parameter, each carrying
    `param_count`, so we know when the set is complete instead of guessing from a
    silence. Missing replies are re-requested individually at the end: on a busy link
    a handful of the ~1600 messages are routinely lost, and a dump with holes in it
    would be published as if it were the aircraft's configuration.
    """
    master.mav.param_request_list_send(master.target_system, master.target_component)
    values: dict = {}
    indexes: set = set()
    expected_count = None
    deadline = time.time() + timeout
    last_message = time.time()

    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        if msg is None:
            # Two seconds of silence after the stream has started means it is over.
            if values and time.time() - last_message > 2.0:
                break
            continue
        last_message = time.time()
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode("utf-8", "replace")
        values[name.strip("\x00")] = msg.param_value
        indexes.add(msg.param_index)
        expected_count = msg.param_count
        if expected_count and len(values) >= expected_count:
            break

    if expected_count and len(values) < expected_count:
        missing = sorted(set(range(expected_count)) - indexes)
        print(f"  {len(missing)} of {expected_count} parameters did not arrive - "
              f"re-requesting them individually ...")
        for index in missing:
            master.mav.param_request_read_send(
                master.target_system, master.target_component, b"", index)
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg:
                name = msg.param_id
                if isinstance(name, bytes):
                    name = name.decode("utf-8", "replace")
                values[name.strip("\x00")] = msg.param_value
        still_missing = expected_count - len(values)
        if still_missing > 0:
            print(f"  WARNING: {still_missing} parameter(s) still missing. This dump is "
                  f"INCOMPLETE - do not publish it; run the tool again.")
    return values


def format_value(value: float) -> str:
    """Integers without a decimal point, everything else with the digits it needs."""
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return f"{value:g}"


def main() -> None:
    argv = sys.argv[1:]
    simulated = "--sim" in argv or "--sitl" in argv
    to_stdout = "--stdout" in argv

    out_path = None
    if "--out" in argv:
        index = argv.index("--out")
        if index + 1 < len(argv):
            out_path = argv[index + 1]
    if out_path is None and not to_stdout:
        out_path = next_version_path(simulated)

    config = Config.sitl() if simulated else Config.pi()
    print(f"Connecting to {config.connection_string} ...")
    master = mavutil.mavlink_connection(config.connection_string, source_system=250)
    if not wait_for_vehicle(master):
        raise SystemExit(1)
    print(f"Connected to system {master.target_system}")

    # Quiet the telemetry first: ~1600 PARAM_VALUE replies compete with a 4 Hz
    # all-streams feed, and the ones that lose look exactly like parameters that do
    # not exist (the same trap preflight.py documents for single reads).
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 0)
    time.sleep(1.5)
    while master.recv_match(blocking=False):
        pass

    print("Downloading the parameter list (this takes a while over a slow link) ...")
    values = fetch_all(master)
    print(f"  {len(values)} parameters received")
    if not values:
        raise SystemExit("No parameters received - nothing written.")

    lines = [f"{name},{format_value(value)}" for name, value in sorted(values.items())]
    if to_stdout:
        print("\n".join(lines))
        return
    tmp_path = f"{out_path}.tmp"
    with open(tmp_path, "w") as handle:
        handle.write("\n".join(lines) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp_path, out_path)
    print(f"\nWrote {len(lines)} parameters to {out_path}")
    if not simulated:
        print("\nNext steps:")
        print("  1. Review the diff against the previous version - a published flight "
              "set is a decision, not a snapshot.")
        print("  2. Regenerate the SITL mirror so the simulator flies the same set:")
        print("       python params/generate_sitl_flight_params.py")
        print("  3. Commit both files and note what changed in params/README.md.")
        print("The mission check and preflight.py pick the highest version "
              "automatically - there is no path to edit in the source.")


if __name__ == "__main__":
    main()
