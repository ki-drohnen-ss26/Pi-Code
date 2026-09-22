"""
sitl.py
=======
Start SITL already carrying the flight parameters, in one command.

    python sitl.py                  # ArduCopter SITL, wiped, flight set loaded
    python sitl.py --speedup 5      # same, five times faster than real time
    python sitl.py --no-wipe        # keep the simulator's existing parameter storage
    python sitl.py --print-only     # print the command instead of running it

Run this from the environment that has `sim_vehicle.py` and `mavproxy.py` on PATH
(our `ardupilot` conda env), and run `main.py --sim` from the Pi-Code environment in a
second terminal. They are separate processes talking over UDP 14550.

WHY THIS SCRIPT EXISTS
----------------------
Until the 2026-08-24 ownership decision the companion wrote the parameters a simulated
run needed, so a bare `sim_vehicle.py` was enough to fly. It no longer writes any, so
the simulator has to arrive already configured, and for a while the only documented way
to do that was to load the mirror into a RUNNING simulator twice with a reboot each
time. That procedure is both fragile and easy to get half-right, and half-right is the
bad case: a SITL missing `RNGFND1_TYPE` streams no rangefinder, the mission aborts with
`NO_RANGEFINDER_DATA`, and nothing says the cause was the parameter load rather than the
code.

Passing the mirror as a **startup defaults file** removes the whole dance. ArduPilot
holds back defaults whose parameter does not exist yet and applies them when the driver
creates it, so `RNGFND1_TYPE` and its `RNGFND1_MIN_CM` / `MAX_CM` / `GNDCLEAR` / `ORIENT`
sub-parameters all land in the SAME boot. One start, no reboots, no second pass.
Verified 2026-09-21: 1646 parameters, `EKF3 IMU0 fusing optical flow` in the banner, and
the position-hold milestone (now milestone 2) green immediately afterwards.

WHAT IT PUTS ON THE COMMAND LINE, AND WHY EACH PIECE IS THERE
-------------------------------------------------------------
* `--add-param-file=params/sitl_flight_v<N>.parm` - the generated mirror of the
  published flight set, so the simulator flies the parameters we fly. Resolved by
  version, so publishing a v3 changes nothing here.
* `--custom-location=<config.origin_lat,lon,alt,0>` - the simulated compass is modelled
  at the simulator's home. Start it anywhere else than the EKF origin the companion
  sends and pre-arm fails with *"Check mag field (z diff:976>200)"*, 976 mGauss being
  the difference between the northern and southern hemisphere.
* `-w` (wipe) - every run starts from the same parameter state. Without it yesterday's
  experiment is still in the simulator's storage and the run is not reproducible. A
  wipe also resets the simulated battery, which drains across runs and otherwise aborts
  a later mission with a puzzling `LOW_BATTERY`.
* `--out=udp:127.0.0.1:14550` - where `Config.sitl()` listens.
"""

import os
import shutil
import subprocess
import sys

import paramcheck
from config import Config

# Where the ArduPilot checkout usually sits, most specific first. `ARDUPILOT_HOME`
# overrides all of them, so a differently laid-out machine needs no source edit.
CANDIDATE_DIRS = (
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 "Simulation", "ardupilot"),
    os.path.expanduser("~/ardupilot"),
    os.path.expanduser("~/Documents/ardupilot"),
)


def find_sim_vehicle() -> str:
    """Absolute path of `sim_vehicle.py`, or exit with the reason it was not found."""
    override = os.environ.get("ARDUPILOT_HOME")
    roots = ([override] if override else []) + list(CANDIDATE_DIRS)
    for root in roots:
        candidate = os.path.join(root, "Tools", "autotest", "sim_vehicle.py")
        if os.path.exists(candidate):
            return candidate
    on_path = shutil.which("sim_vehicle.py")
    if on_path:
        return on_path
    raise SystemExit(
        "Could not find sim_vehicle.py. Looked in:\n  "
        + "\n  ".join(os.path.join(r, "Tools", "autotest") for r in roots)
        + "\nSet ARDUPILOT_HOME to your ArduPilot checkout, or put sim_vehicle.py on PATH."
    )


def build_command(argv: list) -> list:
    """Assemble the sim_vehicle.py command line. Every piece is explained in the
    module docstring; nothing here is optional decoration."""
    config = Config.sitl()

    mirror = config.expected_params_path or paramcheck.newest_flight_set(simulated=True)
    if not mirror:
        raise SystemExit(
            f"No SITL mirror found in {paramcheck.PARAMS_DIR} (expected "
            f"sitl_flight_v<N>.parm). Generate one from the published flight set:\n"
            f"    python params/generate_sitl_flight_params.py"
        )

    speedup = "1"
    if "--speedup" in argv:
        index = argv.index("--speedup")
        if index + 1 < len(argv):
            speedup = argv[index + 1]

    port = config.connection_string.rsplit(":", 1)[-1]

    command = [
        find_sim_vehicle(),
        "-v", "ArduCopter",
        "--no-rebuild",
        "--console",
        f"--custom-location={config.origin_lat},{config.origin_lon},{config.origin_alt},0",
        f"--add-param-file={mirror}",
        f"--out=udp:127.0.0.1:{port}",
        "--speedup", speedup,
    ]
    if "--no-wipe" not in argv:
        command.append("-w")
    return command


def main() -> None:
    command = build_command(sys.argv[1:])
    print("Starting SITL with the published flight parameters:\n")
    print("    " + " ".join(command) + "\n")
    if "--print-only" in sys.argv:
        return
    print("Then, in a second terminal:  python main.py --sim --milestone 1\n")
    # Hand the terminal over: sim_vehicle.py starts MAVProxy, and MAVProxy's console is
    # the thing you want in front of you (`param show`, `mode LOITER`,
    # `param set SIM_BATT_VOLTAGE 10.5` to provoke the battery failsafe).
    try:
        raise SystemExit(subprocess.call(command))
    except FileNotFoundError:
        raise SystemExit(
            "sim_vehicle.py was found but could not be started. It needs MAVProxy on "
            "PATH - run this from the environment that has it (our `ardupilot` conda "
            "env), not from the Pi-Code one."
        )


if __name__ == "__main__":
    main()
