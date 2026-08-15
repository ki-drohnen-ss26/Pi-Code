"""
main.py
Entry point. Wires together configuration, drone, camera, failsafe and mission.
=======

Usage:
    python main.py                 # SITL (udpin:127.0.0.1:14550)
    python main.py --port 14551    # SITL on a second MAVProxy output (QGC keeps 14550)
    python main.py --pi            # real Pi via mavlink-router
    python main.py --pi-serial     # real Pi, direct UART (no mavlink-router)
    python main.py --tele          # only print telemetry (no flight), any of the above

The preset is chosen here, not by editing config.py - so the same checked-out code
runs on the Mac and on the drone.
"""

import logging
import sys

from camera import MockCamera, ScriptedCamera, SimCamera, TimedCamera
from config import Config
from drone import Drone
from failsafe import FailsafeMonitor
from logbook import setup_logging
from mission import DeliveryMission
from release import FcServo, PiServo

log = logging.getLogger(__name__)


def make_camera(config: Config, drone: Drone):
    """Build the Camera implementation selected by config.camera_source.

    "auto"  -> SimCamera when indoor (gps_denied) else MockCamera
    "sim"   -> SimCamera (simulated target at a known local position)
    "mock"  -> MockCamera (always centred)
    "timed" -> TimedCamera (finds after a set time, no real detection)
    """
    src = config.camera_source.lower()
    if src == "auto":
        src = "sim" if config.gps_denied else "mock"
    if src == "sim":
        return SimCamera(drone, config.sim_target_north, config.sim_target_east,
                         config.sim_fov_radius_m)
    if src == "timed":
        return TimedCamera(config.timed_camera_after_s)
    if src == "mock":
        return MockCamera()
    # For the OVER_TARGET correction loop with fixed frames, set ScriptedCamera manually.
    raise ValueError(f"Unknown camera_source '{config.camera_source}'")


def make_release(config: Config, drone: Drone):
    """Build the ReleaseMechanism selected by config.release_mechanism.

    "fc" -> FcServo (drop servo on an FC output, over MAVLink; SITL / tests)
    "pi" -> PiServo (drop servo on a Raspberry Pi GPIO pin, PWM from the Pi)
    """
    mech = config.release_mechanism.lower()
    if mech == "fc":
        return FcServo(drone, config)
    if mech == "pi":
        return PiServo(config)
    raise ValueError(f"Unknown release_mechanism '{config.release_mechanism}'")


def make_config(argv: list) -> Config:
    """Pick the preset from the command line instead of editing source on the drone.

    Both presets connect over UDP: on the Pi, mavlink-router owns /dev/serial0 and
    forwards the FC stream to 127.0.0.1:14550, so the endpoint is the same as in SITL.
    What differs is the drop servo (FC output vs Pi GPIO) and the battery threshold -
    see the constructors in config.py.
    """
    if "--pi" in argv:
        return Config.pi()
    if "--pi-serial" in argv:      # direct UART, only without mavlink-router
        return Config.pi_serial()
    # QGroundControl also binds 14550. To run both, add a second output in the MAVProxy
    # console (`output add 127.0.0.1:14551`) and start with --port 14551.
    if "--port" in argv:
        return Config.sitl(port=int(argv[argv.index("--port") + 1]))
    return Config.sitl()


def main() -> None:
    config = make_config(sys.argv)

    # --- logging: console + timestamped file under config.log_dir ---
    setup_logging(config.log_dir)

    # --- open the connection ---
    drone = Drone(config)
    drone.connect()

    # Telemetry-only mode for a quick link check
    if "--tele" in sys.argv:
        drone.print_telemetry(duration_s=15.0)
        return

    # --- assemble the components ---
    # Every camera implements the same interface (get_target_offset); config picks one.
    camera = make_camera(config, drone)
    release = make_release(config, drone)
    failsafe = FailsafeMonitor(drone, config)
    mission = DeliveryMission(drone, camera, failsafe, config, release)

    # --- start the mission ---
    # DeliveryMission.run() already commands LAND before re-raising, so we only have to
    # make sure the failure is visible and the exit code is non-zero.
    try:
        mission.run()
    except KeyboardInterrupt:
        log.warning("[MAIN] Interrupted by user")
        raise SystemExit(130)
    except Exception:
        log.exception("[MAIN] Mission failed")
        raise SystemExit(1)


if __name__ == "__main__":
    main()