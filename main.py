"""
main.py
Entry point. Wires together configuration, drone, camera, failsafe and mission.
=======

Usage:
    python main.py                 # uses the SITL default from config.py
    python main.py --tele          # only print telemetry (no flight)

To switch to the real Pi: change the connection_string in config.py, or use
Config.pi_serial() instead of Config() below.
"""

import sys

from camera import MockCamera, ScriptedCamera, SimCamera, TimedCamera
from config import Config
from drone import Drone
from failsafe import FailsafeMonitor
from logbook import setup_logging
from mission import DeliveryMission


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


def main() -> None:
    # --- choose configuration ---
    # config = Config.sitl(port=14551)    # NOTE: Since QGroundControl takes 14550, we need to add 14551 at runtime
    config = Config.sitl()                # NOTE: This binds to 14550, therefore QGroundControl cant be used
    # config = Config.pi_serial()         # real Pi on the flight controller (UART)

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
    failsafe = FailsafeMonitor(drone, config)
    mission = DeliveryMission(drone, camera, failsafe, config)

    # --- start the mission ---
    mission.run()


if __name__ == "__main__":
    main()