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

from camera import MockCamera, ScriptedCamera, SimCamera
from config import Config
from drone import Drone
from failsafe import FailsafeMonitor
from logbook import setup_logging
from mission import DeliveryMission


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
    # Every camera implements the same interface (get_target_offset). Pick one:
    if config.gps_denied:
        # Indoor: simulate a target at a known local position so the SEARCH +
        # APPROACH flow can be exercised in SITL before the real AI camera exists.
        camera = SimCamera(drone, config.sim_target_north, config.sim_target_east,
                           config.sim_fov_radius_m)
    else:
        camera = MockCamera()             # GPS path: target already centred
    # Alternative for the OVER_TARGET correction loop with fixed frames:
    # camera = ScriptedCamera([
    #     {"detected": True, "dx": 1.0, "dy": 0.5, "distance": 2.0},
    #     {"detected": True, "dx": 0.0, "dy": 0.0, "distance": 1.5},
    # ])
    failsafe = FailsafeMonitor(drone, config)
    mission = DeliveryMission(drone, camera, failsafe, config)

    # --- start the mission ---
    mission.run()


if __name__ == "__main__":
    main()