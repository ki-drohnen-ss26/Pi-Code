"""
main.py
=======
Entry point. Wires together configuration, drone, camera, failsafe and mission.

Usage:
    python main.py                 # uses the SITL default from config.py
    python main.py --tele          # only print telemetry (no flight)

To switch to the real Pi: change the connection_string in config.py, or use
Config.pi_serial() instead of Config() below.
"""

import sys

from camera import MockCamera
from config import Config
from drone import Drone
from failsafe import FailsafeMonitor
from mission import DeliveryMission


def main() -> None:
    # --- choose configuration ---
    # config = Config.sitl(port=14551)    # NOTE: Since QGroundControl takes 14550, we need to add 14551 at runtime
    config = Config.sitl()                # NOTE: This binds to 14550, therefore QGroundControl cant be used
    # config = Config.pi_serial()         # real Pi on the flight controller (UART)

    # --- open the connection ---
    drone = Drone(config)
    drone.connect()

    # Telemetry-only mode for a quick link check
    if "--tele" in sys.argv:
        drone.print_telemetry(duration_s=15.0)
        return

    # --- assemble the components ---
    camera = MockCamera()                 # later: real camera, same interface
    failsafe = FailsafeMonitor(drone, config)
    mission = DeliveryMission(drone, camera, failsafe, config)

    # --- start the mission ---
    mission.run()


if __name__ == "__main__":
    main()