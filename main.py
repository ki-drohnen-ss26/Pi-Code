"""
main.py
Entry point. Wires together configuration, drone, camera, failsafe and mission.
=======

Usage:
    python main.py                    # REAL AIRCRAFT via mavlink-router (the default)
    python main.py --pi-serial        # real aircraft, direct UART (no mavlink-router)
    python main.py --sim              # SIMULATION against SITL
    python main.py --sim --port 14551 # SITL on a second MAVProxy output (QGC keeps 14550)
    python main.py --tele             # only print telemetry (no flight), any of the above

The preset is chosen here, not by editing config.py - so the same checked-out code
runs on the Mac and on the drone.

**The default is the real aircraft; simulation needs `--sim`.** Forgetting a flag has
to fail safely, and only one of the two mistakes is dangerous - see make_config().
"""

import logging
import sys

from camera import MockCamera, RealCamera, ScriptedCamera, SimCamera, TimedCamera
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
    "real"  -> RealCamera (Raspberry Pi AI Camera / IMX500, Phase 4)

    RealCamera is started here rather than lazily: uploading the model to the sensor
    takes ~45 s the first time, and that must happen on the ground, not mid-mission.
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
    if src == "real":
        camera = RealCamera(config, drone)
        camera.start()
        return camera
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

    **The default is the REAL AIRCRAFT. Simulation is opt-in via `--sim`.**

    This way round on purpose, because forgetting the flag must fail on the safe side,
    and the two mistakes are not equally bad:

    * A real aircraft accidentally running SIMULATION values fails **silently and
      dangerously**. `battery_min_voltage` would be 10.8 V - *below* a 4S Li-Ion's
      empty voltage of 11.2 V - so the low-battery abort could never fire. The drop
      would be commanded on an FC output that carries no servo. And `camera_source`
      "auto" resolves to SimCamera, which reports a target that does not exist, so the
      aircraft flies to an empty spot and drops there.
    * Simulation accidentally running REAL values fails **immediately and harmlessly**:
      `PiServo` raises "needs gpiozero (Raspberry Pi only)" before anything takes off.

    So the dangerous direction is the one that now needs an explicit flag.
    """
    if "--sim" in argv or "--sitl" in argv:
        # QGroundControl also binds 14550. To run both, add a second output in the
        # MAVProxy console (`output add 127.0.0.1:14551`) and start with --port 14551.
        if "--port" in argv:
            return Config.sitl(port=int(argv[argv.index("--port") + 1]))
        return Config.sitl()
    if "--pi-serial" in argv:      # direct UART, only without mavlink-router
        return Config.pi_serial()
    return Config.pi()             # --pi is accepted but redundant: it is the default


def log_profile(config: Config) -> None:
    """State which profile is active, loudly, before anything is commanded.

    A wrong profile is invisible in the air but obvious in a log line. Since the real
    aircraft is now the default, the simulation banner is the one that must stand out -
    seeing it while standing next to an armed drone means stop.
    """
    simulated = config.release_mechanism == "fc"
    if simulated:
        log.warning("=" * 62)
        log.warning("[PROFILE] SIMULATION (SITL) - do NOT use for a real flight")
        log.warning("=" * 62)
    else:
        log.info("[PROFILE] REAL AIRCRAFT")
    log.info(
        f"[PROFILE] link={config.connection_string}  drop={config.release_mechanism}  "
        f"battery_min={config.battery_min_voltage} V  camera={config.camera_source}  "
        f"gps_denied={config.gps_denied}"
    )


def main() -> None:
    config = make_config(sys.argv)

    # --- logging: console + timestamped file under config.log_dir ---
    setup_logging(config.log_dir)
    log_profile(config)

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
    finally:
        # Release the sensor on every exit path, including an abort. A camera left
        # running holds the IMX500 in a half-configured state, and the next start then
        # fails with a flood of "Failed to queue buffer" until the Pi is rebooted.
        if hasattr(camera, "stop"):
            camera.stop()


if __name__ == "__main__":
    main()