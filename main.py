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

Staged bring-up on a new aircraft - each stage adds exactly ONE unknown:
    python main.py --milestone 1   # climb to 0.8m, hold, land        (position hold)
    python main.py --milestone 2   # same + the detector, logging only (detector)
    python main.py --milestone 3   # fly the search pattern            (pattern)
    python main.py --milestone 4   # search + detect + centre, no drop (approach)
    python main.py --milestone 5   # the full delivery                 (release)

Rehearse any of them in the simulator first with `--sim --milestone N`; the simulated
detector is substituted for the IMX500 and the substitution is logged. Individual
switches (--hover / --alt / --no-drop) still work and override a milestone.

The preset is chosen here, not by editing config.py - so the same checked-out code
runs on the Mac and on the drone.

**The default is the real aircraft; simulation needs `--sim`.** Forgetting a flag has
to fail safely, and only one of the two mistakes is dangerous - see make_config().
"""

import logging
import sys

from camera import MockCamera, RealCamera, SimCamera, TimedCamera
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
    "none"  -> MockCamera that never detects (bring-up: no detector in the loop)
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
    if src == "none":
        # Never detects. Milestone 1 and 3 fly without a detector in the loop, so the
        # run tests position hold and the search pattern and nothing else. Expect the
        # pattern to end in TARGET_NOT_FOUND - that IS the pass condition here.
        return MockCamera(detected=False)
    if src == "real":
        camera = RealCamera(config, drone)
        camera.start()
        return camera
    # For the OVER_TARGET correction loop with fixed frames, wire camera.ScriptedCamera
    # in here manually (import it first - it is a test helper, not a config-selectable source).
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
            config = Config.sitl(port=int(argv[argv.index("--port") + 1]))
        else:
            config = Config.sitl()
    elif "--pi-serial" in argv:    # direct UART, only without mavlink-router
        config = Config.pi_serial()
    else:
        config = Config.pi()

    _apply_bringup_flags(config, argv)
    return config


# Staged bring-up. Each milestone adds exactly ONE unknown to the previous one, so a
# failure names its own cause instead of leaving four candidates open. The numbers are
# the flight-test order, not a preference - do not skip ahead.
MILESTONES = {
    1: dict(label="hover only — position hold",
            hover_test_s=20.0, hover_test_alt=0.8, camera_source="none"),
    2: dict(label="hover + detector (logs only, acts on nothing)",
            hover_test_s=20.0, hover_test_alt=1.0, camera_source="real"),
    3: dict(label="search pattern, no detection",
            hover_test_s=0.0, camera_source="none"),
    4: dict(label="search + detect + centre, release nothing",
            hover_test_s=0.0, camera_source="real", skip_drop=True),
    5: dict(label="full delivery",
            hover_test_s=0.0, camera_source="real", skip_drop=False),
}


def _apply_bringup_flags(config: Config, argv: list) -> None:
    """Staged bring-up switches, so a first flight needs no source edit on the drone.

        --milestone N   apply bring-up stage N (1-5), see MILESTONES above
        --hover [s]     climb, hold for s seconds (default 20), land. No search, no drop.
        --alt [m]       takeoff altitude for that hover, overriding search_altitude.
        --no-drop       fly the full search + approach but release nothing.
        --takeover      do NOT arm or take off - wait for the pilot to fly it up, then
                        take over in GUIDED (see mission._wait_for_pilot)

    --milestone sets a whole coherent stage; the individual flags are applied AFTER it so
    a single value can still be overridden (e.g. `--milestone 1 --alt 0.8`).
    """
    def _value_after(flag: str, default: float) -> float:
        i = argv.index(flag)
        if i + 1 < len(argv):
            try:
                return float(argv[i + 1])
            except ValueError:
                pass                      # next token is another flag, not a number
        return default

    if "--milestone" in argv:
        number = int(_value_after("--milestone", 0))
        if number not in MILESTONES:
            raise SystemExit(
                f"Unknown milestone {number}. Available: "
                + ", ".join(f"{n} ({m['label']})" for n, m in sorted(MILESTONES.items()))
            )
        for key, value in MILESTONES[number].items():
            if key != "label":
                setattr(config, key, value)
        config.milestone = number

        if config.release_mechanism == "fc" and config.camera_source == "real":
            config.camera_source = "auto"
            config.milestone_camera_substituted = True

    if "--hover" in argv:
        config.hover_test_s = _value_after("--hover", 20.0)
    if "--alt" in argv:
        config.hover_test_alt = _value_after("--alt", 0.0)
    if "--no-drop" in argv:
        config.skip_drop = True
    if "--takeover" in argv:
        config.takeover_mode = True


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
    # Bring-up modes change what the flight actually does, so they must be as visible as
    # the profile itself - not buried in a config file nobody re-reads before a flight.
    if config.milestone:
        log.warning(f"[PROFILE] BRING-UP MILESTONE {config.milestone}: "
                    f"{MILESTONES[config.milestone]['label']}")
        if config.milestone_camera_substituted:
            log.warning("[PROFILE] ... but this is SIMULATION: the milestone asks for "
                        "the IMX500, the run uses the simulated detector instead")
    if config.hover_test_s > 0:
        alt = config.hover_test_alt if config.hover_test_alt > 0 else (
            config.search_altitude if config.gps_denied else config.cruise_alt)
        log.warning(f"[PROFILE] BRING-UP HOVER: climb to {alt} m, hold "
                    f"{config.hover_test_s:.0f} s, land. No search, no drop.")
    if config.skip_drop:
        log.warning("[PROFILE] BRING-UP: skip_drop is set - the payload will NOT be released")
    if config.takeover_mode:
        log.warning(f"[PROFILE] TAKEOVER: the companion will NOT arm and will NOT take "
                    f"off. The pilot flies it to {config.takeover_min_alt_m} m first.")


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