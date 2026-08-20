"""
test_mission.py
===============
Unit tests for the delivery state machine (mission.py) that run on the Mac in
under a second, WITHOUT SITL or any MAVLink link.

The trick is FakeDrone: a small in-memory stand-in that mirrors every method the
mission and the failsafe call on a real Drone, and records the actions so the
tests can assert on them. Because the mission was written against the Drone
interface (not against the MAVLink connection directly), swapping in FakeDrone is
all it takes.

These tests protect the upcoming refactors (search/approach states, GPS-denied
navigation) from silently breaking the mission flow.
"""

import pytest

from camera import MockCamera, ScriptedCamera, SimCamera, TimedCamera
from config import Config
from failsafe import FailsafeMonitor
from mission import DeliveryMission, State
from release import FcServo


class FakeDrone:
    """In-memory Drone replacement. Defaults to a perfectly healthy drone; pass
    flags to simulate failures (e.g. arm_ok=False)."""

    def __init__(
        self,
        config: Config,
        battery=None,
        ready_ok: bool = True,
        arm_ok: bool = True,
        takeoff_ok: bool = True,
        arrival_ok: bool = True,
        disarm_ok: bool = True,
        link_alive_ok: bool = True,
        origin_ok: bool = True,
        unknown_params=None,
    ):
        self.config = config
        self.calls: list[tuple] = []
        # battery may be a single dict/None (constant) or a list consumed one per
        # read (the last entry repeats), to simulate telemetry hiccups.
        self._battery = battery if battery is not None else {"voltage": 12.0, "current": 1.0, "remaining": 100}
        self._batt_i = 0
        self._armed = False
        self._servo = config.neutral_pwm
        self.ready_ok = ready_ok
        self.arm_ok = arm_ok
        self.takeoff_ok = takeoff_ok
        self.arrival_ok = arrival_ok
        self.disarm_ok = disarm_ok
        self.link_alive_ok = link_alive_ok
        self._link_checks = 0
        self._north = 0.0  # local NED position, updated by goto_local / move_body_offset
        self._east = 0.0
        # Flight mode the FC reports. Tests flip this to simulate the pilot taking over.
        self.mode = "GUIDED"
        self.origin_ok = origin_ok
        # Parameter names the fake FC does NOT know, so tests can exercise the
        # RTL_ALT / RTL_ALT_M fallback across firmware versions.
        self.unknown_params = set(unknown_params or ())

    # --- used by the failsafe ---
    def tick(self):
        self.calls.append(("tick",))

    def get_mode(self):
        return self.mode

    def set_param(self, name, value, timeout=3.0):
        if name in self.unknown_params:
            raise TimeoutError(f"No confirmation for parameter {name}")
        self.calls.append(("set_param", name, value))
        return float(value)

    def get_battery(self, timeout=2.0):
        if isinstance(self._battery, list):
            value = self._battery[min(self._batt_i, len(self._battery) - 1)]
            self._batt_i += 1
            return value
        return self._battery

    def link_alive(self, timeout=3.0):
        # bool = constant. int = this many checks succeed, then the link dies, which
        # lets a test lose the link mid-flight instead of before arming.
        if isinstance(self.link_alive_ok, bool):
            return self.link_alive_ok
        self._link_checks += 1
        return self._link_checks <= self.link_alive_ok

    def move_body_offset(self, forward, right, down=0.0):
        # yaw=0 assumption: body forward=north, body right=east
        self._north += forward
        self._east += right
        self.calls.append(("move_body_offset", forward, right))

    # --- preparation ---
    def configure_drop_servo(self):
        self.calls.append(("configure_drop_servo",))

    def reset_servo(self):
        self._servo = self.config.neutral_pwm
        self.calls.append(("reset_servo",))

    def wait_ready_to_arm(self, timeout=60.0, require_abs=True):
        self.calls.append(("wait_ready_to_arm", require_abs))
        return self.ready_ok

    def set_mode(self, mode_name, timeout=5.0):
        self.mode = mode_name
        self.calls.append(("set_mode", mode_name))
        return True

    def arm(self, timeout=10.0, attempts=5):
        self.calls.append(("arm",))
        if self.arm_ok:
            self._armed = True
        return self.arm_ok

    def disarm(self):
        self._armed = False
        self.calls.append(("disarm",))

    def is_armed(self):
        return self._armed

    # --- navigation ---
    def takeoff(self, altitude, timeout=30.0):
        self.calls.append(("takeoff", altitude))
        return self.takeoff_ok

    def goto(self, lat, lon, alt):
        self.calls.append(("goto", lat, lon, alt))

    def wait_arrival(self, lat, lon, radius_m, timeout=60.0):
        self.calls.append(("wait_arrival",))
        return self.arrival_ok

    def set_origin(self, lat, lon, alt):
        self.calls.append(("set_origin", lat, lon, alt))
        return self.origin_ok

    def land(self):
        self.mode = "LAND"
        self.calls.append(("land",))

    def get_local_position(self, timeout=2.0):
        return {"north": self._north, "east": self._east, "down": -2.0}

    def goto_local(self, north, east, down):
        # teleport to the waypoint (good enough for logic tests)
        self._north, self._east = north, east
        self.calls.append(("goto_local", north, east))

    def wait_local_arrival(self, north, east, radius_m, timeout=60.0):
        self.calls.append(("wait_local_arrival",))
        return self.arrival_ok

    def return_to_launch(self):
        self.mode = "RTL"
        self.calls.append(("return_to_launch",))

    def wait_disarmed(self, timeout=60.0):
        if self.disarm_ok:
            self._armed = False
        return self.disarm_ok

    # --- payload ---
    def drop(self):
        self._servo = self.config.drop_pwm
        self.calls.append(("drop",))

    def read_servo(self, timeout=3.0, expected=None):
        return self._servo

    # --- test helper ---
    def actions(self) -> list[str]:
        return [c[0] for c in self.calls]


@pytest.fixture(autouse=True)
def _no_sleep(monkeypatch):
    """Strip the real waits out of the mission so tests are instant."""
    monkeypatch.setattr("mission.time.sleep", lambda *_a, **_k: None)


def _build_mission(drone: FakeDrone, config: Config, camera=None) -> DeliveryMission:
    camera = camera or MockCamera()  # target detected dead-centre by default
    failsafe = FailsafeMonitor(drone, config)  # the REAL failsafe, fed by FakeDrone
    release = FcServo(drone, config)  # FC servo path; FakeDrone provides the servo calls
    return DeliveryMission(drone, camera, failsafe, config, release)


def test_happy_path_runs_to_done_and_delivers():
    config = Config.sitl()
    config.gps_denied = False  # GPS path: IDLE->TAKEOFF->ENROUTE->OVER_TARGET->DROP->RTL
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason is None

    actions = drone.actions()
    # The delivery happened in the right order and ended with a return to launch.
    assert "drop" in actions
    assert actions.index("takeoff") < actions.index("drop") < actions.index("land")


def test_low_battery_aborts_before_drop_but_still_returns():
    config = Config.sitl()
    # Voltage below battery_min_voltage (10.8 V) -> failsafe fires on the first check.
    drone = FakeDrone(config, battery={"voltage": 10.0, "current": 1.0, "remaining": 15})
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "LOW_BATTERY"
    assert "drop" not in drone.actions()          # no delivery on abort
    assert "land" in drone.actions()  # but it returned safely


def test_failed_arming_ends_without_commanding_flight():
    """A vehicle that never armed is standing on the ground. Commanding LAND or RTL at
    it is pointless - and the old code did exactly that, then logged "landed and
    disarmed" for a drone that had never left the floor."""
    config = Config.sitl()
    drone = FakeDrone(config, arm_ok=False)
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "ARMING_FAILED"   # named, not None
    assert "drop" not in drone.actions()
    assert "land" not in drone.actions()
    assert "return_to_launch" not in drone.actions()


def test_over_target_correction_nudges_then_drops():
    config = Config.sitl()
    config.gps_denied = False  # GPS path so OVER_TARGET consumes all camera frames
    drone = FakeDrone(config)
    # Two off-centre frames (outside centre_tolerance) then a centred one.
    camera = ScriptedCamera([
        {"detected": True, "dx": 1.0, "dy": 0.5, "distance": 2.0},
        {"detected": True, "dx": 0.4, "dy": 0.2, "distance": 1.8},
        {"detected": True, "dx": 0.0, "dy": 0.0, "distance": 1.5},
    ])
    mission = _build_mission(drone, config, camera=camera)

    mission.run()

    assert mission.state is State.DONE
    actions = drone.actions()
    # The drone was nudged for the off-centre frames, then it delivered.
    assert actions.count("move_body_offset") == 2
    assert actions.index("move_body_offset") < actions.index("drop")


def test_link_loss_in_flight_aborts_and_lands():
    """Link loss AFTER takeoff: the mission must abort and bring the aircraft down.
    (link_alive_ok=2 lets the checks before IDLE and TAKEOFF pass, so the link dies
    once we are airborne rather than on the ground.)"""
    config = Config.sitl()
    drone = FakeDrone(config, link_alive_ok=2)
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "LINK_LOSS"
    assert "drop" not in drone.actions()
    assert "land" in drone.actions()
    assert "land" in drone.actions()


def test_sustained_telemetry_loss_aborts():
    config = Config.sitl()
    config.telemetry_max_misses = 2  # abort fast for the test
    drone = FakeDrone(config, battery=[None])  # every read returns no data
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "NO_TELEMETRY"
    assert "drop" not in drone.actions()


def test_single_telemetry_hiccup_does_not_abort():
    config = Config.sitl()
    # One missing read, then healthy again -> the counter resets, no abort.
    healthy = {"voltage": 12.0, "current": 1.0, "remaining": 100}
    drone = FakeDrone(config, battery=[None, healthy])
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason is None
    assert "drop" in drone.actions()


# ----------------------------------------------------------------------
# Indoor / GPS-denied path (Phase 2): search + approach with SimCamera
# ----------------------------------------------------------------------
def test_indoor_search_finds_target_approaches_and_delivers():
    config = Config.sitl()  # gps_denied=True by default
    config.sim_target_north = 2.5  # off a spiral corner, so APPROACH actually nudges
    config.sim_target_east = 1.5
    drone = FakeDrone(config)
    camera = SimCamera(drone, config.sim_target_north, config.sim_target_east, config.sim_fov_radius_m)
    mission = _build_mission(drone, config, camera=camera)

    mission.run()

    assert mission.state is State.DONE
    actions = drone.actions()
    assert "goto_local" in actions         # it flew a search pattern
    assert "move_body_offset" in actions   # it servo-approached the target
    assert "drop" in actions
    assert actions.index("goto_local") < actions.index("drop")
    # ended within tolerance of the target
    assert abs(drone._north - 2.5) <= config.centre_tolerance
    assert abs(drone._east - 1.5) <= config.centre_tolerance


def test_indoor_continuous_cadence_finds_target():
    config = Config.sitl()
    config.detection_cadence = "continuous"
    config.sim_target_north = 2.5
    config.sim_target_east = 1.5
    drone = FakeDrone(config)
    camera = SimCamera(drone, config.sim_target_north, config.sim_target_east, config.sim_fov_radius_m)
    mission = _build_mission(drone, config, camera=camera)

    mission.run()

    assert mission.state is State.DONE
    assert "drop" in drone.actions()


def test_indoor_target_not_found_aborts_home():
    config = Config.sitl()
    far = 20.0  # outside search_max_radius -> no waypoint ever detects it
    config.sim_target_north = far
    config.sim_target_east = far
    drone = FakeDrone(config)
    camera = SimCamera(drone, far, far, config.sim_fov_radius_m)
    mission = _build_mission(drone, config, camera=camera)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "TARGET_NOT_FOUND"
    assert "drop" not in drone.actions()
    assert "land" in drone.actions()


# ----------------------------------------------------------------------
# Phase 3: EKF origin, indoor geofence, camera-less flight (TimedCamera)
# ----------------------------------------------------------------------
def test_set_origin_sent_before_arming_when_enabled():
    config = Config.sitl()  # gps_denied=True
    config.set_origin_on_start = True
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    actions = drone.actions()
    assert "set_origin" in actions
    assert actions.index("set_origin") < actions.index("arm")
    assert ("set_origin", config.origin_lat, config.origin_lon, config.origin_alt) in drone.calls


def test_no_origin_when_disabled():
    config = Config.sitl()
    config.set_origin_on_start = False  # explicit, don't rely on the default
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    assert "set_origin" not in drone.actions()


def test_geofence_sets_altitude_fence():
    config = Config.sitl()
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    assert ("set_param", "FENCE_TYPE", config.fence_type) in drone.calls
    assert ("set_param", "FENCE_ALT_MAX", config.fence_alt_max_m) in drone.calls
    assert ("set_param", "FENCE_ENABLE", 1) in drone.calls


def test_timed_camera_finds_after_time():
    assert TimedCamera(detected_after_s=1000.0).get_target_offset()["detected"] is False
    offset = TimedCamera(detected_after_s=0.0).get_target_offset()
    assert offset["detected"] is True
    assert offset["dx"] == 0.0 and offset["dy"] == 0.0


def test_piservo_pulse_mapping_clamps_to_unit_range():
    """PiServo maps microsecond pulses onto gpiozero's [-1, 1], clamped. This runs
    without gpiozero (only the pure mapping is exercised)."""
    from release import PiServo
    assert PiServo._pw_to_value(1500) == 0.0    # centre
    assert PiServo._pw_to_value(1000) == -1.0   # min band
    assert PiServo._pw_to_value(2000) == 1.0    # max band
    assert PiServo._pw_to_value(1100) == pytest.approx(-0.8)
    assert PiServo._pw_to_value(1900) == pytest.approx(0.8)
    assert PiServo._pw_to_value(2500) == 1.0    # above band -> clamped
    assert PiServo._pw_to_value(500) == -1.0    # below band -> clamped


def test_boot_mode_does_not_abort_before_the_mission_starts():
    """Regression: the mode check must not fire before we have set GUIDED ourselves.

    failsafe.check() runs before the FIRST state, and at that moment the FC is still
    in whatever mode it booted into (STABILIZE). Checking from the start aborted every
    mission with MODE_CHANGED_STABILIZE before it had done anything at all.
    """
    config = Config.sitl()
    drone = FakeDrone(config)
    drone.mode = "STABILIZE"          # as the autopilot reports on boot
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason is None
    assert "drop" in drone.actions()


def test_pilot_override_stops_without_commanding_anything():
    """If the FC leaves GUIDED, someone else is flying: the pilot flipped a switch, or
    an FC failsafe (fence breach, EKF) took over. Commanding LAND or RTL now would
    fight them, so the mission must go quiet. In SITL the old code did the opposite -
    it kept sending waypoints for 60 s to a vehicle that had already left GUIDED."""
    config = Config.sitl()
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    # The pilot takes over right after takeoff.
    original_takeoff = drone.takeoff

    def takeoff_then_pilot_takes_over(altitude, timeout=30.0):
        result = original_takeoff(altitude, timeout)
        drone.mode = "LOITER"
        return result

    drone.takeoff = takeoff_then_pilot_takes_over
    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "MODE_CHANGED_LOITER"
    assert "drop" not in drone.actions()
    # The decisive assertion: we did NOT override the human.
    assert "land" not in drone.actions()
    assert "return_to_launch" not in drone.actions()


def test_safety_envelope_is_enforced_before_flight():
    """The FC defaults are built for open sky: FENCE_ACTION=1 climbs to RTL_ALT (15 m)
    on a breach. Indoors that is the ceiling, so the companion sets its own limits."""
    config = Config.sitl()
    drone = FakeDrone(config)
    _build_mission(drone, config).run()

    assert ("set_param", "WPNAV_SPEED_UP", config.climb_rate_cms) in drone.calls
    assert ("set_param", "FENCE_ACTION", config.fence_action) in drone.calls
    assert ("set_param", "RTL_ALT", config.rtl_alt_m * 100.0) in drone.calls  # cm on 4.6


def test_rtl_altitude_falls_back_to_the_newer_parameter_name():
    """RTL_ALT (cm) was renamed to RTL_ALT_M (m) in ArduPilot 4.7. Setting the wrong
    one is not an error - the autopilot ignores unknown parameters and silently keeps
    its 15 m default - so we try both."""
    config = Config.sitl()
    drone = FakeDrone(config, unknown_params={"RTL_ALT"})   # pretend firmware >= 4.7
    _build_mission(drone, config).run()

    assert ("set_param", "RTL_ALT", config.rtl_alt_m * 100.0) not in drone.calls
    assert ("set_param", "RTL_ALT_M", config.rtl_alt_m) in drone.calls


def test_battery_sag_needs_several_samples_before_aborting():
    """Li-Ion dips hard under load. A single reading below the threshold is a sag, not
    an empty pack - only a run of them may abort a flight."""
    config = Config.sitl()
    low = {"voltage": config.battery_min_voltage - 0.5, "current": 30.0, "remaining": 60}
    ok = {"voltage": config.battery_min_voltage + 1.0, "current": 5.0, "remaining": 60}
    # One dip, then recovery, then healthy readings for the rest of the flight.
    drone = FakeDrone(config, battery=[ok, low, ok])
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.abort_reason is None
    assert "drop" in drone.actions()


def test_unreachable_waypoints_abort_the_search():
    """Not arriving at a waypoint means the vehicle is not following us or the position
    estimate has diverged (optical flow without a valid rangefinder height). Flying the
    rest of the pattern from an unknown place makes it worse, so we stop."""
    config = Config.sitl()
    drone = FakeDrone(config, arrival_ok=False)
    mission = _build_mission(drone, config, camera=MockCamera(detected=False))

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "WAYPOINTS_UNREACHABLE"
    assert "drop" not in drone.actions()
    assert "land" in drone.actions()


def test_crash_in_a_state_still_commands_land():
    """An unhandled exception used to kill the process, leaving the aircraft armed in
    GUIDED holding its last position target - GUID_TIMEOUT does not apply to position
    targets, so it would hover until the battery died."""
    config = Config.sitl()
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    def explode(*_a, **_k):
        raise RuntimeError("simulated MAVLink failure")

    drone.goto_local = explode

    with pytest.raises(RuntimeError):
        mission.run()

    assert "land" in drone.actions()


def test_factories_build_every_configured_variant():
    """main.py's factories are the one path that pytest never exercised - which is how
    an unrunnable release_mechanism='pi' on the Mac stayed green in CI and only blew up
    against SITL."""
    from main import make_camera, make_release

    config = Config.sitl()
    drone = FakeDrone(config)
    for source in ("auto", "sim", "mock", "timed"):
        config.camera_source = source
        assert make_camera(config, drone).get_target_offset() is not None
    assert isinstance(make_release(Config.sitl(), drone), FcServo)

    bad = Config.sitl()
    bad.camera_source = "nonsense"
    with pytest.raises(ValueError):
        make_camera(bad, drone)
    bad = Config.sitl()
    bad.release_mechanism = "nonsense"
    with pytest.raises(ValueError):
        make_release(bad, drone)


def test_camera_less_flight_searches_and_drops():
    """Phase 3 end-to-end: origin set, fly the search pattern, TimedCamera 'finds'
    immediately, drop — all without a real camera."""
    config = Config.sitl()
    config.set_origin_on_start = True
    drone = FakeDrone(config)
    camera = TimedCamera(detected_after_s=0.0)
    mission = _build_mission(drone, config, camera=camera)

    mission.run()

    assert mission.state is State.DONE
    actions = drone.actions()
    assert "set_origin" in actions
    assert "goto_local" in actions   # it flew the search pattern
    assert "drop" in actions


# ======================================================================
# Preset selection: the real aircraft is the DEFAULT, simulation is opt-in
# ======================================================================
# These guard a safety property, not a convenience: the two possible mistakes are
# not equally bad. Simulation values on a real aircraft are silent and dangerous
# (battery_min_voltage 10.8 V is BELOW a 4S Li-Ion's 11.2 V empty voltage, so the
# low-battery abort could never fire; the drop goes to an FC output with no servo;
# SimCamera reports a target that is not there). Real values in simulation fail
# loudly and harmlessly. So the dangerous direction is the one that needs a flag.

def test_dataclass_defaults_are_the_flight_configuration():
    """Plain Config() must describe the real aircraft, not the simulator."""
    config = Config()
    assert config.release_mechanism == "pi"       # servo on the Pi GPIO
    assert config.battery_min_voltage == 12.8     # 4S Li-Ion, not SITL's pack
    assert config.camera_source != "auto"         # "auto" would be SimCamera on a drone


def test_no_flag_selects_the_real_aircraft():
    from main import make_config

    config = make_config(["main.py"])
    assert config.release_mechanism == "pi"
    assert config.battery_min_voltage == 12.8
    assert config.camera_source == "timed"        # honest camera-less default


def test_sim_flag_selects_simulation():
    from main import make_config

    for flag in ("--sim", "--sitl"):
        config = make_config(["main.py", flag])
        assert config.release_mechanism == "fc", flag
        assert config.battery_min_voltage == 10.8, flag
        assert config.camera_source == "auto", flag
        assert config.connection_string == "udpin:127.0.0.1:14550", flag


def test_sim_flag_honours_a_custom_port():
    from main import make_config

    config = make_config(["main.py", "--sim", "--port", "14551"])
    assert config.connection_string == "udpin:127.0.0.1:14551"
    assert config.release_mechanism == "fc"


def test_port_without_sim_does_not_silently_simulate():
    """--port alone used to imply SITL. It must not: a stray --port on the drone
    would otherwise hand the real aircraft the simulator's battery threshold."""
    from main import make_config

    config = make_config(["main.py", "--port", "14551"])
    assert config.release_mechanism == "pi"
    assert config.battery_min_voltage == 12.8


def test_pi_serial_stays_a_real_aircraft_profile():
    from main import make_config

    config = make_config(["main.py", "--pi-serial"])
    assert config.connection_string == "/dev/serial0"
    assert config.baud == 921600
    assert config.release_mechanism == "pi"
    assert config.battery_min_voltage == 12.8


def test_pi_flag_still_accepted_as_a_redundant_alias():
    """--pi is documented in older notes; it must keep meaning 'real aircraft'."""
    from main import make_config

    assert make_config(["main.py", "--pi"]).release_mechanism == "pi"
