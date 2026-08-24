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

These tests cover the GPS path (ENROUTE/OVER_TARGET), the default GPS-denied
navigation) from silently breaking the mission flow.
"""

import json
import os
import tempfile

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
        self.ekf_flag_ok = True    # takeover: does the EKF bit appear once airborne?
        # Global position as read by get_position(); takeover watches rel_alt on it.
        self.position = {"lat": 0.0, "lon": 0.0, "alt": 0.0, "rel_alt": 0.0}
        self.arm_ok = arm_ok
        self.takeoff_ok = takeoff_ok
        self.arrival_ok = arrival_ok
        self.disarm_ok = disarm_ok
        self.link_alive_ok = link_alive_ok
        self._link_checks = 0
        self._north = 0.0  # local NED position, updated by goto_local / move_body_offset
        self._east = 0.0
        # Position sensors as read by failsafe.verify_position_sensors(). Healthy by
        # default; a test that wants the flyaway precondition overrides this.
        # What the FC carried before this run, as read back by read_param(). Includes
        # FENCE_ACTION - the parameter whose unrestored leftover caused the 2026-08-21
        # crash - and the safety-envelope trio, so the restore paths are exercised.
        self.params_before = {"FENCE_ENABLE": 0.0, "FENCE_TYPE": 7.0,
                              "FENCE_ALT_MAX": 120.0, "FENCE_ACTION": 1.0,
                              "WPNAV_SPEED_UP": 250.0, "WPNAV_SPEED": 1000.0,
                              "RTL_ALT": 1500.0}
        # Raw rangefinder reading as read by get_rangefinder(); None = no message.
        self.rangefinder_m = None
        self.position_sensors = {
            "rangefinder_samples": 25, "rangefinder_min": 0.30, "rangefinder_max": 1.20,
            "flow_samples": 25, "flow_quality_max": 180,
        }
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

    def get_position(self, timeout=2.0):
        return self.position

    def get_rangefinder(self, timeout=1.5):
        return self.rangefinder_m

    def wait_ekf_flag(self, flag, timeout=10.0):
        self.calls.append(("wait_ekf_flag", flag))
        return self.ekf_flag_ok

    def read_param(self, name, tries=3, timeout=2.0):
        self.calls.append(("read_param", name))
        return self.params_before.get(name)

    def read_position_sensors(self, duration=5.0):
        self.calls.append(("read_position_sensors", duration))
        return self.position_sensors

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
        return True

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
        return True

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
    # Isolate the failsafe's on-disk param backup per test - a leftover from a real
    # run (or another test) must not leak parameter restores into this one.
    if not config.param_backup_file:
        config.param_backup_file = os.path.join(tempfile.mkdtemp(), "fc_params_backup.json")
    failsafe = FailsafeMonitor(drone, config)  # the REAL failsafe, fed by FakeDrone
    release = FcServo(drone, config)  # FC servo path; FakeDrone provides the servo calls
    return DeliveryMission(drone, camera, failsafe, config, release)


def test_happy_path_runs_to_done_and_delivers():
    config = Config.sitl()
    config.gps_denied = False  # GPS path: IDLE->TAKEOFF->ENROUTE->OVER_TARGET->DROP->RECOVER
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason is None

    actions = drone.actions()
    # The delivery happened in the right order and the aircraft came down (LAND:
    # recovery_action defaults to "land", RECOVER only flies RTL when asked to).
    assert "drop" in actions
    assert actions.index("takeoff") < actions.index("drop") < actions.index("land")


def test_low_battery_aborts_before_drop_but_still_returns():
    config = Config.sitl()
    # Voltage below battery_min_voltage (10.8 V) on every read -> the failsafe fires
    # after battery_low_samples (3) consecutive checks, i.e. just after takeoff.
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
    assert "return_to_launch" not in drone.actions()   # indoor recovery lands, never RTLs


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
    config.geofence_enable = True      # off by default indoors, see config.py
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
    """The FC defaults are built for open sky: WPNAV_SPEED_UP 250 cm/s overshoots a
    2 m takeoff, WPNAV_SPEED 1000 cm/s crosses a hall in a second, and RTL climbs to
    15 m. Indoors all three are wrong, so the companion sets its own limits."""
    config = Config.sitl()
    drone = FakeDrone(config)
    _build_mission(drone, config).run()

    assert ("set_param", "WPNAV_SPEED_UP", config.climb_rate_cms) in drone.calls
    assert ("set_param", "WPNAV_SPEED", config.cruise_speed_cms) in drone.calls
    assert ("set_param", "RTL_ALT", config.rtl_alt_m * 100.0) in drone.calls  # cm on 4.6
    # FENCE_ACTION is NOT part of the envelope any more: meaningless with the fence off,
    # and writing it left an unrestored change on the FC. It moved to setup_geofence().
    assert not any(c[0] == "set_param" and c[1].startswith("FENCE_") for c in drone.calls)


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
    for source in ("auto", "sim", "mock", "timed", "none"):
        config.camera_source = source
        assert make_camera(config, drone).get_target_offset() is not None
    # "none" (milestones 1 and 3: no detector in the loop) must never detect anything.
    config.camera_source = "none"
    assert make_camera(config, drone).get_target_offset()["detected"] is False
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


# ======================================================================
# Staged hardware bring-up: hover test and skip_drop
# ======================================================================
# These exist so a first flight on a new aircraft adds ONE unknown at a time.
# A failure then names its own cause instead of leaving four candidates open.

def test_hover_test_skips_search_and_drop_and_lands():
    """Milestone 1: climb, hold, land. Nothing else may be commanded."""
    config = Config.sitl()
    config.hover_test_s = 0.2          # keep the test fast; the path is what matters
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    actions = drone.actions()
    assert "takeoff" in actions
    assert "land" in actions
    # The whole point: no search pattern, no payload release.
    assert "goto_local" not in actions
    assert "goto" not in actions
    assert "drop" not in actions


def test_hover_test_alt_overrides_the_takeoff_altitude():
    """A bring-up hover must be flyable lower than the search altitude without
    touching the rest of the configuration."""
    config = Config.sitl()
    config.hover_test_s = 0.1
    config.hover_test_alt = 1.0
    config.search_altitude = 2.0
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    assert mission.takeoff_altitude() == 1.0

    mission.run()
    commanded = [c for c in drone.calls if c[0] == "takeoff"]
    assert commanded == [("takeoff", 1.0)]


def test_hover_test_alt_is_ignored_without_hover_mode():
    """hover_test_alt must not silently lower a real mission's takeoff."""
    config = Config.sitl()
    config.hover_test_alt = 1.0        # set, but hover_test_s stays 0
    config.search_altitude = 2.0
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    assert mission.takeoff_altitude() == 2.0


def test_hover_aborts_on_failsafe_instead_of_holding():
    """A failsafe during the hold must end the hover, not sit it out."""
    config = Config.sitl()
    config.hover_test_s = 30.0         # long enough that only the abort can end it
    drone = FakeDrone(config, battery={"voltage": 10.0, "current": 1.0, "remaining": 50})
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "LOW_BATTERY"
    assert "land" in drone.actions()   # airborne abort -> RECOVER -> LAND


def test_skip_drop_centres_but_releases_nothing():
    """Milestone 4: prove search + detect + centring with nothing falling out."""
    config = Config.sitl()
    config.skip_drop = True
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    actions = drone.actions()
    assert "goto_local" in actions     # it really flew the search pattern
    assert "drop" not in actions       # and released nothing
    assert "land" in actions


def test_bringup_flags_are_parsed_from_the_command_line():
    """The flags exist so no source edit is needed on the drone between stages."""
    from main import make_config

    c = make_config(["main.py", "--hover", "30", "--alt", "1.5"])
    assert c.hover_test_s == 30.0
    assert c.hover_test_alt == 1.5
    assert c.release_mechanism == "pi"          # still the real-aircraft profile

    c = make_config(["main.py", "--hover"])     # bare flag -> default duration
    assert c.hover_test_s == 20.0

    c = make_config(["main.py", "--no-drop"])
    assert c.skip_drop is True
    assert c.hover_test_s == 0.0

    c = make_config(["main.py", "--sim"])       # nothing set unless asked for
    assert c.hover_test_s == 0.0
    assert c.skip_drop is False


# ======================================================================
# Flyaway protection
# ======================================================================
# A GPS-denied position estimate that loses its height reference does not fail
# loudly - it drifts, and the position controller then chases an error that only
# exists in the filter. params/README.md records 366 m of drift measured while the
# vehicle stood still, from a rangefinder stuck at 0.00 m. These tests pin the two
# guards against that: refuse to arm on dead sensors, and land if the reported
# position leaves a plausible envelope.

def test_rangefinder_stuck_at_zero_lands_right_after_takeoff():
    """The exact precondition of the documented 366 m drift.

    It cannot be caught on the ground - there a healthy sensor reads 0.00 m too - so
    the abort must happen just after the climb, at takeoff height, before the estimate
    has a search pattern in which to run away."""
    config = Config.sitl()
    drone = FakeDrone(config)
    drone.position_sensors = {
        "rangefinder_samples": 40,     # messages ARE arriving ...
        "rangefinder_min": 0.0,        # ... but the distance never leaves zero
        "rangefinder_max": 0.0,
        "flow_samples": 40, "flow_quality_max": 150,
    }
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.abort_reason == "RANGEFINDER_NOT_TRACKING"
    assert "takeoff" in drone.actions()          # it did leave the ground ...
    assert "land" in drone.actions()             # ... and came straight back down
    assert "goto_local" not in drone.actions()   # no search pattern was ever flown
    assert "drop" not in drone.actions()


def test_zero_rangefinder_on_the_ground_does_not_block_arming():
    """A healthy sensor reads ~0 m sitting on the floor. Blocking on that would refuse
    every takeoff - SITL caught exactly this when the check was written wrong."""
    config = Config.sitl()
    drone = FakeDrone(config)
    drone.position_sensors = {
        "rangefinder_samples": 40, "rangefinder_min": 0.0, "rangefinder_max": 0.0,
        "flow_samples": 40, "flow_quality_max": 150,
    }
    mission = _build_mission(drone, config)

    assert mission.failsafe.verify_position_sensors() is None


def test_missing_optical_flow_refuses_to_arm():
    config = Config.sitl()
    drone = FakeDrone(config)
    drone.position_sensors = {
        "rangefinder_samples": 40, "rangefinder_min": 0.3, "rangefinder_max": 1.1,
        "flow_samples": 0, "flow_quality_max": None,     # no flow at all
    }
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.abort_reason == "NO_OPTICAL_FLOW_DATA"
    assert "arm" not in drone.actions()


def test_silent_rangefinder_refuses_to_arm():
    config = Config.sitl()
    drone = FakeDrone(config)
    drone.position_sensors = {
        "rangefinder_samples": 0, "rangefinder_min": None, "rangefinder_max": None,
        "flow_samples": 40, "flow_quality_max": 150,
    }
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.abort_reason == "NO_RANGEFINDER_DATA"


def test_sensor_gate_is_skipped_outdoors():
    """The gate is about optical flow. With GPS there is nothing for it to check."""
    config = Config.sitl()
    config.gps_denied = False
    drone = FakeDrone(config)
    drone.position_sensors = {
        "rangefinder_samples": 0, "rangefinder_min": None, "rangefinder_max": None,
        "flow_samples": 0, "flow_quality_max": None,
    }
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason is None
    assert "read_position_sensors" not in drone.actions()
    assert "drop" in drone.actions()             # the GPS mission still completes


def test_runaway_position_estimate_lands_the_aircraft():
    """The software counterpart to a horizontal fence: if the reported position leaves
    the envelope, land - whether it is the aircraft running away or the filter."""
    config = Config.sitl()
    config.max_position_radius_m = 15.0
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    # Airborne, then the estimate jumps far outside anything the search pattern reaches.
    original_takeoff = drone.takeoff

    def takeoff_then_diverge(altitude, timeout=30.0):
        result = original_takeoff(altitude, timeout)
        drone._north, drone._east = 300.0, 200.0
        return result

    drone.takeoff = takeoff_then_diverge
    mission.run()

    assert mission.abort_reason == "POSITION_IMPLAUSIBLE"
    assert "land" in drone.actions()             # airborne abort -> RECOVER -> LAND
    assert "drop" not in drone.actions()


def test_position_guard_can_be_disabled():
    config = Config.sitl()
    config.max_position_radius_m = 0.0           # off
    drone = FakeDrone(config)
    drone._north, drone._east = 300.0, 200.0
    mission = _build_mission(drone, config)

    assert mission.failsafe.position_implausible() is None


def test_safety_envelope_limits_horizontal_speed_too():
    """WPNAV_SPEED caps how fast a diverging estimate can be chased. The firmware
    default is 1000 cm/s - a hall crossed in under a second."""
    config = Config.sitl()
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    sets = [c for c in drone.calls if c[0] == "set_param"]
    assert ("set_param", "WPNAV_SPEED", config.cruise_speed_cms) in sets
    assert ("set_param", "WPNAV_SPEED_UP", config.climb_rate_cms) in sets
    # And the FC's own values come back on exit - the envelope must not outlive the
    # run any more than the fence may (the crash fence was exactly such a leftover).
    final = {c[1]: c[2] for c in sets}
    assert final["WPNAV_SPEED"] == 1000.0
    assert final["WPNAV_SPEED_UP"] == 250.0
    assert final["RTL_ALT"] == 1500.0
    assert not os.path.exists(config.param_backup_file)


def test_every_milestone_is_a_coherent_stage():
    """Each milestone must add exactly one unknown to the previous one."""
    from main import make_config, MILESTONES

    stages = {n: make_config(["main.py", "--milestone", str(n)]) for n in MILESTONES}

    assert stages[1].hover_test_s > 0 and stages[1].camera_source == "none"
    assert stages[2].hover_test_s > 0 and stages[2].camera_source == "real"
    assert stages[3].hover_test_s == 0 and stages[3].camera_source == "none"
    assert stages[4].camera_source == "real" and stages[4].skip_drop is True
    assert stages[5].camera_source == "real" and stages[5].skip_drop is False
    # All of them stay on the real-aircraft profile.
    assert all(c.release_mechanism == "pi" for c in stages.values())
    assert all(c.milestone == n for n, c in stages.items())


def test_unknown_milestone_is_rejected_not_ignored():
    """A typo must stop the run, not silently fly a different stage."""
    from main import make_config

    with pytest.raises(SystemExit):
        make_config(["main.py", "--milestone", "9"])


def test_milestone_substitutes_the_camera_in_simulation_and_says_so():
    """A milestone must be rehearsable in SITL - but never silently with a different
    detector than the one it names."""
    from main import make_config

    sim = make_config(["main.py", "--sim", "--milestone", "5"])
    assert sim.camera_source == "auto"            # SimCamera, not RealCamera
    assert sim.milestone_camera_substituted is True

    real = make_config(["main.py", "--milestone", "5"])
    assert real.camera_source == "real"
    assert real.milestone_camera_substituted is False


def test_pilot_takeover_during_approach_stops_the_companion():
    """Flipping out of GUIDED must stop the servo loop immediately, not after the phase
    timeout. Our setpoints are discarded from that moment anyway - what matters is that
    the companion stops talking and the log records when the human took over."""
    config = Config.sitl()
    drone = FakeDrone(config)
    camera = ScriptedCamera([{"detected": True, "dx": 3.0, "dy": 3.0, "distance": 2.0}])
    mission = _build_mission(drone, config, camera=camera)

    original_nudge = drone.move_body_offset

    def nudge_then_pilot_takes_over(forward, right, down=0.0):
        original_nudge(forward, right, down)
        drone.mode = "LOITER"          # pilot flips the mode switch mid-approach

    drone.move_body_offset = nudge_then_pilot_takes_over
    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "MODE_CHANGED_LOITER"
    assert "drop" not in drone.actions()     # never released after the takeover
    assert "land" not in drone.actions()     # and never fought the pilot with a mode


def test_geofence_is_restored_on_every_exit_path():
    """Fence parameters live in the FC and outlive the script. A 4 m 'Always Land'
    fence left behind lands the next MANUAL flight, minutes later, with no visible
    connection to the companion run that set it."""
    config = Config.sitl()
    config.geofence_enable = True      # off by default indoors, see config.py
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    sets = [c for c in drone.calls if c[0] == "set_param"]
    # It was enabled for the mission - including FENCE_ACTION, the parameter whose
    # unrestored leftover caused the 2026-08-21 crash ...
    assert ("set_param", "FENCE_ENABLE", 1) in [(c[0], c[1], int(c[2])) for c in sets]
    assert ("set_param", "FENCE_ACTION", config.fence_action) in sets
    # ... and the FC's own values were put back afterwards.
    final = {}
    for c in sets:
        final[c[1]] = c[2]
    assert final["FENCE_ENABLE"] == 0.0
    assert final["FENCE_TYPE"] == 7.0
    assert final["FENCE_ALT_MAX"] == 120.0
    assert final["FENCE_ACTION"] == 1.0
    # Everything restored -> the on-disk backup must be gone.
    assert not os.path.exists(config.param_backup_file)


def test_geofence_is_restored_even_when_the_mission_crashes():
    config = Config.sitl()
    config.geofence_enable = True
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    def explode(*_args, **_kwargs):
        raise RuntimeError("something went wrong mid-flight")

    drone.takeoff = explode
    with pytest.raises(RuntimeError):
        mission.run()

    final = {c[1]: c[2] for c in drone.calls if c[0] == "set_param"}
    assert final["FENCE_ENABLE"] == 0.0     # restored despite the crash


# ======================================================================
# Pilot-assisted start (takeover)
# ======================================================================
# The companion must never arm or take off in this mode. It inherits an aircraft the
# pilot has already flown up - which both puts the human in charge of the riskiest
# moment and breaks the flow-only deadlock (no position estimate on the ground, no
# takeoff without one).

def _airborne_after(drone, alt_m=1.2):
    """Make FakeDrone report a pilot who has armed and climbed. Rangefinder and EKF
    agree, as they do on a healthy aircraft."""
    drone._armed = True
    drone.position = {"lat": 0.0, "lon": 0.0, "alt": alt_m, "rel_alt": alt_m}
    drone.rangefinder_m = alt_m


def test_takeover_never_arms_and_never_takes_off():
    config = Config.sitl()
    config.takeover_mode = True
    config.hover_test_s = 0.2
    drone = FakeDrone(config)
    _airborne_after(drone)
    mission = _build_mission(drone, config)

    mission.run()

    actions = drone.actions()
    assert "arm" not in actions          # the pilot armed it, not us
    assert "takeoff" not in actions      # and the pilot flew it up
    assert "set_mode" in actions         # we only asked for GUIDED
    assert mission.state is State.DONE


def test_takeover_waits_until_the_pilot_is_high_enough():
    """Handing over at 10 cm would defeat the purpose - flow needs height."""
    config = Config.sitl()
    config.takeover_mode = True
    config.takeover_min_alt_m = 0.8
    config.takeover_timeout_s = 0.5      # pilot never climbs; give up quickly
    drone = FakeDrone(config)
    drone._armed = True
    drone.position = {"lat": 0.0, "lon": 0.0, "alt": 0.1, "rel_alt": 0.1}
    drone.rangefinder_m = 0.1
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.abort_reason == "PILOT_NEVER_TOOK_OFF"
    assert "set_mode" not in drone.actions()   # never grabbed control


def test_takeover_refuses_without_a_position_estimate_in_the_air():
    """Taking over a flying aircraft with no position estimate is the flyaway setup."""
    config = Config.sitl()
    config.takeover_mode = True
    drone = FakeDrone(config)
    _airborne_after(drone)
    drone.ekf_flag_ok = False            # airborne, but the EKF still has nothing
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.abort_reason == "NO_POSITION_ESTIMATE_IN_AIR"
    assert "set_mode" not in drone.actions()
    # It must not command a landing either - the pilot is flying and in control.
    assert "land" not in drone.actions()


def test_takeover_goes_straight_to_the_stage_under_test():
    """The pilot already did the climb, so TAKEOFF must be skipped entirely."""
    config = Config.sitl()
    config.takeover_mode = True
    drone = FakeDrone(config)
    _airborne_after(drone)
    mission = _build_mission(drone, config)

    mission.run()

    assert "goto_local" in drone.actions()     # went straight into SEARCH
    assert "takeoff" not in drone.actions()


def test_geofence_is_off_by_default_indoors():
    """Not a preference - a result. A barometric altitude fence sized for an indoor
    hover sits inside its own sensor's noise band: propeller downwash spiked BAlt to
    4-6.7 m in all five of our flight logs while the aircraft was centimetres off the
    floor. With FENCE_ACTION=2 that forced the vehicle out of the pilot's mode into
    LAND on every single takeoff - and once into the ceiling."""
    for config in (Config(), Config.pi(), Config.sitl(), Config.pi_serial()):
        assert config.geofence_enable is False


def test_geofence_untouched_when_disabled():
    """If we do not enable it, we must not write - and not 'restore' - anything.
    (The one exception is a STALE fence from an earlier run, tested below; with a
    clean FC like this one, no FENCE_* write may happen at all.)"""
    config = Config.sitl()
    assert config.geofence_enable is False
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    written = {c[1] for c in drone.calls if c[0] == "set_param"}
    assert not any(name.startswith("FENCE_") for name in written)


def test_stale_fence_from_an_earlier_run_is_disarmed():
    """The 2026-08-21 crash fence was a LEFTOVER: written by an earlier companion run,
    never restored (battery pulls), then armed during a manual flight. With the fence
    now off by default, the companion must still notice such a leftover and disarm it -
    touching only the ENABLE bit, nothing else."""
    config = Config.sitl()
    assert config.geofence_enable is False
    drone = FakeDrone(config)
    drone.params_before["FENCE_ENABLE"] = 1.0     # the leftover, as the FC reports it
    mission = _build_mission(drone, config)

    mission.run()

    sets = [(c[1], c[2]) for c in drone.calls if c[0] == "set_param"]
    assert ("FENCE_ENABLE", 0) in sets
    # Only the enable bit - shape and action stay for the owner to inspect.
    assert not any(name in ("FENCE_TYPE", "FENCE_ALT_MAX", "FENCE_ACTION")
                   for name, _ in sets)


def test_leftover_backup_from_a_killed_run_is_restored_first():
    """A battery pull ends the process without restore_params(). The on-disk backup
    survives, and the NEXT run must put those values back BEFORE snapshotting its own
    baseline - otherwise it would adopt the leftovers as 'before' values."""
    config = Config.sitl()
    config.param_backup_file = os.path.join(tempfile.mkdtemp(), "fc_params_backup.json")
    with open(config.param_backup_file, "w") as fh:
        json.dump({"WPNAV_SPEED": 1000.0}, fh)    # what the killed run had saved
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)

    mission.run()

    sets = [(c[1], c[2]) for c in drone.calls if c[0] == "set_param"]
    # Restored first, then this run's own envelope value was applied.
    assert sets.index(("WPNAV_SPEED", 1000.0)) < sets.index(("WPNAV_SPEED", config.cruise_speed_cms))
    # And this run cleaned up after itself again.
    assert not os.path.exists(config.param_backup_file)


def test_continuous_cadence_counts_unreached_waypoints():
    """A leg that times out is NOT arrival. The stop_and_look cadence has guarded this
    ('never pretend we got there') from the start; the continuous cadence used to fly
    the rest of the pattern on paper, one full timeout per leg."""
    config = Config.sitl()
    config.detection_cadence = "continuous"
    config.waypoint_timeout_s = 0.05          # keep the test fast
    drone = FakeDrone(config)

    def goto_local_without_moving(north, east, down):
        drone.calls.append(("goto_local", north, east))   # vehicle does NOT follow

    drone.goto_local = goto_local_without_moving
    mission = _build_mission(drone, config, camera=MockCamera(detected=False))

    mission.run()

    assert mission.abort_reason == "WAYPOINTS_UNREACHABLE"
    assert "drop" not in drone.actions()
    assert "land" in drone.actions()


def test_takeover_refuses_when_ekf_and_rangefinder_disagree():
    """The crash signature: EKF altitude +1070 m on the floor, rangefinder truthful.
    Handing a GUIDED aircraft to a filter in that state is the flyaway setup, so the
    takeover must refuse - and must not command anything at the pilot either."""
    config = Config.sitl()
    config.takeover_mode = True
    drone = FakeDrone(config)
    drone._armed = True
    drone.position = {"lat": 0.0, "lon": 0.0, "alt": 1070.0, "rel_alt": 1070.0}
    drone.rangefinder_m = 1.2                 # the pilot is really at ~1.2 m
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.abort_reason == "EKF_ALT_DIVERGED"
    assert "set_mode" not in drone.actions()
    assert "land" not in drone.actions()      # the pilot is flying - stay quiet


def test_inflight_ekf_rangefinder_disagreement_lands():
    """The crash signature, caught IN FLIGHT: after a healthy takeoff the EKF altitude
    runs away from the rangefinder. The continuous cross-check in failsafe.check()
    must land the aircraft after alt_disagree_samples consecutive mismatches - not
    fly the rest of the pattern on a meaningless height estimate."""
    config = Config.sitl()
    drone = FakeDrone(config)
    original_takeoff = drone.takeoff

    def takeoff_then_ekf_diverges(altitude, timeout=30.0):
        result = original_takeoff(altitude, timeout)
        drone.position = {"lat": 0.0, "lon": 0.0, "alt": 50.0, "rel_alt": 50.0}
        drone.rangefinder_m = 2.0            # the sensor stays truthful
        return result

    drone.takeoff = takeoff_then_ekf_diverges
    mission = _build_mission(drone, config, camera=MockCamera(detected=False))

    mission.run()

    assert mission.abort_reason == "EKF_ALT_DIVERGED"
    assert "land" in drone.actions()
    assert "drop" not in drone.actions()


def test_single_altitude_mismatch_does_not_abort():
    """One glitchy comparison (or a box under the flight path for a moment) is not a
    divergence - only a run of consecutive mismatches may abort, like the battery."""
    config = Config.sitl()
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)
    drone.position = {"lat": 0.0, "lon": 0.0, "alt": 50.0, "rel_alt": 50.0}
    drone.rangefinder_m = 2.0

    assert mission.failsafe.altitude_implausible() is None   # 1/3
    drone.position = {"lat": 0.0, "lon": 0.0, "alt": 2.0, "rel_alt": 2.0}
    assert mission.failsafe.altitude_implausible() is None   # agree -> counter resets
    drone.position = {"lat": 0.0, "lon": 0.0, "alt": 50.0, "rel_alt": 50.0}
    assert mission.failsafe.altitude_implausible() is None   # 1/3 again, not 2/3


def test_altitude_check_ignores_missing_or_out_of_range_rangefinder():
    """No reading is the telemetry failsafe's job; beyond ~7.5 m the MTF-01P value is
    not trustworthy. Neither may count as a divergence."""
    config = Config.sitl()
    drone = FakeDrone(config)
    mission = _build_mission(drone, config)
    drone.position = {"lat": 0.0, "lon": 0.0, "alt": 50.0, "rel_alt": 50.0}

    drone.rangefinder_m = None
    assert mission.failsafe.altitude_implausible() is None
    drone.rangefinder_m = 8.0
    assert mission.failsafe.altitude_implausible() is None


def test_takeover_waits_for_a_rangefinder_reading():
    """Without a rangefinder reading there is no trustworthy height to hand over on -
    the EKF altitude alone proved worthless in the crash logs (it read +1070 m on the
    ground). No reading -> keep waiting -> time out, rather than trusting the EKF."""
    config = Config.sitl()
    config.takeover_mode = True
    config.takeover_timeout_s = 0.5
    drone = FakeDrone(config)
    drone._armed = True
    drone.position = {"lat": 0.0, "lon": 0.0, "alt": 2.0, "rel_alt": 2.0}
    drone.rangefinder_m = None                # no RANGEFINDER messages at all
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.abort_reason == "PILOT_NEVER_TOOK_OFF"
    assert "set_mode" not in drone.actions()
