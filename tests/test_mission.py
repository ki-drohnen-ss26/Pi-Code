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
        self._north = 0.0  # local NED position, updated by goto_local / move_body_offset
        self._east = 0.0

    # --- used by the failsafe ---
    def set_param(self, name, value, timeout=3.0):
        self.calls.append(("set_param", name, value))
        return float(value)

    def get_battery(self, timeout=2.0):
        if isinstance(self._battery, list):
            value = self._battery[min(self._batt_i, len(self._battery) - 1)]
            self._batt_i += 1
            return value
        return self._battery

    def link_alive(self, timeout=3.0):
        return self.link_alive_ok

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
    assert actions.index("takeoff") < actions.index("drop") < actions.index("return_to_launch")


def test_low_battery_aborts_before_drop_but_still_returns():
    config = Config.sitl()
    # Voltage below battery_min_voltage (10.8 V) -> failsafe fires on the first check.
    drone = FakeDrone(config, battery={"voltage": 10.0, "current": 1.0, "remaining": 15})
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "LOW_BATTERY"
    assert "drop" not in drone.actions()          # no delivery on abort
    assert "return_to_launch" in drone.actions()  # but it returned safely


def test_failed_arming_aborts_into_rtl():
    config = Config.sitl()
    drone = FakeDrone(config, arm_ok=False)
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert "drop" not in drone.actions()
    assert "return_to_launch" in drone.actions()


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


def test_link_loss_aborts_before_drop():
    config = Config.sitl()
    drone = FakeDrone(config, link_alive_ok=False)
    mission = _build_mission(drone, config)

    mission.run()

    assert mission.state is State.DONE
    assert mission.abort_reason == "LINK_LOSS"
    assert "drop" not in drone.actions()
    assert "return_to_launch" in drone.actions()


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
    assert "return_to_launch" in drone.actions()


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
