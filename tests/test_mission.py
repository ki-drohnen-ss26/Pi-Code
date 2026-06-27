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

from camera import MockCamera, ScriptedCamera
from config import Config
from failsafe import FailsafeMonitor
from mission import DeliveryMission, State


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
        self.calls.append(("move_body_offset", forward, right))

    # --- preparation ---
    def configure_drop_servo(self):
        self.calls.append(("configure_drop_servo",))

    def reset_servo(self):
        self._servo = self.config.neutral_pwm
        self.calls.append(("reset_servo",))

    def wait_ready_to_arm(self, timeout=60.0):
        self.calls.append(("wait_ready_to_arm",))
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
    return DeliveryMission(drone, camera, failsafe, config)


def test_happy_path_runs_to_done_and_delivers():
    config = Config.sitl()
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
