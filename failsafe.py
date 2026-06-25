"""
failsafe.py
===========
Safety monitor. Deliberately separates the safety logic from the mission logic.
On a problem the monitor returns a reason string; the state machine then decides
to switch to ABORT/RTL.

Note: this is it's own abort logic on the companion side. It is independent of
ArduPilot's internal failsafes (BATT_LOW_VOLT, FS_*). Both can - and should -
coexist.
"""

import time
from typing import Optional

from config import Config
from drone import Drone


class FailsafeMonitor:
    def __init__(self, drone: Drone, config: Config):
        self.drone = drone
        self.config = config
        self._phase_start: Optional[float] = None
        self._phase_name: str = ""

    # ------------------------------------------------------------------
    # Geofence (set once before the mission)
    # ------------------------------------------------------------------
    def setup_geofence(self) -> None:
        if self.config.geofence_enable:
            self.drone.set_param("FENCE_ENABLE", 1)
            print("[FAILSAFE] Geofence enabled (FENCE_ENABLE=1)")

    # ------------------------------------------------------------------
    # Phase timeout
    # ------------------------------------------------------------------
    def start_phase(self, name: str) -> None:
        """Mark the start of a mission phase for the timeout check."""
        self._phase_name = name
        self._phase_start = time.time()

    def phase_timed_out(self) -> bool:
        if self._phase_start is None:
            return False
        return (time.time() - self._phase_start) > self.config.phase_timeout_s

    # ------------------------------------------------------------------
    # Battery check
    # ------------------------------------------------------------------
    def battery_critical(self) -> bool:
        batt = self.drone.get_battery()
        if not batt:
            return False  # no data -> don't abort here (handle separately)
        if batt["voltage"] <= self.config.battery_min_voltage:
            return True
        if 0 <= batt["remaining"] <= self.config.battery_min_percent:
            return True
        return False

    # ------------------------------------------------------------------
    # Combined check: call once per loop iteration
    # ------------------------------------------------------------------
    def check(self) -> Optional[str]:
        """
        Returns: a reason string if an abort is required, otherwise None.
        """
        if self.battery_critical():
            return "LOW_BATTERY"
        if self.phase_timed_out():
            return f"TIMEOUT_{self._phase_name}"
        return None