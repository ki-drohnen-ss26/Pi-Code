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

import logging
import time
from typing import Optional

from config import Config
from drone import Drone

log = logging.getLogger(__name__)


class FailsafeMonitor:
    def __init__(self, drone: Drone, config: Config):
        self.drone = drone
        self.config = config
        self._phase_start: Optional[float] = None
        self._phase_name: str = ""
        self._telemetry_misses: int = 0  # consecutive reads with no telemetry

    # ------------------------------------------------------------------
    # Geofence (set once before the mission)
    # ------------------------------------------------------------------
    def setup_geofence(self) -> None:
        if not self.config.geofence_enable:
            return
        # Set the fence TYPE first (default 1 = max-altitude only, which works without a
        # horizontal position estimate - indoor-safe), then the altitude, then enable.
        self.drone.set_param("FENCE_TYPE", self.config.fence_type)
        self.drone.set_param("FENCE_ALT_MAX", self.config.fence_alt_max_m)
        self.drone.set_param("FENCE_ENABLE", 1)
        log.info(
            f"[FAILSAFE] Geofence enabled (type={self.config.fence_type}, "
            f"alt_max={self.config.fence_alt_max_m} m)"
        )

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
    # Link health
    # ------------------------------------------------------------------
    def link_lost(self) -> bool:
        """True if no FC heartbeat arrives within the configured timeout."""
        return not self.drone.link_alive(self.config.heartbeat_timeout_s)

    # ------------------------------------------------------------------
    # Battery check
    # ------------------------------------------------------------------
    def battery_critical(self, batt: dict) -> bool:
        """True if voltage or remaining capacity is at/below the abort threshold.
        Expects a valid battery dict (the None / missing-data case is handled in
        check() via the telemetry-miss counter)."""
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
        Returns a reason string if an abort is required, otherwise None.

        Order: link first (cheapest signal of a dead connection), then telemetry
        health, then battery, then the phase timeout. A SINGLE missing telemetry
        read is tolerated (streams hiccup); only a run of them aborts - this closes
        the old blind spot where missing data silently meant "all good".
        """
        if self.link_lost():
            return "LINK_LOSS"

        batt = self.drone.get_battery()
        if batt is None:
            self._telemetry_misses += 1
            if self._telemetry_misses >= self.config.telemetry_max_misses:
                return "NO_TELEMETRY"
        else:
            self._telemetry_misses = 0
            if self.battery_critical(batt):
                return "LOW_BATTERY"

        if self.phase_timed_out():
            return f"TIMEOUT_{self._phase_name}"
        return None