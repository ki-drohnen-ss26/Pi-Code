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
        self._phase_budget: float = config.phase_timeout_s
        self._telemetry_misses: int = 0  # consecutive reads with no telemetry
        self._battery_lows: int = 0      # consecutive readings below the threshold
        self._mode_monitored: bool = False  # armed by watch_mode() once we set GUIDED

    # ------------------------------------------------------------------
    # FC-side safety envelope (set once before the mission)
    # ------------------------------------------------------------------
    def setup_safety_envelope(self) -> None:
        """Force the flight-controller limits that make an indoor abort survivable.

        This is NOT our own failsafe logic - it is what the AUTOPILOT does on its own
        when something goes wrong, and its defaults are built for open sky:

          * FENCE_ACTION defaults to 1 ("RTL or Land"), and RTL first CLIMBS to
            RTL_ALT (default 1500 cm = 15 m) before returning. In a hall that means
            flying into the ceiling. We set 2 ("Always Land") plus a low RTL_ALT.
          * WPNAV_SPEED_UP defaults to 250 cm/s, which overshoots a 2 m takeoff by
            more than 2 m - enough to breach a low altitude fence on its own.

        We set these before every flight rather than trusting whatever happens to be
        stored in the FC, because a wiped or freshly flashed board silently reverts to
        the outdoor defaults.
        """
        if not self.config.enforce_safety_envelope:
            log.warning("[FAILSAFE] Safety envelope DISABLED by config")
            return
        # A parameter the firmware does not know must not throw us out of a mission on
        # the ground, but it MUST be visible: whatever we failed to set is still at the
        # autopilot's outdoor default.
        self._try_param("WPNAV_SPEED_UP", self.config.climb_rate_cms)
        self._try_param("FENCE_ACTION", self.config.fence_action)
        self._set_rtl_altitude(self.config.rtl_alt_m)
        log.info(
            f"[FAILSAFE] Safety envelope: climb<={self.config.climb_rate_cms} cm/s, "
            f"FENCE_ACTION={self.config.fence_action} (2=Always Land), "
            f"RTL_ALT={self.config.rtl_alt_m} m"
        )

    def _try_param(self, name: str, value: float) -> bool:
        """set_param, but a firmware that does not know the parameter is a warning
        rather than a crash."""
        try:
            self.drone.set_param(name, value)
            return True
        except TimeoutError:
            log.warning(f"[FAILSAFE] {name} not accepted - it keeps the firmware default")
            return False

    def _set_rtl_altitude(self, metres: float) -> None:
        """Set the RTL altitude, coping with the 4.6 -> 4.7 parameter rename.

        ArduPilot 4.5/4.6 has RTL_ALT in CENTIMETRES; 4.7 renamed it to RTL_ALT_M in
        METRES. Setting the wrong one is not an error - the autopilot simply ignores an
        unknown parameter, leaving the 15 m default in place. So try the name our
        flight controller uses first and fall back to the newer one.
        """
        try:
            self.drone.set_param("RTL_ALT", metres * 100.0)   # 4.5 / 4.6: centimetres
            return
        except TimeoutError:
            pass
        try:
            self.drone.set_param("RTL_ALT_M", metres)         # 4.7+: metres
        except TimeoutError:
            log.warning(
                "[FAILSAFE] Could not set the RTL altitude (neither RTL_ALT nor "
                "RTL_ALT_M exists) - an RTL may climb to the firmware default"
            )

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
    def start_phase(self, name: str, budget_s: Optional[float] = None) -> None:
        """Mark the start of a mission phase for the timeout check.

        `budget_s` lets a phase carry its own budget. SEARCH needs this: the default
        spiral has 25 waypoints, so charging the whole phase against phase_timeout_s
        (60 s) aborts the mission long before the pattern has been flown.
        """
        self._phase_name = name
        self._phase_start = time.time()
        self._phase_budget = self.config.phase_timeout_s if budget_s is None else budget_s

    def phase_timed_out(self) -> bool:
        if self._phase_start is None:
            return False
        return (time.time() - self._phase_start) > self._phase_budget

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
    # ------------------------------------------------------------------
    # Mode monitoring
    # ------------------------------------------------------------------
    def watch_mode(self) -> None:
        """Arm the mode check. Call this once the mission has actually put the FC into
        its working mode.

        Before that the vehicle is legitimately in whatever mode it booted into
        (STABILIZE), so checking from the first loop iteration aborts every mission
        before it starts - which is exactly what happened the first time this check
        went in.
        """
        self._mode_monitored = True
        log.info(f"[FAILSAFE] Watching for a mode change away from {self.config.expect_mode}")

    def mode_lost(self) -> Optional[str]:
        """Reason string if the FC is no longer in the mode we command from.

        The autopilot can leave GUIDED without asking us: the pilot flips a switch, a
        fence is breached, an EKF failsafe fires. From that moment every setpoint we
        send is silently discarded - and this is the dangerous part - our own abort
        path would then command RTL and OVERRIDE the human who just took control. So
        losing the mode is an abort reason in its own right, handled by stopping
        rather than by commanding anything.

        We saw exactly this in SITL: a fence breach switched the vehicle to RTL while
        the mission kept sending waypoints for 60 s to a vehicle that had already
        crashed.
        """
        expected = self.config.expect_mode
        if not expected or not self._mode_monitored:
            return None
        mode = self.drone.get_mode()
        if mode and mode != expected:
            return f"MODE_CHANGED_{mode}"
        return None

    # ------------------------------------------------------------------
    # Combined check: call once per loop iteration
    # ------------------------------------------------------------------
    def check(self) -> Optional[str]:
        """
        Returns a reason string if an abort is required, otherwise None.

        Order: link first (cheapest signal of a dead connection), then mode, then
        telemetry health, then battery, then the phase timeout. A SINGLE missing
        telemetry read is tolerated (streams hiccup); only a run of them aborts - this
        closes the old blind spot where missing data silently meant "all good". The
        same applies to battery voltage, which sags under load.
        """
        self.drone.tick()   # heartbeat out, autopilot messages in

        if self.link_lost():
            return "LINK_LOSS"

        mode_reason = self.mode_lost()
        if mode_reason:
            return mode_reason

        batt = self.drone.get_battery()
        if batt is None:
            self._telemetry_misses += 1
            if self._telemetry_misses >= self.config.telemetry_max_misses:
                return "NO_TELEMETRY"
        else:
            self._telemetry_misses = 0
            if self.battery_critical(batt):
                self._battery_lows += 1
                if self._battery_lows >= self.config.battery_low_samples:
                    return "LOW_BATTERY"
            else:
                self._battery_lows = 0

        if self.phase_timed_out():
            return f"TIMEOUT_{self._phase_name}"
        return None