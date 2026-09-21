"""
paramcheck.py
=============
One shared, read-only answer to "does the flight controller carry the parameters we
published?". Used by `preflight.py` (the manual sweep, all documented parameters) and
by `failsafe.verify_flight_parameters()` (the gate the mission runs before it arms, a
curated subset).

WHY THIS EXISTS AS ITS OWN MODULE
---------------------------------
Since the 2026-08-24 ownership decision the companion writes no FC parameter: Mission
Planner plus the published, versioned flight set own the configuration. That decision
only works if something notices when the live aircraft drifts away from the published
file, and noticing has to happen **before** the mission arms, not in a tool a human
remembers to run. Both callers need the same file parser, the same version resolution
and the same comparison, so there is one copy of each here.

Deliberately free of `drone.py` and `pymavlink`: this module reads files and compares
dictionaries. The caller supplies the live values, whichever way it reads them.

WHICH FILE IS "THE PUBLISHED SET"
---------------------------------
Resolved by VERSION, not by a hard-coded name, so publishing `flight_v3.param` needs no
source edit anywhere:

    real aircraft -> params/flight_v<N>.param      (highest N)
    simulation    -> params/sitl_flight_v<N>.parm  (highest N, the generated mirror)

The simulator gets the mirror rather than the flight set because the mirror deliberately
deviates in named places (SITL sensor backends, GPS off, the `RNGFND1_MIN_CM` validity
floor, the simulated pack's battery voltages). Comparing SITL against the real flight set
would report those intended deviations as faults and teach everyone to ignore the check.

CRITICAL vs INFORMATIONAL
-------------------------
Not every difference is a reason to refuse a flight, and a gate that cries wolf gets
switched off. So the subset the mission verifies is split in two:

* CRITICAL: the values the companion's own safety reasoning depends on. The fence, the
  EKF source set, the rangefinder/flow backends and their limits, the speed envelope,
  the battery failsafe actions. A difference here means the aircraft in front of you is
  not the aircraft the code was reasoned about, so the default is to refuse.
* INFORMATIONAL: real but not flight-critical, or knowingly in flux. `ARMING_CHECK` and
  the `FLTMODE` map live here: the team removed the compass bit on the aircraft
  (41350 -> 41346) while the hall's magnetic problem is open, and the switch mapping is
  currently done transmitter-side and lands in flight-set v3. Those are printed, never
  flown into a refusal.
"""

import os
import re
from typing import Dict, Iterable, List, Optional, Tuple

PARAMS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "params")

# Highest N wins. Two patterns because the real flight set and its generated SITL mirror
# are versioned together but named apart (and carry different extensions, which is the
# one visible reminder that they must never be loaded onto each other's target).
_REAL_RE = re.compile(r"^flight_v(\d+)\.param$", re.IGNORECASE)
_SIM_RE = re.compile(r"^sitl_flight_v(\d+)\.parm$", re.IGNORECASE)

# ----------------------------------------------------------------------------------
# What the mission verifies before it arms.
# ----------------------------------------------------------------------------------
# CRITICAL: a difference here means the companion's safety reasoning no longer describes
# the aircraft in front of you. Each line says which piece of reasoning depends on it.
# `FENCE_ENABLE` is deliberately NOT here even though an indoor altitude fence caused
# the 2026-08-21 ceiling crash: `failsafe.verify_fence_disabled()` already refuses to
# fly on it, and does a better job - it reads FENCE_TYPE/ALT_MAX/ACTION too and names
# the fault `UNEXPECTED_FENCE_ENABLED` instead of a generic parameter mismatch. Two
# gates giving one fault two names is worse than one gate giving it the right one.
CRITICAL_PARAMS: Tuple[str, ...] = (
    "EK3_SRC1_POSXY",    # the EKF source set IS the indoor navigation design
    "EK3_SRC1_VELXY",
    "EK3_SRC1_POSZ",
    "EK3_SRC1_VELZ",
    "EK3_SRC1_YAW",
    "AHRS_EKF_TYPE",
    "RNGFND1_TYPE",      # no rangefinder backend -> no height -> flow cannot be scaled
    "RNGFND1_MIN_CM",    # the validity floor: a landed reading below it is discarded
    "RNGFND1_MAX_CM",
    "RNGFND1_ORIENT",
    "RNGFND1_GNDCLEAR",  # the EKF's expected on-ground reading
    "FLOW_TYPE",         # no flow backend -> no horizontal position indoors
    "WPNAV_SPEED",       # the brake on a flyaway, and the speed the timeouts assume
    "WPNAV_SPEED_UP",    # the default 250 cm/s overshot a 2 m takeoff by over 2 m
    "BATT_FS_LOW_ACT",   # must not be RTL indoors: RTL climbs to RTL_ALT first
    "BATT_FS_CRT_ACT",
    "FS_THR_ENABLE",
    "RTL_ALT",           # in case RTL is triggered from somewhere else
)

# INFORMATIONAL: printed, never a refusal. Real differences, but ones the team is
# knowingly carrying (see the module docstring).
INFORMATIONAL_PARAMS: Tuple[str, ...] = (
    "ARMING_CHECK",
    "BATT_MONITOR",
    "BATT_LOW_VOLT",
    "BATT_CRT_VOLT",
    "FS_EKF_ACTION",
    "FS_GCS_ENABLE",
    "FLOW_ORIENT_YAW",
)

VERIFIED_PARAMS: Tuple[str, ...] = CRITICAL_PARAMS + INFORMATIONAL_PARAMS

# Parameters are compared as floats, so the comparison needs a tolerance rather than
# equality. 1e-4 is far below any meaningful step in an ArduPilot parameter and well
# above the float32 round trip a PARAM_VALUE goes through.
TOLERANCE = 1e-4


def newest_flight_set(simulated: bool = False, directory: Optional[str] = None) -> Optional[str]:
    """Path of the highest-numbered published parameter file, or None if there is none.

    `simulated=True` resolves the generated SITL mirror instead of the flight set, for
    the reason in the module docstring: the mirror deviates from the aircraft on purpose,
    so it is the only honest reference for a simulated run.
    """
    directory = directory or PARAMS_DIR
    if not os.path.isdir(directory):
        return None
    pattern = _SIM_RE if simulated else _REAL_RE
    best_version = -1
    best_path = None
    for name in os.listdir(directory):
        match = pattern.match(name)
        if not match:
            continue
        version = int(match.group(1))
        if version > best_version:
            best_version = version
            best_path = os.path.join(directory, name)
    return best_path


def load_param_file(path: Optional[str]) -> Optional[Dict[str, float]]:
    """Parse a published parameter file into {NAME: float}, tolerantly.

    Mission Planner and ArduPilot tooling emit several shapes of the same thing:
    `NAME,VALUE` (our flight sets), `NAME VALUE` or `NAME<tab>VALUE`, sometimes with a
    trailing comment. Blank lines and `#`/`//` comments are skipped. A line whose value
    is not a number is skipped rather than aborting the whole check: the goal is a
    best-effort comparison, not a strict parser. Returns None when the file is absent,
    which every caller must treat as "no verification", never as "nothing differs".
    """
    if not path or not os.path.exists(path):
        return None
    expected: Dict[str, float] = {}
    with open(path) as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#") or line.startswith("//"):
                continue
            parts = line.replace(",", " ").replace("\t", " ").split()
            if len(parts) < 2:
                continue
            try:
                expected[parts[0]] = float(parts[1])
            except ValueError:
                continue
    return expected


def matches(live: Optional[float], want: float) -> bool:
    """True when a live reading equals the published value within TOLERANCE."""
    return live is not None and abs(live - want) <= TOLERANCE


def compare(live: Dict[str, Optional[float]], expected: Dict[str, float],
            names: Optional[Iterable[str]] = None) -> List[Tuple[str, Optional[float], float]]:
    """Every (name, live_value, expected_value) where the two differ.

    A parameter the file does not document is skipped: the file is the reference, so
    "not published" means "no opinion", not "wrong".

    A parameter the FC did not answer for (`live` is None) is also skipped here, and
    the caller reports it separately. It is a finding, but not the same finding: a busy
    link routinely drops a few PARAM_VALUE replies, and that is indistinguishable from
    a name the firmware does not know. Counting silence as a difference would ground a
    flight over a lost packet, which is how a safety gate earns its way into being
    switched off.
    """
    names = list(names) if names is not None else list(live)
    out: List[Tuple[str, Optional[float], float]] = []
    for name in names:
        if name not in expected or name not in live:
            continue
        value = live[name]
        if value is None:
            continue
        if not matches(value, expected[name]):
            out.append((name, value, expected[name]))
    return out
