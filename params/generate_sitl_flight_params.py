#!/usr/bin/env python3
"""Derive the SITL mirror of the published flight set from a SINGLE source file.

WHY THIS SCRIPT EXISTS
----------------------
Daniele's requirement (2026-08-24): SITL must test the SAME parameters we fly, and it
must do so from ONE clean file derived from `flight_v2.param` — not a chain of overlays
(phaseA -> phaseB -> gps_off -> mandate) that drifts out of sync with the flight set every
time Mission Planner publishes a new version. This script is that derivation: read
`flight_v2.param`, apply the split rule below, write `sitl_flight_v2.parm`.

WHY WE CANNOT JUST `param load flight_v2.param` INTO SITL
--------------------------------------------------------
`flight_v2.param` is a full 1159-parameter dump of the REAL aircraft. A pile of its values
are bound to the physical airframe and would break the simulator, not configure it. Three
concrete examples (verified):
  * AHRS_ORIENTATION = 13 — the real board's mounting rotation. SITL's simulated IMU is
    not physically rotated, so replaying this rotates the estimate into attitude garbage.
  * the real INS_*/COMPASS_* calibration offsets — measured against the real, imperfect
    sensors. Loaded onto SITL's already-ideal simulated sensors they MIS-calibrate them.
  * RNGFND1_TYPE = 10 / FLOW_TYPE = 5 — MAVLink hardware backends (the MTF-01P) that do
    not exist in SITL. The simulator needs its own backends (RNGFND1_TYPE 100, FLOW_TYPE
    10, SIM_FLOW_ENABLE 1) and SIM_TERRAIN 0, none of which live in the flight set.
So we MIRROR the behavioural parameters (the ones that define how the aircraft flies) and
leave the PHYSICAL parameters SITL-native (excluded), then bolt the SIM-side backends on.

THE SPLIT RULE (implemented below as a blacklist)
-------------------------------------------------
Everything in the flight set is mirrored EXCEPT the physical classes enumerated in
EXCLUDE_PREFIXES / EXCLUDE_EXACT / the RC-calibration rule — each carries a one-line WHY.
A blacklist (not a whitelist) is deliberate: a behavioural parameter that a future
`flight_v3` adds is then mirrored automatically; only genuinely hardware-bound classes
have to be named here. OVERRIDES then FORCE the values that must differ in sim — the SITL
sensor backends + GPS off, plus the four evidence-backed sim-adaptations (RNGFND1_MIN_CM and
the three battery voltages) that a validity floor / pack physics make wrong for the simulated
pack (verified 2026-08-25) — and APPENDS add the SIM_* parameters the flight set does not
carry at all.

WHY THE OUTPUT MUST BE LOADED TWICE (documented in the generated header too)
----------------------------------------------------------------------------
ArduPilot only creates the RNGFND1_* sub-parameters (MIN_CM/MAX_CM/GNDCLEAR/ORIENT) once
RNGFND1_TYPE is set AND the FC has rebooted. A single alphabetically-sorted file sends
RNGFND1_GNDCLEAR/MAX_CM/MIN_CM BEFORE RNGFND1_TYPE, so on pass one the firmware does not
yet know those names and silently drops them. The procedure is therefore always:
load, reboot, load AGAIN, reboot.

Dependency-free, deterministic (sorted output). Run with the project interpreter:
    /opt/miniconda3/envs/ki_drohnen_pi/bin/python3 params/generate_sitl_flight_params.py
Optional: --src PATH, --out PATH, --date STRING (no datetime.now() — the date is passed in
or omitted so re-running never produces a spurious diff).
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# --------------------------------------------------------------------------------------
# PHYSICAL classes to EXCLUDE (mirror nothing under these prefixes). Each line's comment
# is the WHY: what about this class is bound to the real airframe and would break SITL.
# --------------------------------------------------------------------------------------
EXCLUDE_PREFIXES: dict[str, str] = {
    "INS_":     "IMU accel/gyro calibration offsets+scales measured on the real sensors; corrupt SITL's ideal IMU.",
    "COMPASS_": "magnetometer offsets/orientation/IDs bound to the real compasses; mis-calibrate SITL's ideal mag.",
    "BARO":     "barometer ground-pressure/calibration captured on the real board; SITL runs its own baro model. (BARO_, BARO1_..3_)",
    "SERIAL":   "serial-port protocol/baud map is the physical wiring of the real FC (Pi on 4, MTF-01P on 5); SITL's ports differ.",
    "BRD_":     "board-level config (safety switch, pin/voltage, board type) belongs to the physical autopilot.",
    "MOT_":     "motor PWM ranges + thrust curve are tuned to the real ESCs/props; SITL uses its own motor model.",
    "ATC_":     "attitude-controller PID gains are tuned to the real frame (the halved PIDs); wrong for SITL's model.",
    "PSC_":     "position-controller gains are frame-tuned; SITL's dynamics differ, so keep the sim's own.",
    "SR0_":     "MAVLink stream rates for a physical telemetry port; SITL sets its own per link.",
    "SR1_":     "MAVLink stream rates for a physical telemetry port; SITL sets its own per link.",
    "SR2_":     "MAVLink stream rates for a physical telemetry port; SITL sets its own per link.",
    "SR3_":     "MAVLink stream rates for a physical telemetry port; SITL sets its own per link.",
    "SR4_":     "MAVLink stream rates for a physical telemetry port; SITL sets its own per link.",
    "STAT_":    "accumulated flight-hour/boot statistics of the physical airframe; meaningless in sim.",
    "LOG_":     "onboard logging backend/bitmask tied to the real SD-card setup.",
    "SERVO":    "servo output function/min/max/trim map to the real output rails; SITL keeps its own (SERVO9 is set at runtime by FcServo in SITL).",
    "RCMAP_":   "RC channel-to-function mapping is a transmitter/receiver artefact, not flight behaviour.",
    "RSSI_":    "RSSI input is wired to a physical pin/ADC that SITL does not have.",
    "AHRS_TRIM_": "accelerometer level-trim measured on the real airframe; SITL's IMU is already level.",
    # RNGFND1 hardware sub-parameters (backend/pin/scaling/position). The BEHAVIOURAL
    # RNGFND1 keeps (MIN_CM/MAX_CM/ORIENT/GNDCLEAR) are named individually below so this
    # prefix does not swallow them.
}

# --------------------------------------------------------------------------------------
# Single physical parameters (exact names) to EXCLUDE.
# --------------------------------------------------------------------------------------
EXCLUDE_EXACT: dict[str, str] = {
    "AHRS_ORIENTATION": "real board mounting rotation (13); SITL's IMU is not rotated, so replaying it yields attitude garbage.",
    "FORMAT_VERSION":   "storage-format marker of the physical board's parameter store; irrelevant to SITL.",
    "RC_OPTIONS":       "RC-hardware option bitmask tied to the physical receiver.",
    # RC transmitter calibration (per channel) is excluded by the RC_CALIB_RE rule below.
    # Battery SENSING hardware — pins + multipliers calibrated to the real power module.
    # The battery ACTIONS (BATT_FS_*_ACT, BATT_MONITOR, ...) are behavioural and stay
    # mirrored so the failsafe fires the same way in sim. The battery VOLTAGE thresholds
    # (BATT_ARM_VOLT/LOW_VOLT/CRT_VOLT) are pack physics, not behaviour: the real-pack values
    # refuse arming / would land constantly against SITL's ~12.6 V simulated pack, so they are
    # sim-adapted in OVERRIDES (verified 2026-08-25).
    "BATT_VOLT_PIN":    "battery voltage ADC pin on the real board; SITL reads its simulated pack.",
    "BATT_CURR_PIN":    "battery current ADC pin on the real board; SITL reads its simulated pack.",
    "BATT_VOLT_MULT":   "voltage divider ratio of the real power module; SITL's simulated voltage needs no scaling.",
    "BATT_AMP_PERVLT":  "current-sensor scale of the real power module; not applicable to the simulated pack.",
    "BATT_AMP_OFFSET":  "current-sensor zero offset of the real power module.",
    "BATT_VLT_OFFSET":  "voltage-sense offset of the real power module.",
    "BATT_SERIAL_NUM":  "serial number of the physical battery/power module.",
    # RNGFND1 hardware sub-parameters (behavioural MIN_CM/MAX_CM/ORIENT/GNDCLEAR are kept).
    "RNGFND1_ADDR":     "I2C/backend address of the real rangefinder; the SITL backend (TYPE 100) does not use it.",
    "RNGFND1_FUNCTION": "input transfer function for an analog rangefinder; N/A to the SITL backend.",
    "RNGFND1_OFFSET":   "analog zero-offset of the real sensor; N/A to the SITL backend.",
    "RNGFND1_PIN":      "ADC pin of the real rangefinder; the SITL backend has no pin.",
    "RNGFND1_POS_X":    "physical mounting position of the real rangefinder on the airframe.",
    "RNGFND1_POS_Y":    "physical mounting position of the real rangefinder on the airframe.",
    "RNGFND1_POS_Z":    "physical mounting position of the real rangefinder on the airframe.",
    "RNGFND1_PWRRNG":   "power-save range of the real sensor; N/A to the SITL backend.",
    "RNGFND1_RMETRIC":  "ratiometric-analog flag of the real sensor; N/A to the SITL backend.",
    "RNGFND1_SCALING":  "analog volts->distance scaling of the real sensor; the SITL backend reports distance directly.",
    "RNGFND1_STOP_PIN":  "GPIO stop-pin of the real sensor; the SITL backend has no such pin.",
}

# RC transmitter calibration: RC<n>_MIN/MAX/TRIM/DZ for every channel. These are the
# stick endpoints/centre/deadzone of the REAL transmitter; SITL injects its own RC and
# must keep the simulator's calibration.
RC_CALIB_RE = re.compile(r"^RC(1[0-6]|[1-9])_(MIN|MAX|TRIM|DZ|OPTION)$")
RC_CALIB_WHY = ("RC<n>_MIN/MAX/TRIM/DZ = real transmitter stick calibration; SITL uses its own "
                "injected RC. RC<n>_OPTION is excluded too: option-number validity is "
                "BUILD-dependent - the real FC accepts RC7_OPTION=182, but the default SITL "
                "build rejected it with a boot-halting 'Config Error: Failed to init' "
                "(found 2026-08-25, same failure class as the post-crash baro config error).")

# --------------------------------------------------------------------------------------
# OVERRIDES: parameters that EXIST in the flight set but MUST take a different value in
# sim. (value, why) — forced regardless of the mirrored value.
# --------------------------------------------------------------------------------------
OVERRIDES: dict[str, tuple[str, str]] = {
    "RNGFND1_TYPE": ("100", "flight set has 10 (MTF-01P over MAVLink, absent in SITL); 100 = SITL's native rangefinder backend."),
    "FLOW_TYPE":    ("10",  "flight set has 5 (MTF-01P optical flow over MAVLink, absent in SITL); 10 = SITL's native flow backend."),
    "GPS1_TYPE":    ("0",   "indoor = no GPS; matches the validated companion set_origin() flow (flight set carries 1)."),
    "RNGFND1_MIN_CM": ("0", "flight set has 1 (1 cm); SITL's landed rangefinder reads exactly 0.00 m, so the mirrored 1 cm floor flags it out-of-range-low and blocks ALL on-ground EKF flow/height init (2x2 matrix 2026-08-25: POSZ=2/MIN_CM=1 never fuses, MIN_CM=0 fuses immediately). The real aircraft keeps 1 - it reads 0.02 m, 1 cm above its floor."),
    "BATT_ARM_VOLT": ("0",  "flight set has 12.7 (real 4S Li-Ion arming floor); against SITL's ~12.6 V simulated pack it refuses arming ('Arm: Battery 1 below minimum arming voltage', observed 2026-08-25). 0 = no arming-voltage gate in sim; the failsafe ACTIONS stay mirrored."),
    "BATT_LOW_VOLT": ("10.5", "flight set has 12.4 (real-pack low-voltage threshold); against SITL's ~12.6 V simulated pack it would land constantly, so it is sim-adapted below the simulated pack (2026-08-25). Voltage is pack physics; the failsafe ACTIONS stay mirrored."),
    "BATT_CRT_VOLT": ("10",  "flight set has 12 (real-pack critical threshold); against SITL's ~12.6 V simulated pack it would trip, so it is sim-adapted below the simulated pack (2026-08-25). Voltage is pack physics; the failsafe ACTIONS stay mirrored."),
}

# --------------------------------------------------------------------------------------
# APPENDS: SIM-side parameters the flight set does NOT contain at all. (value, why)
# --------------------------------------------------------------------------------------
APPENDS: dict[str, tuple[str, str]] = {
    "SIM_FLOW_ENABLE": ("1", "turn the simulator's optical-flow model ON (no hardware equivalent in the flight set)."),
    "SIM_TERRAIN":     ("0", "OFF, or SITL measures the rangefinder against a terrain model anchored at CMAC 584 m and it reads a constant 0.00 m in Frankfurt."),
}

VERIFY_LINE = (
    "param show RNGFND1_MIN_CM RNGFND1_MAX_CM RNGFND1_GNDCLEAR EK3_SRC1_POSZ "
    "FENCE_ENABLE WPNAV_SPEED ARMING_CHECK   ->   expected 0 / 800 / 10 / 2 / 0 / 100 / 41350"
)


def exclusion_reason(name: str) -> str | None:
    """Return the WHY string if `name` is a physical parameter to exclude, else None.

    OVERRIDES win over exclusion (an overridden name is always emitted). Exact matches are
    checked before prefixes so a named RNGFND1 keep is never swallowed by a broad prefix.
    """
    if name in OVERRIDES:
        return None
    if name in EXCLUDE_EXACT:
        return EXCLUDE_EXACT[name]
    if RC_CALIB_RE.match(name):
        return RC_CALIB_WHY
    for prefix, why in EXCLUDE_PREFIXES.items():
        if name.startswith(prefix):
            return why
    return None


def parse_param_file(path: Path) -> list[tuple[str, str]]:
    """Tolerant parser: accepts comma-, space- or tab-separated `NAME<sep>VALUE` lines.

    Blank lines and `#`/`;` comments are skipped. The VALUE is kept as the original STRING
    (never re-parsed as a float) so mirrored values are byte-for-byte what the flight set
    published — no precision drift, no `1` becoming `1.0`.
    """
    pairs: list[tuple[str, str]] = []
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line[0] in "#;":
            continue
        # Split on the first comma, tab or run of spaces — whichever separates the pair.
        parts = re.split(r"[,\t ]+", line, maxsplit=1)
        if len(parts) != 2:
            continue
        name, value = parts[0].strip(), parts[1].strip()
        if name:
            pairs.append((name, value))
    return pairs


def build_output(src_pairs: list[tuple[str, str]]) -> tuple[dict[str, str], dict[str, int]]:
    """Apply the split rule. Returns (name->value to emit, count summary)."""
    out: dict[str, str] = {}
    counts = {"kept": 0, "excluded": 0, "overridden": 0, "appended": 0}

    for name, value in src_pairs:
        if name in OVERRIDES:
            out[name] = OVERRIDES[name][0]
            counts["overridden"] += 1
            continue
        if exclusion_reason(name) is not None:
            counts["excluded"] += 1
            continue
        out[name] = value
        counts["kept"] += 1

    # Overrides that were not present in the source still have to be emitted.
    for name, (value, _why) in OVERRIDES.items():
        if name not in out:
            out[name] = value
            counts["overridden"] += 1

    for name, (value, _why) in APPENDS.items():
        out[name] = value
        counts["appended"] += 1

    return out, counts


def render_header(src: Path, date_str: str) -> list[str]:
    """The generated-file banner: provenance, split rule, double-load reason, verify."""
    lines = [
        "# ======================================================================================",
        "# sitl_flight_v2.parm - the SITL MIRROR of the published flight set. GENERATED FILE.",
        "# ======================================================================================",
        "#",
        f"# Source:     params/{src.name}  (the published, Mission-Planner-owned flight set)",
        f"# Generated:  {date_str}",
        "# Generator:  params/generate_sitl_flight_params.py   (edit the generator, never this file)",
        "#",
        "# WHAT THIS IS. The behavioural parameters of the flight set - the ones that define how",
        "# the aircraft flies (EK3_*, FENCE_*, FS_*, WPNAV_*, RTL_*, LAND_*, PILOT_*, ARMING_CHECK,",
        "# the FLTMODE map, the battery failsafe ACTIONS, the rangefinder limits) - mirrored 1:1 so",
        "# SITL tests EXACTLY what we fly. The PHYSICAL parameters (real board mounting, INS_/COMPASS_",
        "# calibrations, MOT_/ATC_/PSC_ frame tuning, SERIAL wiring, board/battery pins, servo",
        "# outputs) are DROPPED so SITL keeps its own - loading them would break the simulator.",
        "#",
        "# SIM-side backends are forced/added on top: RNGFND1_TYPE 100 and FLOW_TYPE 10 replace the",
        "# MTF-01P MAVLink backends that do not exist in SITL, GPS1_TYPE 0 makes it indoor, and",
        "# SIM_FLOW_ENABLE 1 / SIM_TERRAIN 0 turn the sim's flow model on and its terrain trap off.",
        "# Four more values are sim-adapted where a validity floor / pack physics make the mirrored",
        "# value wrong for SITL (verified 2026-08-25): RNGFND1_MIN_CM 0 (SITL's landed reading is",
        "# exactly 0.00 m; the real 1 cm floor blocks ALL on-ground EKF flow/height init) and the",
        "# battery voltages BATT_ARM_VOLT 0 / BATT_LOW_VOLT 10.5 / BATT_CRT_VOLT 10 (the real-pack",
        "# thresholds refuse arming / land constantly against SITL's ~12.6 V pack; ACTIONS stay mirrored).",
        "#",
        "# >>> LOAD THIS FILE TWICE, WITH A REBOOT EACH TIME <<<",
        "#     param load params/sitl_flight_v2.parm",
        "#     reboot",
        "#     param load params/sitl_flight_v2.parm",
        "#     reboot",
        "#   WHY TWICE: ArduPilot only creates the RNGFND1_* sub-parameters (MIN_CM/MAX_CM/GNDCLEAR/",
        "#   ORIENT) after RNGFND1_TYPE is set AND the FC reboots. This file is sorted alphabetically,",
        "#   so on pass one those names are sent BEFORE RNGFND1_TYPE and silently dropped as unknown;",
        "#   pass two - with the backend now present - fills them in. (Untested convenience alternative:",
        "#   sim_vehicle.py --add-param-file <this file> at start with -w. Labelled untested on purpose.)",
        "#",
        "# VERIFY after the second reboot:",
        f"#     {VERIFY_LINE}",
        "#",
        "# NEVER load this onto the real aircraft - it is a SITL artefact. Regenerate when a new",
        "# flight_vN lands: see params/README.md, subsection \"SITL mirror of the flight set\".",
        "# ======================================================================================",
        "",
    ]
    return lines


def main(argv: list[str]) -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description="Generate the SITL mirror of the flight set.")
    ap.add_argument("--src", default=str(here / "flight_v2.param"), help="source flight set")
    ap.add_argument("--out", default=str(here / "sitl_flight_v2.parm"), help="output SITL file")
    ap.add_argument(
        "--date",
        default=None,
        help="date string for the header; omit to write a stable 'regenerate' note (no datetime.now).",
    )
    args = ap.parse_args(argv)

    src = Path(args.src)
    out = Path(args.out)
    if not src.exists():
        print(f"error: source not found: {src}", file=sys.stderr)
        return 1

    date_str = args.date if args.date else "regenerate: see params/README.md"

    src_pairs = parse_param_file(src)
    values, counts = build_output(src_pairs)

    # Deterministic: sort by name. Pad the name column for readability; single space also
    # parses fine, but the aligned column matches the other sitl_*.parm files in this folder.
    width = max((len(n) for n in values), default=0)
    body = [f"{name:<{width}} {values[name]}" for name in sorted(values)]

    text = "\n".join(render_header(src, date_str) + body) + "\n"
    out.write_text(text)

    total_src = len(src_pairs)
    print(f"wrote {out}")
    print(
        f"  source parameters : {total_src}\n"
        f"  kept (mirrored)   : {counts['kept']}\n"
        f"  excluded (physical): {counts['excluded']}\n"
        f"  overridden (sim)  : {counts['overridden']}\n"
        f"  appended (SIM_*)  : {counts['appended']}\n"
        f"  -> output lines   : {len(values)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
