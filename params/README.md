# Flight-controller parameters

A reproducible flight needs a known flight-controller (FC) parameter set. This
folder holds the baseline so a run can always be restored to a defined state — in
SITL and, later, on the real FC.

## Files

| File           | What it is                                                       |
|----------------|------------------------------------------------------------------|
| `default.parm` | Full baseline dump, captured from SITL (or the real FC). *Capture it in the Phase 1 SITL run — see below.* |
| `sitl_flow_phaseA.parm` | SITL: enable simulated optical flow + rangefinder (load, then reboot). |
| `sitl_flow_phaseB.parm` | SITL: configure flow/rangefinder + point the EKF at flow, GPS off (load after phase A reboot). |
| `gps_denied_sitl.parm`  | *(capture after a working indoor run)* the full GPS-denied param set. |

> `default.parm` is captured during validation, not hand-written. The list below
> documents the parameters our companion code relies on, so you know what must be
> present in any baseline.

## Parameters the companion code touches or assumes

Set at runtime by the code (you do **not** need to pre-set these):

| Parameter         | Value | Set by                          | Why                                                              |
|-------------------|-------|---------------------------------|------------------------------------------------------------------|
| `FENCE_ENABLE`    | `1`   | `failsafe.setup_geofence()`     | Geofence on before the mission (`config.geofence_enable`).       |
| `SERVO9_FUNCTION` | `0`   | `drone.configure_drop_servo()`  | "Disabled" = MAVLink/mission-controlled, so `DO_SET_SERVO` works for the drop (`config.drop_servo`). |

Assumed present / worth setting on the FC side (independent of the companion
failsafe — both should coexist):

| Parameter        | Suggested | Why                                                       |
|------------------|-----------|-----------------------------------------------------------|
| `BATT_LOW_VOLT`  | `10.8`    | FC-side low-battery failsafe (mirror of `config.battery_min_voltage`). |
| `BATT_FS_LOW_ACT`| `2` (RTL) | What the FC does on low battery.                          |
| `FS_GCS_ENABLE`  | `1`       | FC reacts if the companion/GCS link drops (our `LINK_LOSS` is the companion-side counterpart). |

Phase 5 extends this baseline with the MTF-01P (rangefinder + optical-flow serial
protocol) and the Pi TELEM-port parameters.

## Capture the baseline from SITL (Phase 1)

With SITL running (`sim_vehicle.py -v ArduCopter --console`), in the MAVProxy
console:

```
param fetch
param save default.parm
```

The file lands in MAVProxy's working directory — copy it here as
`params/default.parm`.

## Restore a baseline

- **MAVProxy:** `param load default.parm`
- **QGroundControl:** Vehicle Setup → Parameters → Tools → *Load from file*

## Reset SITL to firmware defaults (clean slate)

The simplest way to wipe all parameters back to the firmware defaults is to restart
SITL with the wipe flag:

```
sim_vehicle.py -v ArduCopter --console -w
```

(`-w` wipes the simulated EEPROM on startup.) Alternatively, in MAVProxy:
`param set FORMAT_VERSION 0` then `reboot`.

## Configure SITL for the MTF-01P stand-in (optical flow + LiDAR)

Load the two overlays instead of typing params one by one — this avoids the
paste/garbling problems of multi-line input in the MAVProxy console. The values match
ArduPilot's own `Tools/autotest/default_params/copter-optflow.parm`.

```
param load /Users/danieleamore/Studium/Drohnen-mit-KI/Pi-Code/params/sitl_flow_phaseA.parm
reboot
# wait for reconnect, then:
param load /Users/danieleamore/Studium/Drohnen-mit-KI/Pi-Code/params/sitl_flow_phaseB.parm
reboot
```

**The second reboot is required** — the analog rangefinder backend only picks up
`RNGFND1_PIN` on the next boot, otherwise pre-arm reports *"Rangefinder 1: Not
Detected"*.

Notes:
- We do **not** set `SIM_SONAR_SCALE` (its default 12.1212 already matches
  `RNGFND1_SCALING` 12.12 — overriding it breaks the rangefinder).
- We do **not** disable GPS. The EKF uses optical flow for horizontal position/velocity
  (`EK3_SRC1_POSXY 0`, `VELXY 5`); GPS only provides the origin/home so the geofence and
  the "waiting for home" pre-arm are satisfied. (For a *truly* GPS-denied test, set
  `GPS1_TYPE 0` — but then expect to handle home/fence separately.)

After the second reboot, wait until the EKF reports a relative position estimate, then
run `python main.py` (with `config.gps_denied = True`). Once it flies, capture the
working set: `param fetch` → `param save gps_denied_sitl.parm` → copy it here.
