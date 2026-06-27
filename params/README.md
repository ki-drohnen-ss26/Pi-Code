# Flight-controller parameters

A reproducible flight needs a known flight-controller (FC) parameter set. This
folder holds the baseline so a run can always be restored to a defined state — in
SITL and, later, on the real FC.

## Files

| File           | What it is                                                       |
|----------------|------------------------------------------------------------------|
| `default.parm` | Full baseline dump, captured from SITL (or the real FC). *Capture it in the Phase 1 SITL run — see below.* |

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

Phase 4 extends this baseline with the MTF-01P (rangefinder + optical-flow serial
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
