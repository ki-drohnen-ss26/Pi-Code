# Flight-controller parameters

A reproducible flight needs a known flight-controller (FC) parameter set. This
folder holds the baseline so a run can always be restored to a defined state — in
SITL and, later, on the real FC.

> ## ⚠️ Parameter names are firmware-specific
>
> Our flight controller runs **ArduCopter 4.6.3**, so that is what SITL must run too
> (`git checkout Copter-4.6.3`). Names moved in 4.7:
>
> | 4.5 / 4.6 (ours) | 4.7+ |
> |---|---|
> | `RNGFND1_MIN_CM`, `RNGFND1_MAX_CM` (cm) | `RNGFND1_MIN`, `RNGFND1_MAX` (m) |
> | `RTL_ALT` (cm) | `RTL_ALT_M` (m) |
> | `SYSID_MYGCS`, `SYSID_ENFORCE` | `MAV_GCS_SYSID`, `MAV_OPTIONS` |
>
> `param load` **silently skips** names the firmware does not know — no error, the
> parameter just keeps its default. That is how a file can look like it loaded while
> leaving the rangefinder unusable. Always check the firmware banner
> (`ArduCopter V4.6.3`) before loading anything; `Drone.log_autopilot_version()` also
> records it in every mission log.

## Files

| File           | Firmware | What it is                                              |
|----------------|----------|---------------------------------------------------------|
| `sitl_flow_phaseA.parm` | 4.6.x | SITL: enable simulated optical flow + rangefinder (load, then reboot). |
| `sitl_flow_phaseB.parm` | 4.6.x | SITL: configure flow/rangefinder + point the EKF at flow (load after phase A reboot, then reboot again). |
| `sitl_gps_off.parm`     | 4.6.x | SITL: GPS truly off (`GPS1_TYPE 0`) + an `ARMING_CHECK` mask without the GPS bit (Phase 3). |
| `default.parm` | 4.8.0-dev | *Historical.* Full dump from the old master-branch SITL. **Not loadable on 4.6.3.** |
| `gps_denied_sitl.parm`  | 4.8.0-dev | *Historical*, indoor set with GPS still ON (Phase 2). |
| `gps_off_sitl.parm`     | 4.8.0-dev | *Historical*, indoor set with GPS off (Phase 3). |

The three overlays are the ones to use; they are small, commented and version-checked.
The full dumps document the old 4.8-dev runs and are kept for the record only — they
also contain the SITL airframe's PID tuning, which must **never** go onto the real
aircraft. A fresh baseline captured from the real FC is still missing and is the most
valuable file this folder could have.

> `default.parm` is captured during validation, not hand-written. The list below
> documents the parameters our companion code relies on, so you know what must be
> present in any baseline.

## Parameters the companion code touches or assumes

Set at runtime by the code (you do **not** need to pre-set these):

| Parameter         | Value | Set by                          | Why                                                              |
|-------------------|-------|---------------------------------|------------------------------------------------------------------|
| `FENCE_TYPE`      | `1`   | `failsafe.setup_geofence()`     | Altitude-only fence (`config.fence_type`) — works without a horizontal position (indoor-safe). |
| `FENCE_ALT_MAX`   | `4.0` | `failsafe.setup_geofence()`     | Max fence altitude in m (`config.fence_alt_max_m`).              |
| `FENCE_ENABLE`    | `1`   | `failsafe.setup_geofence()`     | Geofence on before the mission (`config.geofence_enable`).       |
| `FENCE_ACTION`    | `2`   | `failsafe.setup_safety_envelope()` | "Always Land". The firmware default `1` ("RTL or Land") **climbs** to `RTL_ALT` on a breach — into the ceiling. |
| `RTL_ALT`         | `200` cm | `failsafe.setup_safety_envelope()` | In case RTL is triggered from elsewhere. **Centimetres on 4.5/4.6**; renamed to `RTL_ALT_M` (metres) in 4.7, so the code tries both. Firmware default is 1500 cm = 15 m. |
| `WPNAV_SPEED_UP`  | `50` cm/s | `failsafe.setup_safety_envelope()` | The default 250 cm/s overshot a 2 m takeoff by more than 2 m in SITL and breached a 4 m fence. |
| `SERVO9_FUNCTION` | `0`   | `drone.configure_drop_servo()`  | "Disabled" = MAVLink/mission-controlled, so `DO_SET_SERVO` works for the drop (`config.drop_servo`). **Only for `release_mechanism="fc"` (SITL);** the real build uses `"pi"` (servo on a Pi GPIO), where the FC has no drop servo. |

Indoors the companion also sends `SET_GPS_GLOBAL_ORIGIN` (a message, not a parameter)
when `config.set_origin_on_start` is set — see Phase 3 in the roadmap.

Assumed present / worth setting on the FC side (independent of the companion
failsafe — both should coexist):

| Parameter        | Suggested | Why                                                       |
|------------------|-----------|-----------------------------------------------------------|
| `BATT_LOW_VOLT`  | `12.8`    | FC-side low-battery failsafe. Our pack is **4S Li-Ion** (4.1 V/cell full = 16.4 V, 2.8 V/cell empty = 11.2 V), so the old 10.8 V — below empty — would never have fired. Mirrors `Config.pi().battery_min_voltage`. |
| `BATT_FS_LOW_ACT`| `1` (Land)| What the FC does on low battery. **Not `2` (RTL)** indoors: RTL climbs to `RTL_ALT` first. |
| `FS_GCS_ENABLE`  | `5` (Land) or `0` | FC reacts if it stops hearing from the companion. Two catches: it only ever fires **after** it has seen a HEARTBEAT from `SYSID_MYGCS` (`Drone.tick()` now sends one every second — before that this option was silently dead), and its action must not be RTL indoors. `0` is defensible in a hall, but then note that a dead Pi has no automatic rescue. |
| `FS_THR_ENABLE`  | `3` (Land)| Radio failsafe. Same reasoning — the default `1` is RTL. |
| `RNGFND1_MIN_CM` | `1`       | **Not the 20 cm default.** The MTF-01P sits a few cm above the floor; below `RNGFND1_MIN_CM` the driver reports "out of range low", the EKF gets no terrain height, optical flow cannot be scaled and arming fails with *"Need Position Estimate"*. |

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
  the "waiting for home" pre-arm are satisfied. (For a *truly* GPS-denied test — Phase 3
  — set `GPS1_TYPE 0` and `config.set_origin_on_start = True`; the companion then sets the
  origin itself and uses the altitude-only fence. Capture that as `gps_off_sitl.parm`.)

After the second reboot, wait until the EKF reports a relative position estimate, then
run `python main.py` (with `config.gps_denied = True`). Once it flies, capture the
working set: `param fetch` → `param save gps_denied_sitl.parm` → copy it here.
