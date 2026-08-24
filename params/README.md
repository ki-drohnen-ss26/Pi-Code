# Flight-controller parameters

A reproducible flight needs a known flight-controller (FC) parameter set. This folder
holds the SITL baseline so a simulated run can always be restored to a defined state,
**plus** — since 2026-08-22 — the recovered baseline of the **real** flight controller:
`fc_baseline_463_20260821.parm` was reconstructed from the crash day's dataflash log
(the FC writes every parameter into its own log at boot) after a custom-firmware flash
wiped the live values. It is the only surviving full dump of the real aircraft's
configuration, including the accel calibration, ESC/servo setup, the MTF-01P serial
config and the Pi companion link. **Never load it without `fc_safe_overrides.parm` on
top** — see *Restoring the real FC after the wipe* below.

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
>
> This check is about *names*, not about *targets*: a matching banner does not make a
> file safe to load, because the real FC and SITL both run 4.6.3. See the second
> warning below — no file in this folder goes onto the real aircraft.

## Files

| File           | Firmware | What it is                                              |
|----------------|----------|---------------------------------------------------------|
| `fc_baseline_463_20260821.parm` | 4.6.3 | **REAL FC.** Full baseline of the actual aircraft, recovered from the 2026-08-21 dataflash log (1154 parameters). Contains the CRASH configuration — load only together with the overrides below. |
| `fc_safe_overrides.parm` | 4.6.3 | **REAL FC.** The corrections from the crash analysis: fence off, `EK3_SRC1_POSZ 1`, arming checks on, 4S-Li-Ion battery failsafe. Load after the baseline, then reboot. |
| `sitl_flow_phaseA.parm` | 4.6.x | SITL: enable simulated optical flow + rangefinder (load, then reboot). |
| `sitl_flow_phaseB.parm` | 4.6.x | SITL: configure flow/rangefinder + point the EKF at flow (load after phase A reboot, then reboot again). |
| `sitl_gps_off.parm`     | 4.6.x | SITL: GPS truly off (`GPS1_TYPE 0`) + an `ARMING_CHECK` mask without the GPS bit (Phase 3). |
| `sitl_indoor_463.parm`  | 4.6.3 | **SITL restore point — never load onto the real FC.** Full dump captured from the first fully green indoor *SITL* mission on 4.6.3; carries the SITL airframe tuning, sensor backends and serial config (see the warning below). Load this to get the *simulator* back to a known-good state in one step. |
| `default.parm` | 4.8.0-dev | *Historical.* Full dump from the old master-branch SITL. **Not loadable on 4.6.3.** |
| `gps_denied_sitl.parm`  | 4.8.0-dev | *Historical*, indoor set with GPS still ON (Phase 2). |
| `gps_off_sitl.parm`     | 4.8.0-dev | *Historical*, indoor set with GPS off (Phase 3). |

The three overlays are the ones to use in SITL; they are small, commented and
version-checked. The full dumps document the old 4.8-dev runs and are kept for the
record only.

> ## ⚠️ Every `sitl_*` / historical file in this folder is a SITL artefact
>
> Not just the historical 4.8-dev dumps — `sitl_indoor_463.parm` and the three overlays
> too. **None of them may be loaded onto the real flight controller.** (The two
> `fc_*.parm` files above are the exception: they are FOR the real FC and carry the
> same danger in reverse — they would misconfigure SITL.) The SITL files carry:
>
> - the SITL airframe's PID tuning (`ATC_RAT_RLL_P`/`ATC_RAT_PIT_P 0.135`) and 364
>   `SIM_*` parameters, plus simulated compass IDs;
> - SITL sensor backends — `RNGFND1_TYPE 100` ("SITL"), `FLOW_TYPE 10`,
>   `SIM_FLOW_ENABLE 1` — where the real aircraft reads an MTF-01P over MAVLink
>   (`RNGFND1_TYPE 10`, `FLOW_TYPE 5`);
> - SITL's untouched serial ports — `SERIAL4_PROTOCOL 5` / `SERIAL4_BAUD 230` and
>   `SERIAL5_PROTOCOL -1` (disabled) — where the real FC has the Pi on SERIAL4 (MAVLink2
>   @921600) and the MTF-01P on SERIAL5 (MAVLink1 @115200). Loading this would sever the
>   companion link and switch off the sensor port in one go;
> - `BATT_LOW_VOLT 10.5`, below the empty voltage of our 4S Li-Ion pack — the table
>   further down asks for `12.8` on the real FC for exactly that reason.
>
> **A matching firmware version is not a guard.** The real FC and SITL both run
> ArduCopter 4.6.3, so these files load *cleanly* onto the aircraft and silently replace
> its tuning, sensor backends and serial wiring. The version check at the top of this
> file protects against the 4.8-dev dumps only; nothing protects against this.
>
> The same warning applies to `mav.parm` in the repo root: despite living outside this
> folder it is a **SITL capture** (`FLOW_TYPE 10`, `RNGFND1_TYPE 100`, `SERIAL5`
> disabled) and must never go onto the aircraft.

## Restoring the real FC after the wipe (2026-08-22)

Flashing the colleague's custom 4.8.0-dev build **reset every parameter to firmware
defaults** (Mission Planner showed `New mission / New rally / New fence`; the battery
monitor read 0 V). After flashing back to **stock ArduCopter 4.6.3** — which requires
the barometer problem to be fixed first, see `../docs/SIM_TO_REAL.md` §5c — restore
the aircraft like this, in Mission Planner (CONFIG → Full Parameter List → Load from
file → Write params), or MAVProxy `param load`:

```
1. load  fc_baseline_463_20260821.parm      # the aircraft as it was (incl. calibration)
2. load  fc_safe_overrides.parm             # fence off, EK3_SRC1_POSZ 1, checks on
3. reboot
4. python preflight.py                      # sensor health + EKF-drift verdict
```

The baseline deliberately stays byte-faithful to the crash-day state (it is also the
forensic record); every safety-relevant deviation lives visibly in the small,
commented overrides file. Do a compass calibration afterwards if the GPS/compass
module was replaced.

> Full dumps are captured from a working run, not hand-written. The list below
> documents the parameters our companion code relies on, so you know what must be
> present in any baseline.

## Parameters the companion code touches or assumes

Set at runtime by the code (you do **not** need to pre-set these). Everything the
failsafe writes is **saved first and restored on exit** (`restore_params()`, mirrored
to `logs/fc_params_backup.json` so even a killed run is cleaned up by the next one):

| Parameter         | Value | Set by                          | Why                                                              |
|-------------------|-------|---------------------------------|------------------------------------------------------------------|
| `WPNAV_SPEED_UP`  | `50` cm/s | `failsafe.setup_safety_envelope()` | The default 250 cm/s overshot a 2 m takeoff by more than 2 m in SITL. |
| `WPNAV_SPEED`     | `100` cm/s | `failsafe.setup_safety_envelope()` | The default 1000 cm/s crosses a hall in under a second; also the brake on a flyaway. |
| `RTL_ALT`         | `200` cm | `failsafe.setup_safety_envelope()` | In case RTL is triggered from elsewhere. **Centimetres on 4.5/4.6**; renamed to `RTL_ALT_M` (metres) in 4.7, so the code tries both. Firmware default is 1500 cm = 15 m. |
| `FENCE_TYPE`      | `1`   | `failsafe.setup_geofence()` — **only when `config.geofence_enable=True` (default False, see SIM_TO_REAL §5c)** | Altitude-only fence — works without a horizontal position. |
| `FENCE_ALT_MAX`   | `4.0` | `failsafe.setup_geofence()` (same gate) | Max fence altitude in m (`config.fence_alt_max_m`). |
| `FENCE_ACTION`    | `2`   | `failsafe.setup_geofence()` (same gate; moved out of the envelope after the crash) | "Always Land". The firmware default `1` ("RTL or Land") **climbs** to `RTL_ALT` on a breach — into the ceiling. |
| `FENCE_ENABLE`    | `1`   | `failsafe.setup_geofence()` (same gate) | Geofence on before the mission. With the fence *disabled*, the companion instead checks for — and disarms — a stale `FENCE_ENABLE=1` left by an earlier run. |
| `SERVO9_FUNCTION` | `0`   | `drone.configure_drop_servo()`  | "Disabled" = MAVLink/mission-controlled, so `DO_SET_SERVO` works for the drop (`config.drop_servo`). **Only for `release_mechanism="fc"` (SITL);** the real build uses `"pi"` (servo on a Pi GPIO), where the FC has no drop servo. Deliberately not restored. |

Indoors the companion also sends `SET_GPS_GLOBAL_ORIGIN` (a message, not a parameter)
when `config.set_origin_on_start` is set — see Phase 3 in the roadmap.

Assumed present / worth setting on the FC side (independent of the companion
failsafe — both should coexist):

| Parameter        | Suggested | Why                                                       |
|------------------|-----------|-----------------------------------------------------------|
| `BATT_LOW_VOLT`  | `12.8`    | FC-side low-battery failsafe. Our pack is **4S Li-Ion** (4.1 V/cell full = 16.4 V, 2.8 V/cell empty = 11.2 V), so the old 10.8 V — below empty — would never have fired. Mirrors `config.battery_min_voltage`, whose default of `12.8` **is** the real-aircraft value; only `Config.sitl()` lowers it to 10.8 V for the simulated pack. |
| `BATT_FS_LOW_ACT`| `1` (Land)| What the FC does on low battery. **Not `2` (RTL)** indoors: RTL climbs to `RTL_ALT` first. |
| `FS_GCS_ENABLE`  | `5` (Land) or `0` | FC reacts if it stops hearing from the companion. Two catches: it only ever fires **after** it has seen a HEARTBEAT from `SYSID_MYGCS` (`Drone.tick()` now sends one every second — before that this option was silently dead), and its action must not be RTL indoors. `0` is defensible in a hall, but then note that a dead Pi has no automatic rescue. |
| `FS_THR_ENABLE`  | `3` (Land)| Radio failsafe. Same reasoning — the default `1` is RTL. |
| `RNGFND1_MIN_CM` | `1`       | **Not the 20 cm default.** The MTF-01P sits a few cm above the floor; below `RNGFND1_MIN_CM` the driver reports "out of range low", the EKF gets no terrain height, optical flow cannot be scaled and arming fails with *"Need Position Estimate"*. |

Phase 5 extends this baseline with the MTF-01P (rangefinder + optical-flow serial
protocol) and the Pi TELEM-port parameters.

## Capture a baseline

After a run that actually flew, capture the working set so it can be restored:

```
param fetch
param save sitl_indoor_463.parm
```

The file lands in MAVProxy's working directory (`Simulation/ardupilot/`) — copy it here.
MAVProxy also keeps a live cache in `mav.parm` in the same directory, which is usually
already up to date.

`sitl_indoor_463.parm` in this folder is such a capture: the SITL state of the **first
fully green indoor mission on 4.6.3** (origin set by the companion, GPS off, optical flow +
rangefinder, search → approach → drop → land). It includes the three safety-envelope
parameters the companion sets at runtime (`FENCE_ACTION`, `RTL_ALT`, `WPNAV_SPEED_UP`)
as well as the geofence parameters (`FENCE_TYPE`, `FENCE_ALT_MAX`, `FENCE_ENABLE`) and
— this being a SITL capture — `SERVO9_FUNCTION`, which the code only sets when
`release_mechanism="fc"`. That is intentional: it is a restore point for a known-good
run, not a minimal overlay.

## Restore a baseline

**In SITL only.** Every file here is a SITL artefact, so both routes below are for the
simulator. Never point them at the real flight controller — and do not let a matching
firmware banner talk you into it, since the aircraft runs the same 4.6.3 and the file
would load without a single complaint (see the warning under *Files*).

- **MAVProxy:** `param load sitl_indoor_463.parm` (the known-good indoor SITL state),
  then `reboot`
- **QGroundControl:** connect to SITL, then Vehicle Setup → Parameters → Tools →
  *Load from file*

Check the firmware version too — a dump only loads cleanly onto the release it was taken
from (see the warning at the top). That check catches the 4.8-dev dumps; it does not
catch loading a SITL file onto the aircraft.

## Reset SITL to firmware defaults (clean slate)

The simplest way to wipe all parameters back to the firmware defaults is to restart
SITL with the wipe flag:

```
sim_vehicle.py -v ArduCopter --console -w
```

(`-w` wipes the simulated EEPROM on startup.) Alternatively, in MAVProxy:
`param set FORMAT_VERSION 0` then `reboot`.

## Configure SITL for the MTF-01P stand-in (optical flow + LiDAR)

Load the three overlays instead of typing parameters one by one — this avoids the
paste/garbling problems of multi-line input in the MAVProxy console. Reboot after each,
and wait for MAVProxy to reconnect before the next. **These go into the simulator only:**
they switch the FC to the simulator's own sensor backends (`RNGFND1_TYPE 100`,
`FLOW_TYPE 10`, `SIM_FLOW_ENABLE 1`) and would blind the real aircraft, which has a
physical MTF-01P on SERIAL5. In the MAVProxy console attached to SITL:

```
param load /Users/danieleamore/Studium/Drohnen-mit-KI/Pi-Code/params/sitl_flow_phaseA.parm
reboot
param load /Users/danieleamore/Studium/Drohnen-mit-KI/Pi-Code/params/sitl_flow_phaseB.parm
reboot
param load /Users/danieleamore/Studium/Drohnen-mit-KI/Pi-Code/params/sitl_gps_off.parm
reboot
```

Why three, and why the reboots:

| Overlay | What it does | Why it needs its own reboot |
|---|---|---|
| `phaseA` | `RNGFND1_TYPE 100`, `FLOW_TYPE`, `SIM_FLOW_ENABLE`, **`SIM_TERRAIN 0`** | the `RNGFND1_*` sub-parameters only come into existence once `RNGFND1_TYPE` is set and the FC reboots |
| `phaseB` | rangefinder limits, EKF sources → optical flow, `EK3_SRC_OPTIONS 0` | the rangefinder backend is only re-read on boot |
| `sitl_gps_off` | `GPS1_TYPE 0`, `ARMING_CHECK` without the GPS bit | GPS is torn down on boot |

**`SIM_TERRAIN 0` is the one that is easy to miss and impossible to debug.** With terrain
enabled, SITL measures the rangefinder against a terrain model anchored at
`SIM_OPOS_ALT` — default **584 m, the altitude of the default SITL home at CMAC** — and
`--custom-location` does *not* change it. Start the simulator in Frankfurt (112 m) and
the vehicle sits ~470 m below the modelled ground, so the rangefinder reports a constant
**0.00 m** with no warning. Optical flow then cannot be scaled into a velocity and the
position estimate drifts away (we measured 366 m of drift while the vehicle stood still),
waypoints are never reached and altitude control oscillates.

We use `RNGFND1_TYPE 100` ("SITL", the simulator's native backend) rather than type 1
("Analog") from ArduPilot's `copter-optflow.parm`. Type 1 emulates a voltage on an ADC
pin which the driver converts back via `RNGFND1_SCALING`; type 100 skips that round trip.
Both read zero when `SIM_TERRAIN` is wrong — that is in fact how we found the real cause,
after first suspecting the driver.

Start SITL at the same coordinates the companion uses as its EKF origin, otherwise
pre-arm fails with *"Check mag field"* (the simulated compass is modelled at the SITL
home position):

```
sim_vehicle.py -v ArduCopter --console --custom-location=50.131196,8.692972,112,0
```

**Verify before flying:** in QGroundControl's MAVLink Inspector, `DISTANCE_SENSOR` must
follow the actual altitude. In a dataflash log, `RFND.Dist` must track `CTUN.Alt`. A
rangefinder reading 0.00 m at every altitude means `SIM_TERRAIN` is still on.

Then run `python main.py --sim` (SITL; `python main.py` with no flag is the real
aircraft). `config.gps_denied = True` is the default.
