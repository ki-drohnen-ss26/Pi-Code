# Flight-controller parameters

A reproducible flight needs a known flight-controller (FC) parameter set. This folder
holds the SITL mirror of the flight set (`sitl_flight_v2.parm`) so a simulated run always
flies the same parameters we fly, **plus** — since 2026-08-22 — the recovered baseline of
the **real** flight controller:
`fc_baseline_463_20260821.parm` was reconstructed from the crash day's dataflash log
(the FC writes every parameter into its own log at boot) after a custom-firmware flash
wiped the live values. It is the only surviving full dump of the real aircraft's
configuration, including the accel calibration, ESC/servo setup, the MTF-01P serial
config and the Pi companion link. **Never load it without `fc_safe_overrides.parm` on
top** — see *Restoring the real FC after the wipe* below.

## Parameter ownership (team decision, 2026-08-24)

FC parameters have exactly **one owner**: **Mission Planner plus the published, versioned
flight parameter set** (`flight_v2.param`). The companion computer **no longer writes any
FC parameter** — it **verifies** them read-only in `preflight.py`, which loads
`flight_v2.param` (override with `--expected PATH`) and prints every parameter on the live
FC that differs from the published set. Two reasons: a **single source of truth** for the
flight configuration, and **no surprise overwrites** — the 2026-08-21 crash fence was
precisely a companion-written parameter that outlived its run. So when the live FC differs
from the flight set, the fix is to **change it in Mission Planner** (or capture the
aircraft and publish a new versioned file), never to let the companion patch it. The old
write/restore machinery in `failsafe.py` (`enforce_safety_envelope`, `setup_safety_envelope()`,
`restore_params()`, the `logs/fc_params_backup.json` mirror) was **deleted on 2026-08-25** —
the companion is now structurally unable to change an FC parameter. A v3 of the flight set,
adding the `FLTMODE` switch mapping, is expected next.

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
| `flight_v2.param` | 4.6.3 | **THE PUBLISHED FLIGHT SET — the single source of truth (2026-08-24).** Full 1159-parameter dump copied from the aircraft today, and the versioned set that now **owns** the FC configuration (see the ownership note below). Carries the deliberate team values: `ARMING_CHECK 41350`, `BATT_LOW_VOLT 12.4`, the halved `ATC_RAT_PIT`/`ATC_RAT_RLL` tune, `EK3_SRC1_POSZ 2`. `preflight.py` verifies the live FC against this file. **v3 with the `FLTMODE` switch mapping is expected next** (mapping is currently done transmitter-side). Owned by Mission Planner, not written by the companion. |
| `fc_baseline_463_20260821.parm` | 4.6.3 | **REAL FC — forensic/rescue record.** Full baseline of the actual aircraft, recovered from the 2026-08-21 dataflash log (1154 parameters). Contains the CRASH configuration. Superseded as the live configuration by `flight_v2.param`; kept for the crash forensics and the post-wipe rescue path. |
| `fc_safe_overrides.parm` | 4.6.3 | **RECOMMENDATION RECORD (no longer param-loaded wholesale, 2026-08-24).** The corrections proposed by the crash analysis. Several were **declined** by the team: `BATT_LOW_VOLT` (kept at `12.4`, not the suggested `12.8`) and `ARMING_CHECK` (kept at `41350`, the fuller mask declined). `RNGFND1_GNDCLEAR 2` is **still open**. Apply any adopted line **individually via Mission Planner** — do **not** load the whole file onto the FC any more; the live flight configuration is `flight_v2.param`. |
| `sitl_flight_v2.parm` | 4.6.3 | **THE SITL MIRROR OF THE FLIGHT SET — the one file for daily SITL use (2026-08-24).** Generated from `flight_v2.param` by `generate_sitl_flight_params.py`: the behavioural parameters mirrored 1:1, the physical ones dropped, the SITL sensor backends bolted on. **Load it TWICE with a reboot each time** (see *SITL mirror of the flight set* below). Supersedes the step-by-step phase overlays for everyday work (they stay for the incremental sensor bring-up). **SITL only — never onto the real FC.** |
| `generate_sitl_flight_params.py` | — | The generator for `sitl_flight_v2.parm`. Dependency-free; reads `flight_v2.param`, applies the split rule (see below), writes the mirror. Re-run it whenever a new `flight_vN` is published — edit the generator, never the generated `.parm`. |

For SITL use, load `sitl_flight_v2.parm` — the generated mirror of the flight set (see
*SITL mirror of the flight set* below). It is the **only** SITL parameter file: the
MTF-01P stand-in (simulated flow + rangefinder), GPS-off and the mandated rangefinder
height source (`EK3_SRC1_POSZ 2`) are all baked in. The historical step-by-step overlays
(`sitl_flow_phaseA/B.parm`, `sitl_gps_off.parm`) were superseded by the mirror and
removed on 2026-08-25; their hard-won lessons (`SIM_TERRAIN 0`, the choice of
`RNGFND1_TYPE 100`) live on in *Why the mirror configures the MTF-01P stand-in* below.

> ## ⚠️ Every `sitl_*` file in this folder is a SITL artefact
>
> `sitl_flight_v2.parm` is SITL-only.
> **It may never be loaded onto the real flight controller.** (The two
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
> its tuning, sensor backends and serial wiring. The firmware-name check at the top of
> this file only catches a version *mismatch*; nothing protects against loading a 4.6.3
> SITL file onto the 4.6.3 aircraft.
>
> The same warning applies to `mav.parm` in the repo root: despite living outside this
> folder it is a **SITL capture** (`FLOW_TYPE 10`, `RNGFND1_TYPE 100`, `SERIAL5`
> disabled) and must never go onto the aircraft.

## Restoring the real FC after the wipe (2026-08-22; superseded 2026-08-24 — read the update first)

> ### Update 2026-08-24 — the FC has been rebuilt and freshly calibrated by hand: DO NOT load the baseline
>
> The team has meanwhile rebuilt the real FC directly on the aircraft. Today's fresh
> dump (`2026_08_24_params.param`) confirms the sensor/serial/EKF setup now matches the
> docs (`EK3_SRC1_POSZ 2`, `FLOW_TYPE 5`, `RNGFND1` configured, `SERIAL5_OPTIONS 1024`),
> and it carries **fresh calibrations that are better than the pre-crash baseline**: a
> new accel calibration, a real compass calibration **for the first time** (the pre-crash
> compass was never calibrated at all), new initial-tuning values, and
> `MOT_BAT_VOLT_MAX/_MIN 16.4/11.2` correctly matching the 4S Li-Ion.
>
> **From this rebuilt state, do NOT load `fc_baseline_463_20260821.parm` any more.** The
> baseline is byte-faithful to the crash day, so loading it would **overwrite the new
> accel/compass calibration and tuning with the stale pre-crash values** — the
> baseline-first procedure below is obsolete for this FC state. Instead load only the
> overrides, then set up the manual-test modes:
>
> ```
> 1. load  fc_safe_overrides.parm             # fence off, EK3_SRC1_POSZ 2 (mandated) + RNGFND1_GNDCLEAR 2, checks on, batt failsafe
> 2. reboot
> 3. python setparam.py FLTMODE4 2 FLTMODE6 5 # map the modes for the manual tests (4 = AltHold, 6 = Loiter)
> 4. python preflight.py                      # sensor health + EKF-drift verdict
> ```
>
> This is necessary because today's dump still carried unsafe leftovers that the overrides
> file exists to neutralise: `ARMING_CHECK 0` (all pre-arm checks off); an **ENABLED**
> fence in the Mission-Planner-suggested default shape (`FENCE_ENABLE 1`, `FENCE_TYPE 7`,
> `FENCE_ACTION 3` = SmartRTL-or-RTL-or-Land, `FENCE_ALT_MAX 120`) — an
> uncommanded-mode-change trap indoors, the **same class as the crash**, which the
> overrides' `FENCE_ENABLE 0` currently neutralises; `BATT_FS_LOW_ACT 2` (RTL climbs
> toward the ceiling indoors) and a late `BATT_LOW_VOLT 12.4` for a 4S Li-Ion;
> `RNGFND1_GNDCLEAR 10` against the real ~2 cm mounting; and all six `FLTMODE`s at 0.
>
> `fc_baseline_463_20260821.parm` still earns its place for two things: **(a)** the
> **forensic record** of the crash-day aircraft, and **(b)** the **rescue path after a
> future parameter wipe** — in that wiped case (bare firmware defaults, no calibration)
> the original two-step load order below still applies, because then there is no fresh
> calibration to protect.

Flashing the colleague's custom 4.8.0-dev build **reset every parameter to firmware
defaults** (Mission Planner showed `New mission / New rally / New fence`; the battery
monitor read 0 V). After flashing back to **stock ArduCopter 4.6.3** — which requires
the barometer problem to be fixed first, see `../docs/SIM_TO_REAL.md` §5c — restore
the aircraft like this **(only from a genuine wipe — see the 2026-08-24 update above; a
freshly rebuilt FC must NOT be sent the baseline)**, in Mission Planner (CONFIG → Full
Parameter List → Load from file → Write params), or MAVProxy `param load`:

```
1. load  fc_baseline_463_20260821.parm      # the aircraft as it was (incl. calibration)
2. load  fc_safe_overrides.parm             # fence off, EK3_SRC1_POSZ 2 (mandated) + RNGFND1_GNDCLEAR 2, checks on
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

> **Since the 2026-08-24 ownership decision the companion writes NONE of the flight
> parameters, and the write/restore machinery that once could was deleted on 2026-08-25.**
> The envelope and fence values now live in `flight_v2.param` and are set via Mission
> Planner; `preflight.py` verifies them read-only. The table below documents the values
> and *why* they matter; "Owned by" is where each is set. The only FC write the companion
> still makes is the SITL drop-servo setup at the bottom of the table.

The envelope/fence values (now owned by the flight set, verified by `preflight.py`):

| Parameter         | Value | Owned by                        | Why                                                              |
|-------------------|-------|---------------------------------|------------------------------------------------------------------|
| `WPNAV_SPEED_UP`  | `50` cm/s | `flight_v2.param` (Mission Planner) | The default 250 cm/s overshot a 2 m takeoff by more than 2 m in SITL. |
| `WPNAV_SPEED`     | `100` cm/s | `flight_v2.param` (Mission Planner) | The default 1000 cm/s crosses a hall in under a second; also the brake on a flyaway. |
| `RTL_ALT`         | `200` cm | `flight_v2.param` (Mission Planner) | In case RTL is triggered from elsewhere. **Centimetres on 4.5/4.6**; renamed to `RTL_ALT_M` (metres) in 4.7. Firmware default is 1500 cm = 15 m. |
| `FENCE_ENABLE`    | `0`   | `flight_v2.param` (Mission Planner) | Fence **off** — a barometric altitude fence indoors caused the 2026-08-21 crash. The companion never sets it; `verify_fence_disabled()` only reads it and **aborts** (`UNEXPECTED_FENCE_ENABLED`) if an unasked-for fence is armed. If you ever want a fence, size the altitude limit well above the downwash spike (> 8 m) and set `FENCE_ACTION` to "Always Land" (`2`) — the firmware default `1` **climbs** to `RTL_ALT` on a breach, into the ceiling — all in Mission Planner. |
| `SERVO9_FUNCTION` | `0`   | `FcServo.setup()` / `drone.configure_drop_servo()` | "Disabled" = MAVLink/mission-controlled, so `DO_SET_SERVO` works for the drop (`config.drop_servo`). **The one remaining companion FC write, and SITL only** (`release_mechanism="fc"`); the real build uses `"pi"` (servo on a Pi GPIO), where the FC has no drop servo. |

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

## Reset SITL to firmware defaults (clean slate)

The simplest way to wipe all parameters back to the firmware defaults is to restart
SITL with the wipe flag:

```
sim_vehicle.py -v ArduCopter --console -w
```

(`-w` wipes the simulated EEPROM on startup.) Alternatively, in MAVProxy:
`param set FORMAT_VERSION 0` then `reboot`.

## SITL mirror of the flight set

**Use `sitl_flight_v2.parm` for everyday SITL — it is the one file that makes the
simulator fly the SAME parameters we fly.** It is generated from the published flight set
(`flight_v2.param`) by `generate_sitl_flight_params.py`, so there is no longer a chain of
overlays to keep in sync with what Mission Planner publishes.

**Why not just `param load flight_v2.param` into SITL?** Because a full dump of the real
aircraft is loaded with values bound to the physical airframe that would *break* the
simulator rather than configure it. Three concrete ones:

- `AHRS_ORIENTATION 13` — the real board's mounting rotation. SITL's simulated IMU is not
  physically rotated, so replaying this rotates the estimate into attitude garbage.
- the **real `INS_*` / `COMPASS_*` calibrations** — offsets/scales measured against the
  real, imperfect sensors. Loaded onto SITL's already-ideal sensors they *mis*-calibrate
  them.
- `RNGFND1_TYPE 10` / `FLOW_TYPE 5` — **MAVLink hardware backends** (the MTF-01P) that do
  not exist in SITL. The simulator needs its own (`RNGFND1_TYPE 100`, `FLOW_TYPE 10`,
  `SIM_FLOW_ENABLE 1`) plus `SIM_TERRAIN 0`.

**The split rule (a blacklist).** Everything in the flight set is mirrored 1:1 *except* the
physical classes, which are dropped so SITL keeps its own: `AHRS_ORIENTATION`+`AHRS_TRIM_*`,
all `INS_*`, all `COMPASS_*`, `BARO*`, `SERIALn_*`, `BRD_*`, the RC transmitter calibration
(`RCn_MIN/MAX/TRIM/DZ/OPTION`, `RCMAP_*`, `RSSI_*`, `RC_OPTIONS`), `MOT_*`, `ATC_*`, `PSC_*`,
`SR0..4_*`, `STAT_*`, `LOG_*`, `SERVO*`, `FORMAT_VERSION`, the battery sensing pins/multipliers
and the RNGFND1 hardware sub-parameters. (`RC<n>_OPTION` is dropped because option-number
validity is *build-dependent* — the real FC's `RC7_OPTION 182` boot-looped the default SITL
build, found 2026-08-25.) On top, seven values are **forced** to differ in sim — the sensor
backends `RNGFND1_TYPE 100`, `FLOW_TYPE 10`, `GPS1_TYPE 0` (indoor/no-GPS), and four
evidence-backed sim-adaptations: `RNGFND1_MIN_CM 0` (SITL's landed reading is exactly 0.00 m;
the mirrored 1 cm floor blocks all on-ground EKF fusion) and the battery voltages
`BATT_ARM_VOLT 0` / `BATT_LOW_VOLT 10.5` / `BATT_CRT_VOLT 10` (real-pack thresholds vs SITL's
~12.6 V simulated pack; the failsafe **actions** stay mirrored) — and two SIM-side lines are
**appended** (`SIM_FLOW_ENABLE 1`, `SIM_TERRAIN 0`). A blacklist (not a whitelist) means a
behavioural parameter a future `flight_v3` adds is mirrored automatically.

**Regenerate when a new flight set lands** (e.g. the coming v3 with the `FLTMODE` mapping):

```
/opt/miniconda3/envs/ki_drohnen_pi/bin/python3 params/generate_sitl_flight_params.py --date 2026-08-24
```

(`--date` is optional and only stamps the header; omit it and the header carries a stable
"regenerate: see params/README.md" note — the script never calls `datetime.now()`, so a
re-run produces no spurious diff.)

**Load it TWICE, rebooting each time** — this is not optional:

```
param load /Users/danieleamore/Studium/Drohnen-mit-KI/Pi-Code/params/sitl_flight_v2.parm
reboot
param load /Users/danieleamore/Studium/Drohnen-mit-KI/Pi-Code/params/sitl_flight_v2.parm
reboot
```

ArduPilot only creates the `RNGFND1_*` sub-parameters (`MIN_CM`/`MAX_CM`/`GNDCLEAR`/`ORIENT`)
once `RNGFND1_TYPE` is set **and** the FC reboots. The file is sorted alphabetically, so on
pass one those names are sent *before* `RNGFND1_TYPE` and silently dropped as unknown; pass
two — with the backend now present — fills them in.

**Verify after the second reboot:**

```
param show RNGFND1_MIN_CM RNGFND1_MAX_CM RNGFND1_GNDCLEAR EK3_SRC1_POSZ FENCE_ENABLE WPNAV_SPEED ARMING_CHECK
```

Expected: `0 / 800 / 10 / 2 / 0 / 100 / 41350`. **`RNGFND1_MIN_CM` is `0` in SITL but `1` on the
aircraft** — the one behavioural-looking value the mirror deliberately deviates on: SITL's
landed rangefinder reads exactly `0.00 m`, so the flight set's `1` (1 cm) floor would flag it
out-of-range-low and block all on-ground EKF fusion; the real sensor reads `0.02 m`, a cm above
its floor, so it keeps `1`. (Start the simulator at the companion's EKF origin —
`--custom-location=50.131196,8.692972,112,0` — or pre-arm fails with *"Check mag field"*; the
`SIM_TERRAIN 0` trap that pins the rangefinder at a constant 0.00 m is documented in the next
section.)

### What the mirror taught us (SITL session, 2026-08-25)

Loading the mirror onto a wiped SITL and flying milestone 1 under the mandated
`EK3_SRC1_POSZ 2` surfaced four places the sim must deviate from the flight set — and one
clean result that reframed the whole on-ground fusion question.

**The 2×2 matrix (wiped, mirror-loaded SITL; each cell after reboot, 60 s of
`EKF_STATUS_REPORT` on the ground):**

| | `RNGFND1_MIN_CM = 0` | `RNGFND1_MIN_CM = 1` |
|---|---|---|
| `EK3_SRC1_POSZ = 1` (baro) | `POS_HORIZ_REL` immediately | — |
| `EK3_SRC1_POSZ = 2` (rangefinder) | `POS_HORIZ_REL` immediately | **never** (flags `0x0027`, no `PRED_REL`, no `VERT_AGL`) |

The on-ground blocker in SITL is **not** the height source `POSZ 2` — it is the
`RNGFND1_MIN_CM` **validity floor**. SITL's landed rangefinder reads exactly `0.00 m`; with
`MIN_CM 1` the driver flags it out-of-range-low, the EKF receives no range at all, optical
flow cannot be scaled, and no relative position (nor, under `POSZ 2`, any height) ever
initialises. Set `MIN_CM 0` and fusion comes up immediately under either height source. The
real aircraft sits **1 cm above** the same floor (reads `0.02 m` with `MIN_CM 1`), so the same
gate is the leading suspect for the crash-day divergence — to be checked against the colleague
diff, with `RNGFND1_GNDCLEAR` (10 vs 2) a second suspect.

**Two more deviations the mirror generator now bakes in (each with its finding):**

- **`RC<n>_OPTION` excluded.** Option-number validity is *build-dependent*: mirroring the
  real FC's `RC7_OPTION 182` boot-looped the default SITL build with
  *"Config Error: Failed to init: RC7_OPTION: 182"* (same failure class as the post-crash
  baro config error). The generator's RC-calibration rule now drops `RC<n>_OPTION` too.
- **Battery voltages sim-adapted.** `BATT_ARM_VOLT 0`, `BATT_LOW_VOLT 10.5`, `BATT_CRT_VOLT 10`
  in SITL vs the real-pack `12.7 / 12.4 / 12`. Voltage thresholds are pack physics: the real
  values refuse arming (*"Arm: Battery 1 below minimum arming voltage"*) / would land constantly
  against SITL's ~12.6 V simulated pack. The failsafe **actions** (`BATT_FS_*_ACT`) stay
  mirrored.

After adapting these, the full milestone-1 mission ran **green** in SITL under `POSZ 2`: EKF
ready on the ground, GUIDED, armed, climb to 0.8 m, rangefinder-track check passed (0.84 m),
20 s hover with **0.03 m** worst horizontal drift, LAND, disarm.

### Experiments — compassless yaw (SITL, 2026-08-25)

Prompted by the hall's magnetic-anomaly problem (`project-docs` → Problems → "Loiter
drifts in the hall"), we tried the obvious escape in SITL: flying **without a compass**.
Overlaying `COMPASS_USE`/`USE2`/`USE3 = 0` and `EK3_SRC1_YAW = 0` on the `flight_v2`
mirror (ArduCopter 4.6.3) **refused arming**: EKF3 optical-flow aiding flapped on and off
in a ~1 s cycle ("started relative aiding" / "stopped aiding"), the position estimate
never stabilised, and pre-arm failed with *"Need Position Estimate"* on all five attempts
— while the identical mirror **with** the compass flew milestone 1 green the same day. On
this stack (flow + rangefinder, no GPS) the EKF needs a yaw source of bounded uncertainty,
so **the compass stays mandatory** and this overlay is deliberately **kept out of the
mirror**.

## Why the mirror configures the MTF-01P stand-in (optical flow + LiDAR)

The mirror switches the FC to the simulator's own sensor backends (`RNGFND1_TYPE 100`,
`FLOW_TYPE 10`, `SIM_FLOW_ENABLE 1`, `GPS1_TYPE 0`, `SIM_TERRAIN 0`) — **simulator
only**: these would blind the real aircraft, which has a physical MTF-01P on SERIAL5.
Two of those values encode lessons that cost real debugging days:

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
