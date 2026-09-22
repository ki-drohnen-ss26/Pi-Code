# Flight-controller parameters

A reproducible flight needs a known flight-controller (FC) parameter set. This folder
holds the SITL mirror of the flight set (`sitl_flight_v3.parm`) so a simulated run always
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
flight parameter set** (`flight_v3.param`). The companion computer **no longer writes any
FC parameter** — it **verifies** them read-only in `preflight.py`, which loads
`flight_v3.param` (override with `--expected PATH`) and prints every parameter on the live
FC that differs from the published set. Two reasons: a **single source of truth** for the
flight configuration, and **no surprise overwrites** — the 2026-08-21 crash fence was
precisely a companion-written parameter that outlived its run. So when the live FC differs
from the flight set, the fix is to **change it in Mission Planner** (or capture the
aircraft and publish a new versioned file), never to let the companion patch it. The old
write/restore machinery in `failsafe.py` (`enforce_safety_envelope`, `setup_safety_envelope()`,
`restore_params()`, the `logs/fc_params_backup.json` mirror) was **deleted on 2026-08-25** —
the companion is now structurally unable to change an FC parameter. `flight_v3.param`
(2026-09-22) captured the rangefinder-dropout troubleshooting changes; a `flight_v4`
adding the `FLTMODE` switch mapping is still expected.

## Publishing a new flight set (two commands)

Ownership only works if publishing a new version is **easy**. If it is not, the honest
answer to "the aircraft changed" quietly becomes "switch the check off". So after a
Mission Planner change the whole update is two commands, run from the repo root:

```
python dumpparams.py                            # -> params/flight_v<next>.param
python params/generate_sitl_flight_params.py    # -> the regenerated SITL mirror
```

`dumpparams.py` downloads the live FC's complete parameter list and writes it in Mission
Planner's `NAME,VALUE` format, re-requesting individually any reply the link lost (a
handful of the ~1600 `PARAM_VALUE` messages routinely go missing on a busy link, and a
dump with holes in it would be published as if it were the aircraft's configuration). It
is **read-only on the aircraft**: it requests parameters and writes a local file, it
never sends a `PARAM_SET`. `--sim` captures SITL instead, `--out PATH` writes elsewhere,
`--stdout` prints without writing.

Then **review the diff and commit both files**. A published flight set is a decision, not
a snapshot, and the point of versioning it is that a flight log can be matched to the
exact configuration it was flown under.

**No source edit is needed anywhere.** Both the pre-mission parameter check and
`preflight.py` resolve the published set by VERSION: the highest
`params/flight_v<N>.param` for the aircraft, the highest `params/sitl_flight_v<N>.parm`
for a simulated run (`paramcheck.newest_flight_set()`). `preflight.py`'s hard-coded
`flight_v2.param` default is gone; `--expected PATH` still pins one file, as does
`config.expected_params_path`.

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
| `flight_v3.param` | 4.6.3 | **THE PUBLISHED FLIGHT SET — the single source of truth (2026-09-22).** Full 1161-parameter dump captured via Mission Planner's own "Save to file" (not `dumpparams.py` — the capture happened on a teammate's laptop mid test session) after two live changes made straight to the aircraft during real milestone-2 test flights: `RNGFND1_GNDCLEAR 10 -> 5` and `EK3_RNG_M_NSE 0.5 -> 0.2`, both attempts at the intermittent-rangefinder-dropout problem (see [project-docs → Problems → Rangefinder dropout](https://github.com/ki-drohnen-ss26/project-docs/blob/main/docs/problems/rangefinder-dropout-2026-09-22.md) — **neither fixed it**, the dropout is still open). Everything else is routine auto-learned drift (accel/gyro/compass calibration, `MOT_THST_HOVER`, boot/flight counters) between the same two capture days. `preflight.py` and the pre-mission check verify the live FC against this file (version-resolved automatically, no source change needed). |
| `flight_v2.param` | 4.6.3 | **Superseded by `flight_v3.param` (2026-09-22).** Published 2026-08-24, the first versioned set to **own** the FC configuration after the crash. Carries `ARMING_CHECK 41346`, `BATT_LOW_VOLT 12.4`, the halved `ATC_RAT_PIT`/`ATC_RAT_RLL` tune, `EK3_SRC1_POSZ 2`, but the pre-2026-09-21 `RNGFND1_GNDCLEAR 10`. Kept for history / diffing. |
| `fc_baseline_463_20260821.parm` | 4.6.3 | **REAL FC — forensic/rescue record.** Full baseline of the actual aircraft, recovered from the 2026-08-21 dataflash log (1154 parameters). Contains the CRASH configuration. Superseded as the live configuration by `flight_v3.param`; kept for the crash forensics and the post-wipe rescue path. |
| `fc_safe_overrides.parm` | 4.6.3 | **RECOMMENDATION RECORD (no longer param-loaded wholesale, 2026-08-24).** The corrections proposed by the crash analysis. Several were **declined** by the team: `BATT_LOW_VOLT` (kept at `12.4`, not the suggested `12.8`) and `ARMING_CHECK` (kept at `41346`, the fuller mask declined). `RNGFND1_GNDCLEAR` was **adopted at `5`** (2026-09-21, now carried in `flight_v3.param`): the true mounting is ~2 cm, but Mission Planner refuses anything below 5, the parameter's own minimum, so 5 is the closest achievable value, not a re-measurement — and, per the 2026-09-22 test flights, not the fix either. Apply any further adopted line **individually via Mission Planner** — do **not** load the whole file onto the FC any more; the live flight configuration is `flight_v3.param`. |
| `sitl_flight_v3.parm` | 4.6.3 | **THE SITL MIRROR OF THE FLIGHT SET — the one file for daily SITL use (2026-09-22).** Generated from `flight_v3.param` by `generate_sitl_flight_params.py`: the behavioural parameters mirrored 1:1, the physical ones dropped, the SITL sensor backends bolted on. **Start the simulator with it as a startup defaults file** (`python sitl.py`; see *SITL mirror of the flight set* below). **SITL only — never onto the real FC.** |
| `sitl_flight_v2.parm` | 4.6.3 | **Superseded by `sitl_flight_v3.parm`.** Kept for history / diffing. |
| `generate_sitl_flight_params.py` | — | The generator for `sitl_flight_v3.parm`. Dependency-free; reads `flight_v3.param`, applies the split rule (see below), writes the mirror. Re-run it whenever a new `flight_vN` is published — edit the generator, never the generated `.parm`. |

For SITL use, start the simulator with `sitl_flight_v3.parm`, the generated mirror of the
flight set (`python sitl.py`; see *SITL mirror of the flight set* below). It is the
**only** SITL parameter file: the MTF-01P stand-in (simulated flow + rangefinder),
GPS-off and the rangefinder height source (`EK3_SRC1_POSZ 2`) are all baked in.

> **`EK3_SRC1_POSZ 2` is our own choice, not an assignment requirement.** Earlier versions
> of these notes called the rangefinder height source "mandated", and said the barometer
> was not permitted. That is wrong. Aufgabe 4 requires that the LiDAR and the optical flow
> be USED for altitude hold and position hold; it says nothing about which EKF source
> parameter carries the vertical position, and the professor's course document
> (AI_Drones.pdf §11.2 Step C) sets `POSZ 2` only in its SRC2/aux-switch recipe, never in
> the exclusively-indoor SRC1 one. `EK3_SRC1_POSZ 1` (barometer) therefore remains an
> available option. Nothing else changes: the 2026-08-21 crash history and the safety
> protocol that a `POSZ 2` flight runs under stand exactly as written.

The historical step-by-step overlays
(`sitl_flow_phaseA/B.parm`, `sitl_gps_off.parm`) were superseded by the mirror and
removed on 2026-08-25; their hard-won lessons (`SIM_TERRAIN 0`, the choice of
`RNGFND1_TYPE 100`) live on in *Why the mirror configures the MTF-01P stand-in* below.

> ## ⚠️ Every `sitl_*` file in this folder is a SITL artefact
>
> `sitl_flight_v3.parm` is SITL-only.
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
> 1. load  fc_safe_overrides.parm             # fence off, EK3_SRC1_POSZ 2 (our choice) + RNGFND1_GNDCLEAR 5, checks on, batt failsafe
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
> `RNGFND1_GNDCLEAR 10` against the real ~2 cm mounting (the closest settable value turned
> out to be 5, not 2 - see the update below); and all six `FLTMODE`s at 0.
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
2. load  fc_safe_overrides.parm             # fence off, EK3_SRC1_POSZ 2 (our choice) + RNGFND1_GNDCLEAR 5, checks on
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
> The envelope and fence values now live in `flight_v3.param` and are set via Mission
> Planner; `preflight.py` verifies them read-only. The table below documents the values
> and *why* they matter; "Owned by" is where each is set. The only FC write the companion
> still makes is the SITL drop-servo setup at the bottom of the table.

The envelope/fence values (now owned by the flight set, verified by `preflight.py`):

| Parameter         | Value | Owned by                        | Why                                                              |
|-------------------|-------|---------------------------------|------------------------------------------------------------------|
| `WPNAV_SPEED_UP`  | `50` cm/s | `flight_v3.param` (Mission Planner) | The default 250 cm/s overshot a 2 m takeoff by more than 2 m in SITL. |
| `WPNAV_SPEED`     | `100` cm/s | `flight_v3.param` (Mission Planner) | The default 1000 cm/s crosses a hall in under a second; also the brake on a flyaway. |
| `RTL_ALT`         | `200` cm | `flight_v3.param` (Mission Planner) | In case RTL is triggered from elsewhere. **Centimetres on 4.5/4.6**; renamed to `RTL_ALT_M` (metres) in 4.7. Firmware default is 1500 cm = 15 m. |
| `FENCE_ENABLE`    | `0`   | `flight_v3.param` (Mission Planner) | Fence **off** — a barometric altitude fence indoors caused the 2026-08-21 crash. The companion never sets it; `verify_fence_disabled()` only reads it and **aborts** (`UNEXPECTED_FENCE_ENABLED`) if an unasked-for fence is armed. If you ever want a fence, size the altitude limit well above the downwash spike (> 8 m) and set `FENCE_ACTION` to "Always Land" (`2`) — the firmware default `1` **climbs** to `RTL_ALT` on a breach, into the ceiling — all in Mission Planner. |
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

### The pre-mission parameter check (2026-09-21)

The companion **still writes nothing**. Since 2026-09-21 it does, however, **read the
published set back before every mission**: `FailsafeMonitor.verify_flight_parameters()`
runs in `mission._idle()`, right after the fence check and before arming, reads a curated
subset of parameters off the FC and compares them against the published file. This is the
other half of the ownership decision. Handing the parameters to Mission Planner removed
the surprise-overwrite failure mode that caused the 2026-08-21 crash, but it opened a
quieter one: nothing then noticed when the aircraft in front of you stopped being the
aircraft the code was reasoned about. A parameter changed for one experiment and left
behind is invisible in the air and obvious in a diff, so the mission takes the diff before
it arms. The shared file parser, version resolution and comparison live in `paramcheck.py`,
which `preflight.py` uses too.

**CRITICAL** (a difference aborts the mission with `FC_PARAMS_MISMATCH`), the values the
companion's own safety reasoning depends on:

| Group | Parameters | Why a difference is a refusal |
|---|---|---|
| EKF source set | `EK3_SRC1_POSXY`, `EK3_SRC1_VELXY`, `EK3_SRC1_POSZ`, `EK3_SRC1_VELZ`, `EK3_SRC1_YAW`, `AHRS_EKF_TYPE` | The source set **is** the indoor navigation design. |
| Rangefinder | `RNGFND1_TYPE`, `RNGFND1_MIN_CM`, `RNGFND1_MAX_CM`, `RNGFND1_ORIENT`, `RNGFND1_GNDCLEAR` | No backend means no height; no height means the optical flow cannot be scaled. `MIN_CM` is the validity floor that decides whether a landed reading is discarded at all. |
| Optical flow | `FLOW_TYPE` | No flow backend means no horizontal position indoors. |
| Speed envelope | `WPNAV_SPEED`, `WPNAV_SPEED_UP` | The brake on a flyaway, and the speeds every phase timeout assumes. |
| Failsafe actions | `BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT`, `FS_THR_ENABLE`, `RTL_ALT` | Indoors none of these may climb: RTL goes up to `RTL_ALT` first. |

**INFORMATIONAL** (logged, never blocking): `ARMING_CHECK`, `BATT_MONITOR`,
`BATT_LOW_VOLT`, `BATT_CRT_VOLT`, `FS_EKF_ACTION`, `FS_GCS_ENABLE`, `FLOW_ORIENT_YAW`.
`ARMING_CHECK` sits in this half **deliberately**: the team removed the compass bit on the
aircraft (`41350` to `41346`) while the hall's magnetic problem is open, and the `FLTMODE`
map is still done transmitter-side — `flight_v3.param` turned out to be the
rangefinder-dropout troubleshooting capture instead (2026-09-22), so the switch mapping
is now expected in a `flight_v4`. A gate that cries
wolf over a difference the team is knowingly carrying gets switched off, and then it
guards nothing.

Three more rules are worth knowing:

- **`FENCE_ENABLE` is deliberately NOT in the list**, even though an indoor altitude fence
  caused the 2026-08-21 ceiling crash. `verify_fence_disabled()` runs first and already
  refuses on it, and does a better job: it reads `FENCE_TYPE`/`ALT_MAX`/`ACTION` too, logs
  the fence's shape and names the fault `UNEXPECTED_FENCE_ENABLED` instead of a generic
  mismatch. Two gates giving one fault two names is worse than one gate giving it the
  right one.
- **A parameter the FC does not answer for does not block.** It is reported as "could not
  read" and left unverified: a busy link routinely drops `PARAM_VALUE` replies, and that
  is indistinguishable from a name the firmware does not know. Grounding a flight over a
  lost packet is how a safety gate earns its way into being switched off.
- **A simulated run is compared against the MIRROR, not the flight set**, because the
  mirror deviates from the aircraft on purpose (SITL sensor backends, GPS off, the
  `RNGFND1_MIN_CM` validity floor, the simulated pack's battery voltages). Comparing SITL
  against the flight set would report those intended deviations as faults and teach
  everyone to ignore the check.

`config.param_check` chooses what a CRITICAL difference does: `"abort"` (the default),
`"warn"` (report everything and fly anyway) or `"off"`. `config.expected_params_path` pins
one file; empty (the default) resolves the highest version, as described under *Publishing
a new flight set* above. When the reference file is missing the check downgrades to "no
verification" with a warning, never to "all good".

> **`RNGFND1_GNDCLEAR` is decided at `5` — published in `flight_v3.param` (2026-09-22).**
> The value was `10` before; this section used to say the prose recommended `2` (the
> MTF-01P's true mounting height) with the decision still open. It is decided, but not
> at `2`: on 2026-09-21 the team tried to set the real aircraft to `2` in Mission Planner
> and found the parameter refuses anything below `5`, its own valid range floor. `5` is
> therefore the closest achievable value, not a re-measurement of the mounting, which is
> still physically ~2 cm.
>
> **This did not fix the problem it was chasing.** `RNGFND1_GNDCLEAR 5` and (separately)
> `EK3_RNG_M_NSE 0.2` were both tried, live, during the 2026-09-22 real milestone-2 test
> session, specifically to address an EKF-altitude-runs-away-from-the-real-rangefinder
> pattern seen in an earlier flight that same day. It recurred in most of the flights
> made *after* both changes were live. The dataflash logs show why neither parameter
> could have helped: the rangefinder value itself froze (bit-for-bit identical for 6+
> seconds while the current draw proved the aircraft was genuinely climbing), so there
> was no fresh measurement for either parameter to act on. See
> [project-docs → Problems → Rangefinder dropout](https://github.com/ki-drohnen-ss26/project-docs/blob/main/docs/problems/rangefinder-dropout-2026-09-22.md)
> for the full log evidence and the still-open root cause. (`RNGFND1_GNDCLEAR` was also
> the second suspect in the crash-day divergence, see *What the mirror taught us* below —
> a different mechanism, same symptom class: the EKF trusting a rangefinder reading that
> was not the ground truth.)

## Reset SITL to firmware defaults (clean slate)

The simplest way to wipe all parameters back to the firmware defaults is to restart
SITL with the wipe flag:

```
sim_vehicle.py -v ArduCopter --console -w
```

(`-w` wipes the simulated EEPROM on startup.) Alternatively, in MAVProxy:
`param set FORMAT_VERSION 0` then `reboot`. `python sitl.py` already passes `-w`, so an
everyday simulated run starts from a wiped EEPROM and then loads the flight parameters;
`--no-wipe` keeps the existing storage.

## SITL mirror of the flight set

**Use `sitl_flight_v3.parm` for everyday SITL — it is the one file that makes the
simulator fly the SAME parameters we fly.** It is generated from the published flight set
(`flight_v3.param`) by `generate_sitl_flight_params.py`, so there is no longer a chain of
overlays to keep in sync with what Mission Planner publishes.

**Why not just `param load flight_v3.param` into SITL?** Because a full dump of the real
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
behavioural parameter a future `flight_v4` adds is mirrored automatically.

**Regenerate when a new flight set lands** (e.g. the coming v4 with the `FLTMODE` mapping):

```
/opt/miniconda3/envs/ki_drohnen_pi/bin/python3 params/generate_sitl_flight_params.py --date 2026-08-24
```

(`--date` is optional and only stamps the header; omit it and the header carries a stable
"regenerate: see params/README.md" note — the script never calls `datetime.now()`, so a
re-run produces no spurious diff.)

**Hand the mirror to the simulator as a STARTUP DEFAULTS FILE. `python sitl.py` does
exactly that**, and is the only command you need:

```
python sitl.py                 # ArduCopter SITL, wiped, flight parameters loaded
python sitl.py --speedup 5     # five times faster than real time
python sitl.py --no-wipe       # keep the simulator's existing parameter storage
python sitl.py --print-only    # print the command instead of running it
```

It resolves the ArduPilot checkout (`ARDUPILOT_HOME`, else `../Simulation/ardupilot`, else
`~/ardupilot`) and the highest-numbered mirror, then runs:

```
sim_vehicle.py -v ArduCopter --no-rebuild --console \
  --custom-location=50.13119602511582,8.692972038286195,112.0,0 \
  --add-param-file=<repo>/params/sitl_flight_v3.parm \
  --out=udp:127.0.0.1:14550 --speedup 1 -w
```

Run it from the environment that has `sim_vehicle.py` **and** `mavproxy.py` on PATH (our
`ardupilot` conda env), and run `python main.py --sim` from the Pi-Code environment in a
second terminal. The `-w` wipe earns its place twice: every run starts from the same
parameter state, and it resets the simulated battery, which drains across runs and
otherwise aborts a later mission with a puzzling `LOW_BATTERY`.

**Why ONE boot is now enough.** ArduPilot **holds back** a startup default whose parameter
does not exist yet and applies it the moment the driver creates it. So `RNGFND1_TYPE` and
its `RNGFND1_MIN_CM` / `MAX_CM` / `GNDCLEAR` / `ORIENT` sub-parameters all land in the
**same** boot. That is precisely what a `param load` into a running simulator cannot do:
the file is sorted alphabetically, the sub-parameters are sent *before* `RNGFND1_TYPE`,
and the backend that owns them does not exist yet, so they are silently dropped as
unknown.

> **The two-pass `param load` + `reboot` procedure is superseded, and it was observed to
> FAIL on 2026-09-21.** After the first reboot the `RNGFND1_*` sub-parameters were still
> reported unknown, the second pass changed nothing ("changed 0"), and after the second
> reboot MAVProxy lost the link and never got it back. Do not fall back to it. Starting a
> bare `sim_vehicle.py` instead is the other way to get this wrong: on firmware defaults
> (`RNGFND1_TYPE 0`, `FLOW_TYPE 0`) the mission aborts in IDLE with `NO_RANGEFINDER_DATA`,
> which is the code behaving correctly about a simulator nobody configured.

**Verify after starting** (in the MAVProxy console `sitl.py` hands you):

```
param show RNGFND1_MIN_CM RNGFND1_MAX_CM RNGFND1_GNDCLEAR EK3_SRC1_POSZ FENCE_ENABLE WPNAV_SPEED ARMING_CHECK
```

Expected: `0 / 800 / 10 / 2 / 0 / 100 / 41350`. **`RNGFND1_MIN_CM` is `0` in SITL but `1` on the
aircraft** — the one behavioural-looking value the mirror deliberately deviates on: SITL's
landed rangefinder reads exactly `0.00 m`, so the flight set's `1` (1 cm) floor would flag it
out-of-range-low and block all on-ground EKF fusion; the real sensor reads `0.02 m`, a cm above
its floor, so it keeps `1`. (`sitl.py` starts the simulator at the companion's EKF origin
for you; anywhere else and pre-arm fails with *"Check mag field"*. The `SIM_TERRAIN 0` trap
that pins the rangefinder at a constant 0.00 m is documented in the next section.)

Measured on 2026-09-21 with this one-command start: **1646 parameters** loaded, the boot
banner carrying *"EKF3 IMU0 fusing optical flow"* and *"started relative aiding"*, and the
mission's own pre-arm check reporting *"Flight parameters match sitl_flight_v3.parm"*.

### What the mirror taught us (SITL session, 2026-08-25)

Loading the mirror onto a wiped SITL and flying milestone 2 under our chosen
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
diff, with `RNGFND1_GNDCLEAR` (10 vs 2, the value proposed at the time) a second suspect.
(2026-09-21 update: `2` turned out not to be settable — the adopted value is `5`, the
parameter's own minimum; see the note earlier in this file.)

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

After adapting these, the full milestone-2 mission ran **green** in SITL under `POSZ 2`: EKF
ready on the ground, GUIDED, armed, climb to 0.8 m, rangefinder-track check passed (0.84 m),
20 s hover with **0.03 m** worst horizontal drift, LAND, disarm.

### Experiments — compassless yaw (SITL, 2026-08-25)

Prompted by the hall's magnetic-anomaly problem (`project-docs` → Problems → "Loiter
drifts in the hall"), we tried the obvious escape in SITL: flying **without a compass**.
Overlaying `COMPASS_USE`/`USE2`/`USE3 = 0` and `EK3_SRC1_YAW = 0` on the `flight_v2`
mirror (ArduCopter 4.6.3) **refused arming**: EKF3 optical-flow aiding flapped on and off
in a ~1 s cycle ("started relative aiding" / "stopped aiding"), the position estimate
never stabilised, and pre-arm failed with *"Need Position Estimate"* on all five attempts
— while the identical mirror **with** the compass flew milestone 2 green the same day. On
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
home position). `python sitl.py` puts the origin on the command line for you, together
with the mirror; spelled out, that part of it is:

```
sim_vehicle.py -v ArduCopter --console --custom-location=50.131196,8.692972,112,0
```

**Verify before flying:** in QGroundControl's MAVLink Inspector, `DISTANCE_SENSOR` must
follow the actual altitude. In a dataflash log, `RFND.Dist` must track `CTUN.Alt`. A
rangefinder reading 0.00 m at every altitude means `SIM_TERRAIN` is still on.

Then run `python main.py --sim` (SITL; `python main.py` with no flag is the real
aircraft). `config.gps_denied = True` is the default.
