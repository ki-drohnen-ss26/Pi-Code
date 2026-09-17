# From Simulation (SITL) to the Real World

The whole point of the architecture is that going from SITL to the real drone changes
**as little as possible**. This page lists exactly what changes, what must be
verified on hardware, and what stays the same. See [ARCHITECTURE.md](ARCHITECTURE.md)
for how the components fit together.

## The one thing that changes: a command-line flag

Nothing in the source changes. `main.py` picks a preset:

```bash
python main.py            # the real aircraft, via mavlink-router  ← the DEFAULT
python main.py --sim      # SITL on the Mac
```

> **The real aircraft is the default; simulation is opt-in.** This is a safety property,
> not a preference. The two possible mistakes are not equally bad:
>
> | Mistake | What happens |
> |---|---|
> | Real aircraft runs **simulation** values | **Silent and dangerous.** `battery_min_voltage` 10.8 V is *below* the 4S Li-Ion's 11.2 V empty voltage → the low-battery **voltage** abort can never fire. Only the 20 % remaining-capacity arm of `battery_critical()` would be left, and that depends on the FC's current sensor and `BATT_CAPACITY` being right (it is skipped entirely when the FC reports `-1`). The drop is commanded on an FC output with no servo. `SimCamera` reports a target that is not there, so the aircraft flies to an empty spot and drops. |
> | Simulation runs **real** values | **Immediate and harmless.** `PiServo` raises "needs gpiozero (Raspberry Pi only)" before takeoff. |
>
> So the dangerous direction needs the flag. `main.py` also logs the active profile on
> every start, with the simulation banner deliberately loud — seeing it next to an armed
> aircraft means stop.

> **The Pi link is UDP, not serial** — this surprises people. `mavlink-router` runs as a
> systemd service on the Pi, owns `/dev/serial0` and forwards the FC stream to
> `127.0.0.1:14550`. Two processes cannot share a UART, so the script binds to the same
> UDP endpoint it uses against SITL. `Config.pi_serial()` (`python main.py --pi-serial`)
> exists only for a setup **without** a router.

Everything else (drone / mission / failsafe / camera) is identical. The **dataclass
defaults in `config.py` are the flight configuration** — reading that file tells you how
the real aircraft is set up. `Config.pi()` therefore overrides *nothing* but the
endpoint, and every deviation from reality lives in exactly one place, `Config.sitl()`:

| | `Config.sitl()` (the deviation) | `Config.pi()` = the defaults |
|---|---|---|
| `release_mechanism` | `"fc"` — servo on an FC output (there is no GPIO on a Mac), and SITL echoes the value back so the drop can be verified | `"pi"` — servo on a Pi GPIO pin |
| `battery_min_voltage` | `10.8 V` — matches SITL's simulated ~12.6 V pack; the real threshold would abort on the first reading | `12.8 V` — our 4S Li-Ion (16.4 V full, 11.2 V empty) |
| `camera_source` | `"auto"` — resolves to `SimCamera`/`MockCamera` | `"timed"` — honest camera-less default; `"real"` once an `.rpk` is on board |

## What stays the same vs what changes

| Area | SITL | Real hardware |
|------|------|---------------|
| Connection | `udpin:127.0.0.1:14550` | `udpin:127.0.0.1:14550` (via mavlink-router) |
| Mission logic | identical | identical |
| Pre-arm | GPS/EKF converge in seconds | Flight set runs `ARMING_CHECK = 41350` (was `0` on the crash-day FC); the companion's `wait_ready_to_arm()` is an added gate (§2) |
| Position source | simulated GPS | GPS (outdoor) **or** MTF-01P optical flow + LiDAR (indoor) |
| Camera | Mock / Sim (`Config.sitl()` sets `"auto"`) | `TimedCamera` today (`camera_source="timed"` — no detection at all); `RealCamera` (IMX500, model on the sensor NPU, §3a) once an `.rpk` is aboard |
| Drop servo | `FcServo` (FC output, value echoed back) | `PiServo` (servo on Pi GPIO) — **calibrate PWM, test on bench** |

## 1. Physical connection
Pi UART (GPIO TX/RX) ↔ FC TELEM port: **TX↔RX crossed, common GND**. Or a USB-UART
adapter to the FC.
- **FC** (once, via a ground station): `SERIALx_PROTOCOL = 2` (MAVLink2),
  `SERIALx_BAUD` matching the Pi (e.g. `921` = 921600).
- **Pi:** enable the UART in `raspi-config`, disable the serial login console on that
  port so `/dev/serial0` is free.

## 2. Pre-arm & EKF (position estimate)
- **Outdoor / GPS:** `wait_ready_to_arm()` waits for `EKF_POS_HORIZ_ABS` (absolute,
  from GPS). The crash-day FC read `ARMING_CHECK = 0` — all pre-arm checks disabled, so
  the autopilot would arm with an unhealthy EKF, an unhealthy compass or no position
  estimate at all, leaving only the companion's `wait_ready_to_arm()` (a single
  `EKF_STATUS_REPORT` bit) as a gate. The published flight set now runs
  **`ARMING_CHECK = 41350`** (Baro + Compass + Board voltage + Battery + System +
  RangeFinder — the deliberate 2026-08-24 team choice; the fuller `786390` mask that also
  restores the INS/RC checks was recommended but declined for now, recorded in
  project-docs). Change it in Mission Planner / via the flight set — the companion no
  longer writes it (§5a) — and do **not** load `../params/sitl_flight_v2.parm` (the SITL
  mirror) onto the flight controller, it is a SITL file (see `../params/README.md`).
  `arm()` retries with pauses instead of giving up or blocking.
- **Indoor / no GPS (Phase 2, implemented):** `config.gps_denied = True` makes
  `wait_ready_to_arm(require_abs=False)` wait for `EKF_POS_HORIZ_REL` (relative, from
  optical flow), and navigation uses `goto_local()` (local NED) instead of `goto()`
  (lat/lon). On the real drone this runs on the MTF-01P.

  **Know the on-ground deadlock:** a flow-only vehicle may never set
  `EKF_POS_HORIZ_REL` while it sits on the floor — optical flow needs a height above
  ground before the filter trusts it, and there is no height without a takeoff. On
  the real aircraft `wait_ready_to_arm(require_abs=False)` can therefore wait forever
  (exactly what happened on the 2026-08-20 test day). The designed escape is
  `--takeover` (`mission._wait_for_pilot()`): the pilot flies the first metre by
  hand and the companion takes over in the air — gated on the **rangefinder** height
  and on EKF/rangefinder agreement, not on the EKF altitude alone (§5c explains why).

  **To validate the indoor path in SITL, load the generated mirror of the flight set.**
  `../params/sitl_flight_v2.parm` is derived from the published `flight_v2.param` by
  `generate_sitl_flight_params.py`, so SITL tests the SAME parameters we fly: the
  behavioural ones are mirrored 1:1 while the physical ones (real board mounting,
  `INS_`/`COMPASS_` calibrations, the MTF-01P MAVLink backends, serial wiring,
  `MOT_`/`ATC_`/`PSC_` frame tuning) are dropped and replaced by SITL's own — that is why
  the flight set cannot be loaded wholesale. The mirror also carries the SITL sensor
  backends on, the EKF sources on optical flow + rangefinder, GPS off (`GPS1_TYPE 0`) and
  `SIM_TERRAIN 0`, so one file covers what used to take three hand-loaded phase overlays.
  Still start the simulator at the coordinates the companion uses as its EKF origin
  (`--custom-location=50.131196,8.692972,112,0`), otherwise pre-arm fails with *"Check mag
  field"*. Load it **twice** with a reboot between, on **ArduCopter 4.6.3** (the release our
  flight controller runs):

  ```
  param load .../params/sitl_flight_v2.parm
  reboot
  param load .../params/sitl_flight_v2.parm
  reboot
  ```

  The double load is not optional: the file is an alphabetically-sorted dump, and the
  `RNGFND1_*` sub-parameters only exist after `RNGFND1_TYPE` is set and the FC reboots, so a
  single pass leaves `RNGFND1_MIN_CM`/`MAX_CM`/`GNDCLEAR` at firmware defaults and the
  rangefinder reads 0.00 m on the ground (it aborted a mission this way on 2026-08-24). The
  second pass — with the backend now present — fills them. Verify after the second reboot
  with `param show RNGFND1_MIN_CM RNGFND1_MAX_CM RNGFND1_GNDCLEAR EK3_SRC1_POSZ FENCE_ENABLE
  WPNAV_SPEED ARMING_CHECK` → expected `0 / 800 / 10 / 2 / 0 / 100 / 41350`. The mandated
  rangefinder height source (`EK3_SRC1_POSZ 2`) is already in the mirror, so there is no
  longer a separate mandate overlay to top it with.

  **Real hardware (no GPS reception) — handled in Phase 3:** indoors there is no GPS to
  set the origin/home, so the companion does it itself:
  - **EKF origin without GPS** — set `config.set_origin_on_start = True` (+ `origin_lat`/
    `origin_lon`/`origin_alt`). `_idle` then calls `Drone.set_origin()` →
    `SET_GPS_GLOBAL_ORIGIN` before arming, so `LOCAL_POSITION_NED`, home and
    `goto_local()` have a reference. Our SITL setup now runs GPS-off too (the mirror
    `../params/sitl_flight_v2.parm` sets `GPS1_TYPE 0`), so this path is exercised there as
    well; `set_origin()` reads the origin back and warns if the autopilot kept a different
    one. Set
    `origin_lat`/`origin_lon` to the **real hall** coordinate
    so the magnetic declination matches. For SITL you **must** launch the sim at the same
    spot (`sim_vehicle.py ... --custom-location=lat,lon,alt,0`) — the simulated compass is
    modelled at the SITL home, so a mismatch is not a small yaw
    offset but a hard pre-arm block: *"PreArm: Check mag field (z diff:976>200)"*. The
    976 mGauss is exactly the difference between the northern and southern hemisphere
    (Frankfurt vs the default home at CMAC, Canberra). Also set `SIM_TERRAIN 0` when you
    use `--custom-location` — see `../params/README.md`.
  - **Geofence** — **off** in the published flight set (`FENCE_ENABLE 0`) since the
    2026-08-21 incident: an indoor-sized barometric altitude fence breaches on the
    downwash spike of every takeoff, and `FENCE_ACTION` turns that into an uncommanded
    mode change (§5c). If you ever want one, size the altitude fence well above the
    spike (> 8 m from our logs) and set it **in Mission Planner** — the companion no
    longer writes fence parameters. At startup the companion still checks for a stale
    fence left enabled by an earlier run (`verify_fence_disabled()`); per the
    2026-08-24 ownership decision it never writes `FENCE_ENABLE=0` to clear it; it
    **refuses to fly** (`UNEXPECTED_FENCE_ENABLED`) and tells you to disable it in
    Mission Planner (§5a, §5c).
  - **Camera-less flight test** — set `config.camera_source = "timed"` to fly the search
    pattern and drop after `timed_camera_after_s`, with no AI camera (Phase 3).

  The flow *navigation* is GPS-independent and already validated.

## 3. Camera axis mapping ← MUST be calibrated
The camera reports where the target is in the **image** (`dx` = right of centre,
`dy` = up/down). The mission turns that into a **body-frame move** (forward/right) in
`mission._nudge_from_offset()` (used by both OVER_TARGET and APPROACH). Current
assumption (downward-facing camera, image-top = nose):

| Image value | → Body move |
|-------------|-------------|
| `dx` (target to the right) | `right` |
| `dy` (target ahead) | `forward` |

**This depends on how the camera is physically mounted/rotated, and on sign
conventions.** Calibrate once:

1. Hover and place the target clearly to the drone's **right**.
2. Read and log the camera's `dx / dy` (`RealCamera` logs every detection as
   `[CAM] '<label>' p=… -> dx=… dy=… m`).
3. Confirm a positive `dx` makes the drone move **toward** the target (right), not away.
4. If reversed or swapped, set the mounting parameters — **no source edit**:

| Symptom | Fix |
|---|---|
| Drone moves left when the pad is right | `cam_invert_x = True` |
| Drone moves back when the pad is ahead | `cam_invert_y = True` |
| Left/right and forward/back are interchanged | `cam_swap_axes = True` (camera rotated 90° in its mount) |

Getting this wrong means the drone "corrects" **away** from the target. Verify in SITL
with `ScriptedCamera`, then re-verify on the real camera (mounting may differ).

### 3a. `RealCamera` (IMX500) — units and the model format

Two things about the real camera are not obvious and both have bitten us:

**The IMX500 cannot run a `.tflite`.** It executes the network on the *sensor's* NPU and
only loads Sony's packaged **`.rpk`** format. A `.tflite` would have to run on the Pi's
CPU, and a Zero 2 W cannot do that at a useful rate while also serving MAVLink. Produce
the `.rpk` from the trained weights:

```bash
yolo export model=<trained>.pt format=imx data=<dataset>.yaml   # Linux / Docker
imx500-package -i packerOut.zip -o ~/models/pad                 # on the Pi
```
`imx500-package` ships with `imx500-all`, so install that on the Pi first.

**The box format of the `.rpk` is a config value, not a constant.** Two conventions
exist: the IMX500/picamera2 samples emit `(y0, x0, y1, x1)` normalised to 0..1, while
Ultralytics `format=imx` exports have been seen emitting `(x0, y0, x1, y1)` in
input-tensor **pixels**. `config.cam_box_order` selects the order (`"yxyx"` default /
`"xyxy"`); pixel-valued boxes are normalised automatically. `RealCamera` logs the
first raw box next to its decoded form — check that line on the bench (milestone 2)
before trusting any dx/dy: a wrong order shows up as swapped axes, a wrong scale as
corrections that are far too large.

**`RealCamera` returns ground METRES, not image fractions.** `SimCamera` — against which
the whole mission was validated in SITL — reports metres, and `centre_tolerance`,
`approach_gain` and `max_nudge_m` are tuned for metres. A camera natively reports "30 %
of the frame to the right", so `RealCamera` converts once, using
`tan(fraction × FOV/2) × height`. The height comes from the drone; without one it falls
back to `search_altitude` **and warns**, because every correction is then mis-scaled by
the ratio of assumed to real height. `cam_hfov_deg` / `cam_vfov_deg` must match the lens
and the crop actually in use, or the same scaling error appears silently.

> **`camera_source` on the real aircraft.** It is `"timed"` by the dataclass default —
> `Config.pi()` does not override it — and deliberately not `"auto"`. On the drone `"auto"` resolves to `SimCamera`, which **invents** a target at
> `sim_target_north/east` and reports it detected — the aircraft would fly to a spot
> where nothing is and drop there. Set `"real"` only once `camera_model_path` points at
> a real `.rpk`.

> Related open design question: downward (nadir) vs slightly tilted camera. Nadir keeps
> this mapping trivial and is the recommended starting point; a forward tilt helps the
> search but complicates centring. See [ROADMAP.md](ROADMAP.md) Phase 2/4.

## 4. Drop servo: where it is wired + calibration
The release lives behind the `ReleaseMechanism` protocol (`release.py`), selected by
`config.release_mechanism`, so the mission logic is identical either way:

| `release_mechanism` | Implementation | Servo wired to | Signal path |
|---------------------|----------------|----------------|-------------|
| `"fc"` (SITL / tests) | `FcServo` | a flight-controller AUX output (`SERVO9`) | Pi → MAVLink `DO_SET_SERVO` → FC → servo |
| `"pi"` (our indoor build) | `PiServo` | a **Raspberry Pi GPIO pin** (`config.drop_gpio_pin`, BCM, default 18) | Pi generates the PWM directly; the FC is not involved |

**Our build uses `"pi"`** — the servo hangs off the Pi. On the Pi:
- Install the GPIO libs: `sudo apt install python3-gpiozero python3-lgpio`.
  > **Do not follow older guides that tell you to install `pigpio`.** The pigpio daemon
  > was **removed from Debian 13 (trixie)**, which our Raspberry Pi OS is based on —
  > `apt install pigpio` fails with *"has no installation candidate"*, and setting
  > `GPIOZERO_PIN_FACTORY=pigpio` would then break the release outright. Verified on the
  > real Pi: gpiozero already selects **`LGPIOFactory`** by default, which drives the pin
  > through the kernel's GPIO character device. That is the current, supported backend —
  > no daemon, no extra configuration. Check with:
  > ```bash
  > python3 -c "from gpiozero import Device; Device.ensure_pin_factory(); print(type(Device.pin_factory).__name__)"
  > # -> LGPIOFactory
  > ```
- **Power the servo from a separate 5 V BEC, not the Pi's 5 V pin** — stall/inrush
  current can brown out a Pi Zero 2 W. Only the **signal** wire goes to the GPIO pin,
  with a **common ground** between the BEC and the Pi.
- The Pi release is **open-loop** (no servo read-back), so `confirm()` trusts the pulse.
  SITL still verifies the drop via the FC read-back on the `"fc"` path.

Calibration (either path), on the **bench, no props**:
- Does `config.neutral_pwm` hold the hatch closed and `config.drop_pwm` open it cleanly?
  Adjust the raw PWM values (µs) to the actual servo + mechanism.
- `"fc"` only: `SERVO9_FUNCTION = 0` keeps the FC output MAVLink-controlled (set
  automatically by `FcServo.setup()` → `configure_drop_servo()`). Not needed for `"pi"`.

## 5. Link loss: companion vs flight controller
- Our **`LINK_LOSS`** = the **Pi stops receiving heartbeats from the FC** (UART dead /
  unplugged / baud mismatch). The companion notices and stops — but if the link is
  truly dead it cannot command anything either.
- The **FC's own** GCS failsafe (`FS_GCS_ENABLE`, `FS_GCS_TIMEOUT` 5 s) is the
  counterpart: it reacts when the *autopilot* stops hearing from *us*. Two things about
  it are easy to get wrong:
  - **It never fires unless we send heartbeats.** ArduPilot only starts monitoring after
    it has seen at least one HEARTBEAT from the GCS system id (`SYSID_MYGCS`, default
    255); setpoint traffic does not count. `Drone.tick()` sends one every second, so the
    option is now actually usable — before that, setting `FS_GCS_ENABLE = 1` was silently
    dead.
  - **Its action must not be RTL indoors.** Values are
    `0:Disabled, 1:RTL, 3:SmartRTL or RTL, 4:SmartRTL or Land, 5:Land`. Use `5` (Land)
    in a hall, or leave it `0` and rely on the pilot — but then write down that a dead
    Pi has no automatic rescue.

## 5c. Incident 2026-08-21 — full-power climb into the ceiling

**What happened.** During a MANUAL flight in Stabilize, with no companion script running,
the aircraft came off the ground and then went to full throttle and hit the hall ceiling.

**An earlier version of this section blamed a cable under the rangefinder. That was
wrong**, and the dataflash logs disprove it. The correct chain has three links, and all
three are visible in all five logs from those two days.

**Link 1 — the EKF's altitude was already meaningless, and had been for every flight.**
`EK3_SRC1_POSZ = 2` makes the rangefinder the vertical position source. At rest that
rangefinder reports a constant 0.02 m, and a constant reading carries no absolute height
information: vehicle height and terrain offset become jointly unobservable and drift
together while the filter integrates uncorrected vertical acceleration. `CTUN.Alt`
diverged in **every** log — 14 m, 44 m, 37 m, 20 m, and finally **1147 m** — while
`CTUN.BAlt`, the barometric altitude sitting right beside it, stayed within ±0.4 m the
whole time. The truth was in the log; it was simply not a configured source.

**Link 2 — the barometer spikes on takeoff, and the fence was set below the spike.**
Propeller downwash raises the local pressure, so barometric altitude jumps the moment the
aircraft comes light on its skids. At ~15 % throttle, centimetres off the floor:

| log 1 | log 2 | log 3 | log 4 | log 5 (crash takeoff) |
|---|---|---|---|---|
| `BAlt` 4.05 m | 4.65 m | 5.26 m | 4.73 m | 6.73 m (with `RFND` reading 0.24 m; 14.1 m at the ceiling impact) |

`FENCE_ALT_MAX = 4.0` — written at the time by our own then-existing `setup_geofence()`
(the companion no longer writes it) — sat **inside that noise band**. It breached on every
single takeoff.

**Link 3 — the fence action is a mode change.** `FENCE_ACTION = 2` forced the vehicle out
of the pilot's Stabilize into **LAND**, an altitude-controlled mode. LAND read the EKF
altitude from link 1, concluded it was 1147 m below target and descending at 12.9 m/s,
and saturated `CTUN.ThO` at **1.000** trying to arrest a descent that was not happening.
Impact at 4.8 g.

On the earlier flights the same three links produced the *opposite-looking* symptom: LAND
happened to command a descent, the aircraft auto-disarmed, and re-arming was then refused
with **"Arm: LAND mode not armable"** until the battery was pulled. That was never a
battery failsafe and never a latched EKF error — it was simply a vehicle still sitting in
LAND mode.

**The rangefinder was not at fault.** During the fatal climb it tracked correctly:
0.02 → 0.16 → 0.28 → 0.96 → 3.89 → **4.94 m** and back down, 48 consecutive plausible
samples. The barometer was healthy in all five logs (`Health = 1` throughout). Both
sensors did their job.

**What changed as a result**
- The fence is **off** in the published flight set (`FENCE_ENABLE 0`). A barometric
  altitude fence sized for an indoor hover sits inside its own sensor's noise band, and a
  fence whose action is a *mode change* converts a bad measurement into a manoeuvre nobody
  commanded. If you ever want one, set it in Mission Planner with the altitude limit well
  above the downwash spike (> 8 m from our data).
- The companion **stopped writing FC parameters altogether** (team decision 2026-08-24,
  §5a). The crash fence was a companion-written parameter that outlived its run; the
  durable fix is that the companion no longer owns any FC parameter — Mission Planner and
  the published `../params/flight_v2.param` do, and `preflight.py` verifies them. With the
  fence off, a stale `FENCE_ENABLE=1` found on the FC at startup is no longer cleared by
  the companion: it **refuses to fly** (`UNEXPECTED_FENCE_ENABLED`) and asks the operator
  to disable the fence in Mission Planner. The old write/restore/backup path
  (`setup_safety_envelope()`, `restore_params()`, `logs/fc_params_backup.json`,
  `failsafe.recover_stale_params()`) was **removed entirely on 2026-08-25** — the
  ownership doctrine made it dead code, so it no longer exists even as an opt-in (§5a).
- `preflight.py` reports the EKF altitude drift while the aircraft is disarmed and
  standing still. The divergence is **quadratic** — pure inertial integration of
  accelerometer bias, with no height measurement fused at all: −268 m ninety seconds
  after boot, −1070 m three minutes in, the EKF "climb rate" at −12.6 m/s while the
  aircraft stood motionless. Even its first seconds are visible from the ground;
  twenty seconds of `preflight.py` would have shown it before any of these flights.
- The `--takeover` gate no longer trusts the EKF altitude (which read +1070 m *on the
  floor* at the moment of arming — it would have handed over instantly). It now gates
  on the **rangefinder** and refuses the handover when EKF and rangefinder disagree
  (`EKF_ALT_DIVERGED`).

**What is still open.** `EK3_SRC1_POSZ = 2` — the rangefinder as the EKF's height
source — is **mandated by the assignment** (the barometer as the EKF source is not
permitted), so it stays on the FC by requirement, not by oversight. It is also the
configuration in which the vertical estimate diverged above, so we fly it only under the
**safety protocol** (see *What changed as a result*: preflight drift gate before every
arming, geofence off, the rangefinder-gated `--takeover`, and the in-flight
EKF-vs-rangefinder cross-check) while we investigate why EKF3 never fused a height. The
open investigation: a colleague team flies the **same** sensor with `POSZ = 2`
successfully, so the next step is a full parameter diff against their aircraft — prime
suspect `RNGFND1_GNDCLEAR` (the EKF's expected on-ground reading; ours was the default
10 cm while the sensor sits ~2 cm up, now corrected to 2 in `fc_safe_overrides.parm`),
plus `RNGFND1_MIN_CM` and `EK3_ALT_M_NSE` — weighed against the alternative that their
EKF drifts on the ground too and nobody ever left it standing for minutes.

**New lead (2026-08-24, from the professor's own course document `AI_Drones.pdf`,
§11.2 "Configuration Step C (EK3)"):** the document gives *two* recipes. The
switchable variant (GNSS on SRC1, indoor on SRC2 + aux switch `RCn_OPTION 90`) sets
`EK3_SRC2_POSZ = 2`. The **exclusively-indoor** variant (SRC1) sets POSXY=0, VELXY=5,
VELZ=0, YAW=1 — and **contains no POSZ line at all**, i.e. the height source stays at
its default (baro) there. If the colleague team followed that exclusive-indoor list
verbatim, they fly `POSZ = 1` with the rangefinder doing flow-scaling/terrain — which
would explain their stable ground behaviour completely. **Check `EK3_SRC1_POSZ` first
in their diff, and clarify with the professor which variant the assignment intends.**

The barometer — the reference that was correct throughout — stopped being *detected*
after the crash: stock 4.6.3 halts with **"Config Error: Baro: unable to initialise
driver"** (in that state the FC streams no sensor data at all, so the zeros in
FC_Check.pdf prove nothing about the rangefinder or flow — both were demonstrably
healthy in the same day's logs). That cost us the barometer as an independent altitude
**witness** and stopped stock 4.6.3 from booting at all (it is not, and by the mandate
cannot be, the EKF height source). Two findings point at repairable wiring rather than a
dead baro chip:

1. The crash **tore off the GPS connector** (FC_Check.pdf §1.4: "in der Nähe des
   abgerissenen GPS-Anschlusses").
2. The FlywooF745 has exactly **one I2C bus** (hwdef: "only one I2C bus", PB6/PB7),
   shared by the onboard barometer (BMP280/SPL06/DPS310 at 0x76) and the external
   GPS-module compass. A torn-off connector whose SDA/SCL lines short or hang the bus
   makes the baro probe fail at boot — and on the colleague's custom 4.8.0-dev build
   (baro requirement bypassed) it *also* explains the **"Bad Compass Health"** shown in
   Mission Planner: two symptoms, one bus.

**Zero-cost test:** disconnect the GPS module's I2C wires (or the whole GPS plug),
boot **stock 4.6.3**, and watch the messages. Baro back → the chip is fine, repair the
GPS wiring (or fly indoor without compass) instead of replacing the FC. Baro still
missing → the chip itself died and a new FC/external baro is needed. Note the custom
4.8.0-dev flash also **wiped all parameters to defaults** — after returning to stock,
reload the recovered baseline with the safe overrides (see `../params/README.md`).
See [`ROADMAP.md`](ROADMAP.md) ("Incident 2026-08-21") for the decision tracking.

> **Update 2026-08-24 — barometer recovery RESOLVED, the I2C hypothesis held
> completely.** Bent pins were found in the GPS connector and straightened
> (2026-08-23); "Bad Compass Health" on the custom 4.8.0-dev build disappeared. Stock
> **4.6.3** was then re-flashed and **detects the barometer again**: the chip was never
> dead, only unreachable behind the hung single I2C bus. No new FC needed. This restores
> the barometer as our independent altitude **witness** and lets stock 4.6.3 boot — it
> does **not** change the EKF height source, which is the rangefinder
> (`EK3_SRC1_POSZ = 2`) by assignment. Remaining before flight: reload the recovered
> baseline + safe overrides (which now set `POSZ = 2` and `RNGFND1_GNDCLEAR = 2`,
> `../params/README.md`), recalibrate the compass, run `python preflight.py` and clear
> its ground-drift gate, and do the hand-lift fusion test — then fly only under the
> safety protocol above. Still open: the parameter diff against the colleague team's
> working `POSZ = 2` aircraft (hot suspect `RNGFND1_GNDCLEAR`). The active path is this
> main repo (stock 4.6.3, rangefinder height source under the protocol, barometer as
> witness).

> **Refinement 2026-08-25 — why the EKF fused no height, pinned down in SITL.** The
> three-link causal chain above is unchanged history: the leftover fence
> (`FENCE_ACTION = 2`), an EKF fusing no height on the ground, and `ARMING_CHECK = 0`
> still add up to a full-throttle LAND into the ceiling. What this refines is *only*
> Link 1's sub-cause — **why** no height was fused, which the crash-day analysis left
> open. A clean **2×2 matrix** on a wiped, mirror-loaded SITL (baro vs rangefinder
> `POSZ` × `RNGFND1_MIN_CM` 0 vs 1, each cell after reboot, 60 s of `EKF_STATUS_REPORT`
> on the ground) isolated it to the **`RNGFND1_MIN_CM` validity floor**, *not* the
> `POSZ = 2` source choice: with `MIN_CM = 0` fusion comes up immediately under *either*
> height source; with `MIN_CM = 1` **and** `POSZ = 2` no relative position (nor any
> height) ever initialises. SITL's landed rangefinder reads exactly `0.00 m`, so a
> non-zero `MIN_CM` flags it out-of-range-low, the driver hands the EKF nothing, optical
> flow cannot be scaled, and — with the rangefinder as the sole height source — the
> vertical estimate has nothing to correct it. In SITL this is **provably** the blocker.
> On the **real aircraft** the same mechanism is the **leading hypothesis**, still to be
> confirmed via the colleague parameter diff: the landed reading is `0.02 m`, clearing a
> `0.01 m` (`MIN_CM 1`) floor by a single centimetre. `POSZ = 2` is therefore
> **exonerated as the blocker per se** — it stays **mandated**, and under this one
> deviation the mirror flew a **fully green milestone 1** in SITL (EKF ready on the
> ground, climb to 0.8 m, rangefinder track confirmed, 20 s hover at 0.03 m worst drift,
> LAND). None of this revises the Link 1–3 history above; it names the fusion sub-cause
> that history left incomplete. See `ROADMAP.md` ("Incident 2026-08-21", the 2026-08-25
> breakthrough entry) for the matrix and the next real-aircraft steps.

## 5b. Flyaway: why it happens indoors, and what stops it

Another team reported their aircraft going to full power and flying away. That is not a
random failure — it is the standard end state of a GPS-denied position estimate that has
lost its height reference, and every precondition for it can be present while the
aircraft looks healthy.

**The mechanism.** In `GUIDED` the autopilot flies to a *position target*. Optical flow
measures an **angular** rate; turning it into a velocity needs a height, which comes from
the rangefinder. With no valid height the flow cannot be scaled, the position estimate
drifts, and the controller sees a large and growing error to its target — so it
accelerates to correct it. The aircraft is not malfunctioning; it is chasing an error
that exists only in the filter. `../params/README.md` records **366 m of drift measured
while the vehicle stood still**, from a rangefinder stuck at `0.00 m`.

**Why nothing catches it by default.** `EKF_POS_HORIZ_REL` can come up on a drifting
estimate, so `wait_ready_to_arm()` passes. The crash-day FC ran `ARMING_CHECK = 0` (§2),
which removed the autopilot's own "Need Position Estimate" refusal entirely; the flight
set's `41350` restores several checks but a drifting *relative* estimate can still pass.
And the geofence we can set
indoors is **altitude-only** (`FENCE_TYPE = 1`) — it guards the ceiling, not the walls.

**What the companion now does about it:**

| Guard | Where | What it catches |
|---|---|---|
| `verify_position_sensors()` | before arming | The rangefinder or the flow produce **no messages at all**. Note it does **not** abort on a reading of `0.00 m`: on the floor a healthy sensor reads zero too, so the value proves nothing there. |
| `verify_rangefinder_tracks_altitude()` | right after the climb | The rangefinder does not follow the height — at 1 m it still reads `0.00`. **This is the check that catches the flyaway precondition**, and it runs at takeoff altitude where an abort is a short descent instead of after a whole search pattern's worth of drift. |
| `position_implausible()` | every failsafe check in flight | The reported position leaves `max_position_radius_m` (15 m). This is the software stand-in for the horizontal fence we cannot set. Whether the aircraft is running away or the estimate is, the answer is the same: land. |
| `altitude_implausible()` | every failsafe check in flight | The EKF altitude and the raw rangefinder disagree by more than `alt_disagree_max_m` (2 m) for `alt_disagree_samples` (3) consecutive checks — the exact signature of the 2026-08-21 crash (EKF −1070 m, rangefinder 0.02 m). The sensor check above proves the SENSOR responds; this one watches the ESTIMATE for the rest of the flight. |
| `WPNAV_SPEED` | flight set (`flight_v2.param`) | Caps horizontal speed at 100 cm/s. The firmware default is **1000 cm/s** — a hall crossed in under a second, and the difference between a drift you can take over from and one you cannot. Now owned by Mission Planner + the flight set, verified by `preflight.py` (§5a). |

**What they do not replace.** A pilot with a kill switch, and `ARMING_CHECK` restored.
The guards make the failure survivable and diagnosable; they do not make an
unconfigured MTF-01P safe to fly.

## 5a. The safety envelope — now owned by Mission Planner, verified by the companion

**Team decision 2026-08-24 (parameter ownership).** The companion **no longer writes any
FC parameter.** FC parameters have exactly one owner: **Mission Planner plus the
published, versioned flight parameter set** (`../params/flight_v2.param` — the full 1159-
parameter dump copied from the aircraft; a v3 with the `FLTMODE` switch mapping follows).
The companion's job is now to **verify** that set read-only (`preflight.py`), not to
enforce it.

Two reasons, both learned the hard way:

- **Single source of truth.** One place holds the flight configuration. A parameter is
  what Mission Planner shows and what `flight_v2.param` records — not something a
  companion run quietly changed and (maybe) changed back.
- **No surprise overwrites.** The 2026-08-21 crash fence was *precisely* a
  companion-written parameter (`FENCE_ALT_MAX = 4.0`, written by the old
  `setup_geofence()`) that outlived its run and breached on the next takeoff (§5c). A
  companion that never writes cannot leave that kind of trap behind.

**The three limits still matter — they now live in the published flight set**, and
`preflight.py` checks the live FC against it and prints every mismatch:

| Parameter | Flight-set value | Firmware default | Why the default is dangerous indoors |
|---|---|---|---|
| `WPNAV_SPEED_UP` | `50` cm/s | `250` cm/s | overshot a 2 m takeoff by >2 m in SITL |
| `WPNAV_SPEED` | `100` cm/s | `1000` cm/s | a hall crossed in under a second; also the single most effective brake on a flyaway |
| `RTL_ALT` | `200` cm | `1500` cm (15 m) | RTL **climbs** to `RTL_ALT` before returning — into the ceiling |

If the live FC differs from the flight set, the fix is now to **change the FC back in
Mission Planner** (or capture the aircraft and publish a new versioned param file) — not
to let the companion silently patch it.

> **`RTL_ALT` name-versioning still applies to whoever sets it.** It is **centimetres** on
> ArduPilot 4.5/4.6 and was renamed to `RTL_ALT_M` (metres) in 4.7 — our FC runs 4.6.3, so
> `RTL_ALT` in centimetres is correct. `param load` silently skips names the firmware does
> not know, so a value written under the wrong name keeps the 15 m default.

### The old set/restore/backup machinery — removed 2026-08-25
This path **once existed** and was, for a while, kept behind an explicit opt-in flag
(`config.enforce_safety_envelope`, default **False** everywhere). With it on,
`failsafe.setup_safety_envelope()` wrote the three limits, saved each FC value first,
restored every one on exit (`restore_params()`), and mirrored the saved baseline to
`logs/fc_params_backup.json` so a run that died without restoring (battery pull, `kill -9`,
Pi brownout) was cleaned up by the *next* run's `failsafe.recover_stale_params()`. It was
useful for a throwaway SITL setup whose parameters no overlay otherwise carried.

That gap was then closed by the flight set itself: `../params/sitl_flight_v2.parm` mirrors
the envelope/fence limits (`FENCE_ENABLE 0`, `WPNAV_SPEED 100`, the mandated
`EK3_SRC1_POSZ 2`), so a SITL run flies correctly with no companion writes at all. With the
opt-in reduced to dead code, **the whole write/restore/backup machinery was deleted on
2026-08-25** — `setup_safety_envelope()`, `restore_params()`, `recover_stale_params()`,
the `logs/fc_params_backup.json` mirror and the `enforce_safety_envelope` config field are
gone. The flight companion now writes **no FC flight parameter** (its one remaining
PARAM_SET anywhere is `FcServo`'s SITL-only `SERVO9_FUNCTION=0`, §4); on both the real
aircraft and SITL, Mission Planner plus the published flight set own the flight
configuration.

The stale-fence check is the one place the companion still *reacts* to an FC parameter —
and it has never written. `verify_fence_disabled()` (renamed from the old `setup_geofence()`
when its write path was stripped) looks for a `FENCE_ENABLE = 1` this run did not ask for,
and if it finds one it **refuses to fly** (abort reason `UNEXPECTED_FENCE_ENABLED`) with a
message telling you to disable the fence in Mission Planner. Detection stayed; the old
"companion writes `FENCE_ENABLE=0`" remedy became the operator's job (§5c).

## 6. Data rates
Over a serial link telemetry is slower than SITL's local UDP;
`Drone.request_data_streams()` sets a fixed rate so position/battery arrive reliably.

## 7. Sensors (Phase 5)
MTF-01P configured via the CP2102 USB-UART adapter; FC params for the rangefinder +
optical-flow serial protocol. See `../params/README.md` and the project sensor docs.

## 8. What the companion does when things go wrong

The mission never silently keeps flying. Every abort names itself in the log
(`[ABORT] Reason: ...`) and then takes one of three exits:

| Situation | What happens | Why |
|---|---|---|
| Never armed (pre-arm failed, GUIDED refused) | stop, command nothing | the aircraft is on the ground; commanding a flight mode at it is noise, and the old code then logged "landed and disarmed" for a drone that never left the floor |
| **Mode changed** (`MODE_CHANGED_*`) | stop, command nothing | the pilot or an FC failsafe has control — commanding RTL now would **override the human** |
| Anything else while airborne | `RECOVER` → **LAND** | LAND needs no position, no home and no altitude headroom |

`config.recovery_action` switches `RECOVER` between `"land"` (default, indoor) and
`"rtl"` (outdoor). An unhandled exception in the state machine also commands LAND before
propagating — otherwise the process dies and leaves the aircraft armed in GUIDED holding
its last position target, which `GUID_TIMEOUT` does **not** clear (it only applies to
velocity/acceleration/attitude targets, not position ones). It would hover until the
battery ran out.

## 9. Reading a flight log
`Drone.tick()` mirrors every autopilot `STATUSTEXT` into the mission log with its
severity, so pre-arm rejections and failsafe reasons are in the file you already have:

```
[FC] ArduPilot flight software 4.6.3
[FC/WARNING] PreArm: Check mag field (z diff:976>200)
[ARM] Arming failed - see the [FC/...] messages above for the reason
```

Without this, a rejected arming is a bare `result=4` and the reason is only visible in a
MAVProxy console — which you do not have on a flying aircraft. The firmware version is
logged on connect for the same reason: parameter names differ between releases, so every
log should state what it was flown against.

## Safety checklist (real hardware)
- [ ] First flights **without propellers**.
- [ ] Independent kill switch on the transmitter (mode switch / disarm). The companion
      failsafe does **not** replace it.
- [ ] `ARMING_CHECK` sane on the FC. The team **deliberately runs `41350`**
      (2026-08-24 decision) = Baro 2 + Compass 4 + Board voltage 128 + Battery 256 +
      System 8192 + RangeFinder 32768, carried by `../params/flight_v2.param`. The fuller
      `786390` mask (adds the INS/RC checks, every check except the GPS lock) was
      **recommended but declined for now** — kept on record in project-docs, not adopted.
      Do **not** rely on the old crash-day `0` (every pre-arm check off): that is exactly
      what the flight set replaces.
- [ ] ArduPilot's own failsafes set: `BATT_LOW_VOLT`, `BATT_FS_LOW_ACT` (Land, not RTL),
      `FS_GCS_ENABLE` (Land, or 0 — see §5), radio failsafe (`FS_THR_ENABLE = 3`, Land).
- [ ] `RNGFND1_MIN_CM` low enough (**1**, not the 20 cm default) — otherwise the
      MTF-01P never reads "good" near the floor, the EKF gets no terrain height and
      arming fails with *"Need Position Estimate"*.
- [ ] Baseline params captured **from the real FC** and known-good. The files in
      `../params/` are SITL dumps and are **not** loadable onto the flight controller.
- [ ] Camera **axis mapping** verified on the real camera (§3).
- [ ] Drop servo PWM verified on the bench (§4).
- [ ] Firmware version in the mission log matches the FC you flashed (§9).
