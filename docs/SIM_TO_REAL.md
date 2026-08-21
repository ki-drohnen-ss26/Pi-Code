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
| Pre-arm | GPS/EKF converge in seconds | `ARMING_CHECK = 0` measured on our FC — every autopilot check is off, the companion's `wait_ready_to_arm()` is the only gate left (§2) |
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
  from GPS). On our FC `ARMING_CHECK` is currently **0** — all pre-arm checks are
  disabled, so the autopilot arms with an unhealthy EKF, an unhealthy compass or no
  position estimate at all. The only gate left is the companion's
  `wait_ready_to_arm()`, and that is a single `EKF_STATUS_REPORT` bit. Restore
  `ARMING_CHECK = 1` before any flight, or set the value `786390` (every check except
  the GPS lock). Set the value by hand — do **not** load `../params/sitl_gps_off.parm`
  onto the flight controller, it is a SITL file (see `../params/README.md`). `arm()` retries with pauses instead
  of giving up or blocking.
- **Indoor / no GPS (Phase 2, implemented):** `config.gps_denied = True` makes
  `wait_ready_to_arm(require_abs=False)` wait for `EKF_POS_HORIZ_REL` (relative, from
  optical flow), and navigation uses `goto_local()` (local NED) instead of `goto()`
  (lat/lon). On the real drone this runs on the MTF-01P.

  **To validate the indoor path in SITL** load the three ready-made overlays, each
  followed by a reboot — see `../params/README.md` for what each one does and why:

  ```
  param load .../params/sitl_flow_phaseA.parm   # sensors on + SIM_TERRAIN 0
  reboot
  param load .../params/sitl_flow_phaseB.parm   # limits + EKF sources = optical flow
  reboot
  param load .../params/sitl_gps_off.parm       # GPS truly off (Phase 3)
  reboot
  ```
  Start the simulator at the same coordinates the companion uses as its EKF origin
  (`--custom-location=50.131196,8.692972,112,0`), otherwise pre-arm fails with
  *"Check mag field"*. The two pitfalls that cost us the most time are documented in
  `../params/README.md`: `SIM_TERRAIN` must be 0 (otherwise the rangefinder reads a
  constant 0.00 m and optical flow silently cannot be scaled), and parameter names
  differ between 4.6 and 4.7.

  Verified working on **ArduCopter 4.6.3** — the same release our flight controller
  runs. `sitl_indoor_463.parm` is the captured state of that green run — load it to get
  straight back to a known-good simulator.

  **Real hardware (no GPS reception) — handled in Phase 3:** indoors there is no GPS to
  set the origin/home, so the companion does it itself:
  - **EKF origin without GPS** — set `config.set_origin_on_start = True` (+ `origin_lat`/
    `origin_lon`/`origin_alt`). `_idle` then calls `Drone.set_origin()` →
    `SET_GPS_GLOBAL_ORIGIN` before arming, so `LOCAL_POSITION_NED`, home and
    `goto_local()` have a reference. Our SITL setup now runs GPS-off too
    (`sitl_gps_off.parm`), so this path is exercised there as well; `set_origin()` reads
    the origin back and warns if the autopilot kept a different one. Set
    `origin_lat`/`origin_lon` to the **real hall** coordinate
    so the magnetic declination matches. For SITL you **must** launch the sim at the same
    spot (`sim_vehicle.py ... --custom-location=lat,lon,alt,0`) — the simulated compass is
    modelled at the SITL home, so a mismatch is not a small yaw
    offset but a hard pre-arm block: *"PreArm: Check mag field (z diff:976>200)"*. The
    976 mGauss is exactly the difference between the northern and southern hemisphere
    (Frankfurt vs the default home at CMAC, Canberra). Also set `SIM_TERRAIN 0` when you
    use `--custom-location` — see `../params/README.md`.
  - **Geofence** — `setup_geofence()` sets an **altitude-only** fence by default
    (`fence_type = 1`, `fence_alt_max_m`), which needs no horizontal position. Size
    `fence_alt_max_m` to the flight, or set `config.geofence_enable = False` indoors.
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
estimate, so `wait_ready_to_arm()` passes. `ARMING_CHECK = 0` (measured on our FC, §2)
removes the autopilot's own "Need Position Estimate" refusal. And the geofence we can set
indoors is **altitude-only** (`FENCE_TYPE = 1`) — it guards the ceiling, not the walls.

**What the companion now does about it:**

| Guard | Where | What it catches |
|---|---|---|
| `verify_position_sensors()` | before arming | The rangefinder or the flow produce **no messages at all**. Note it does **not** abort on a reading of `0.00 m`: on the floor a healthy sensor reads zero too, so the value proves nothing there. |
| `verify_rangefinder_tracks_altitude()` | right after the climb | The rangefinder does not follow the height — at 1 m it still reads `0.00`. **This is the check that catches the flyaway precondition**, and it runs at takeoff altitude where an abort is a short descent instead of after a whole search pattern's worth of drift. |
| `position_implausible()` | every failsafe check in flight | The reported position leaves `max_position_radius_m` (15 m). This is the software stand-in for the horizontal fence we cannot set. Whether the aircraft is running away or the estimate is, the answer is the same: land. |
| `WPNAV_SPEED` | safety envelope | Caps horizontal speed at `cruise_speed_cms` (100 cm/s). The firmware default is **1000 cm/s** — a hall crossed in under a second, and the difference between a drift you can take over from and one you cannot. |

**What they do not replace.** A pilot with a kill switch, and `ARMING_CHECK` restored.
The guards make the failure survivable and diagnosable; they do not make an
unconfigured MTF-01P safe to fly.

## 5a. The safety envelope the companion enforces
`failsafe.setup_safety_envelope()` writes three FC parameters before every flight,
because the autopilot's defaults are built for open sky and a freshly flashed or wiped
board silently reverts to them:

| Parameter | We set | Default | Why the default is dangerous indoors |
|---|---|---|---|
| `FENCE_ACTION` | `2` (Always Land) | `1` (RTL or Land) | RTL **climbs** to `RTL_ALT` before returning — into the ceiling |
| `RTL_ALT` | `200` cm | `1500` cm (15 m) | see above; set in case RTL is triggered from elsewhere |
| `WPNAV_SPEED_UP` | `50` cm/s | `250` cm/s | overshot a 2 m takeoff by >2 m in SITL and breached a 4 m fence |

> `RTL_ALT` is **centimetres** on ArduPilot 4.5/4.6 and was renamed to `RTL_ALT_M`
> (metres) in 4.7. Setting the wrong one is not an error — the autopilot ignores unknown
> parameters and keeps its 15 m default. `_set_rtl_altitude()` therefore tries both.

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
- [ ] `ARMING_CHECK` restored on the FC — measured as **0**, i.e. every pre-arm check is
      disabled and the autopilot would arm with an unhealthy EKF or compass (§2).
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
