# From Simulation (SITL) to the Real World

The whole point of the architecture is that going from SITL to the real drone changes
**as little as possible**. This page lists exactly what changes, what must be
verified on hardware, and what stays the same. See [ARCHITECTURE.md](ARCHITECTURE.md)
for how the components fit together.

## The one thing that changes: a command-line flag

Nothing in the source changes. `main.py` picks a preset:

```bash
python main.py            # SITL on the Mac
python main.py --pi       # the real Pi, via mavlink-router
```

> **The Pi link is UDP, not serial** — this surprises people. `mavlink-router` runs as a
> systemd service on the Pi, owns `/dev/serial0` and forwards the FC stream to
> `127.0.0.1:14550`. Two processes cannot share a UART, so the script binds to the same
> UDP endpoint it uses against SITL. `Config.pi_serial()` (`python main.py --pi-serial`)
> exists only for a setup **without** a router.

Everything else (drone / mission / failsafe / camera) is identical. `Config.sitl()` and
`Config.pi()` also carry the two genuine hardware differences, so they stay in one place:

| | `Config.sitl()` | `Config.pi()` |
|---|---|---|
| `release_mechanism` | `"fc"` — servo on an FC output (there is no GPIO on a Mac) | `"pi"` — servo on a Pi GPIO pin |
| `battery_min_voltage` | `10.8 V` — matches SITL's simulated ~12.6 V pack | `12.8 V` — our 4S Li-Ion (16.4 V full, 11.2 V empty) |

## What stays the same vs what changes

| Area | SITL | Real hardware |
|------|------|---------------|
| Connection | `udpin:127.0.0.1:14550` | `udpin:127.0.0.1:14550` (via mavlink-router) |
| Mission logic | identical | identical |
| Pre-arm | GPS/EKF converge in seconds | real GPS fix / EKF / compass must be healthy |
| Position source | simulated GPS | GPS (outdoor) **or** MTF-01P optical flow + LiDAR (indoor) |
| Camera | Mock / Sim / Timed | RealCamera (AI camera) |
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
  from GPS). Pre-arm checks are real now: GPS fix, EKF, compass must be healthy —
  `arm()` retries with pauses instead of giving up or blocking.
- **Indoor / no GPS (Phase 2, implemented):** `config.gps_denied = True` makes
  `wait_ready_to_arm(require_abs=False)` wait for `EKF_POS_HORIZ_REL` (relative, from
  optical flow), and navigation uses `goto_local()` (local NED) instead of `goto()`
  (lat/lon). On the real drone this runs on the MTF-01P.

  **To validate the indoor path in SITL** load the two ready-made overlays (values
  match ArduPilot's own `copter-optflow.parm`), each followed by a reboot — see
  `../params/README.md`:

  ```
  param load .../params/sitl_flow_phaseA.parm   # RNGFND1_TYPE, SIM_FLOW_ENABLE, FLOW_TYPE
  reboot
  param load .../params/sitl_flow_phaseB.parm   # RNGFND1_PIN/SCALING/MIN/MAX, EK3_SRC1_* = flow
  reboot                                         # 2nd reboot REQUIRED (rangefinder reads PIN on boot)
  ```
  Pitfalls we hit: don't override `SIM_SONAR_SCALE` (default 12.1212 matches
  `RNGFND1_SCALING`), and the rangefinder is only detected after the second reboot.

  **GPS stays ON in SITL** as a shortcut: it only seeds the EKF origin/home so the
  geofence and "waiting for home" pre-arm pass. The actual XY position/velocity still
  come from optical flow (`EK3_SRC1_POSXY=0`, `VELXY=5`), so the flow navigation is
  genuinely exercised. Once it flies, capture the working set into
  `../params/gps_denied_sitl.parm`.

  **Real hardware (no GPS reception) — handled in Phase 3:** indoors there is no GPS to
  set the origin/home, so the companion does it itself:
  - **EKF origin without GPS** — set `config.set_origin_on_start = True` (+ `origin_lat`/
    `origin_lon`/`origin_alt`). `_idle` then calls `Drone.set_origin()` →
    `SET_GPS_GLOBAL_ORIGIN` before arming, so `LOCAL_POSITION_NED`, home and
    `goto_local()` have a reference. (Leave it `False` in SITL while GPS is still on — it
    seeds the origin itself.) Set `origin_lat`/`origin_lon` to the **real hall** coordinate
    so the magnetic declination matches; for SITL, launch the sim at the same spot
    (`sim_vehicle.py ... --custom-location=lat,lon,alt,0`) or expect a few-degrees yaw
    offset (the mission still works — the local frame is just rotated).
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
2. Read and log the camera's `dx / dy`.
3. Confirm a positive `dx` makes the drone move **toward** the target (right), not away.
4. If reversed or swapped, flip the sign / swap the axes in `_nudge_from_offset()`.

Getting this wrong means the drone "corrects" **away** from the target. Verify in SITL
with `ScriptedCamera`, then re-verify on the real camera (mounting may differ).

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
- Install the GPIO libs: `pip install gpiozero pigpio`. For jitter-free pulses run the
  pigpio daemon (`sudo apt install pigpio && sudo systemctl enable --now pigpiod`) and
  export `GPIOZERO_PIN_FACTORY=pigpio`; otherwise gpiozero uses software PWM and the
  servo may twitch.
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
