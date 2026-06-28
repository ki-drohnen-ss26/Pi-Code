# From Simulation (SITL) to the Real World

The whole point of the architecture is that going from SITL to the real drone changes
**as little as possible**. This page lists exactly what changes, what must be
verified on hardware, and what stays the same. See [ARCHITECTURE.md](ARCHITECTURE.md)
for how the components fit together.

## The one thing the code changes

In `main.py`:

```python
# SITL (Mac)
config = Config.sitl()
# Real Pi on the flight controller (UART)
config = Config.pi_serial("/dev/serial0", baud=921600)
```

Everything else (drone / mission / failsafe / camera) is identical.

## What stays the same vs what changes

| Area | SITL | Real hardware |
|------|------|---------------|
| Connection | `udpin:127.0.0.1:14550` | `/dev/serial0` @ 921600 (UART) |
| Mission logic | identical | identical |
| Pre-arm | GPS/EKF converge in seconds | real GPS fix / EKF / compass must be healthy |
| Position source | simulated GPS | GPS (outdoor) **or** MTF-01P optical flow + LiDAR (indoor) |
| Camera | Mock / Sim / Timed | RealCamera (AI camera) |
| Drop servo | values just echoed back | real servo — **calibrate PWM, test on bench** |

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

## 4. Drop servo calibration
`config.drop_pwm` / `config.neutral_pwm` are raw PWM values. On real hardware:
- Test on the **bench, no props**: does `neutral_pwm` hold the hatch closed and
  `drop_pwm` open it cleanly?
- Adjust the values to the actual servo + mechanism. `SERVO9_FUNCTION = 0` keeps the
  output MAVLink-controlled (set automatically by `configure_drop_servo()`).

## 5. Link loss: companion vs flight controller
- Our **`LINK_LOSS`** = the **Pi stops receiving heartbeats from the FC** (UART dead /
  unplugged / baud mismatch). The companion notices and stops — but if the link is
  truly dead it cannot command RTL either.
- Therefore configure the **FC's own** GCS failsafe (`FS_GCS_ENABLE = 1`, action
  RTL/Land) so the aircraft recovers on its own. This is different from the RC/radio
  failsafe (transmitter ↔ FC).

## 6. Data rates
Over a serial link telemetry is slower than SITL's local UDP;
`Drone.request_data_streams()` sets a fixed rate so position/battery arrive reliably.

## 7. Sensors (Phase 5)
MTF-01P configured via the CP2102 USB-UART adapter; FC params for the rangefinder +
optical-flow serial protocol. See `../params/README.md` and the project sensor docs.

## Safety checklist (real hardware)
- [ ] First flights **without propellers**.
- [ ] Independent kill switch on the transmitter (mode switch / disarm). The companion
      failsafe does **not** replace it.
- [ ] ArduPilot's own failsafes set: `BATT_LOW_VOLT`, `BATT_FS_LOW_ACT`,
      `FS_GCS_ENABLE`, radio failsafe.
- [ ] Baseline params loaded (`../params/default.parm`) and known-good.
- [ ] Camera **axis mapping** verified on the real camera (§3).
- [ ] Drop servo PWM verified on the bench (§4).
