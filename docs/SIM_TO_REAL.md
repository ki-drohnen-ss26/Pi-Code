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
| Camera | Mock / Scripted | RealCamera (AI camera) |
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
- **Indoor / no GPS (Phase 2):** switch to `EKF_POS_HORIZ_REL` (relative, from optical
  flow), configure `EK3_SRCx_*`, run on the MTF-01P. Navigation moves from `goto()`
  (lat/lon) to `goto_local()` (local NED).

## 3. Camera axis mapping ← MUST be calibrated
The camera reports where the target is in the **image** (`dx` = right of centre,
`dy` = up/down). The mission turns that into a **body-frame move** (forward/right) in
`mission._over_target()`. Current assumption (downward-facing camera, image-top = nose):

| Image value | → Body move |
|-------------|-------------|
| `dx` (target to the right) | `right` |
| `dy` (target ahead) | `forward` |

**This depends on how the camera is physically mounted/rotated, and on sign
conventions.** Calibrate once:

1. Hover and place the target clearly to the drone's **right**.
2. Read and log the camera's `dx / dy`.
3. Confirm a positive `dx` makes the drone move **toward** the target (right), not away.
4. If reversed or swapped, flip the sign / swap the axes in `_over_target()`.

Getting this wrong means the drone "corrects" **away** from the target. Verify in SITL
with `ScriptedCamera`, then re-verify on the real camera (mounting may differ).

> Related open design question: downward (nadir) vs slightly tilted camera. Nadir keeps
> this mapping trivial and is the recommended starting point; a forward tilt helps the
> search but complicates centring. See [ROADMAP.md](ROADMAP.md) Phase 2/3.

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

## 7. Sensors (Phase 4)
MTF-01P configured via the CP2102 USB-UART adapter; FC params for the rangefinder +
optical-flow serial protocol. See `params/README.md` and the project sensor docs.

## Safety checklist (real hardware)
- [ ] First flights **without propellers**.
- [ ] Independent kill switch on the transmitter (mode switch / disarm). The companion
      failsafe does **not** replace it.
- [ ] ArduPilot's own failsafes set: `BATT_LOW_VOLT`, `BATT_FS_LOW_ACT`,
      `FS_GCS_ENABLE`, radio failsafe.
- [ ] Baseline params loaded (`params/default.parm`) and known-good.
- [ ] Camera **axis mapping** verified on the real camera (§3).
- [ ] Drop servo PWM verified on the bench (§4).
