# Variant: fw480dev-nobaro

**Firmware target: ArduCopter 4.8.0-dev, on an airframe with NO usable barometer.**

This is a copy of the companion-computer code adapted for a flight controller running a
custom **4.8.0-dev** build on a drone whose **barometer is absent or unusable**. The
safety logic is identical to the main Pi-Code (4.6.3); what differs is (a) the FC
parameter *names* changed between 4.6 and 4.7+, and (b) with no barometer the rangefinder
is the EKF's **only** height source, which is the exact configuration that caused the
2026-08-21 crash - so this variant carries an extra pre-arm gate and a strict flight
protocol.

## When to use which

| Use... | ...when the FC runs | ...and the airframe |
| --- | --- | --- |
| **main Pi-Code (4.6.3)** | ArduCopter 4.6.3 | any (uses 4.6 param names: `RTL_ALT`, `RNGFND1_MIN_CM`, `SYSID_MYGCS`) |
| **variant fw480dev-baro** | ArduCopter 4.8.0-dev | has a **working barometer** (baro is the height source) |
| **variant fw480dev-nobaro** (this one) | ArduCopter 4.8.0-dev | has **no usable barometer** (rangefinder is the only height source) |

Firmware and code must match. 4.8.0-dev uses 4.7+ parameter names
(`RTL_ALT_M`, `RNGFND1_MIN`/`RNGFND1_MAX`, `MAV_GCS_SYSID`), and ArduPilot **silently
ignores** a parameter name it does not know - so running the wrong variant loads none of
the safety envelope and gives no error. `drone.log_autopilot_version()` warns on a
major.minor mismatch, but it is your job to pick the right folder. **Never mix param
files across variants, or with the 4.6.3 files in the main repo.**

## Why this variant is different (the 2026-08-21 crash)

The crash happened with `EK3_SRC1_POSZ=2` - the rangefinder as the EKF's only height
source. EKF3 never fused a height while the aircraft sat on the ground, so the vertical
estimate diverged **quadratically** (past -1000 m by the time it armed) while the
rangefinder sensor itself stayed healthy the whole time. The first altitude-controlled
mode (a fence-forced LAND) then commanded full throttle and flew into the ceiling.

On a build **with** a barometer this cannot happen - the baro holds the height. On
**this** build there is no baro to fall back to, so `EK3_SRC1_POSZ=2` is not a mistake to
be fixed, it is the **only** configuration this airframe can have. The danger is real and
unchanged, so the code guards it two ways the main repo does not need:

- **Pre-arm altitude-drift gate** (`failsafe.verify_position_sensors`): the reported EKF
  altitude is watched while the aircraft stands still. If it *moves* on the ground
  (`|trend| > max_ground_alt_drift` = 5 cm/s, or `span > max_ground_alt_span` = 0.5 m),
  the run refuses to arm with `EKF_ALT_DRIFTING`. This is the divergence starting, caught
  on the floor where a refusal is free.
- **Forced takeover start**: on the real profiles the companion never arms or takes off
  itself. A flow-only no-baro EKF may not report a position estimate on the ground, which
  would deadlock `wait_ready_to_arm`. The pilot flies the first metre by hand; the
  companion takes over in the air, where its rangefinder/EKF cross-gate can refuse a
  diverged filter.

## Parameter load order

Load in this exact order, on the flight controller for this airframe:

1. `params/fc_480dev_baseline_translated.parm` - the recovered real-FC baseline,
   translated to 4.8 parameter names.
2. `params/fc_480dev_overrides_nobaro.parm` - the no-baro safety overrides (fence off,
   `EK3_SRC1_POSZ=2`, `ARMING_CHECK` with the baro bit removed, `BATT_LOW_VOLT`,
   `RTL_ALT_M`).
3. **Reboot the FC** (`python setparam.py --reboot`) - several of these only take effect
   after a reboot.
4. **Verify the critical values** - 4.8-dev may have moved further parameter names, and a
   silently-ignored name reads back as its old default, so check them explicitly:

   ```
   python setparam.py --show EK3_SRC1_POSZ FENCE_ENABLE ARMING_CHECK \
       RNGFND1_MIN RNGFND1_MAX RTL_ALT_M FLOW_TYPE RNGFND1_TYPE \
       SERIAL5_PROTOCOL BATT_LOW_VOLT
   ```

   If any of these read as `not present` or an unexpected value, stop - the name has
   moved again and that safety setting is NOT in effect.

## Test-day quickstart

1. Load the params in the order above, reboot, and verify the critical values.
2. `python preflight.py` - read-only. Confirm: barometer shows *not present - EXPECTED*,
   the rangefinder/optical-flow are healthy, and the **altitude-drift line is stable**
   while the aircraft stands still. A running-away altitude here means **DO NOT FLY**.
3. **Bench hand-lift test**: lift the airframe by hand and confirm the EKF altitude
   *follows* the real lift before any flight. This proves the EKF actually fuses the
   rangefinder height - the thing that was missing in the crash.
4. Start flights **only via `--takeover`**: `python main.py --takeover [--milestone N]`.
   The pilot flies the first metre; the companion inherits a stable aircraft.
5. Keep flights short and staged (`--milestone 1..5`).

## WARNINGS

- **Dev-branch firmware.** 4.8.0-dev has had no release QA. Parameter names and defaults
  can change between builds; re-verify the critical values (step 4 above) after any
  firmware update.
- **Switching firmware wipes parameters.** Flashing a different build resets the FC to
  defaults - reload the params in order and reboot before trusting anything.
- **Never mix param files.** Do not load this variant's params on a baro airframe, the
  baro variant's params here, or any 4.6.3 files from the main repo onto a 4.8 FC. Wrong
  names are silently ignored and the safety envelope silently does not apply.
- **No barometer means no backstop.** The rangefinder is the only height source. The
  pre-arm drift gate and the bench hand-lift test are the only things standing between a
  diverging EKF and a full-throttle climb. Do not skip them, and do not fly if either
  fails.
