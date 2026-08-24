# Variant: fw480dev-baro (ArduCopter 4.8.0-dev, working barometer)

## What this variant is

A copy of the companion-computer code adapted to run against **ArduCopter 4.8.0-dev**
on an aircraft whose **barometer is healthy**. The flight logic - state machine,
failsafe monitor, flyaway guards, everything - is **identical to the main Pi-Code
(4.6.3)**. The *only* thing that differs is the **flight-controller parameter names**,
which moved on the 4.7+/4.8-dev branch:

| main Pi-Code (4.6.3) | this variant (4.8.0-dev) | unit change |
|----------------------|--------------------------|-------------|
| `RTL_ALT`            | `RTL_ALT_M`              | cm -> m     |
| `RNGFND1_MIN_CM`     | `RNGFND1_MIN`            | cm -> m     |
| `RNGFND1_MAX_CM`     | `RNGFND1_MAX`            | cm -> m     |
| `SYSID_MYGCS`        | `MAV_GCS_SYSID`          | (renamed)   |
| `SYSID_ENFORCE`      | folded into `MAV_OPTIONS`| (dropped)   |

Because the safe height source is a **working barometer**, `EK3_SRC1_POSZ = 1` (baro)
comes straight from the params overrides file and there is nothing else to change: with
a barometer feeding the EKF height, the safety envelope is exactly the 4.6.3 code.
**This variant exists so a test day on 4.8.0-dev needs no source changes at all** - pick
the folder, load the params, fly. Same behaviour, different firmware names.

## When to use this vs. the alternatives

- **Main Pi-Code (4.6.3):** use it whenever the flight controller runs the released
  4.6.3 firmware. That is the primary, QA'd target. Do **not** point this 4.8-dev code
  at a 4.6.3 FC - `RTL_ALT_M` / `RNGFND1_MIN` are unknown there and silently ignored.
- **This variant (fw480dev-baro):** the FC runs 4.8.0-dev **and** its barometer is
  healthy (confirm with `python preflight.py` - the SYS_STATUS barometer line must read
  `ok`).
- **Sibling variant (fw480dev-nobaro):** the FC runs 4.8.0-dev but the barometer is
  **dead/unhealthy**. That variant keeps the extra no-baro safety handling; this one
  does not need it. If the barometer is not `ok`, use the sibling, not this folder.

`drone.log_autopilot_version()` prints a loud warning if the FC does not report a
version starting with `4.8` - that is the automated tripwire for "wrong folder".

## Parameter-load order (exact)

Load into the FC **in this order**, then reboot, then verify:

1. `params/fc_480dev_baseline_translated.parm`   (the full recovered baseline)
2. `params/fc_480dev_overrides_baro.parm`          (the safety overrides - loaded SECOND
   so they win over the baseline)
3. **Reboot the flight controller** (many of these only take effect on reboot).
4. Verify the critical values actually stuck - 4.8-dev is a moving target and may have
   renamed further parameters that then loaded as nothing:

```
python setparam.py --show EK3_SRC1_POSZ FENCE_ENABLE ARMING_CHECK RNGFND1_MIN \
    RNGFND1_MAX RTL_ALT_M FLOW_TYPE RNGFND1_TYPE SERIAL5_PROTOCOL BATT_LOW_VOLT
```

Expected after a correct load: `EK3_SRC1_POSZ=1`, `FENCE_ENABLE=0`,
`ARMING_CHECK=786390`, `RNGFND1_MIN=0.01`, `RNGFND1_MAX=8`, `RTL_ALT_M=2`,
`FLOW_TYPE=5`, `RNGFND1_TYPE=10`, `SERIAL5_PROTOCOL=1`, `BATT_LOW_VOLT=12.8`. Any of
these coming back "not present" or at a default means the name moved again on this
dev build - stop and fix the name before flying, an ignored safety param is invisible
in the air.

## Test-day quickstart

1. `python preflight.py` - confirm firmware starts with `4.8`, barometer `ok`, and the
   height estimate is stable while disarmed.
2. Load the two param files in order, reboot, run the `setparam.py --show` verify above.
3. Rehearse in SITL: `python main.py --sim --milestone 1` (then 2..5).
4. Fly the milestones on the aircraft in order: `python main.py --milestone 1` up to
   `--milestone 5`. Each adds one unknown - do not skip ahead.

## WARNINGS

- **4.8.0-dev is a development branch.** It has had no release QA. Parameter names and
  defaults can move between builds, and ArduPilot **silently ignores unknown parameter
  names** - a param you think you set may simply have vanished. Always run the
  `setparam.py --show` verify after loading, before every flight.
- **Switching firmware wipes parameters.** Reflashing the FC (to or from 4.8.0-dev)
  clears the parameter store. Always reload BOTH param files and re-verify afterwards.
- **Never mix param files across variants.** Do not load this folder's `.parm` files on
  the sibling `fw480dev-nobaro` FC (or vice versa), and never load the main repo's 4.6.3
  `.parm` files here - the cm/m unit differences (`RTL_ALT` 200 vs `RTL_ALT_M` 2) turn a
  correct value into a dangerously wrong one, and the renamed params load as nothing.
