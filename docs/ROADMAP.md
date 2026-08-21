# Pi-Code Roadmap — Companion-Computer Control

This roadmap describes how we develop the Raspberry Pi (companion computer) control
software so that, at the very end, putting it on the real drone is a low-risk,
mechanical step — **not** a debugging session on a flying aircraft.

## Guiding principle

> Everything that can be developed and tested against **SITL** on the Mac is finished
> there. On the real hardware only the following remains: flash the Pi → clone the
> code → run `python main.py` (no flag = the real aircraft) → wire the Pi to the
> flight controller → run incremental flight tests. No logic debugging on the flying
> object.

The architecture already supports this: the dataclass defaults in `config.py` **are**
the real-aircraft configuration, and apart from the endpoint the simulation deviates in
exactly three named ways — the drop servo (`release_mechanism`), the battery threshold
and the camera source — all confined to `Config.sitl()`. `drone.py` is
hardware-agnostic, `camera.py` is a `Protocol` that several implementations satisfy
(the mock/scripted/simulated cameras, the camera-less `TimedCamera`, and `RealCamera`
for the IMX500), and `mission.py` is a clean state machine. The roadmap builds on
exactly these seams.

## Target scenario (decided)

- **Indoor flight in a hall, WITHOUT GPS.** Position comes from the MicroAir
  **MTF-01P** (optical flow + LiDAR), fused by the ArduPilot EKF. This is the core
  of Task 4 and therefore the critical path.
- **The delivery target (landing pad) location is NOT known in advance.** The drone
  must *search* for it with a flight pattern and *find* it with the AI camera, then
  approach and centre over it before dropping. The companion is the brain (sends flight
  commands), the camera is the sense (detects the target).
- **Navigation style:** companion-driven `GUIDED` — the Pi sends each step (takeoff /
  goto / drop) live over MAVLink. (Current design; kept.)
- **MAVLink transport on the Pi:** `mavlink-router` distributes the FC stream to the
  script + QGroundControl (over Wi-Fi) + a log simultaneously, so we can monitor and
  debug live during flight.

Because the standard SITL world is GPS-based, the GPS mission is used only as a
**logic shakedown** (Phase 1) and is then converted to GPS-denied (Phase 2), which
is the real target.

---

## Phases

### Phase 0 — Foundation & hygiene
**Goal:** make the codebase safe to refactor and self-documenting before we add complexity.
- Remove `__pycache__/` from version control; add a `requirements.txt` for `Pi-Code`
  with a pinned `pymavlink` version.
- **`FakeDrone` + `pytest`:** drive the `mission.py` state machine with a fake drone so
  the full mission logic can be tested on the Mac in under a second, without SITL.
- **Structured logging:** companion-side flight log to a timestamped file (in addition
  to console). Directly reusable for documentation and post-flight analysis.
- Finalize the architecture note (the SITL/Pi split) in the README.

**Done when:** `pytest` is green and a logged dry run prints a full mission timeline.

### Phase 1 — Full mission in standard SITL (GPS) *(Mac)*
**Goal:** validate the whole pipeline end-to-end with the least friction. This is a
deliberate stepping stone, not the final scenario.
- Harden the failsafe: heartbeat / link-loss detection, and define behaviour when
  telemetry is missing (the old `battery_critical()` returned `False` when telemetry was
  missing, so no data silently meant "all good"; `check()` now counts consecutive misses
  and aborts with `NO_TELEMETRY`).
- Implement the **real `OVER_TARGET` correction loop** using
  `SET_POSITION_TARGET_LOCAL_NED` in the body frame, exercised by a *scriptable* mock
  camera that feeds changing `dx/dy`.
- Verify the drop-servo read-back in SITL.
- Save a **baseline FC parameter file** under `params/`.

**Done when:** IDLE→TAKEOFF→ENROUTE→OVER_TARGET→DROP→RTL runs reproducibly in SITL and
the run is fully logged.

### Phase 2 — GPS-denied navigation + target search in SITL *(Mac, Task 4 core)*
**Goal:** the actual indoor scenario, fully in simulation — including finding a target
whose position is not known in advance.

The delivery pad location is unknown, so the state machine grows a search stage:

```
IDLE → TAKEOFF → SEARCH → APPROACH → DROP → RECOVER
                    ↑__________|   (target lost → back to SEARCH)
                          (ABORT on failsafe → RECOVER, i.e. LAND indoors)
```

*Navigation (GPS-denied):*
- Configure EKF sources for non-GPS: `EK3_SRCx_*` to optical flow + rangefinder, GPS
  disabled; run SITL with simulated flow + rangefinder (`SIM_FLOW_ENABLE`, simulated
  `RNGFND`).
- Switch `wait_ready_to_arm()` to the relative position flag (`EKF_POS_HORIZ_REL`).
- Replace global `goto()` with a **`goto_local()`** based on
  `SET_POSITION_TARGET_LOCAL_NED`; mission positions become local NED offsets, not
  lat/lon.
- Validate the indoor **takeoff path** (GUIDED behaves differently without GPS) plus
  **Position Hold / Altitude Hold**.

*Target search & approach:*
- **`SearchPattern`** (`search.py`): generates local-NED waypoints relative to launch.
  - `ExpandingSquare` (**default**): spirals outward; needs only a step size + max
    radius, not the hall dimensions — but size that radius for the hall by hand. The
    configured geofence is altitude-only (`FENCE_TYPE=1`) and will not stop a
    horizontal excursion, and the pattern reaches one `step` beyond `max_radius`.
  - `Lawnmower`: back-and-forth over a configured rectangle (even full coverage).
  - Selected via `config.py` (`search_pattern`).
- **Detection cadence** (config-selected): `stop_and_look` (**default** — fly to a
  waypoint, hover, detect, decide; robust, low motion blur) or `continuous` (poll while
  flying each leg, abort the leg on detection).
- **`APPROACH`** state: visual servoing — turn the camera's `dx/dy` into body-frame
  nudges until centred, then **`DROP`** (the indoor path does not use `OVER_TARGET`;
  that is the GPS path's fine-centring state). On sustained target loss → back to SEARCH.
- **`SimCamera`** (`camera.py`, satisfies the existing `Camera` protocol): models a
  target at a known local NED position and "detects" it once the drone is within a
  field-of-view footprint. This lets the whole search → approach → drop logic be
  validated in SITL/`FakeDrone` **before** the real camera exists. The
  `{detected, dx, dy, distance}` contract is unchanged, so Phase 4 only swaps the
  detector.

**Done when:** the full indoor delivery mission flies in SITL using optical flow + LiDAR
for position, with no GPS, and finds the target by search before dropping. Both search
patterns and both detection cadences are exercised via config.

### Phase 3 — GPS-denied for real hardware: EKF origin, fence, camera-less search *(SITL → real)*
**Goal:** make the indoor (no-GPS) path work on the real drone, and let the search
pattern + drop be flight-tested *before* the AI camera is ready.

In SITL we leave GPS on as a shortcut to seed the EKF origin/home. Indoors there is no
GPS, so the companion must establish the reference itself:
- **`Drone.set_origin()`** — send `SET_GPS_GLOBAL_ORIGIN` (a chosen reference lat/lon) so
  the EKF origin/home, `LOCAL_POSITION_NED` and `goto_local()` work without GPS.
- **Geofence indoors** — the GPS-style fence needs a position; use an altitude-only fence
  or disable it indoors (`config.geofence_enable`). Decide and wire it.
- **`TimedCamera`** (config-selectable camera source) — "finds" the target after a set
  time (centred), so a real flight can exercise the search-pattern flight + the drop
  mechanism with **no AI camera**. Same `Camera` interface, so the mission is untouched.

**Done when:** in SITL with GPS truly off (`GPS1_TYPE 0` + `set_origin`) the indoor
mission arms, flies the search pattern and drops; and `TimedCamera` lets the same run
complete with no camera. This unlocks the first camera-less real indoor flight (flown in
the flight-test phase).

### Phase 4 — Real AI camera *(Pi, developed in parallel)*
**Goal:** replace the mock camera without touching mission logic.
- Implement `RealCamera` (IMX500 / YOLO) that satisfies the existing `Camera` protocol:
  turn a detection bounding box into `dx/dy/distance`. ☑ *implemented*
- Test standalone on the Pi (no flight): point at the target object and print offsets. ☐

**Done when:** selecting `config.camera_source = "real"` is the only change and the
mission code is untouched.

*Status:* `RealCamera` is written and wired in (`camera_source="real"`), the IMX500 is
detected on the Pi (`imx500 [4056x3040]`), and the mounting calibration is exposed as
`cam_swap_axes` / `cam_invert_x` / `cam_invert_y` so it needs no source edit between
test flights. **Blocked on the model:** the trained pad detector exists only as
`pad_320_int8.tflite` (YOLO11, 320 px, int8), and the IMX500 loads **only** Sony's
`.rpk` format — a `.tflite` would have to run on the Pi's CPU, which a Zero 2 W cannot
sustain alongside MAVLink. A re-export (`yolo export … format=imx` → `imx500-package`)
is pending. Also still open: `imx500-all` is not installed on the Pi (needs internet),
and the axis calibration itself must be flown.

### Phase 5 — Pi provisioning & hardware-in-the-loop prep
**Goal:** a reproducible Pi image and a configured flight controller.
- Configure the MTF-01P on the bench via the CP2102 USB-UART adapter (sensor doc).
- Pi OS image: enable UART, disable the serial login console, install pymavlink, clone
  the code, set up a **systemd autostart service**, and **`mavlink-router`** (FC →
  script + QGC + log).
- FC parameters: assign the Pi's TELEM port (`SERIALx_PROTOCOL=2`, matching baud) and
  the MTF-01P (rangefinder + optical-flow serial protocol). Extend the `params/` file.

**Done when:** the Pi boots, routes MAVLink, and the FC is parameterised for flow +
LiDAR + companion link.

### Phase 6 — Bench integration *(no propellers)*
**Goal:** prove the real link and sensors before anything spins.
- Pi ↔ FC: heartbeat, `--tele` telemetry, mode changes, servo drop, arm/disarm.
- MTF-01P live: `RNGFND` distance and `OPTFLOW_QUALITY` healthy; move the airframe by
  hand and confirm the EKF is actually using the flow for position.

**Done when:** every companion action works against the real FC on the bench, props off.

### Phase 7 — Flight tests *(incremental, props on, in the hall)*
**Goal:** earn trust step by step. Each stage adds exactly ONE unknown, selected from
the command line with `--milestone N` so no source is edited between flights.

1. Manual `AltHold` → confirms LiDAR altitude hold.
2. Manual `PosHold` → confirms optical-flow position hold.
3. `--milestone 1` — companion hover at 1 m. The logged **drift** is the result, not the
   fact that it hovered: a broken flow setup still flies, it just walks away.
4. `--milestone 2` — same flight with the detector running, logging only. Verify the
   **sign** of `dx`/`dy` here (pad to the right → positive `dx`), on the ground first.
5. `--milestone 3` — the search pattern, no detector. Ends in `TARGET_NOT_FOUND`; that
   is the pass condition. Note the spiral reaches `search_max_radius_m` **plus one**
   `search_step_m` and nothing bounds it horizontally.
6. `--milestone 4` — search, detect and centre with nothing able to fall out.
7. `--milestone 5` — the full indoor delivery.

Flyaway guards run throughout (`SIM_TO_REAL.md` §5b): the rangefinder must track altitude
after the climb, the reported position must stay inside `max_position_radius_m`, and
`WPNAV_SPEED` caps horizontal speed at 1 m/s instead of the firmware's 10 m/s.

**Blocker for all of it:** `ARMING_CHECK = 0` on the FC must be restored first.
Every test is logged and documented (what was tested, parameters, outcome).

### Phase 8 — Documentation & deliverables *(continuous, finalised here)*
- Fill the mkdocs stub pages, write the tutorials, keep the project journal, complete
  `results/limitations`, and prepare the poster + live demo.

---

### Phase 3b — Diagnostics & safety hardening *(done)*
**Goal:** never again debug a flight from a log that does not say what happened, and
never let a companion-side problem turn into a crash.

Driven entirely by things that actually went wrong in SITL:

- **`STATUSTEXT` into the mission log.** A rejected arming used to be a bare `result=4`;
  the reason (`PreArm: Check mag field`) was only visible in a MAVProxy console we do
  not have in flight. `Drone.tick()` now mirrors every autopilot message with its
  severity, and logs the firmware version on connect.
- **Companion heartbeat.** Without one the FC's `FS_GCS_*` failsafe can never fire —
  ArduPilot only starts monitoring after it has seen a heartbeat from `SYSID_MYGCS`.
- **Verified instead of assumed.** `set_origin()` reads the origin back (ArduPilot drops
  the message silently when one is already set — a "green" Phase 3 run had in fact been
  flying on the simulator's origin), and `takeoff()` checks the altitude is *held*, not
  just crossed (a 2 m takeoff sailed to 4.5 m and breached the fence).
- **Mode monitoring.** If the FC leaves GUIDED — pilot, fence breach, EKF failsafe — the
  mission stops instead of sending waypoints into the void or, worse, commanding RTL
  over a human who has just taken control.
- **LAND instead of RTL, and `ABORT` split three ways** (never armed / mode changed /
  airborne). RTL climbs to `RTL_ALT` first, which indoors is the ceiling.
- **FC safety envelope** set + verified before every flight (`FENCE_ACTION`, `RTL_ALT`
  with the 4.6/4.7 name fallback, `WPNAV_SPEED_UP`).
- **Presets over source edits:** `python main.py` runs the **real aircraft**, `--sim`
  the simulator; the dataclass defaults carry the aircraft's drop-servo path and
  battery threshold, and `Config.sitl()` overrides both for the simulator
  (`release_mechanism="fc"`, 10.8 V).

**Done when:** `pytest` covers each of the above and a SITL run logs the
firmware version, the autopilot's own messages and a named abort reason. ☑

## Cross-cutting

- **Documentation is written per phase, not at the end.** Each phase produces or
  updates a docs page (English).
- **Safety:** first hardware tests without propellers; always keep an independent kill
  switch on the transmitter; configure ArduPilot's own failsafes (`BATT_LOW_VOLT`,
  `FS_*`) in addition to the companion failsafe.

## Status

| Phase | Title                          | State |
|-------|--------------------------------|-------|
| 0     | Foundation & hygiene           | ☑ done |
| 1     | Full mission in SITL (GPS)     | ☑ done (SITL: full mission + LOW_BATTERY abort verified) |
| 2     | GPS-denied nav + target search | ☑ done (SITL: optical-flow search→approach→drop verified) |
| 3     | GPS-denied real HW: origin, fence, TimedCamera | ☑ done (SITL on ArduCopter **4.6.3**: origin set + verified by the companion, GPS off, full indoor mission green) |
| 3b    | Diagnostics & safety hardening | ☑ done (STATUSTEXT logging, companion heartbeat, verified origin/takeoff, mode monitoring, LAND instead of RTL, FC safety envelope) |
| 4     | Real AI camera                 | ◑ `RealCamera` implemented + wired; IMX500 detected on the Pi. Blocked on an `.rpk` model (only `.tflite` exists) and `imx500-all` |
| 5     | Pi provisioning & HIL prep     | ◑ Pi image, `mavlink-router` (systemd, `/dev/serial0` @ 921600 → `127.0.0.1:14550`), pymavlink, gpiozero/lgpio all **verified on hardware**. **MTF-01P delivers no data** — see below |
| 6     | Bench integration (no props)   | ◑ Companion↔FC link verified against the real FC (heartbeat, `--tele`, ArduPilot 4.6.3). Servo drop + arm/disarm still open |
| 7     | Flight tests                   | ☐ **blocked:** `ARMING_CHECK = 0` on the FC (all pre-arm checks disabled) must be restored before any flight |
| 8     | Documentation & deliverables   | ☐     |

> **Correction (bench session, real hardware).** Phase 5 previously claimed "MTF-01P
> configured + serial verified". Measured against the actual flight controller, that is
> **not** the case: `RANGEFINDER` streams a constant `0.00 m` and no `OPTICAL_FLOW`
> messages arrive at all, so the EKF sits in `CONST_POS_MODE` without `POS_HORIZ_REL` —
> the indoor mission would abort at `NO_POSITION_ESTIMATE` before ever leaving the
> ground. The **flight-controller side is provably correct** (`SERIAL5` = MAVLink1 @
> 115200, `FLOW_TYPE` 5, `RNGFND1_TYPE` 10, `RNGFND1_MIN_CM` 1, `RNGFND1_ORIENT` 25 —
> every value matches `project-docs/hardware/drone/InitialSetup.md`), so the fault is on
> the sensor side: the MTF-01P supports both MSP and MAVLink and was, per the team's own
> week-3 journal, never configured. Next steps: check the sensor's power LED, then an
> MSP counter-test on the FC, then the CP2102 adapter.
