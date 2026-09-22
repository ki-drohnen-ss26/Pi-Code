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

**Starting SITL (changed 2026-09-21).** Because the companion writes no FC parameter, the
simulator has to arrive already configured, and for a while the only documented way to do
that was to load the mirror into a RUNNING simulator twice with a reboot each time. That
is what "the simulation is broken" turned out to be. A bare `sim_vehicle.py` runs firmware
defaults (`RNGFND1_TYPE=0`, `FLOW_TYPE=0`), so the mission correctly aborts in IDLE with
`NO_RANGEFINDER_DATA`, and the two-pass procedure failed in practice: after the first
reboot the `RNGFND1_*` sub-parameters were still reported unknown, the second pass changed
nothing ("changed 0"), and after the second reboot MAVProxy lost the link and never got it
back. The fix is the new launcher `sitl.py`, which passes the mirror as a **startup
defaults file**. ArduPilot holds back a default whose parameter does not exist yet and
applies it when the driver creates it, so `RNGFND1_TYPE` and its `MIN_CM` / `MAX_CM` /
`GNDCLEAR` / `ORIENT` sub-parameters all land in the SAME boot. One start, no reboot, no
second pass.

```bash
python sitl.py                 # ArduCopter SITL, wiped, flight parameters loaded
python sitl.py --speedup 5
python sitl.py --no-wipe
python sitl.py --print-only
```

It resolves the ArduPilot checkout (`ARDUPILOT_HOME`, else `../Simulation/ardupilot`, else
`~/ardupilot`) and the highest-numbered mirror, then runs:

```bash
sim_vehicle.py -v ArduCopter --no-rebuild --console \
  --custom-location=<config.origin_lat,lon,alt,0> \
  --add-param-file=params/sitl_flight_v<N>.parm \
  --out=udp:127.0.0.1:14550 --speedup 1 -w
```

Run it from the environment that has `sim_vehicle.py` **and** `mavproxy.py` on PATH (the
`ardupilot` conda env), and run `main.py --sim` from the Pi-Code environment in a second
terminal. The `-w` wipe matters twice: every run starts from the same parameter state, and
it resets the simulated battery, which drains across runs and otherwise aborts a later
mission with a puzzling `LOW_BATTERY`. Verified 2026-09-21: 1646 parameters loaded, and
`EKF3 IMU0 fusing optical flow` plus `started relative aiding` in the boot banner.

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
    radius, not the hall dimensions — but size that radius for the hall by hand. No
    geofence guards the walls (the fence is off in the flight set; even an altitude fence
    would only guard the ceiling), so nothing stops a horizontal excursion, and the
    pattern reaches one `step` beyond `max_radius`.
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
  detector. *Corrected 2026-09-21:* that `dx`/`dy` is a **body-frame** error, not
  north/east. `SimCamera` reported earth frame while the approach loop flew it as body
  frame, which only cancels out while the nose points north (the finding is under Phase 3b).

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
- **Geofence indoors** — the GPS-style fence needs a position, so at most an altitude-only
  fence is possible indoors. *Decided after the 2026-08-21 incident: **off by default***
  (`FENCE_ENABLE 0` in the flight set, owned by Mission Planner) — the barometric altitude
  spikes 4–6.7 m under propeller downwash, so an indoor-sized altitude fence breaches on
  every takeoff and `FENCE_ACTION` converts that into an uncommanded mode change (see
  `SIM_TO_REAL.md` §5c). The companion only verifies read-only that no stale fence is armed
  (`verify_fence_disabled()`); it never writes fence parameters.
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
test flights. **The model blocker is resolved (2026-09-21):** the `.rpk` re-export
(`yolo export … format=imx` → `imx500-package`) has landed, and object detection now
works on the real aircraft. Still ahead: the companion's own autonomous `--milestone`
bring-up flights with the real camera (milestones 2, 4 and 5) have not been flown yet,
and the axis calibration (`cam_swap_axes`/`cam_invert_x`/`cam_invert_y`) still needs
confirming in the air rather than only on the bench.

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

1. Manual `AltHold` → confirms LiDAR altitude hold. **☑ flying (2026-08-24 evening)**
   under the chosen `EK3_SRC1_POSZ=2`.
2. Manual `PosHold`/`Loiter` → confirms optical-flow position hold. **☑ Loiter flying
   (2026-08-24 evening)**; the earlier oscillation was cured by halving
   `ATC_RAT_PIT`/`ATC_RAT_RLL` P/I (0.135→0.0675) and D (0.0036→0.0018) (v1→v2 tune).
3. `--milestone 1` — companion hover at 0.8 m. The logged **drift** is the result, not the
   fact that it hovered: a broken flow setup still flies, it just walks away.
4. `--milestone 2` — same flight with the detector running, logging only. Verify the
   **sign** of `dx`/`dy` here (pad to the right → positive `dx`), on the ground first.
5. `--milestone 3` — the search pattern, no detector. Ends in `TARGET_NOT_FOUND`; that
   is the pass condition. Note the spiral reaches `search_max_radius_m` **plus one**
   `search_step_m` and nothing bounds it horizontally.
6. `--milestone 4` — search, detect and centre with nothing able to fall out.
7. `--milestone 5` — the full indoor delivery.

- **2026-08-25 (from the hall dataflash logs):** **AltHold verified clean in the hall**
  (36 s stable, 0.12 m position excursion). **Loiter is blocked by the hall's magnetics** —
  it drifted and was abandoned within seconds on all three attempts. The hall's field
  magnitude swings 195–565 mGauss across the flight volume (rebar/steel; ~340 mGauss on the
  floor vs ~480 Earth field), and the EKF never *rejected* the compass — it fused a wrong
  heading (yaw snapped +83° in one second on landing against only +50° of gyro rotation, an
  `ERR COMPASS` event; a full 360° yaw at up to 182°/s during Loiter). Optical-flow position
  control needs a correct yaw to convert flow into NED, so a wrong heading becomes drift;
  AltHold is yaw-independent, hence clean. *Next:* (1) bench yaw sanity check + confirm
  `COMPASS_ORIENT = 2` is correct for the mast-mounted HGLRC M100; (2) in-hall compass
  calibration / MAGFit from this log (vehicle-fixed offsets only — the spatial gradient
  cannot be calibrated away); (3) pick a flight area away from floor rebar/walls, consider an
  elevated takeoff spot. The team has removed the compass bit from `ARMING_CHECK`
  (41350 → 41346) to clear compass prearm failures — a decision to record and revisit; all
  parameter changes via Mission Planner. A follow-up measurement (recal in the *middle* of
  the hall) found the field magnitude climbing from ~360 mGauss on the floor to ~850 mGauss
  at ~2 m over one and the same spot — a vertical gradient larger than the ~480 mGauss Earth
  field, which no calibration can remove. Flying compassless is not an escape either: on the
  `flight_v2` SITL mirror (4.6.3) `COMPASS_USE=0` + `EK3_SRC1_YAW=0` flapped EKF aiding on/off
  and refused arming with *"Need Position Estimate"* (5/5), while the same mirror with the
  compass flew milestone 1 green — so the compass is mandatory and the fix must make *it*
  usable in the hall. The ranked options (field-mapping a stable takeoff zone, the
  colleague-team comparison, an `EK3_MAG_M_NSE` damping experiment, a hand-lift
  building-vs-self test, or escalation) live in `project-docs` → Problems → "Loiter drifts in
  the hall"; the compass bit returns to `ARMING_CHECK` (41346 → 41350) once one of them
  lands as a proper fix. **Update 2026-09-21:** a session flown deliberately in Stabilize
  reproduced the identical signature (three sudden 30-44° EKF yaw resets, each logged
  `ground mag anomaly, yaw re-aligned`), and confirmed a cheap mitigation in the meantime:
  minimising ground dwell before climbing ("arm and go") measurably improves Loiter,
  consistent with the field-state settling onto the hall's local distortion the longer the
  aircraft sits still before the climb exposes the vertical gradient. With that procedure
  **both AltHold and Loiter now fly on the real aircraft** — Loiter good in the lab, still
  imperfect but flyable in the hall. Full mechanism and log evidence in `project-docs` →
  Problems → [Loiter drifts in the hall](https://github.com/ki-drohnen-ss26/project-docs/blob/main/docs/problems/hall-magnetics.md).
  Companion `--milestone` flights in the hall are the next step, not yet attempted.
- **2026-09-21 (SITL sweep on ArduCopter 4.6.3): every `--milestone` stage is green IN
  SIMULATION.** Milestone 1: climb to 0.8 m, rangefinder track 0.84 m, 20 s hover, worst
  horizontal drift 0.04 to 0.06 m, LAND, disarm. Milestone 2: the same at 1.0 m, and the
  substituted simulated detector now logs *"Camera sees the target"* every second (the
  rehearsal moves the simulated pad under the hover spot, so the detection path and its log
  line are actually exercised instead of staring at empty floor). Milestone 3: all 25 spiral
  waypoints flown in 219 s, ending in `TARGET_NOT_FOUND`, which is the pass condition.
  Milestone 4: detects, centres, releases nothing. Milestone 5: detects, centres in 3
  nudges, `[DROP] Release confirmed`, LAND. The full mission (`main.py --sim`, no milestone
  flag) is green too, and the pre-mission parameter check reports *"Flight parameters match
  sitl_flight_v2.parm"*. This validates the companion logic of every stage end to end. It
  says nothing about the hall on its own; see the 2026-09-21 update above for why manual
  AltHold/Loiter now work there too, and companion `--milestone` flights are the step
  that follows.

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
- **FC safety envelope** (`WPNAV_SPEED_UP`, `WPNAV_SPEED`, `RTL_ALT` with the 4.6/4.7 name
  fallback) was originally *set + restored* by the companion before/after every flight,
  mirrored to disk so even a killed run was cleaned up by the next one. **Superseded on
  2026-08-25** by the parameter-ownership decision: those limits now live in
  `params/flight_v2.param` (owned by Mission Planner) and the companion only verifies them
  read-only. The stale-fence trigger of the 2026-08-21 incident (a leftover `FENCE_ACTION=2`)
  is now caught read-only by `verify_fence_disabled()`, which refuses to fly rather than
  writing anything.
- **Presets over source edits:** `python main.py` runs the **real aircraft**, `--sim`
  the simulator; the dataclass defaults carry the aircraft's drop-servo path and
  battery threshold, and `Config.sitl()` overrides both for the simulator
  (`release_mechanism="fc"`, 10.8 V).
- **The approach loop was flying in the wrong frame (found and fixed 2026-09-21).**
  `SimCamera` returned the target's NORTH/EAST error, but `mission._nudge_from_offset()`
  feeds `dx`/`dy` to `Drone.move_body_offset()`, which sends `MAV_FRAME_BODY_OFFSET_NED`
  and is therefore rotated by the vehicle's yaw. A real downward camera sees the target in
  the IMAGE, and the image is bolted to the airframe, so the body frame is the correct
  contract and `SimCamera` was the unfaithful side. It stays invisible while the nose points
  north. ArduCopter yaws toward each waypoint by default, so after a few legs of the search
  pattern the aircraft sat at yaw -139 degrees: every correction went off at 139 degrees to
  the error, and the drone chased the pad out of its own field of view, fell back to SEARCH,
  re-detected and repeated. 146 nudges, four re-detections, never converged, until the
  simulated battery died. Two things had hidden it: the simulated pad sat at (2.0, 2.0),
  exactly on a spiral corner, so the aircraft arrived already centred and the servo loop
  never ran, and the unit-test `FakeDrone` assumed yaw = 0 on both sides, so the two errors
  cancelled. Fixed: `SimCamera` rotates the earth-frame error into the body frame using a
  new `Drone.get_yaw()` (ATTITUDE, radians, from north, clockwise positive), `FakeDrone`
  rotates `move_body_offset()` by its yaw the way the real autopilot does, and a
  parametrised regression test covers yaw 0, 45, -139, 90 and 180 degrees. The default
  simulated target moved to (2.5, 1.5), off a spiral corner, so a SITL rehearsal actually
  exercises the servo loop. Milestone 5 then converged in 3 nudges and released.

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
| 2     | GPS-denied nav + target search | ☑ done (SITL: optical-flow search→approach→drop verified; the approach loop's earth-frame/body-frame bug found and fixed 2026-09-21) |
| 3     | GPS-denied real HW: origin, fence, TimedCamera | ☑ done (SITL on ArduCopter **4.6.3**: origin set + verified by the companion, GPS off, full indoor mission green) |
| 3b    | Diagnostics & safety hardening | ☑ done (STATUSTEXT logging, companion heartbeat, verified origin/takeoff, mode monitoring, LAND instead of RTL, FC safety envelope) |
| 4     | Real AI camera                 | ☑ `RealCamera` implemented + wired; IMX500 detected on the Pi. The `.rpk` re-export landed 2026-09-21, and **object detection now works on the real aircraft**. Open: fly the axis calibration (milestone 2), not yet confirmed in the air |
| 5     | Pi provisioning & HIL prep     | ☑ Pi image, `mavlink-router` (systemd, `/dev/serial0` @ 921600 → `127.0.0.1:14550`), pymavlink, gpiozero/lgpio all **verified on hardware**. MTF-01P configured and streaming (see note below) |
| 6     | Bench integration (no props)   | ◑ Companion↔FC link verified against the real FC (heartbeat, `--tele`, ArduPilot 4.6.3). Servo drop + arm/disarm still open |
| 7     | Flight tests                   | ◑ **manual modes flying on the real aircraft:** both **AltHold and Loiter** work with the chosen `EK3_SRC1_POSZ=2`; the v1→v2 oscillation was fixed by **halving `ATC_RAT_PIT`/`ATC_RAT_RLL` P/I (0.135→0.0675) and D (0.0036→0.0018)**. Loiter is good in the lab and, as of 2026-09-21, flyable (though still imperfect) in the hall too with an "arm and go" procedure that minimises ground dwell before climbing; see [project-docs → Problems → Loiter drifts in the hall](https://github.com/ki-drohnen-ss26/project-docs/blob/main/docs/problems/hall-magnetics.md) for the mechanism. **All five `--milestone` stages and the full mission are green in SITL (2026-09-21, 4.6.3)**, so the companion logic is validated; the companion's own autonomous `--milestone` flights (GUIDED, software controlled) have not been flown on the real aircraft yet, now that AltHold and Loiter work well enough to attempt them |
| 8     | Documentation & deliverables   | ◑ Pi-Code docs current; project-docs (mkdocs site) being filled |

> **MTF-01P history.** An earlier bench session found `RANGEFINDER` at a constant
> `0.00 m` and no `OPTICAL_FLOW` at all (sensor still on MSP, never configured for
> MAVLink). That was subsequently fixed: in all five dataflash logs from the 2026-08-20/21
> flight days both streams are healthy — rangefinder status "good" in 3441/3441 samples
> (and truthfully tracking the crash climb 0.02 → 4.94 m), flow quality 45–113. The
> sensor side of Task 4 works.

## Incident 2026-08-21 and the EKF height source

The full analysis lives in [`SIM_TO_REAL.md` §5c](SIM_TO_REAL.md); this section tracks
the *decisions and blockers* that came out of it.

**Decisions taken:**
- Fence **off** indoors (`FENCE_ENABLE 0` in the flight set; fence breached on the
  downwash spike on every takeoff, and `FENCE_ACTION=2` then forced LAND over the pilot).
- **Parameter ownership (2026-08-24): the companion no longer writes any FC parameter.**
  FC parameters now have one owner — Mission Planner plus the published, versioned flight
  parameter set (`params/flight_v2.param`); the companion verifies them read-only
  (`preflight.py`). Two reasons: (1) a **single source of truth** for the flight
  configuration, and (2) **no surprise overwrites** — the 2026-08-21 crash fence was
  precisely a companion-written parameter that outlived its run. A
  stale fence found enabled at startup is no longer cleared by the companion: it now
  **refuses to fly** (`UNEXPECTED_FENCE_ENABLED`) and asks the operator to disable it in
  Mission Planner. Deliberate values kept on the real aircraft: `BATT_LOW_VOLT=12.4`
  (recommendation was 12.8) and `ARMING_CHECK=41350` (the fuller 786390 mask declined for
  now); the `FLTMODE` switch mapping is done transmitter-side and lands in flight-set v3.
- **Legacy write/restore machinery deleted (2026-08-25).** The old opt-in
  (`enforce_safety_envelope`, `setup_safety_envelope()`, `restore_params()`,
  `recover_stale_params()`, the `logs/fc_params_backup.json` mirror) is gone; the flight
  companion now writes no FC flight parameter (its only PARAM_SET is `FcServo`'s SITL-only
  `SERVO9_FUNCTION=0`).
- **The mission verifies the flight parameters before every arming (2026-09-21).** Handing
  the FC parameters to Mission Planner removed the surprise-overwrite failure mode, but it
  opened a quieter one: nothing then noticed when the aircraft in front of you stopped being
  the aircraft the code was reasoned about. The new module `paramcheck.py` and the new gate
  `FailsafeMonitor.verify_flight_parameters()`, called from `mission._idle()` right after
  the fence check and before arming, read a curated subset back from the FC and compare it
  against the published set. Still strictly read-only: it reports, it never writes.
  **CRITICAL** (a difference aborts with `FC_PARAMS_MISMATCH`): `EK3_SRC1_POSXY`,
  `EK3_SRC1_VELXY`, `EK3_SRC1_POSZ`, `EK3_SRC1_VELZ`, `EK3_SRC1_YAW`, `AHRS_EKF_TYPE`,
  `RNGFND1_TYPE`, `RNGFND1_MIN_CM`, `RNGFND1_MAX_CM`, `RNGFND1_ORIENT`, `RNGFND1_GNDCLEAR`,
  `FLOW_TYPE`, `WPNAV_SPEED`, `WPNAV_SPEED_UP`, `BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT`,
  `FS_THR_ENABLE`, `RTL_ALT`. **INFORMATIONAL** (reported, never blocking): `ARMING_CHECK`,
  `BATT_MONITOR`, `BATT_LOW_VOLT`, `BATT_CRT_VOLT`, `FS_EKF_ACTION`, `FS_GCS_ENABLE`,
  `FLOW_ORIENT_YAW`. `ARMING_CHECK` sits there deliberately: the team removed the compass
  bit on the aircraft (41350 to 41346) while the hall's magnetic problem is open, and the
  `FLTMODE` map is done transmitter-side. `FENCE_ENABLE` is deliberately **not** in the list
  because `verify_fence_disabled()` already refuses on it with the better-named
  `UNEXPECTED_FENCE_ENABLED` and logs the fence's shape, which is why the fence check runs
  first. A parameter the FC does not answer for is reported as "could not read" and does not
  block: a busy link drops `PARAM_VALUE` replies, and that is indistinguishable from a name
  the firmware does not know. `config.param_check` selects "abort" (default), "warn" or
  "off".
- **The published set is resolved by VERSION, and publishing a new one is two commands.**
  An empty `config.expected_params_path` means the highest `params/flight_v<N>.param` for
  the aircraft and the highest `params/sitl_flight_v<N>.parm` for a simulated run, so a v3
  needs no source edit anywhere. `preflight.py` resolves the same way; its hard-coded
  `flight_v2.param` default is gone and `--expected PATH` still overrides. SITL is compared
  against the **mirror**, not the flight set, because the mirror deviates from the aircraft
  on purpose (SITL sensor backends, GPS off, the `RNGFND1_MIN_CM` validity floor, the
  simulated pack's battery voltages), and comparing it against the flight set would report
  those intended deviations as faults and teach everyone to ignore the check. The new tool
  `dumpparams.py` closes the loop: it downloads the live FC's full parameter list,
  re-requesting any reply that was lost, and writes `params/flight_v<next>.param` in Mission
  Planner's `NAME,VALUE` format. So updating the code after a Mission Planner change is
  `python dumpparams.py`, then `python params/generate_sitl_flight_params.py` to regenerate
  the mirror, then review the diff and commit both. It is read-only on the aircraft.
- The takeover gate (`--takeover`) now trusts the **rangefinder**, not the EKF
  altitude, and refuses the handover when the two disagree (`EKF_ALT_DIVERGED`).

**The EKF height source: our choice, not a mandate (corrected 2026-09-21).** This section
used to say that the assignment **mandates** the rangefinder as the EKF height source
(`EK3_SRC1_POSZ = 2`) and that the barometer (`POSZ = 1`) is not permitted. That reading
was wrong, and the project owner has confirmed it. Aufgabe 4 requires that the LiDAR and
the optical flow be **used** for altitude hold and position hold. It says nothing about
which EKF source parameter carries the vertical position, so `EK3_SRC1_POSZ = 2` is the
team's own configuration choice and `EK3_SRC1_POSZ = 1` (barometer) remains an available
option. `POSZ = 2` is also the verified 2026-08-21 configuration in which the vertical
estimate diverged: EKF3 never fused a height from the rangefinder, so it ran away
quadratically on the ground (−1070 m at arming, "climb rate" −12.6 m/s while stationary).
So `POSZ = 2` stays, but as a decision we made and could still revisit, operated under a
**safety protocol**. `params/fc_safe_overrides.parm` now sets `EK3_SRC1_POSZ = 2` and
`RNGFND1_GNDCLEAR = 5` (the EKF's expected on-ground reading in cm; the default 10 did not
match our ~2 cm mounting, but Mission Planner refused anything below 5 - the parameter's
own minimum - when the team applied this on the real aircraft on 2026-09-21, so 5 is the
closest achievable value, not the true mounting height), loaded on top of the recovered
baseline before the next flight.

*The protective net that makes `POSZ = 2` operable:*
- `preflight.py` ground-drift verdict before **every** arming — drift on the ground is
  the crash failure mode, so it is the GO/NO-GO gate.
- `ARMING_CHECK = 786390`, geofence off.
- continuous in-flight EKF-vs-rangefinder cross-check (`EKF_ALT_DIVERGED` /
  `altitude_implausible()`).
- rangefinder-gated `--takeover` starts — the EKF altitude is never trusted for the
  handover.
- a bench hand-lift test proving the EKF altitude follows a real lift, before the first
  flight of a session.

The barometer stays a logged **witness** (BAlt, sanity reference), and because nothing
forbids it as the EKF source, `POSZ = 1` stays on the table as an option the team may pick
if the rangefinder source ever proves unworkable in the hall.

*Clarification, closed 2026-09-21:* the professor's course document (`AI_Drones.pdf`,
§11.2 Step C) sets `POSZ = 2` only in its **SRC2**/aux-switch recipe; its
exclusively-indoor **SRC1** recipe has no POSZ line at all (height stays at the baro
default). That looked like an open question about which variant the assignment intends. It
is not one: the assignment mandates neither value, so both recipes are legitimate and ours
is a choice. The observation still matters for the colleague-team parameter diff, where
`EK3_SRC1_POSZ` is the **first** value to check: if they followed the SRC1 recipe verbatim,
they fly baro height without knowing it, which would fully explain their stable ground
behaviour.

*Open investigation:* a colleague team flies the **same** sensor with `POSZ = 2`
successfully. The next step is a full parameter diff against their aircraft (hot suspects:
`RNGFND1_GNDCLEAR`, `RNGFND1_MIN_CM`, `EK3_ALT_M_NSE`) plus a 3-minute ground-drift test
on *their* aircraft — the alternative explanation being that their EKF drifts on the
ground too and nobody ever left it standing for minutes.

*Documented lead (2026-08-25):* a **working** reference script from the drone lab
(`fly_800CM_and_land.py`, flown on another aircraft in the same lab) states its EKF
source set as `EK3_SRC1_POSXY = 3` (**GPS**), `VELXY = 5`, `POSZ = 2` — i.e. that
aircraft flies rangefinder height with the *horizontal* source left on GPS despite
having no indoor GPS fix, where ours runs `EK3_SRC1_POSXY = 0`. This is the first
documented evidence of a concrete working parameter set in this lab, so
`EK3_SRC1_POSXY` (ours: 0) now joins `EK3_SRC1_POSZ` and `RNGFND1_GNDCLEAR` as concrete
values to compare in that diff. The same script also warns that optical flow is
typically poor below ~0.5 m ("keep above ~0.5 m") — which bears on our minimum test
altitudes, including the 0.8 m first-flight choice for milestone 1.

*Breakthrough (2026-08-25) — the on-ground blocker in SITL is the `RNGFND1_MIN_CM`
validity floor, not `POSZ`.* A clean 2×2 matrix on a wiped, mirror-loaded SITL (each cell
after reboot, 60 s of `EKF_STATUS_REPORT` on the ground) isolated it:

| | `RNGFND1_MIN_CM = 0` | `RNGFND1_MIN_CM = 1` |
|---|---|---|
| `EK3_SRC1_POSZ = 1` (baro) | `POS_HORIZ_REL` immediately | — |
| `EK3_SRC1_POSZ = 2` (rangefinder) | `POS_HORIZ_REL` immediately | **never** (`0x0027`, no `PRED_REL`/`VERT_AGL`) |

`POSZ = 2` is **exonerated for the on-ground phase in SITL**: with `MIN_CM 0` fusion comes
up immediately under either height source. What blocks it is the `MIN_CM` floor — SITL's
landed rangefinder reads exactly `0.00 m`, so `MIN_CM 1` flags it out-of-range-low, the EKF
gets no range, optical flow cannot be scaled, and no relative position (nor, under `POSZ 2`,
any height) initialises. The **real aircraft sits 1 cm above** the same floor (reads
`0.02 m` with `MIN_CM 1`), so this gate is now the leading suspect for the crash-day
divergence. With this deviation the mirror flew a **fully green milestone 1** in SITL under
the chosen `POSZ 2`: EKF ready on the ground, climb to 0.8 m, rangefinder track confirmed,
20 s hover at **0.03 m** worst horizontal drift, LAND — the milestone-1 logic is validated
end-to-end. *Next real-aircraft steps:* the colleague parameter diff
(`EK3_SRC1_POSZ` / `EK3_SRC1_POSXY` / `RNGFND1_MIN_CM` / `RNGFND1_GNDCLEAR`) and a team
decision whether to set `RNGFND1_MIN_CM = 0` on the aircraft — applied via Mission Planner
only, per the ownership rule.

**Current blockers (in repair order):**
1. **Baro/I2C damage.** After the crash the stock 4.6.3 firmware halts with
   `Config Error: Baro: unable to initialise driver` (FC_Check.pdf), and a colleague's
   custom 4.8.0-dev build (baro requirement bypassed) shows `Baro: no sensors found`
   **and** `Bad Compass Health`. The FlywooF745 has exactly **one I2C bus** shared by
   the onboard baro and the external GPS-module compass — and the crash **tore off the
   GPS connector** (FC_Check.pdf §1.4). A damaged compass/GPS wiring holding SDA/SCL
   down would explain both symptoms with an intact baro chip.
   **RESOLVED 2026-08-24 — the hypothesis held completely.** Bent pins in the GPS
   connector were found and straightened (2026-08-23); "Bad Compass Health"
   disappeared. Stock 4.6.3 was re-flashed and **detects the barometer again**: the
   chip was never dead, only unreachable behind the hung single I2C bus. No new FC
   needed. Full story: `project-docs` → Problems → "Crash & barometer recovery".
2. **Parameter wipe.** Flashing the custom 4.8.0-dev build reset ALL parameters to
   defaults (Mission Planner shows `New mission / New rally / New fence`, battery
   monitor unconfigured). After returning to stock 4.6.3, reload the recovered baseline
   and then `params/fc_safe_overrides.parm` on top: `FENCE_ENABLE=0`, `EK3_SRC1_POSZ=2`
   (the rangefinder height source the team chose) + `RNGFND1_GNDCLEAR=5` (the closest
   achievable to the true ~2 cm mounting; the parameter refuses anything below 5),
   arming checks on (see `params/README.md`).
3. **`ARMING_CHECK = 0`** must be restored (1, or 786390 = everything except GPS lock)
   before any flight — the crash flight armed with an EKF vertical error of 1000 m that
   an enabled EKF pre-arm check would have refused.
