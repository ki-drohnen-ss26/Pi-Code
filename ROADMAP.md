# Pi-Code Roadmap — Companion-Computer Control

This roadmap describes how we develop the Raspberry Pi (companion computer) control
software so that, at the very end, putting it on the real drone is a low-risk,
mechanical step — **not** a debugging session on a flying aircraft.

## Guiding principle

> Everything that can be developed and tested against **SITL** on the Mac is finished
> there. On the real hardware only the following remains: flash the Pi → clone the
> code → switch to `Config.pi_serial()` → wire the Pi to the flight controller →
> run incremental flight tests. No logic debugging on the flying object.

The architecture already supports this: `config.py` isolates the only SITL ↔ Pi
difference (the `connection_string`), `drone.py` is hardware-agnostic, `camera.py`
is a `Protocol` (mock now, real camera later), and `mission.py` is a clean state
machine. The roadmap builds on exactly these seams.

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

### Phase 0 — Foundation & hygiene *(Mac, now)*
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
  telemetry is missing (today `battery_critical()` returns `False` on `None` — a blind
  spot).
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
IDLE → TAKEOFF → SEARCH → APPROACH → OVER_TARGET → DROP → RTL
                    ↑__________|   (target lost → back to SEARCH)
                                              (ABORT → RTL on failsafe)
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
    radius (bounded by the geofence), not the hall dimensions.
  - `Lawnmower`: back-and-forth over a configured rectangle (even full coverage).
  - Selected via `config.py` (`search_pattern`).
- **Detection cadence** (config-selected): `stop_and_look` (**default** — fly to a
  waypoint, hover, detect, decide; robust, low motion blur) or `continuous` (poll while
  flying each leg, abort the leg on detection).
- **`APPROACH`** state: visual servoing — turn the camera's `dx/dy` into local-NED
  nudges and descend until the existing `OVER_TARGET` fine-centre takes over.
- **`SimCamera`** (`camera.py`, satisfies the existing `Camera` protocol): models a
  target at a known local NED position and "detects" it once the drone is within a
  field-of-view footprint. This lets the whole search → approach → drop logic be
  validated in SITL/`FakeDrone` **before** the real camera exists. The
  `{detected, dx, dy, distance}` contract is unchanged, so Phase 3 only swaps the
  detector.

**Done when:** the full indoor delivery mission flies in SITL using optical flow + LiDAR
for position, with no GPS, and finds the target by search before dropping. Both search
patterns and both detection cadences are exercised via config.

### Phase 3 — Real AI camera *(Pi, developed in parallel)*
**Goal:** replace the mock camera without touching mission logic.
- Implement `RealCamera` (IMX500 / YOLO from `yolo-imx500/`) that satisfies the
  existing `Camera` protocol: turn a detection bounding box into `dx/dy/distance`.
- Test standalone on the Pi (no flight): point at the target object and print offsets.

**Done when:** swapping `MockCamera` → `RealCamera` in `main.py` is the only change
and the mission code is untouched.

### Phase 4 — Pi provisioning & hardware-in-the-loop prep
**Goal:** a reproducible Pi image and a configured flight controller.
- Configure the MTF-01P on the bench via the CP2102 USB-UART adapter (sensor doc).
- Pi OS image: enable UART, disable the serial login console, install pymavlink, clone
  the code, set up a **systemd autostart service**, and **`mavlink-router`** (FC →
  script + QGC + log).
- FC parameters: assign the Pi's TELEM port (`SERIALx_PROTOCOL=2`, matching baud) and
  the MTF-01P (rangefinder + optical-flow serial protocol). Extend the `params/` file.

**Done when:** the Pi boots, routes MAVLink, and the FC is parameterised for flow +
LiDAR + companion link.

### Phase 5 — Bench integration *(no propellers)*
**Goal:** prove the real link and sensors before anything spins.
- Pi ↔ FC: heartbeat, `--tele` telemetry, mode changes, servo drop, arm/disarm.
- MTF-01P live: `RNGFND` distance and `OPTFLOW_QUALITY` healthy; move the airframe by
  hand and confirm the EKF is actually using the flow for position.

**Done when:** every companion action works against the real FC on the bench, props off.

### Phase 6 — Flight tests *(incremental, props on, in the hall)*
**Goal:** earn trust step by step.
1. Manual `AltHold` → confirms LiDAR altitude hold.
2. Manual `PosHold` → confirms optical-flow position hold.
3. `GUIDED` hover → `goto_local` → short delivery.
4. Full indoor delivery mission.

Every test is logged and documented (what was tested, parameters, outcome).

### Phase 7 — Documentation & deliverables *(continuous, finalised here)*
- Fill the mkdocs stub pages, write the tutorials, keep the project journal, complete
  `results/limitations`, and prepare the poster + live demo.

---

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
| 1     | Full mission in SITL (GPS)     | ☐     |
| 2     | GPS-denied nav + target search | ☐     |
| 3     | Real AI camera                 | ☐     |
| 4     | Pi provisioning & HIL prep     | ☐     |
| 5     | Bench integration (no props)   | ☐     |
| 6     | Flight tests                   | ☐     |
| 7     | Documentation & deliverables   | ☐     |
