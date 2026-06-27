# Architecture & Flow

How the companion-computer code fits together: the components, how they relate, and
the end-to-end mission flow. See also [SIM_TO_REAL.md](SIM_TO_REAL.md) for the
simulation→hardware transition and [ROADMAP.md](ROADMAP.md) for the phased plan.

## Design rule

> **Only `config.py` knows whether SITL or a real flight controller is on the other
> end.** Everything else — `drone.py`, `mission.py`, `failsafe.py`, `camera.py` — is
> identical in both worlds. That is what makes the move to real hardware a small step.

## Components and their relationships

```mermaid
flowchart TB
    main["main.py — entry point, wiring"]
    cfg["Config — config.py"]
    logb["logbook.py — console + file log"]
    mission["DeliveryMission — mission.py state machine"]
    drone["Drone — drone.py MAVLink actions"]
    fs["FailsafeMonitor — failsafe.py"]
    fc[("Flight Controller / SITL")]

    subgraph cam["Camera — Protocol, camera.py"]
        mock["MockCamera"]
        scripted["ScriptedCamera"]
        real["RealCamera — Phase 3"]
    end

    main --> logb
    main --> cfg
    main --> drone
    main --> cam
    main --> fs
    main --> mission

    mission --> drone
    mission --> cam
    mission --> fs
    mission -.->|reads| cfg
    fs --> drone
    fs -.->|reads| cfg
    drone -.->|reads| cfg
    drone <-->|"MAVLink: UDP in SITL, UART on Pi"| fc
```

| Component | Responsibility |
|-----------|----------------|
| `main.py` | Chooses the `Config`, sets up logging, wires the objects, starts the mission. The single line that differs SITL vs Pi lives here. |
| `Config` | All parameters + the `connection_string`. `Config.sitl()` vs `Config.pi_serial()`. |
| `Drone` | Wraps the MAVLink link. Low-level actions: heartbeat, telemetry, mode/arm/takeoff/goto/RTL, body-frame nudges, servo drop. Hardware-agnostic. |
| `Camera` | A `Protocol` returning `{detected, dx, dy, distance}`. `MockCamera`/`ScriptedCamera` now; `RealCamera` (AI camera) later — same contract, so the mission never changes. |
| `FailsafeMonitor` | Companion-side safety: link loss, telemetry loss, battery, phase timeout. Returns a reason string; the mission decides to ABORT. |
| `DeliveryMission` | The state machine that sequences the delivery and runs the failsafe check before each state. |
| `logbook` | Configures logging to console + a timestamped file under `logs/`. |

## Mission flow (current — Phase 1)

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> TAKEOFF: GUIDED + armed
    TAKEOFF --> ENROUTE: altitude reached
    ENROUTE --> OVER_TARGET: arrived
    OVER_TARGET --> DROP: centred
    DROP --> RTL: released
    RTL --> [*]: landed + disarmed
    IDLE --> ABORT: pre-arm / arm fails
    TAKEOFF --> ABORT: climb timeout
    ENROUTE --> ABORT: arrival timeout
    OVER_TARGET --> ABORT: align timeout
    ABORT --> RTL: controlled return
```

- **Failsafe before every state:** the loop calls `FailsafeMonitor.check()` before
  each state (except while already aborting/landing). It can return `LINK_LOSS`,
  `NO_TELEMETRY`, `LOW_BATTERY` or `TIMEOUT_<phase>` → the machine jumps to `ABORT`.
- **`OVER_TARGET` is the visual-servoing loop:** read the camera's `dx/dy`, nudge the
  drone in the body frame (`Drone.move_body_offset()`), repeat until centred, then
  `DROP`. With `MockCamera` (dx=dy=0) the first frame is already centred; with
  `ScriptedCamera` you can watch it correct over several steps.

## Mission flow (planned — Phase 2, indoor + search)

The hall delivery has no known target position, so the machine grows a search stage
(see [ROADMAP.md](ROADMAP.md)):

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> TAKEOFF
    TAKEOFF --> SEARCH
    SEARCH --> APPROACH: target detected
    APPROACH --> OVER_TARGET: close + roughly centred
    APPROACH --> SEARCH: target lost
    OVER_TARGET --> DROP
    DROP --> RTL
    RTL --> [*]
```

`SEARCH` flies a pattern in local NED (expanding spiral by default) and polls the
camera; `APPROACH` does visual servoing toward a detected target. The `Camera`
contract is unchanged, so only the detector is swapped (`RealCamera`).

## Call sequence — who calls what (Phase 1 happy path)

This is the same flow as above, but at the level of **which method of which class
calls which method**, with the key parameters. It mirrors the code in `main.py` and
`mission.py`; each `->>FC` is a MAVLink message to the flight controller.

```mermaid
sequenceDiagram
    autonumber
    participant Main as main.py
    participant Mis as DeliveryMission
    participant FS as FailsafeMonitor
    participant Dr as Drone
    participant Cam as Camera
    participant FC as FC / SITL

    Main->>Dr: Drone(config).connect()
    Dr->>FC: wait_heartbeat()
    Dr->>FC: request_data_streams(rate_hz=4)
    Main->>Mis: DeliveryMission(drone, camera, failsafe, config).run()

    note over Mis,FS: before EVERY state
    Mis->>FS: check()
    FS->>Dr: link_alive(heartbeat_timeout_s=3.0)
    FS->>Dr: get_battery()
    FS-->>Mis: None or "LINK_LOSS"/"NO_TELEMETRY"/"LOW_BATTERY"/"TIMEOUT_x"

    note over Mis: IDLE
    Mis->>FS: setup_geofence()
    FS->>Dr: set_param("FENCE_ENABLE", 1)
    Mis->>Dr: configure_drop_servo()
    Dr->>FC: set_param("SERVO9_FUNCTION", 0)
    Mis->>Dr: reset_servo()
    Mis->>Dr: wait_ready_to_arm()
    Dr->>FC: wait for EKF_POS_HORIZ_ABS
    Mis->>Dr: set_mode("GUIDED")
    Mis->>Dr: arm()

    note over Mis: TAKEOFF
    Mis->>Dr: takeoff(config.cruise_alt=10.0)
    Dr->>FC: MAV_CMD_NAV_TAKEOFF

    note over Mis: ENROUTE
    Mis->>Dr: goto(target_lat, target_lon, cruise_alt)
    Dr->>FC: SET_POSITION_TARGET_GLOBAL_INT
    Mis->>Dr: wait_arrival(lat, lon, arrival_radius_m=1.5)

    note over Mis: OVER_TARGET (loop until centred)
    Mis->>Cam: get_target_offset()
    Cam-->>Mis: {detected, dx, dy, distance}
    Mis->>Dr: move_body_offset(forward, right)
    Dr->>FC: SET_POSITION_TARGET_LOCAL_NED (BODY_OFFSET)

    note over Mis: DROP
    Mis->>Dr: drop()
    Dr->>FC: MAV_CMD_DO_SET_SERVO(drop_servo=9, drop_pwm=1900)
    Mis->>Dr: read_servo(expected=drop_pwm)
    Mis->>Dr: reset_servo()

    note over Mis: RTL
    Mis->>Dr: return_to_launch()
    Dr->>FC: set_mode("RTL")
    Mis->>Dr: wait_disarmed(phase_timeout_s=60.0)
```

> Parameters shown are the defaults from `config.py`. On an abort the flow jumps from
> the current state straight to `ABORT → RTL` (the `_rtl` calls above), skipping the
> remaining steps.

