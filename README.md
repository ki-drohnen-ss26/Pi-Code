# Delivery Drone – Pi-Code

Companion-computer control code for the automated delivery of objects with an FPV
drone. It runs in testing against **SITL** (simulated flight controller) and
later, unchanged, on the **Raspberry Pi** connected to the real flight controller.
The flight code is identical. The dataclass defaults in `config.py` **are** the
real-aircraft configuration; besides the endpoint, the simulation deviates in exactly
three named ways — the drop servo (`release_mechanism`), the battery threshold
(`battery_min_voltage`) and the camera source (`camera_source`) — and all three are
confined to `Config.sitl()`.

## Project structure

| File             | Task                                                        |
|------------------|-------------------------------------------------------------|
| `config.py`      | Connection + all parameters (SITL vs. Pi in one place)      |
| `drone.py`       | Heartbeat, telemetry, basic commands, payload release       |
| `camera.py`      | Camera interface: `MockCamera`, `ScriptedCamera`, `SimCamera`, `TimedCamera`, `RealCamera` (IMX500) |
| `failsafe.py`    | Link loss, telemetry loss, battery, phase timeout, read-only fence check (`verify_fence_disabled`) |
| `mission.py`     | State machine — GPS path + indoor SEARCH/APPROACH path      |
| `release.py`     | Drop mechanism: `FcServo` (servo on FC) / `PiServo` (servo on Pi GPIO) |
| `search.py`      | Search patterns (expanding spiral / lawnmower) in local NED |
| `logbook.py`     | Logging setup: console + timestamped file under `logs/`     |
| `main.py`        | Entry point, wires everything together                      |
| `preflight.py`   | Read-only FC inspection: parameters, EKF flags, sensor health, EKF-altitude drift check, live `STATUSTEXT`. Also **verifies the live FC against the published flight set** (`params/flight_v2.param`, `--expected PATH` to override) and reports every mismatch — the companion no longer writes FC parameters (team decision 2026-08-24), it checks them. Run it, then try to arm — the autopilot's own objection appears verbatim |
| `setparam.py`    | Set FC parameters with read-back verification (`python setparam.py NAME VALUE [--reboot]`). Refuses to run while armed |
| `fclog.py`       | Continuous FC recorder (own router endpoint). Logs every `STATUSTEXT`, mode change, arm/disarm and EKF-flag transition, plus a "why did it land" dump before each disarm |
| `tests/`         | `pytest` unit tests for the mission logic (no SITL needed)  |
| `params/`        | The published, versioned flight parameter set (`flight_v2.param`) that **owns** the FC configuration, plus the generated SITL mirror (`sitl_flight_v2.parm`) and the recovered real-FC baseline — see `params/README.md` |
| `requirements.txt` | Pinned dependencies (pymavlink, pytest)                   |
| `docs/ARCHITECTURE.md` | Components, relationships & mission flow (Mermaid diagrams) |
| `docs/SIM_TO_REAL.md`  | Concrete SITL→hardware transition guide + safety checklist |
| `docs/ROADMAP.md`      | Phased development plan toward the real Pi + flight tests   |

> The indoor target position is not known in advance, so the state machine has a
> `SEARCH` stage (fly a pattern, look with the camera) and an `APPROACH` stage (visual
> servoing onto a detected target). `config.gps_denied` selects the indoor path
> (default) vs the GPS path — see `docs/ARCHITECTURE.md` and `docs/ROADMAP.md`.

## Prerequisites

1. **A working SITL simulation.** Setting it up is documented in the **ProjectDocs**
   repo under `docs/software/SetupSimulation.md`
   ([link](https://github.com/ki-drohnen-ss26/project-docs/blob/main/docs/software/SetupSimulation.md)).
   That guide is written for Windows/WSL; on macOS or Linux you can run SITL
   natively and skip the WSL section – the `sim_vehicle.py` commands are the same.
2. **Python with pymavlink.** Pi-Code has its own lightweight environment, separate
   from the heavier `ardupilot` (SITL) env. It mirrors the real Pi runtime — the Pi
   later uses the same `requirements.txt` with plain `pip` (no conda):
   ```bash
   conda create -n ki_drohnen_pi python=3.11 -y
   conda activate ki_drohnen_pi
   pip install -r requirements.txt
   ```
   Any environment with `pymavlink` works, but keeping Pi-Code's deps minimal and
   explicit is the point. Run SITL from the `ardupilot` env (Terminal 1) and the
   Pi-Code from `ki_drohnen_pi` (Terminal 2) — they are separate processes.

## Running the unit tests (no SITL, no hardware)

The mission logic is covered by `pytest` tests that run in well under a second using a
`FakeDrone` stand-in — no SITL and no flight controller required. Run them from this
folder:

```bash
pip install -r requirements.txt   # once
pytest
```

They drive the full state machine through the happy path and through failsafe/arming
aborts, so they catch regressions before any simulator or hardware is involved.

## Workflow: testing against SITL

You need **two terminals**, both with your Python environment activated. Gazebo is
not required – SITL alone covers the entire logic.

### 1. Start SITL (Terminal 1)

Start SITL as described in `SetupSimulation.md`, for example:

```bash
sim_vehicle.py -v ArduCopter --console
```

Wait until the console shows something like "EKF3 IMU0 is using GPS" / "Ready to
Fly". Only then are the pre-arm checks (GPS/EKF position) satisfied.

> **macOS tip:** if `--map` throws `No module named 'map'`, the MAVProxy map
> module isn't installed. Just leave `--map` off and use a ground station for the
> map (see "Live map" below).

### 2. Run the Pi-Code (Terminal 2)

```bash
cd Pi-Code
python main.py --sim
```

> **`--sim` is not optional.** Without it `main.py` runs the **real-aircraft** profile
> (see "From SITL to the real drone" below). `--sim` connects on port 14550, which is
> the port `sim_vehicle.py` outputs to, and switches the drop servo, the battery
> threshold and the camera to their simulation values. Every run prints which profile it
> picked, and the simulation banner is deliberately loud.

> **GPS vs indoor path & camera.** `config.gps_denied` defaults to **True**, so `main.py`
> runs the indoor SEARCH/APPROACH path. The camera is chosen by `config.camera_source`
> (`auto` → `SimCamera` indoor; `timed` = camera-less test; `mock` = always centred). For
> a plain GPS SITL run (Phase 1), set `gps_denied = False`. To validate the indoor path
> against SITL, configure SITL for optical flow first — see `docs/SIM_TO_REAL.md` §2.

### Expected output

Indoor run (default `gps_denied=True`):

```
[FC] ArduPilot flight software 4.6.3
[ORIGIN] EKF origin confirmed: lat=50.131196 lon=8.692972
[IDLE] FC parameters are Mission-Planner-owned (team decision 2026-08-24): the companion verifies them but does not write any. See params/README.md.
[PREARM] Waiting for relative (optical flow) EKF position estimate ...
[PREARM] EKF position estimate ready
[MODE] Mode is now GUIDED
[FAILSAFE] Watching for a mode change away from GUIDED
[ARM] Motors are armed
[TAKEOFF] Altitude reached (2.0 m), settling ...
[TAKEOFF] Altitude stable at 2.0 m
[SEARCH] spiral pattern, 25 waypoints, cadence=stop_and_look
[SEARCH] Target detected near (2.0, 2.0)
[APPROACH] Centred over target (dx=-0.08, dy=0.10)
[DROP] Release confirmed
[RECOVER] Landing here
[RECOVER] Landed and disarmed
```

Lines prefixed `[FC/...]` are messages from the autopilot itself (pre-arm rejections,
fence breaches, EKF failsafes), mirrored into the log with their severity. They are the
first place to look when something is refused — see
[docs/SIM_TO_REAL.md](docs/SIM_TO_REAL.md) §9.

## Live map with a ground station (optional)

To watch the drone on a map, use a ground station (QGroundControl or Mission
Planner). It connects automatically on port 14550 – the same port the script uses
by default. To run both at once, give the script its own port:

1. In the MAVProxy console (Terminal 1, `STABILIZE>` prompt), add a second output:
   ```
   output add 127.0.0.1:14551
   ```
2. Point the script at it:
   ```bash
   python main.py --sim --port 14551
   ```
   `--port` only takes effect together with `--sim` – on its own it does not select
   the simulation profile.

Now the ground station (14550) shows the live map while the script (14551) flies
the mission.

## Just checking the link

Before running the full mission you can test the link:

```bash
python main.py --sim --tele
```

Prints mode, armed state, position and battery for 15 s. If the values change,
the link is up.

## Testing the failsafe

While the mission is running (ideally mid-mission, e.g. SEARCH or ENROUTE), type into
the MAVProxy console in Terminal 1:

```
param set SIM_BATT_VOLTAGE 10.5
```

`Config.sitl()` uses a 10.8 V threshold (SITL's simulated pack sits at 12.6 V; the real
4S Li-Ion threshold of 12.8 V would trip instantly). The abort needs
`battery_low_samples` consecutive low readings — a single sag under load is not an empty
battery — and then the machine goes `ABORT → RECOVER`, which **lands** rather than
flying RTL. Indoors that matters: RTL first climbs to `RTL_ALT`.

Other things worth provoking in SITL:

```
mode LOITER          # simulates the pilot taking over -> MODE_CHANGED_LOITER,
                     # and the mission must then command NOTHING further
```

> **No Ctrl-C parameter-restore drill any more.** The companion no longer writes FC
> parameters (team decision 2026-08-24 — Mission Planner + `params/flight_v2.param` own
> them), so there is nothing to restore on exit and the old "kill mid-run, check the
> parameters were put back" drill is obsolete. The save/restore/backup machinery
> (`enforce_safety_envelope`, `restore_params()`, the `logs/fc_params_backup.json` mirror)
> was **deleted on 2026-08-25** — the flight companion now writes no FC flight parameter
> (its one remaining PARAM_SET is `FcServo`'s SITL-only `SERVO9_FUNCTION=0`), so there is no
> such drill and no opt-in flag to run it under.

## Staged bring-up on a new aircraft

The full mission has four unknowns at once — position hold, detector, search pattern,
release. Each milestone adds exactly **one**, so a failure names its own cause instead of
leaving four candidates open:

```bash
python main.py --milestone 1   # climb to 0.8 m, hold, land        → position hold
python main.py --milestone 2   # same + detector, logging only     → detector
python main.py --milestone 3   # fly the search pattern            → pattern
python main.py --milestone 4   # search + detect + centre, no drop → approach
python main.py --milestone 5   # the full delivery                 → release
```

Rehearse each one against SITL first (`--sim --milestone N`). The simulator has no
IMX500, so the simulated detector is substituted — and that substitution is logged,
because a milestone that silently used a different camera than the one it names would be
worse than no rehearsal.

**How to read the results:**

| Milestone | Pass condition |
|---|---|
| 1 | `[HOVER] Done. Worst horizontal drift: …` — a working optical-flow hold stays within tens of centimetres. A drifting one walks away steadily *while still looking like it is flying*, which is why this number matters more than "it hovered". |
| 2 | `[HOVER] Camera sees the target: dx=… dy=… m` with the pad below. Check the **sign**: pad to the drone's right must give a **positive `dx`**. Wrong sign → set `cam_invert_x`, or the aircraft will correct *away* from the pad. |
| 3 | The pattern is flown to the end and the run stops with `TARGET_NOT_FOUND`. That abort **is** the pass condition — there is no detector in the loop. |
| 4 | `[DROP] skip_drop is set — releasing NOTHING`, after the aircraft centred over the pad. |
| 5 | `[DROP] Release confirmed`. |

Milestone 3 flies the spiral, which reaches `search_max_radius_m` **plus one**
`search_step_m` — 7 m with the defaults. The geofence is **off** in the published flight
set (a barometric altitude fence indoors caused the 2026-08-21 crash — see
`docs/SIM_TO_REAL.md` §5c); the companion never writes fence parameters and only checks,
read-only, that no stale fence is armed. The only horizontal guard is the software check
`max_position_radius_m` (15 m), so size the hall (or the radius) for that.

## When something goes wrong: recording the flight controller

The reason for an unexplained landing is nearly always something the autopilot *said* —
`EKF variance`, `Battery failsafe`, `PreArm: ...`. Those are MAVLink `STATUSTEXT`
messages and are gone the moment nobody is listening. `main.py` mirrors them, but only
while a mission runs; the interesting failures happen while the pilot is flying manually
or after the script has exited.

```bash
python preflight.py     # snapshot + 20 s live listen. Try to arm while it runs.
python fclog.py         # continuous recorder, meant to be left running
```

`fclog.py` needs its own endpoint, because only one process can bind a UDP port and
`main.py` uses 14550. Add a second output to `/etc/mavlink-router/main.conf`:

```ini
[UdpEndpoint logger]
Mode = Normal
Address = 127.0.0.1
Port = 14551
```
then `sudo systemctl restart mavlink-router`. One FC stream, several independent
consumers — that is what the router is for.

Set `LOG_DISARMED = 1` on the flight controller as well: its own dataflash log is the
authoritative record and captures far more than MAVLink carries, but by default it only
records while armed — which excludes every pre-arm problem.

## Starting from the pilot's hands instead of the ground

```bash
python main.py --milestone 1 --takeover
```

The companion then **does not arm and does not take off**. It waits until the pilot has
flown the aircraft above `takeover_min_alt_m` (0.8 m) *and* the EKF reports a usable
position estimate, and only then asks for GUIDED.

Two reasons, and the second is not about preference:

1. It puts the human in charge of the riskiest moment.
2. A flow-only vehicle may not report `EKF_POS_HORIZ_REL` while it sits on the floor —
   optical flow needs height before the filter trusts it. A companion that insists on a
   position estimate *before* arming then deadlocks: no height without a takeoff, no
   takeoff without a position estimate. Flying the first metre by hand breaks the circle.

> **Throttle stick during the handover.** In GUIDED the stick is ignored — the autopilot
> controls throttle itself, so its position does not matter while the script flies. But
> the moment you switch back to a manual mode it takes effect **immediately**: centred
> gives you an altitude hold, at zero AltHold descends at full rate and Stabilize drops
> to idle. Keep it near centre, so taking over hands you a hover and not a fall.

## From SITL to the real drone

**Nothing in the code changes** — and the *default* is the real aircraft:

```bash
python main.py            # REAL AIRCRAFT (via mavlink-router)
python main.py --sim      # simulation against SITL
```

> **Why the real drone is the default.** Forgetting a flag must fail on the safe side,
> and the two mistakes are not equally bad. Simulation values on a real aircraft fail
> **silently**: `battery_min_voltage` would be 10.8 V — *below* a 4S Li-Ion's empty
> voltage of 11.2 V — so the low-battery **voltage** abort could never fire. Only the
> 20 % remaining-capacity arm of `battery_critical()` would be left, and that depends on
> the FC's current sensor and `BATT_CAPACITY` being right (it is skipped entirely when
> the FC reports `-1`). The drop would be commanded on an FC output that carries no
> servo, and `SimCamera` would report a target that does not exist, so the aircraft
> flies to an empty spot and drops there. Real
> values in simulation fail **immediately and harmlessly**: `PiServo` raises because a
> Mac has no GPIO. So the dangerous direction is the one that needs the flag.

Note that the Pi link is **UDP, not serial**: `mavlink-router` owns `/dev/serial0` and
forwards the FC stream to `127.0.0.1:14550`, so the script binds to the same endpoint it
uses against SITL. Use `--pi-serial` only if no router is running.

The full transition — wiring, FC params, EKF/pre-arm differences, **camera axis
mapping**, drop-servo calibration, link-loss / `FS_GCS`, and the safety checklist — is
documented in **[docs/SIM_TO_REAL.md](docs/SIM_TO_REAL.md)**. For how the components fit
together and the mission flow (with diagrams), see **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**.