# Delivery Drone – Pi-Code

Companion-computer control code for the automated delivery of objects with an FPV
drone. It runs in testing against **SITL** (simulated flight controller) and
later, unchanged, on the **Raspberry Pi** connected to the real flight controller.
The only difference is the `connection_string` in `config.py`.

## Project structure

| File             | Task                                                        |
|------------------|-------------------------------------------------------------|
| `config.py`      | Connection + all parameters (SITL vs. Pi in one place)      |
| `drone.py`       | Heartbeat, telemetry, basic commands, payload release       |
| `camera.py`      | Camera interface: `MockCamera`, `ScriptedCamera`, `SimCamera`, `TimedCamera`, `RealCamera` (IMX500) |
| `failsafe.py`    | Link loss, telemetry loss, battery, phase timeout, geofence |
| `mission.py`     | State machine — GPS path + indoor SEARCH/APPROACH path      |
| `release.py`     | Drop mechanism: `FcServo` (servo on FC) / `PiServo` (servo on Pi GPIO) |
| `search.py`      | Search patterns (expanding spiral / lawnmower) in local NED |
| `logbook.py`     | Logging setup: console + timestamped file under `logs/`     |
| `main.py`        | Entry point, wires everything together                      |
| `tests/`         | `pytest` unit tests for the mission logic (no SITL needed)  |
| `params/`        | FC parameter baseline (capture/restore) — see `params/README.md` |
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
[FAILSAFE] Safety envelope: climb<=50.0 cm/s, FENCE_ACTION=2 (2=Always Land), RTL_ALT=2.0 m
[FAILSAFE] Geofence enabled (type=1, alt_max=4.0 m)
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
2. Point the script at it – in `main.py`:
   ```python
   config = Config.sitl(port=14551)
   ```

Now the ground station (14550) shows the live map while the script (14551) flies
the mission.

## Just checking the link

Before running the full mission you can test the link:

```bash
python main.py --tele
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

## From SITL to the real drone

**Nothing in the code changes** — and the *default* is the real aircraft:

```bash
python main.py            # REAL AIRCRAFT (via mavlink-router)
python main.py --sim      # simulation against SITL
```

> **Why the real drone is the default.** Forgetting a flag must fail on the safe side,
> and the two mistakes are not equally bad. Simulation values on a real aircraft fail
> **silently**: `battery_min_voltage` would be 10.8 V — *below* a 4S Li-Ion's empty
> voltage of 11.2 V — so the low-battery abort could never fire; the drop would be
> commanded on an FC output that carries no servo; and `SimCamera` would report a target
> that does not exist, so the aircraft flies to an empty spot and drops there. Real
> values in simulation fail **immediately and harmlessly**: `PiServo` raises because a
> Mac has no GPIO. So the dangerous direction is the one that needs the flag.

Note that the Pi link is **UDP, not serial**: `mavlink-router` owns `/dev/serial0` and
forwards the FC stream to `127.0.0.1:14550`, so the script binds to the same endpoint it
uses against SITL. Use `--pi-serial` only if no router is running.

The full transition — wiring, FC params, EKF/pre-arm differences, **camera axis
mapping**, drop-servo calibration, link-loss / `FS_GCS`, and the safety checklist — is
documented in **[docs/SIM_TO_REAL.md](docs/SIM_TO_REAL.md)**. For how the components fit
together and the mission flow (with diagrams), see **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**.