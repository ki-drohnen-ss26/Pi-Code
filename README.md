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
| `camera.py`      | Camera interface: `MockCamera` + `ScriptedCamera` (replay offsets) |
| `failsafe.py`    | Battery threshold, phase timeout, geofence                  |
| `mission.py`     | State machine IDLE→TAKEOFF→ENROUTE→OVER_TARGET→DROP→RTL      |
| `logbook.py`     | Logging setup: console + timestamped file under `logs/`     |
| `main.py`        | Entry point, wires everything together                      |
| `tests/`         | `pytest` unit tests for the mission logic (no SITL needed)  |
| `params/`        | FC parameter baseline (capture/restore) — see `params/README.md` |
| `requirements.txt` | Pinned dependencies (pymavlink, pytest)                   |
| `docs/ARCHITECTURE.md` | Components, relationships & mission flow (Mermaid diagrams) |
| `docs/SIM_TO_REAL.md`  | Concrete SITL→hardware transition guide + safety checklist |
| `docs/ROADMAP.md`      | Phased development plan toward the real Pi + flight tests   |

> The indoor target is found by search (the pad position is not known in advance), so
> the state machine will grow `SEARCH` and `APPROACH` stages in Phase 2 — see
> `docs/ROADMAP.md`. The `Camera` interface already supports this (`detected`/`dx`/`dy`).

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
python main.py
```

By default `main.py` connects on port 14550 (`Config.sitl()`), which is the port
`sim_vehicle.py` outputs to.

### Expected output

```
[PREARM] Waiting for position estimate (GPS fix) ...
[PREARM] Position ready (...)
[MODE] Mode is now GUIDED
[ARM] Motors are armed
[TAKEOFF] Altitude reached (...)
[GOTO] Target reached (...)
[CAM] Centred (...)
[DROP] Read-back servo value: 1900 PWM
[DROP] Release confirmed
[RTL] Landed and disarmed
```

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

While the mission is running (ideally in the ENROUTE phase), type into the
MAVProxy console in Terminal 1:

```
param set SIM_BATT_VOLTAGE 10.5
```

The threshold is 10.8 V, so the `LOW_BATTERY` abort fires immediately and the
state machine switches to `ABORT → RTL`.

## From SITL to the real Pi

Conceptually **nothing in the code changes** except the endpoint in `main.py`:

```python
config = Config.pi_serial("/dev/serial0", baud=921600)
```

The full transition — wiring, FC params, EKF/pre-arm differences, **camera axis
mapping**, drop-servo calibration, link-loss / `FS_GCS`, and the safety checklist — is
documented in **[docs/SIM_TO_REAL.md](docs/SIM_TO_REAL.md)**. For how the components fit
together and the mission flow (with diagrams), see **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**.