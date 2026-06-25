# Delivery Drone – Companion Code

Control code for the automated delivery of objects with an FPV drone. It runs in
testing against **SITL** (simulated flight controller) and later, unchanged, on
the **Raspberry Pi** connected to the real flight controller. The only difference
is the `connection_string` in `config.py`.

## Project structure

| File           | Task                                                        |
|----------------|-------------------------------------------------------------|
| `config.py`    | Connection + all parameters (SITL vs. Pi in one place)      |
| `drone.py`     | Heartbeat, telemetry, basic commands, payload release       |
| `camera.py`    | Camera interface as a mock (`get_target_offset()`)          |
| `failsafe.py`  | Battery threshold, phase timeout, geofence                  |
| `mission.py`   | State machine IDLE→TAKEOFF→ENROUTE→OVER_TARGET→DROP→RTL      |
| `main.py`      | Entry point, wires everything together                      |

## Installation

```bash
conda activate ardupilot
pip install pymavlink
```

## Workflow: testing against SITL

You need **two terminals**, both with the conda environment activated. Gazebo is
not required for any of these steps – SITL alone covers the entire logic.

### 1. Start SITL (Terminal 1)

```bash
conda activate ardupilot
cd "Studium/Drohnen-mit-KI/Simulation/ardupilot"
Tools/autotest/sim_vehicle.py -v ArduCopter --console
```

The first start compiles the SITL binary (takes a few minutes). `--map` is
deliberately omitted – the MAVProxy map module is not installed under macOS and
would only throw an error. We use QGroundControl for the map instead (see below).

Wait until the console shows something like "EKF3 IMU0 is using GPS" / "Ready to
Fly". Only then are the pre-arm checks (GPS/EKF position) satisfied.

### 2. Add a second output port (Terminal 1, MAVProxy console)

`sim_vehicle.py` outputs MAVLink only on **14550** by default. So that
QGroundControl (14550) and our script (14551) can run at the same time, add a
second port at the `STABILIZE>` prompt:

```
output add 127.0.0.1:14551
```

### 3. Open QGroundControl

QGC connects automatically on port 14550 to the running SITL and shows the map,
artificial horizon and status messages.

### 4. Start the script (Terminal 2)

```bash
conda activate ardupilot
cd <path>/delivery_drone
python main.py
```

`main.py` is configured for port 14551 (`Config.sitl(port=14551)`).

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

In QGC you see the drone take off, fly to the target and return, all live.

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

Conceptually **nothing in the code changes**, only the endpoint the script
connects to.

**Physical connection:** the Pi connects via its UART (GPIO TX/RX) to the
flight controller's TELEM port – TX↔RX crossed, common ground (GND).
Alternatively a USB-UART adapter to the FC.

**On the flight controller** (once, e.g. via QGroundControl):
- `SERIALx_PROTOCOL = 2` (MAVLink2) for the port the Pi is on
- `SERIALx_BAUD` matching the Pi side (e.g. 921 = 921600)

**On the Pi:**
- Enable the UART in `raspi-config`, disable the serial login console on that
  port so that `/dev/serial0` is free.

**In the code** – just this one line in `main.py`:
```python
config = Config.pi_serial("/dev/serial0", baud=921600)
```

### What is different from SITL
- **Pre-arm checks are real:** GPS fix, EKF, compass must be healthy. `arm()`
  retries several times with a pause instead of giving up at once or blocking.
- **Indoors without GPS:** `goto()` uses global coordinates and needs GPS.
  Indoors the MTF-01P provides the position via optical flow + LiDAR. Then you
  fly via `SET_POSITION_TARGET_LOCAL_NED` in the local NED frame instead of
  lat/lon, and `wait_ready_to_arm()` checks EKF readiness instead of the GPS fix.
  This is the later adaptation tied to Task 4.
- **Data rates:** over the serial link telemetry streams are often slower;
  `request_data_streams()` therefore sets a fixed rate.

## Safety (real hardware)
- First tests **without propellers**.
- Always keep an independent kill switch on the transmitter (mode switch /
  disarm). The companion failsafe does not replace it.
- Also configure ArduPilot's own failsafes (`BATT_LOW_VOLT`, `FS_*`).