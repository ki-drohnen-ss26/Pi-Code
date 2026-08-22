"""
fclog.py
========
Continuous flight-controller recorder. Runs alongside the mission (or entirely without
one) and writes everything the autopilot says about itself to a timestamped file.

    python fclog.py                    # real FC, second router endpoint (port 14551)
    python fclog.py --sim              # SITL
    python fclog.py --port 14552       # a different endpoint

WHY THIS EXISTS
---------------
When a flight ends in an unexplained landing, the reason is nearly always something the
autopilot *said* - "EKF variance", "Battery failsafe", "PreArm: ...". Those arrive as
MAVLink STATUSTEXT and are gone the moment nobody is listening. `main.py` mirrors them,
but only while a mission is running: the interesting failures happen while the pilot is
flying manually, between runs, or after the script has already exited.

So this listens ALL THE TIME and records:
  * every STATUSTEXT with its severity
  * every mode change and every arm/disarm transition
  * every change in the EKF status flags (the bits the mission gates on)
  * the rangefinder, optical-flow quality and battery, sampled once a second
  * a "why did it land" summary of the last events before a disarm

It changes nothing on the vehicle. It only requests data streams and listens.

RUNNING IT ALONGSIDE main.py
----------------------------
Only one process can bind a UDP port, and `main.py` uses 127.0.0.1:14550. Give the
logger its own endpoint by adding a second UDP output to mavlink-router
(/etc/mavlink-router/main.conf):

    [UdpEndpoint logger]
    Mode = Normal
    Address = 127.0.0.1
    Port = 14551

then `sudo systemctl restart mavlink-router`. That is exactly what a router is for -
one FC stream, several independent consumers.

To have it always running, install it as a systemd service (see docs/SIM_TO_REAL.md).
"""

import sys
import time
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil

SEVERITY = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
            4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}

EKF_FLAGS = [
    (0x01, "ATTITUDE"), (0x02, "VEL_HORIZ"), (0x04, "VEL_VERT"),
    (0x08, "POS_HORIZ_REL"), (0x10, "POS_HORIZ_ABS"), (0x20, "POS_VERT_ABS"),
    (0x40, "POS_VERT_AGL"), (0x80, "CONST_POS_MODE"),
    (0x100, "PRED_POS_HORIZ_REL"), (0x200, "PRED_POS_HORIZ_ABS"), (0x400, "UNINITIALISED"),
]



def wait_for_vehicle(master, timeout=30):
    """Wait for a heartbeat FROM THE AUTOPILOT, not from whatever speaks first.

    pymavlink's wait_heartbeat() returns on the first heartbeat of any kind. Over
    mavlink-router that can be a ground station or another tool, and then
    target_system stays 0 - every later parameter request goes to nobody and the
    script simply hangs. drone.py has guarded against this for a while; these
    diagnostic tools had not.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        master.wait_heartbeat(timeout=2)
        if master.target_system != 0:
            return True
    print("\nNo autopilot heartbeat - target_system stayed 0.")
    print("Something is on the link, but nothing that identifies itself as a vehicle.")
    print("Check that:")
    print("  * the flight controller is powered (USB or battery),")
    print("  * mavlink-router is running:  systemctl status mavlink-router")
    print("  * it actually sees the FC:    journalctl -u mavlink-router -n 20")
    return False

def describe_ekf(flags: int) -> str:
    return " ".join(name for bit, name in EKF_FLAGS if flags & bit) or "(none)"


class Recorder:
    """Writes to console and file at once, so a live watcher and the archive agree."""

    def __init__(self, path: Path):
        self.path = path
        self.file = open(path, "a", encoding="utf-8", buffering=1)  # line buffered
        # Line buffering matters: if the Pi loses power mid-flight, everything written
        # up to that instant has to already be on disk. A crash that eats its own log
        # is how the interesting failures stay unexplained.

    def write(self, tag: str, message: str) -> None:
        line = f"{datetime.now():%Y-%m-%d %H:%M:%S.%f}"[:-3] + f"  {tag:<9} {message}"
        print(line, flush=True)
        self.file.write(line + "\n")


def main() -> None:
    argv = sys.argv
    if "--sim" in argv or "--sitl" in argv:
        port = 14550
    elif "--port" in argv:
        port = int(argv[argv.index("--port") + 1])
    else:
        port = 14551          # the second router endpoint, so main.py keeps 14550

    log_dir = Path("logs")
    log_dir.mkdir(parents=True, exist_ok=True)
    rec = Recorder(log_dir / f"fclog_{datetime.now():%Y%m%d_%H%M%S}.log")
    rec.write("START", f"listening on udpin:127.0.0.1:{port}")

    master = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=253)
    wait_for_vehicle(master) or sys.exit(1)
    rec.write("LINK", f"autopilot system {master.target_system} is alive")
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    mode = None
    armed = None
    ekf = None
    last_sample = 0.0
    # Rolling window of the last events, printed when the vehicle disarms. A disarm you
    # did not command is the moment you want the preceding ten seconds of context, and
    # scrolling back through a live console to find it never works.
    recent: list = []
    rng = flow_q = volt = alt = None

    while True:
        msg = master.recv_match(blocking=True, timeout=2)
        if msg is None:
            continue
        kind = msg.get_type()

        if kind == "STATUSTEXT":
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", "replace")
            label = SEVERITY.get(msg.severity, str(msg.severity))
            rec.write(f"FC/{label}", text.strip())
            recent.append((time.time(), f"[{label}] {text.strip()}"))

        elif kind == "HEARTBEAT":
            new_mode = master.flightmode
            new_armed = master.motors_armed()
            if mode is not None and new_mode != mode:
                rec.write("MODE", f"{mode} -> {new_mode}")
                recent.append((time.time(), f"mode {mode} -> {new_mode}"))
            if armed is not None and new_armed != armed:
                rec.write("ARM", "ARMED" if new_armed else "DISARMED")
                if not new_armed:
                    # The disarm is the event; what came before it is the explanation.
                    rec.write("WHY", "--- last 15 s before this disarm ---")
                    cutoff = time.time() - 15
                    for when, what in recent:
                        if when >= cutoff:
                            rec.write("WHY", f"  -{time.time() - when:4.1f}s  {what}")
                    rec.write("WHY", f"  last sample: alt={alt} rng={rng} "
                                     f"flow_q={flow_q} volt={volt}")
                    rec.write("WHY", "--- end ---")
            mode, armed = new_mode, new_armed

        elif kind == "EKF_STATUS_REPORT":
            if msg.flags != ekf:
                # Only on CHANGE: at 4 Hz this would otherwise bury everything else,
                # and it is the transitions that explain a failure, not the steady state.
                rec.write("EKF", f"flags=0x{msg.flags:04x}  {describe_ekf(msg.flags)}")
                if ekf is not None:
                    lost = [n for b, n in EKF_FLAGS if (ekf & b) and not (msg.flags & b)]
                    if lost:
                        rec.write("EKF", f"  LOST: {', '.join(lost)}")
                        recent.append((time.time(), f"EKF lost {', '.join(lost)}"))
                ekf = msg.flags

        elif kind == "RANGEFINDER":
            rng = round(msg.distance, 2)
        elif kind in ("OPTICAL_FLOW", "OPTICAL_FLOW_RAD"):
            flow_q = getattr(msg, "quality", None)
        elif kind == "SYS_STATUS":
            volt = round(msg.voltage_battery / 1000.0, 2)
        elif kind == "GLOBAL_POSITION_INT":
            alt = round(msg.relative_alt / 1000.0, 2)

        now = time.time()
        if now - last_sample >= 1.0:
            last_sample = now
            # POS_HORIZ_REL and POS_VERT_AGL on every sample, not just on change: the
            # hand-lift test is exactly "at what height do these appear", and reading
            # that off a separate EKF line further up the scrollback does not work.
            rel = "-" if ekf is None else ("YES" if ekf & 0x08 else "no ")
            agl = "-" if ekf is None else ("YES" if ekf & 0x40 else "no ")
            rec.write("SAMPLE", f"mode={mode} armed={armed} alt={alt} m  "
                                f"rng={rng} m  flow_q={flow_q}  batt={volt} V  "
                                f"POS_HORIZ_REL={rel} POS_VERT_AGL={agl}")
        recent = [(w, m) for w, m in recent if w >= now - 60]


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
