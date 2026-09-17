"""
fctools.py
==========
Shared flight-controller helpers for the STANDALONE command-line tools
(preflight.py, fclog.py, setparam.py): wait for a real autopilot heartbeat, and
read/write a single parameter with read-back. One canonical copy of each, so a fix to
the heartbeat-guard or the read-back retry loop lands in one place instead of three.

WHY THESE LIVE OUTSIDE drone.py
-------------------------------
drone.py is the mission stack: importing it pulls in the state machine, the failsafe
monitor, the camera and the release mechanism. The diagnostic CLIs must run on a bare
Pi (or a laptop) with nothing but pymavlink, precisely to answer "is the FC even
talking, and what does it say its parameters are" when the mission itself will NOT
start. Reaching into Drone for wait_for_vehicle() would drag the whole flight stack
behind three tools whose entire job is to work when that stack does not - so these
three helpers live here as the standalone tools' shared, dependency-free copy. drone.py
keeps its own equivalents, tuned for the mission; these are deliberately separate.

Each function takes tries/timeout as arguments so a caller can tune how hard it leans on
a flaky link; the defaults are the values the tools have used all along.
"""

import time

from pymavlink import mavutil


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


def read_param(master, name, tries=4, timeout=2.0):
    """Read one parameter, retrying the request. Returns the value or None."""
    for _ in range(tries):
        master.mav.param_request_read_send(
            master.target_system, master.target_component, name.encode(), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg and msg.param_id == name:
                return msg.param_value
    return None


def set_param(master, name, value, tries=4, timeout=2.0):
    """Write, then read back. Returns the confirmed value or None."""
    for _ in range(tries):
        master.mav.param_set_send(
            master.target_system, master.target_component, name.encode(),
            float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg and msg.param_id == name:
                return msg.param_value
    return None
