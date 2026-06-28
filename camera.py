"""
camera.py
=========
Mock of the camera interface. What matters here is not the implementation but
the CONTRACT (the data structure). Your colleague later replaces only the body
of get_target_offset() with real inference (AI camera module / IMX500). The
signature and the returned dict stay the same -> the mission logic needs no
changes.

Return format:
    {
        "detected": bool,    # target detected in the image?
        "dx": float,         # horizontal offset from image centre (m or normalised)
        "dy": float,         # vertical offset from image centre
        "distance": float,   # estimated distance to target in metres
    }
"""

import math
import time
from typing import Protocol


class Camera(Protocol):
    """Interface the mission programs against. Both implementations (mock now,
    real later) satisfy this protocol type."""

    def get_target_offset(self) -> dict:
        ...


class MockCamera:
    """Returns fixed values for now: target detected dead centre, 1.5 m away.
    This lets the mission logic run through without any real hardware."""

    def __init__(self, detected: bool = True, dx: float = 0.0, dy: float = 0.0, distance: float = 1.5):
        self._detected = detected
        self._dx = dx
        self._dy = dy
        self._distance = distance

    def get_target_offset(self) -> dict:
        return {
            "detected": self._detected,
            "dx": self._dx,
            "dy": self._dy,
            "distance": self._distance,
        }


class ScriptedCamera:
    """Replays a fixed sequence of offset frames, one per call. Used to exercise
    the OVER_TARGET correction loop in tests and SITL dry-runs: feed a few
    off-centre frames that converge to the centre and watch the mission nudge the
    drone and then drop. After the script is exhausted it keeps returning the last
    frame (typically the centred one), so the mission proceeds deterministically.

    Each frame is a dict with the same keys as MockCamera:
        {"detected": bool, "dx": float, "dy": float, "distance": float}
    """

    def __init__(self, frames: list[dict]):
        if not frames:
            raise ValueError("ScriptedCamera needs at least one frame")
        self._frames = frames
        self._i = 0

    def get_target_offset(self) -> dict:
        frame = self._frames[min(self._i, len(self._frames) - 1)]
        self._i += 1
        return frame


class SimCamera:
    """Simulated detector for SITL and tests. Pretends a target sits at a fixed local
    NED position and "detects" it once the drone is within `fov_radius` (ground
    distance). Returns the ground offset to the target as dx (east error) / dy (north
    error), so the same APPROACH / OVER_TARGET servo loop drives the drone onto it.

    This lets the whole search → approach → drop flow be validated before the real AI
    camera exists. `drone` only needs a `get_local_position()` returning
    `{"north": .., "east": ..}` — the real Drone and the test FakeDrone both provide it.
    """

    def __init__(self, drone, target_north: float, target_east: float, fov_radius: float = 1.5):
        self._drone = drone
        self._tn = target_north
        self._te = target_east
        self._fov = fov_radius

    def get_target_offset(self) -> dict:
        pos = self._drone.get_local_position()
        if not pos:
            return {"detected": False, "dx": 0.0, "dy": 0.0, "distance": float("inf")}
        dn = self._tn - pos["north"]
        de = self._te - pos["east"]
        dist = math.hypot(dn, de)
        detected = dist <= self._fov
        return {
            "detected": detected,
            "dx": de if detected else 0.0,  # east error  -> body "right"
            "dy": dn if detected else 0.0,  # north error -> body "forward"
            "distance": dist,
        }


class TimedCamera:
    """Pretends to "find" the target (centred) after `detected_after_s` seconds. There is
    NO real detection - it lets a real flight exercise the search-pattern flight and the
    drop mechanism without the AI camera (Phase 3 / camera-less flight test).

    The timer starts on the first call (i.e. when SEARCH first polls), not at
    construction, so takeoff time does not count.
    """

    def __init__(self, detected_after_s: float = 20.0):
        self._after = detected_after_s
        self._start: float | None = None

    def get_target_offset(self) -> dict:
        if self._start is None:
            self._start = time.monotonic()
        found = (time.monotonic() - self._start) >= self._after
        # Centred (dx=dy=0): APPROACH drops immediately where the drone currently is.
        return {"detected": found, "dx": 0.0, "dy": 0.0, "distance": 0.0}