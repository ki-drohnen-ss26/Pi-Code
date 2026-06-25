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