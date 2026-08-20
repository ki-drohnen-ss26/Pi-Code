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

import logging
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


class RealCamera:
    """Raspberry Pi AI Camera (Sony IMX500) running the landing-pad detector.

    The neural network runs **on the sensor's own NPU**, not on the Pi's CPU. That is
    the whole reason this module exists: a Pi Zero 2 W cannot run YOLO on its CPU at a
    usable rate, and its four cores are already busy with MAVLink and the mission state
    machine. The IMX500 delivers finished detections as frame *metadata*, so the Pi only
    reads a small result structure per frame.

    Two design decisions are worth stating explicitly, because both are easy to get
    wrong and neither is visible from the outside:

    **1. Why this returns METRES, not image fractions.**
    A camera natively reports "the pad is 30 % of the frame to the right". `SimCamera`,
    against which the whole mission was validated in SITL, reports *ground metres*. The
    mission's tuning (`centre_tolerance`, `approach_gain`, `max_nudge_m`) is calibrated
    for metres. If this class returned image fractions instead, every one of those
    numbers would silently change meaning and the SITL validation would no longer carry
    over — which defeats the project's core principle that the flight code is identical
    in simulation and reality. So we convert here, once, using the pinhole relation:

        ground_offset = tan(image_fraction * half_FOV) * height_above_ground

    The height comes from the drone (rangefinder / relative altitude). Without a height
    there is no way to turn an angle into a distance, so we fall back to the configured
    search altitude and say so in the log.

    **2. Why the axis mapping is configuration, not code.**
    `docs/SIM_TO_REAL.md` §3 requires the image→body axis mapping to be *calibrated* on
    the real airframe, because it depends on how the camera is physically rotated in its
    mount. Hard-coding it would mean editing source on a drone between test flights. The
    four `cam_*` settings in `config.py` cover every 90° mounting and both sign
    conventions, so calibration is a parameter change.
    """

    def __init__(self, config, drone=None, model_path: str | None = None):
        self._c = config
        self._drone = drone
        self._model_path = model_path or config.camera_model_path
        self._imx500 = None
        self._picam2 = None
        self._labels: list[str] = []
        self._warned_no_height = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        """Load the model onto the sensor and start the camera.

        Kept out of __init__ so that constructing the object (and importing this
        module) works on a machine without picamera2 - the Mac runs the same file.
        """
        try:
            from picamera2 import Picamera2
            from picamera2.devices import IMX500
        except ImportError as exc:
            raise RuntimeError(
                "RealCamera needs picamera2 (Raspberry Pi only). Install it on the Pi: "
                "sudo apt install python3-picamera2 imx500-all. For SITL use "
                "config.camera_source='sim' or 'timed'."
            ) from exc

        # Uploading the .rpk to the sensor takes ~45 s the first time a given model is
        # selected. Doing that lazily inside get_target_offset() would stall the mission
        # mid-flight, so it happens here, on the ground.
        log = logging.getLogger(__name__)
        log.info(f"[CAM] Loading model {self._model_path} onto the IMX500 ...")
        self._imx500 = IMX500(self._model_path)
        self._picam2 = Picamera2(self._imx500.camera_num)
        cfg = self._picam2.create_preview_configuration(
            controls={"FrameRate": self._c.camera_fps}, buffer_count=4
        )
        # show_preview=False: the Pi is headless. A DRM preview fails with
        # "Failed to reserve DRM plane" over SSH.
        self._picam2.start(cfg, show_preview=False)

        if self._c.camera_labels_path:
            try:
                with open(self._c.camera_labels_path, "r", encoding="utf-8") as fh:
                    self._labels = [line.strip() for line in fh if line.strip()]
                log.info(f"[CAM] {len(self._labels)} labels loaded")
            except OSError as exc:
                log.warning(f"[CAM] Could not read labels: {exc}")

        # Let auto-exposure/white-balance settle. The first frames after start are
        # dark or washed out and produce spurious (non-)detections.
        time.sleep(self._c.camera_warmup_s)
        log.info("[CAM] Camera ready")

    def stop(self) -> None:
        if self._picam2 is not None:
            try:
                self._picam2.stop()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------
    def get_target_offset(self) -> dict:
        """One frame -> the mission's {detected, dx, dy, distance} contract."""
        log = logging.getLogger(__name__)
        if self._picam2 is None:
            raise RuntimeError("RealCamera.start() was never called")

        metadata = self._picam2.capture_metadata()
        detection = self._best_detection(metadata)
        if detection is None:
            return {"detected": False, "dx": 0.0, "dy": 0.0, "distance": float("inf")}

        x0, y0, x1, y1, score, class_id = detection

        # Detection centre as a fraction of the frame, measured from the centre:
        # -1 = left/top edge, +1 = right/bottom edge.
        frac_x = (x0 + x1) - 1.0
        frac_y = (y0 + y1) - 1.0

        height = self._height_above_ground()
        east_like = self._angle_to_ground(frac_x, self._c.cam_hfov_deg, height)
        north_like = self._angle_to_ground(frac_y, self._c.cam_vfov_deg, height)

        dx, dy = self._map_axes(east_like, north_like)
        label = (self._labels[class_id] if 0 <= class_id < len(self._labels)
                 else str(class_id))
        log.info(f"[CAM] '{label}' p={score:.2f} -> dx={dx:+.2f} dy={dy:+.2f} m "
                 f"(h={height:.2f} m)")
        return {"detected": True, "dx": dx, "dy": dy, "distance": height}

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _best_detection(self, metadata):
        """Highest-confidence detection above the threshold, or None.

        Returns (x0, y0, x1, y1, score, class_id) with coordinates normalised to
        [0, 1]. Only the wanted class counts when `camera_target_class` is set - a
        model trained on several classes must not send the drone after the wrong one.
        """
        log = logging.getLogger(__name__)
        outputs = self._imx500.get_outputs(metadata, add_batch=True)
        if outputs is None:
            return None   # no inference result in this frame yet - normal right after start

        # Ultralytics' IMX export emits three tensors: boxes, scores, classes (NMS
        # already applied on the sensor). Guard the shape rather than trusting it: a
        # differently exported model would otherwise fail deep inside numpy with an
        # error that says nothing about the actual cause.
        if len(outputs) < 3:
            log.warning(f"[CAM] Unexpected model output ({len(outputs)} tensors, "
                        f"expected >=3: boxes, scores, classes). The .rpk was probably "
                        f"exported with different post-processing - see docs.")
            return None

        boxes, scores, classes = outputs[0][0], outputs[1][0], outputs[2][0]
        best = None
        for box, score, class_id in zip(boxes, scores, classes):
            if score < self._c.camera_confidence:
                continue
            class_id = int(class_id)
            if (self._c.camera_target_class is not None
                    and class_id != self._c.camera_target_class):
                continue
            if best is None or score > best[4]:
                y0, x0, y1, x1 = (float(v) for v in box)   # IMX500 order: y,x,y,x
                best = (x0, y0, x1, y1, float(score), class_id)
        return best

    def _height_above_ground(self) -> float:
        """Height used to scale image angles into ground metres."""
        log = logging.getLogger(__name__)
        if self._drone is not None:
            try:
                pos = self._drone.get_local_position()
                if pos and pos.get("down") is not None and -pos["down"] > 0.05:
                    return -pos["down"]      # NED: down is negative when airborne
            except Exception:
                pass
        if not self._warned_no_height:
            self._warned_no_height = True
            log.warning(
                f"[CAM] No usable altitude - scaling offsets with the configured "
                f"search altitude ({self._c.search_altitude} m). Corrections will be "
                f"off by the ratio of assumed to real height."
            )
        return self._c.search_altitude

    @staticmethod
    def _angle_to_ground(fraction: float, fov_deg: float, height: float) -> float:
        """Image fraction (-1..+1 from centre) -> ground offset in metres."""
        return math.tan(math.radians(fraction * fov_deg / 2.0)) * height

    def _map_axes(self, image_right: float, image_down: float) -> tuple:
        """Apply the calibrated mounting rotation. See config.cam_* and
        docs/SIM_TO_REAL.md §3 for how to determine these."""
        if self._c.cam_swap_axes:
            image_right, image_down = image_down, image_right
        dx = image_right * (-1.0 if self._c.cam_invert_x else 1.0)
        dy = image_down * (-1.0 if self._c.cam_invert_y else 1.0)
        return dx, dy