"""
test_camera.py
==============
Unit tests for RealCamera's detection-to-metres pipeline. All of it is pure logic
(box decode, angle-to-ground conversion, mounting axis map), so it runs on the Mac
without picamera2 or an IMX500 - which matters, because this code otherwise only
ever executes in the air, where a wrong sign or scale flies the aircraft AWAY from
the pad instead of over it.

RealCamera.__init__ deliberately does not import picamera2 (that happens in
start()), so the object can be built here and fed a fake IMX500.
"""

import math

import pytest

from camera import RealCamera
from config import Config


class FakeImx500:
    """Stands in for picamera2.devices.IMX500: returns canned output tensors."""

    def __init__(self, boxes, scores, classes, input_size=(640, 640)):
        # get_outputs(add_batch=True) shape: outputs[i][0] is the tensor.
        self._outputs = [[boxes], [scores], [classes]]
        self._input_size = input_size

    def get_outputs(self, metadata, add_batch=True):
        return self._outputs

    def get_input_size(self):
        return self._input_size


def _camera(config=None, **fake_kwargs):
    config = config or Config()
    cam = RealCamera(config, drone=None)
    cam._imx500 = FakeImx500(**fake_kwargs)
    return cam, config


# ----------------------------------------------------------------------
# _best_detection: thresholding, class filter, box decode
# ----------------------------------------------------------------------
def test_best_detection_picks_highest_score_above_threshold():
    cam, config = _camera(
        boxes=[[0.1, 0.1, 0.3, 0.3], [0.4, 0.4, 0.6, 0.6], [0.0, 0.0, 0.9, 0.9]],
        scores=[0.6, 0.9, 0.3],          # third is below camera_confidence (0.5)
        classes=[0, 0, 0],
    )
    best = cam._best_detection({})
    assert best is not None
    assert best[4] == pytest.approx(0.9)
    # yxyx order (default): box [0.4, 0.4, 0.6, 0.6] -> x0=0.4, y0=0.4 ...
    assert best[0] == pytest.approx(0.4)


def test_best_detection_filters_on_target_class():
    config = Config()
    config.camera_target_class = 2
    cam, _ = _camera(
        config,
        boxes=[[0.1, 0.1, 0.3, 0.3], [0.4, 0.4, 0.6, 0.6]],
        scores=[0.9, 0.8],
        classes=[0, 2],                  # the higher score is the WRONG class
    )
    best = cam._best_detection({})
    assert best is not None and best[5] == 2
    assert best[4] == pytest.approx(0.8)


def test_best_detection_returns_none_without_inference_result():
    cam, _ = _camera(boxes=[], scores=[], classes=[])
    cam._imx500.get_outputs = lambda metadata, add_batch=True: None
    assert cam._best_detection({}) is None


# ----------------------------------------------------------------------
# _decode_box: coordinate order and pixel-vs-normalised scale
# ----------------------------------------------------------------------
def test_decode_box_default_order_is_yxyx():
    cam, _ = _camera(boxes=[], scores=[], classes=[])
    x0, y0, x1, y1 = cam._decode_box([0.2, 0.1, 0.6, 0.5])   # y0, x0, y1, x1
    assert (x0, y0, x1, y1) == pytest.approx((0.1, 0.2, 0.5, 0.6))


def test_decode_box_xyxy_order_for_ultralytics_exports():
    config = Config()
    config.cam_box_order = "xyxy"
    cam, _ = _camera(config, boxes=[], scores=[], classes=[])
    x0, y0, x1, y1 = cam._decode_box([0.1, 0.2, 0.5, 0.6])
    assert (x0, y0, x1, y1) == pytest.approx((0.1, 0.2, 0.5, 0.6))


def test_decode_box_normalises_pixel_coordinates():
    """Ultralytics `format=imx` exports emit input-tensor PIXELS. A pixel-valued box
    cannot be repaired by the cam_* mounting calibration - it mis-scales every
    correction - so the decode normalises it against the model's input size."""
    config = Config()
    config.cam_box_order = "xyxy"
    cam, _ = _camera(config, boxes=[], scores=[], classes=[], input_size=(640, 640))
    x0, y0, x1, y1 = cam._decode_box([64.0, 128.0, 320.0, 384.0])
    assert (x0, y0, x1, y1) == pytest.approx((0.1, 0.2, 0.5, 0.6))


# ----------------------------------------------------------------------
# _angle_to_ground: the pinhole conversion the whole metre contract rests on
# ----------------------------------------------------------------------
def test_angle_to_ground_centre_is_zero():
    assert RealCamera._angle_to_ground(0.0, 66.0, 2.0) == 0.0


def test_angle_to_ground_full_edge_hits_half_fov():
    # fraction=1.0 means the full half-FOV angle: tan(33 deg) * 2 m
    expected = math.tan(math.radians(33.0)) * 2.0
    assert RealCamera._angle_to_ground(1.0, 66.0, 2.0) == pytest.approx(expected)


def test_angle_to_ground_scales_linearly_with_height():
    one = RealCamera._angle_to_ground(0.5, 66.0, 1.0)
    two = RealCamera._angle_to_ground(0.5, 66.0, 2.0)
    assert two == pytest.approx(2.0 * one)


# ----------------------------------------------------------------------
# _map_axes: every mounting flag must do exactly what its name says
# ----------------------------------------------------------------------
def test_map_axes_identity_by_default():
    cam, _ = _camera(boxes=[], scores=[], classes=[])
    assert cam._map_axes(0.3, -0.2) == (0.3, -0.2)


def test_map_axes_inversions_and_swap():
    config = Config()
    config.cam_invert_x = True
    cam, _ = _camera(config, boxes=[], scores=[], classes=[])
    assert cam._map_axes(0.3, -0.2) == (-0.3, -0.2)

    config = Config()
    config.cam_invert_y = True
    cam, _ = _camera(config, boxes=[], scores=[], classes=[])
    assert cam._map_axes(0.3, -0.2) == (0.3, 0.2)

    config = Config()
    config.cam_swap_axes = True
    cam, _ = _camera(config, boxes=[], scores=[], classes=[])
    assert cam._map_axes(0.3, -0.2) == (-0.2, 0.3)


# ----------------------------------------------------------------------
# _height_above_ground: fallback must warn-and-assume, never crash
# ----------------------------------------------------------------------
def test_height_falls_back_to_search_altitude_without_a_drone():
    cam, config = _camera(boxes=[], scores=[], classes=[])
    assert cam._height_above_ground() == config.search_altitude


def test_height_uses_the_drone_when_airborne():
    class FakePositionDrone:
        def get_local_position(self, timeout=2.0):
            return {"north": 0.0, "east": 0.0, "down": -1.7}   # NED: 1.7 m up

    config = Config()
    cam = RealCamera(config, drone=FakePositionDrone())
    assert cam._height_above_ground() == pytest.approx(1.7)
