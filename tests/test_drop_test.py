"""Hardware-independent checks for the disarmed camera/drop bench test."""

import logging

import pytest

from drop_test import RELEASE_HOLD_S, wait_for_target_and_drop


class FakeCamera:
    def __init__(self, detections):
        self.detections = iter(detections)
        self.events = []

    def start(self):
        self.events.append("camera_start")

    def get_detection(self):
        self.events.append("detect")
        return next(self.detections)

    def stop(self):
        self.events.append("camera_stop")


class FakeServo:
    def __init__(self):
        self.events = []

    def setup(self):
        self.events.append("setup_neutral")

    def drop(self):
        self.events.append("drop")

    def reset(self):
        self.events.append("reset")


def test_detection_releases_for_three_seconds_then_resets(caplog):
    camera = FakeCamera([
        {"detected": False, "confidence": 0.2, "label": "pad"},
        {"detected": True, "confidence": 0.9, "label": "pad"},
    ])
    servo = FakeServo()
    sleeps = []

    with caplog.at_level(logging.INFO):
        dropped = wait_for_target_and_drop(
            camera, servo, timeout=10, interval=0.5,
            threshold=0.7,
            sleep=sleeps.append, monotonic=lambda: 0.0,
        )

    assert dropped is True
    assert servo.events == ["setup_neutral", "drop", "reset"]
    assert sleeps == [0.5, RELEASE_HOLD_S]
    assert camera.events == ["camera_start", "detect", "detect", "camera_stop"]
    camera_reports = [r.message for r in caplog.records if "CAMERA |" in r.message]
    assert len(camera_reports) == 2
    assert "NOT FOUND | confidence=0.200" in camera_reports[0]
    assert "FOUND | confidence=0.900" in camera_reports[1]


def test_exception_during_release_still_resets_and_stops_camera():
    camera = FakeCamera([
        {"detected": True, "confidence": 0.9, "label": "pad"},
    ])
    servo = FakeServo()

    def interrupted_sleep(_seconds):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        wait_for_target_and_drop(
            camera, servo, timeout=10, interval=0.5,
            threshold=0.7,
            sleep=interrupted_sleep, monotonic=lambda: 0.0,
        )

    assert servo.events == ["setup_neutral", "drop", "reset"]
    assert camera.events[-1] == "camera_stop"
