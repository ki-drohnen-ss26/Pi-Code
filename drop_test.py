"""Disarmed camera-to-payload bench test for the Raspberry Pi.

The flight controller is never contacted. The GPIO servo starts in its neutral
position; when the IMX500 first recognises the configured target, the servo moves
to the release position for exactly three seconds and then returns to neutral.

Usage:
    python drop_test.py
    python drop_test.py --timeout 60 --interval 0.5
    python drop_test.py --timeout 0       # wait until detection or Ctrl-C
"""

import argparse
import logging
import time
from collections.abc import Callable

from camera import RealCamera
from config import Config
from logbook import setup_logging
from release import PiServo


log = logging.getLogger(__name__)
RELEASE_HOLD_S = 3.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Release the GPIO payload servo after IMX500 target detection, "
            "without contacting or arming the flight controller"
        )
    )
    parser.add_argument(
        "--timeout", type=float, default=30.0,
        help="seconds to wait for a target; 0 waits until Ctrl-C (default: 30)",
    )
    parser.add_argument(
        "--interval", type=float, default=0.5,
        help="seconds between camera polls (default: 0.5)",
    )
    parser.add_argument(
        "--confidence", type=float,
        help="temporary detection threshold from 0 to 1 (default: config.py value)",
    )
    args = parser.parse_args()
    if args.timeout < 0:
        parser.error("--timeout must not be negative")
    if args.interval <= 0:
        parser.error("--interval must be greater than zero")
    if args.confidence is not None and not 0.0 <= args.confidence <= 1.0:
        parser.error("--confidence must be between 0 and 1")
    return args


def wait_for_target_and_drop(
    camera,
    servo,
    timeout: float,
    interval: float,
    *,
    threshold: float | None = None,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
) -> bool:
    """Run one detection/release cycle and leave the hardware in a safe state.

    Returns ``True`` after a target-triggered release, or ``False`` when the
    timeout expires. A zero timeout waits indefinitely. Dependencies are accepted
    as arguments so the hardware-independent sequence can be unit-tested.
    """
    servo_ready = False
    servo_neutral = False
    camera_started = False

    try:
        # Close the mechanism before camera initialisation, which can take tens of
        # seconds while an IMX500 model is uploaded.
        servo.setup()
        servo_ready = True
        servo_neutral = True

        # stop() is safe after a partial start, so mark ownership before calling
        # start() in case model upload or warm-up is interrupted.
        camera_started = True
        camera.start()
        started = monotonic()

        while timeout == 0 or monotonic() - started < timeout:
            result = camera.get_detection()
            confidence = result.get("confidence")
            confidence_text = "n/a" if confidence is None else f"{confidence:.3f}"
            threshold_text = "n/a" if threshold is None else f"{threshold:.3f}"
            log.info(
                "[DROP_TEST] CAMERA | %s | confidence=%s | threshold=%s | "
                "class=%s | box=%s",
                "FOUND" if result.get("detected", False) else "NOT FOUND",
                confidence_text,
                threshold_text,
                result.get("label"),
                result.get("box"),
            )
            if result.get("detected", False):
                log.warning(
                    "[DROP_TEST] Target detected: class=%s, confidence=%s; releasing",
                    result.get("label"), confidence_text,
                )
                servo.drop()
                servo_neutral = False
                try:
                    sleep(RELEASE_HOLD_S)
                finally:
                    servo.reset()
                    servo_neutral = True
                    log.info("[DROP_TEST] Servo returned to neutral after 3.0 s")
                return True

            sleep(interval)

        log.warning("[DROP_TEST] Timed out without detecting the target; no release")
        return False
    finally:
        # A Ctrl-C or camera error must not leave the hatch commanded open.
        if servo_ready and not servo_neutral:
            try:
                servo.reset()
                log.info("[DROP_TEST] Safety cleanup: servo returned to neutral")
            except Exception:
                log.exception("[DROP_TEST] SAFETY CLEANUP FAILED: check servo position")
        if camera_started:
            camera.stop()
            log.info("[DROP_TEST] Camera stopped")


def main() -> None:
    args = parse_args()
    config = Config.pi()
    if args.confidence is not None:
        config.camera_confidence = args.confidence

    setup_logging(config.log_dir)
    log.warning(
        "[DROP_TEST] BENCH TEST ONLY: the FC is not contacted; no arming, motor, "
        "or takeoff command can be sent"
    )
    log.warning("[DROP_TEST] Keep the release area clear and remove all propellers")
    log.info(
        "[DROP_TEST] model=%s, threshold=%.3f, GPIO%d BCM, release=%d us, "
        "neutral=%d us, hold=%.1f s",
        config.camera_model_path,
        config.camera_confidence,
        config.drop_gpio_pin,
        config.drop_pwm,
        config.neutral_pwm,
        RELEASE_HOLD_S,
    )

    try:
        wait_for_target_and_drop(
            RealCamera(config),
            PiServo(config),
            timeout=args.timeout,
            interval=args.interval,
            threshold=config.camera_confidence,
        )
    except KeyboardInterrupt:
        log.warning("[DROP_TEST] Interrupted by user")
        raise SystemExit(130)
    except Exception:
        log.exception("[DROP_TEST] Test failed")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
