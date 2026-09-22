"""Standalone bench test for the Raspberry Pi payload-release servo.

The flight controller is not contacted and the aircraft is never armed. The test
drives the GPIO servo repeatedly between release and neutral positions.

Usage:
    python servo_test.py
    python servo_test.py --cycles 5
    python servo_test.py --cycles 3 --release-hold 1.5 --neutral-hold 1.0
"""

import argparse
import logging
import time

from config import Config
from logbook import setup_logging
from release import PiServo


log = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Cycle the configured GPIO payload servo without connecting to the FC"
    )
    parser.add_argument("--cycles", type=int, default=3,
                        help="number of release/neutral cycles (default: 3)")
    parser.add_argument("--release-hold", type=float, default=1.5,
                        help="seconds to hold the release position (default: 1.5)")
    parser.add_argument("--neutral-hold", type=float, default=1.0,
                        help="seconds to hold neutral between cycles (default: 1.0)")
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error("--cycles must be at least 1")
    if args.release_hold < 0 or args.neutral_hold < 0:
        parser.error("hold times must not be negative")
    return args


def cycle_servo(servo, cycles: int, release_hold: float, neutral_hold: float,
                sleep=time.sleep) -> None:
    """Run the requested cycles and always leave the command at neutral."""
    servo.setup()  # setup itself commands neutral once
    try:
        for number in range(1, cycles + 1):
            log.info(f"[SERVO_TEST] Cycle {number}/{cycles}: release")
            servo.drop()
            sleep(release_hold)

            log.info(f"[SERVO_TEST] Cycle {number}/{cycles}: neutral")
            servo.reset()
            sleep(neutral_hold)
    finally:
        servo.reset()
        log.info("[SERVO_TEST] Final command: neutral")


def main() -> None:
    args = parse_args()
    config = Config.pi()

    setup_logging(config.log_dir)
    log.warning("[SERVO_TEST] BENCH TEST ONLY: no FC connection, no arming, no "
                "takeoff")
    log.warning("[SERVO_TEST] GPIO control is open-loop; observe the mechanism and "
                "keep the release area clear")
    log.info(f"[SERVO_TEST] GPIO{config.drop_gpio_pin} BCM, "
             f"release={config.drop_pwm} us, neutral={config.neutral_pwm} us, "
             f"cycles={args.cycles}")

    try:
        cycle_servo(
            PiServo(config),
            cycles=args.cycles,
            release_hold=args.release_hold,
            neutral_hold=args.neutral_hold,
        )
    except KeyboardInterrupt:
        log.warning("[SERVO_TEST] Interrupted by user")
        raise SystemExit(130)
    except Exception:
        log.exception("[SERVO_TEST] Test failed")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
