"""Standalone, disarmed test for the Raspberry Pi AI Camera (IMX500).

This program only starts the camera and reads inference metadata.  It never
imports the flight-controller, mission, arming, or payload-release modules.

Usage:
    python camera_test.py
    python camera_test.py --duration 60 --interval 0.5
    python camera_test.py --duration 0       # run until Ctrl-C
"""

import argparse
import logging
import time

from camera import RealCamera
from config import Config
from logbook import setup_logging


log = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Report IMX500 target detection without contacting or arming the FC"
    )
    parser.add_argument(
        "--duration", type=float, default=20.0,
        help="test duration in seconds; 0 runs until Ctrl-C (default: 20)",
    )
    parser.add_argument(
        "--interval", type=float, default=0.5,
        help="seconds between reports (default: 0.5)",
    )
    parser.add_argument(
        "--confidence", type=float,
        help="temporary detection threshold from 0 to 1 (default: config.py value)",
    )
    args = parser.parse_args()
    if args.duration < 0:
        parser.error("--duration must not be negative")
    if args.interval <= 0:
        parser.error("--interval must be greater than zero")
    if args.confidence is not None and not 0.0 <= args.confidence <= 1.0:
        parser.error("--confidence must be between 0 and 1")
    return args


def report_detection(result: dict, threshold: float) -> None:
    confidence = result["confidence"]
    if confidence is None:
        log.info("[CAMERA_TEST] NOT FOUND | confidence=n/a | no target candidate")
        return

    state = "FOUND" if result["detected"] else "NOT FOUND"
    log.info(
        f"[CAMERA_TEST] {state} | confidence={confidence:.3f} "
        f"| threshold={threshold:.3f} | class={result['label']}"
    )


def main() -> None:
    args = parse_args()
    config = Config.pi()
    if args.confidence is not None:
        config.camera_confidence = args.confidence

    setup_logging(config.log_dir)
    log.warning("[CAMERA_TEST] CAMERA ONLY: the FC is not contacted; no arming, "
                "motor, servo, or takeoff command can be sent")
    log.info(f"[CAMERA_TEST] model={config.camera_model_path}")
    log.info(f"[CAMERA_TEST] threshold={config.camera_confidence:.3f}, "
             f"target_class={config.camera_target_class}, interval={args.interval:.2f} s")
    if config.camera_target_class is None:
        log.warning("[CAMERA_TEST] camera_target_class=None: any model class may count "
                    "as the target (appropriate only for a single-class model)")

    camera = RealCamera(config)
    try:
        camera.start()
        started = time.monotonic()
        while args.duration == 0 or time.monotonic() - started < args.duration:
            report_detection(camera.get_detection(), config.camera_confidence)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        log.warning("[CAMERA_TEST] Interrupted by user")
    except Exception:
        log.exception("[CAMERA_TEST] Test failed")
        raise SystemExit(1)
    finally:
        camera.stop()
        log.info("[CAMERA_TEST] Camera stopped")


if __name__ == "__main__":
    main()
