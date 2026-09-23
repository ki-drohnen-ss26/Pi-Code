"""Standalone bench test: detect the landing pad, then release the payload.

The flight controller is never contacted and the aircraft is never armed. This
joins the two halves that `camera_test.py` and `servo_test.py` exercise
separately, so the handover between them can be checked on a table before it is
checked in the air.

It answers two questions:

  1. At what confidence does this camera actually report this pad? The run ends
     with the distribution of every score seen, so the threshold can be chosen
     from data instead of guessed.
  2. Does a confirmed detection release the payload? Only with --arm-drop.

THE DROP IS OFF BY DEFAULT. Without --arm-drop the servo is never touched and
the script only reports where it would have fired.

A single frame never triggers anything. The sensor emits occasional impossible
boxes - stuck to the frame edge, far too long and thin - that last one frame,
while a real pad holds for dozens. The release therefore needs the pad in
--hold of the last --of frames, in roughly the same place, and each box has to
be a plausible shape. That filter is the point of this script as much as the
servo is.

Usage:
    python pad_drop_test.py                        # survey only, no servo
    python pad_drop_test.py --confidence 0.5       # try a different threshold
    python pad_drop_test.py --arm-drop             # really release on a confirmed pad
    python pad_drop_test.py --arm-drop --countdown 5
    python pad_drop_test.py --duration 0           # until Ctrl-C
"""

import argparse
import logging
import time
from collections import deque

from camera import RealCamera
from config import Config
from logbook import setup_logging
from release import PiServo

log = logging.getLogger(__name__)

# A pad seen from above is roughly square. Anything much longer than this is
# either two things merged into one box or noise.
MAX_ASPECT = 3.0
# A box touching the frame edge is usually a fragment of something larger that
# happens to be dark, and its centre is meaningless anyway.
EDGE_MARGIN = 0.02
# How far the centre may wander between frames and still count as the same pad,
# as a fraction of the frame.
MAX_CENTRE_DRIFT = 0.25


def plausible(box):
    """Reject shapes a pad seen from above cannot produce.

    `box` is (x0, y0, x1, y1) normalised to the frame.
    Returns (True, "") or (False, reason).
    """
    x0, y0, x1, y1 = box
    w, h = x1 - x0, y1 - y0
    if w <= 0 or h <= 0:
        return False, "empty box"
    if min(x0, y0) < EDGE_MARGIN or max(x1, y1) > 1.0 - EDGE_MARGIN:
        return False, "touches the frame edge"
    aspect = max(w / h, h / w)
    if aspect > MAX_ASPECT:
        return False, f"aspect 1:{aspect:.1f}"
    return True, ""


def centre_of(box):
    x0, y0, x1, y1 = box
    return ((x0 + x1) / 2.0, (y0 + y1) / 2.0)


def settled(centres):
    """True if every centre in the window sits close to the first one."""
    cx0, cy0 = centres[0]
    return all(abs(cx - cx0) <= MAX_CENTRE_DRIFT and abs(cy - cy0) <= MAX_CENTRE_DRIFT
               for cx, cy in centres)


def read_detection(camera):
    """One frame -> (found, score, box) with score/box None when unavailable.

    `get_target_offset()` is the answer the mission itself acts on, so the
    decision below is made on exactly that. It does not carry the confidence or
    the box, which this script needs to report, so those come from the decoder
    underneath it. If that internal changes, the script keeps working and simply
    reports no score.
    """
    offset = camera.get_target_offset()
    score = box = None
    try:
        metadata = camera._picam2.capture_metadata()
        best = camera._best_detection(metadata)
        if best is not None:
            box, score = best[:4], best[4]
    except Exception:  # reporting only - never let it break the test
        pass
    return bool(offset.get("detected")), score, box, offset


def summarise(accepted, rejected, reject_reasons):
    """Print what the camera reported, so a threshold can be chosen from it."""
    log.info("")
    log.info("=" * 64)
    log.info("CONFIDENCE SURVEY")
    log.info("=" * 64)

    if not accepted and not rejected:
        log.info("No detections at all. Either the pad was never in view, or the "
                 "threshold is above everything this camera reports.")
        return

    def histogram(title, scores):
        if not scores:
            log.info(f"{title}: none")
            return
        scores = sorted(scores)
        log.info(f"{title}: {len(scores)} frames, "
                 f"lowest {scores[0]:.2f}, median {scores[len(scores)//2]:.2f}, "
                 f"highest {scores[-1]:.2f}")
        buckets = {}
        for s in scores:
            buckets[round(s, 2)] = buckets.get(round(s, 2), 0) + 1
        widest = max(buckets.values())
        for value in sorted(buckets):
            bar = "#" * max(1, round(20 * buckets[value] / widest))
            log.info(f"    {value:.2f}  {bar} {buckets[value]}")

    histogram("Accepted as a pad ", accepted)
    histogram("Rejected as noise ", rejected)

    if reject_reasons:
        log.info("Rejected because:")
        for reason, n in sorted(reject_reasons.items(), key=lambda kv: -kv[1]):
            log.info(f"    {n:4d}  {reason}")

    if accepted and rejected:
        log.info("")
        log.info(f"A threshold between {max(rejected):.2f} and {min(accepted):.2f} "
                 f"separates everything seen in this run.")
        if min(accepted) <= max(rejected):
            log.info("  ...except these overlap, so confidence alone does not "
                     "separate them here. The persistence filter is doing the work.")


def main():
    ap = argparse.ArgumentParser(
        description="Detect the pad on the bench and, optionally, release the payload")
    ap.add_argument("--arm-drop", action="store_true",
                    help="really move the servo on a confirmed pad. Off by default")
    ap.add_argument("--confidence", type=float, default=None,
                    help="override config.camera_confidence for this run (0..1)")
    ap.add_argument("--hold", type=int, default=3,
                    help="frames that must show the pad (default 3)")
    ap.add_argument("--of", type=int, default=4, dest="window",
                    help="out of this many most recent frames (default 4)")
    ap.add_argument("--interval", type=float, default=0.2,
                    help="seconds between frames (default 0.2)")
    ap.add_argument("--duration", type=float, default=60.0,
                    help="seconds to run, 0 = until Ctrl-C (default 60)")
    ap.add_argument("--countdown", type=float, default=3.0,
                    help="seconds between confirming the pad and moving the servo")
    args = ap.parse_args()

    if args.hold > args.window:
        ap.error("--hold cannot be larger than --of")

    config = Config.pi()
    if args.confidence is not None:
        if not 0.0 < args.confidence <= 1.0:
            ap.error("--confidence is a fraction from 0 to 1, so 0.5 rather than 50")
        config.camera_confidence = args.confidence

    setup_logging(config.log_dir)

    log.info("[PAD_DROP] model=%s", config.camera_model_path)
    log.info("[PAD_DROP] threshold=%.2f, confirm on %d of %d frames, interval=%.2f s",
             config.camera_confidence, args.hold, args.window, args.interval)

    if args.arm_drop:
        log.warning("")
        log.warning("  THE DROP SERVO IS ARMED. It will move on a confirmed pad.")
        log.warning("  Keep hands clear of the hatch. Propellers should be off.")
        log.warning("")
    else:
        log.info("[PAD_DROP] Survey only: the servo is NOT touched. "
                 "Add --arm-drop to release for real.")

    camera = RealCamera(config, drone=None)
    servo = PiServo(config) if args.arm_drop else None

    accepted, rejected, reasons = [], [], {}
    window = deque(maxlen=args.window)
    centres = deque(maxlen=args.window)
    dropped = False

    try:
        camera.start()
        if servo is not None:
            servo.setup()          # attaches the pin and goes to neutral

        log.info("[PAD_DROP] Running. Ctrl-C to stop.")
        started = time.monotonic()

        while not dropped:
            if args.duration and time.monotonic() - started >= args.duration:
                log.info("[PAD_DROP] Time is up.")
                break

            found, score, box, offset = read_detection(camera)

            ok = False
            if found and box is not None:
                ok, why = plausible(box)
                if ok:
                    accepted.append(score if score is not None else 0.0)
                    centres.append(centre_of(box))
                else:
                    rejected.append(score if score is not None else 0.0)
                    reasons[why] = reasons.get(why, 0) + 1
                    log.info("[PAD_DROP] ignored (%s) p=%s", why,
                             f"{score:.2f}" if score is not None else "?")
            elif found:
                # No box available to check, so trust the mission's own answer.
                ok = True
                accepted.append(score if score is not None else 0.0)
                centres.append((0.5, 0.5))

            window.append(ok)
            if not ok:
                centres.clear()

            hits = sum(window)
            if found and ok:
                log.info("[PAD_DROP] pad p=%s dx=%+.2f dy=%+.2f m  [%d/%d]",
                         f"{score:.2f}" if score is not None else "?",
                         offset.get("dx", 0.0), offset.get("dy", 0.0),
                         hits, args.window)

            confirmed = (hits >= args.hold
                         and len(centres) >= args.hold
                         and settled(list(centres)[-args.hold:]))

            if confirmed:
                log.info("[PAD_DROP] CONFIRMED: pad in %d of the last %d frames, "
                         "centre steady", hits, args.window)
                if servo is None:
                    log.info("[PAD_DROP] WOULD RELEASE HERE. Re-run with --arm-drop "
                             "to do it for real.")
                    window.clear()
                    centres.clear()
                else:
                    for remaining in range(int(args.countdown), 0, -1):
                        log.warning("[PAD_DROP] releasing in %d ...", remaining)
                        time.sleep(1.0)
                    servo.drop()
                    log.warning("[PAD_DROP] RELEASED (confirmed=%s)", servo.confirm())
                    dropped = True

            time.sleep(args.interval)

    except KeyboardInterrupt:
        log.info("[PAD_DROP] Stopped.")
    except Exception as exc:
        log.error("[PAD_DROP] %s", exc)
        raise
    finally:
        if servo is not None:
            try:
                time.sleep(1.0)     # let the hatch finish opening before closing it
                servo.reset()
                log.info("[PAD_DROP] Servo back to neutral.")
            except Exception as exc:
                log.error("[PAD_DROP] Could not return the servo to neutral: %s", exc)
        try:
            camera.stop()
        except Exception:
            pass
        summarise(accepted, rejected, reasons)


if __name__ == "__main__":
    main()
