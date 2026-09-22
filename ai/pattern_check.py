"""
Mustererkennung: verify the red X inside a detection.

The detector answers "dark quadrilateral on a light floor", which a rucksack, a
poster and a chair back all satisfy. The pad additionally carries a red cross
from corner to corner, and nothing else in the room does. Checking for it turns
a shape detector into a pad detector.

Two measurements on the inner region of the box (the border is excluded, so
only the arms of the X remain):

  red fraction   how much saturated red is present at all
  cross energy   whether that red lies along two perpendicular directions

Cross energy is computed from a histogram of pixel angles about the centre,
taken modulo 180 degrees so a line and its opposite arm fall in the same bin.
Two bins 90 degrees apart are summed, and the best such pair is scored against
the total. An ideal X puts everything into that pair; scattered red clutter
spreads across all bins. Because the score maximises over the offset, it does
not care how the pad is rotated in frame.

Pure numpy and OpenCV, no model, ~0.2 ms per box — negligible on a Pi Zero 2 W.

    python3 pattern_check.py              # separation on real vs pad-free crops
"""

import warnings
from pathlib import Path

import cv2
import numpy as np

warnings.filterwarnings("ignore")

HERE = Path(__file__).parent
N_BINS = 36                  # 5 degrees per bin over 0..180
INNER = 0.72                 # keep the middle of the box, drop the red border
MIN_RED = 0.02
MIN_PIXELS = 40


def red_mask(bgr):
    """Saturated red/orange, both ends of the hue circle."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return (((h <= 12) | (h >= 168)) & (s >= 80) & (v >= 60)).astype(np.uint8)


def x_score(crop):
    """Return (score, red_fraction). score in [0,1]; ~1 for a clean X."""
    if crop.size == 0 or min(crop.shape[:2]) < 12:
        return 0.0, 0.0

    h, w = crop.shape[:2]
    my, mx = int(h * (1 - INNER) / 2), int(w * (1 - INNER) / 2)
    inner = crop[my:h - my, mx:w - mx]
    if inner.size == 0:
        return 0.0, 0.0

    mask = red_mask(inner)
    frac = float(mask.mean())
    ys, xs = np.nonzero(mask)
    if len(xs) < MIN_PIXELS or frac < MIN_RED:
        return 0.0, frac

    ih, iw = inner.shape[:2]
    dx = (xs - iw / 2) / (iw / 2)
    dy = (ys - ih / 2) / (ih / 2)
    r = np.hypot(dx, dy)
    keep = r > 0.15                      # centre pixels have no stable angle
    if keep.sum() < MIN_PIXELS:
        return 0.0, frac
    dx, dy, r = dx[keep], dy[keep], r[keep]

    ang = np.degrees(np.arctan2(dy, dx)) % 180.0
    bins = np.clip((ang / 180.0 * N_BINS).astype(int), 0, N_BINS - 1)
    hist = np.bincount(bins, weights=r, minlength=N_BINS)   # weight by radius
    total = hist.sum()
    if total <= 0:
        return 0.0, frac

    quarter = N_BINS // 4                # 90 degrees
    # Arms are a few degrees thick, so each direction is a 3-bin window.
    arm = np.array([sum(hist[(i + d) % N_BINS] for d in (-1, 0, 1))
                    for i in range(N_BINS)])

    # Both arms must be present. Summing them would let a single red line --
    # a floor marking -- score as high as a cross, which is what the first
    # version of this did and why it separated nothing.
    best = max(min(arm[i], arm[(i + quarter) % N_BINS]) for i in range(N_BINS))
    return float(min(2.0 * best / total, 1.0)), frac


def verify(frame, box, min_score=0.35):
    """True if the detection at `box` really carries the pad's X."""
    x1, y1, x2, y2 = (int(v) for v in box[:4])
    score, _ = x_score(frame[max(y1, 0):y2, max(x1, 0):x2])
    return score >= min_score


def main():
    from ultralytics import YOLO

    model = YOLO("runs/D_drone_fpv_320/weights/best.pt")
    rows = {"real pads (test)": [], "pad-free crops": []}

    for f in sorted((HERE / "pad-dataset" / "test" / "images").glob("*")):
        img = cv2.imread(str(f))
        for b in model.predict(img, imgsz=320, conf=0.25, device="mps",
                               verbose=False)[0].boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = (int(v) for v in b)
            rows["real pads (test)"].append(x_score(img[y1:y2, x1:x2])[0])

    for f in sorted((HERE / "pad-negatives-bench" / "images").glob("*")):
        img = cv2.imread(str(f))
        for b in model.predict(img, imgsz=320, conf=0.25, device="mps",
                               verbose=False)[0].boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = (int(v) for v in b)
            rows["pad-free crops"].append(x_score(img[y1:y2, x1:x2])[0])

    print(f"\n{'':<20}{'n':>5}{'min':>8}{'median':>9}{'max':>8}")
    print("-" * 50)
    for k, v in rows.items():
        a = np.array(v) if v else np.array([0.0])
        print(f"{k:<20}{len(v):>5}{a.min():>8.2f}{np.median(a):>9.2f}{a.max():>8.2f}")

    print(f"\n{'threshold':<12}{'pads kept':>12}{'false pads kept':>18}")
    print("-" * 44)
    pads = np.array(rows["real pads (test)"] or [0.0])
    negs = np.array(rows["pad-free crops"] or [0.0])
    for t in (0.25, 0.30, 0.35, 0.40, 0.50, 0.60):
        print(f"{t:<12.2f}{(pads >= t).mean():>12.2f}"
              f"{(negs >= t).sum():>10} / {len(negs)}")


if __name__ == "__main__":
    main()
