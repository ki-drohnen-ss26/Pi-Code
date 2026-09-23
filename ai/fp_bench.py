"""
False positives on pad-free images, against recall on real pads.

A landing controller cares about both, and they trade against each other via
the confidence threshold. This prints the whole curve so the threshold can be
chosen from data instead of guessed.

Negatives are held-out crops from val/test (negatives.py). Recall is on the
real test images, and on those images zoomed out x0.3 — because a threshold
that only works when the pad fills the frame is no use during the approach.

    python3 fp_bench.py runs/*/weights/best.pt
"""

import os
import sys
import warnings
from pathlib import Path

import cv2
import numpy as np

warnings.filterwarnings("ignore")

# ueberschreibbar, damit dieselben Sonden auch das quantisierte
# IMX/ONNX-Modell messen koennen — das laeuft nicht auf MPS
DEVICE = os.environ.get("PROBE_DEVICE", "mps")

from ultralytics import YOLO  # noqa: E402

from robustness import polygons, zoom_out  # noqa: E402

HERE = Path(__file__).parent
BENCH = HERE / "pad-negatives-bench" / "images"
TEST = HERE / "pad-dataset" / "test" / "images"
THRESHOLDS = (0.25, 0.4, 0.5, 0.6, 0.7, 0.8)
IMGSZ = 320


def iou(a, b):
    lt, rb = np.maximum(a[:2], b[:2]), np.minimum(a[2:], b[2:])
    inter = np.prod(np.clip(rb - lt, 0, None))
    area = lambda x: max((x[2] - x[0]) * (x[3] - x[1]), 0.0)  # noqa: E731
    return inter / (area(a) + area(b) - inter + 1e-9)


def false_positives(model, files):
    """Detections per image on images that contain no pad at all."""
    counts = {t: 0 for t in THRESHOLDS}
    worst = 0.0
    for f in files:
        r = model.predict(str(f), imgsz=IMGSZ, conf=min(THRESHOLDS),
                          device=DEVICE, verbose=False)[0]
        confs = r.boxes.conf.cpu().numpy() if r.boxes is not None else np.array([])
        worst = max(worst, confs.max() if len(confs) else 0.0)
        for t in THRESHOLDS:
            counts[t] += int((confs >= t).sum())
    return {t: counts[t] / len(files) for t in THRESHOLDS}, worst


def recall(model, files, zoom=1.0):
    hits = {t: 0 for t in THRESHOLDS}
    total = 0
    for f in files:
        img = cv2.imread(str(f))
        h, w = img.shape[:2]
        gts = [np.array([xy[:, 0].min(), xy[:, 1].min(),
                         xy[:, 0].max(), xy[:, 1].max()])
               for xy in polygons(f.parent.parent / "labels" / f"{f.stem}.txt", w, h)]
        if zoom < 1.0:
            img, gts = zoom_out(img, gts, zoom)
        r = model.predict(img, imgsz=IMGSZ, conf=min(THRESHOLDS),
                          device=DEVICE, verbose=False)[0]
        if r.boxes is None or not len(r.boxes):
            total += len(gts)
            continue
        boxes = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        for g in gts:
            total += 1
            ok = [c for b, c in zip(boxes, confs) if iou(g, b) >= 0.5]
            best = max(ok) if ok else 0.0
            for t in THRESHOLDS:
                hits[t] += int(best >= t)
    return {t: hits[t] / max(total, 1) for t in THRESHOLDS}


def main(weights):
    negs = sorted(BENCH.glob("*"))
    tests = sorted(TEST.glob("*"))
    print(f"{len(negs)} pad-free crops, {len(tests)} real test images, "
          f"inference at {IMGSZ}px\n")

    for wp in weights:
        model = YOLO(wp)
        name = Path(wp).parent.parent.name
        fp, worst = false_positives(model, negs)
        rec = recall(model, tests)
        rec_far = recall(model, tests, zoom=0.3)

        print(f"{name}   (highest confidence on a pad-free image: {worst:.2f})")
        print(f"  {'conf':<8}" + "".join(f"{t:>9.2f}" for t in THRESHOLDS))
        print(f"  {'FP/img':<8}" + "".join(f"{fp[t]:>9.2f}" for t in THRESHOLDS))
        print(f"  {'recall':<8}" + "".join(f"{rec[t]:>9.2f}" for t in THRESHOLDS))
        print(f"  {'rec x0.3':<8}" + "".join(f"{rec_far[t]:>9.2f}" for t in THRESHOLDS))
        print()


if __name__ == "__main__":
    main(sys.argv[1:] or ["runs/D_drone_fpv_320/weights/best.pt"])
