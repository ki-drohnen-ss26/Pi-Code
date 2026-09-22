"""
Landing-pad detector: controlled comparison of training recipes.

All runs use the same leak-free split from build_dataset.py, so the only thing
that changes between them is the recipe. Run A reproduces the recipe that
produced the current best.pt; B and C are the proposed one.

Recipe changes and why they matter for a drone:

  degrees 0 -> 180   A multirotor's yaw is arbitrary, so the pad arrives at the
                     camera at every rotation. The old recipe only ever showed
                     the network the +/-15 deg that Roboflow had baked in.
                     Labels are polygons, so rotation yields tight boxes.
  flipud  0 -> 0.5   A near-nadir view is flip-symmetric; free extra data.
  scale  0.5 -> 0.8  The pad must be found at approach altitude (tiny) and at
                     touchdown (filling the frame). Wider scale jitter spans
                     the whole descent.
  translate .1 -> .2 The pad is rarely centred while the drone is manoeuvring.
  perspective -> 5e-4  It is a flat plane viewed obliquely from a moving camera.
  hsv_v .4 -> .5     Sports hall, office and outdoor lighting all appear here.
  cos_lr, 150 ep     73 source photos is tiny; a cosine anneal with a large
                     patience beats an 80-epoch step schedule. Validation mAP
                     saturates around epoch 40, so 150 is already generous.
  close_mosaic 25    Mosaic manufactures the small-pad cases the dataset lacks,
                     but the last epochs should see real, uncropped framing.

multi_scale was tried and dropped: on MPS it forces a graph rebuild at every
new input size and cost ~3x per epoch for no measurable gain over scale=0.8.

Usage:  python3 train.py [run_name ...]     (default: all)
"""

import sys
import warnings
from pathlib import Path

warnings.filterwarnings("ignore")

from ultralytics import YOLO  # noqa: E402

HERE = Path(__file__).parent
DATA = str(HERE / "pad-dataset" / "data.yaml")
DEVICE = "mps"

COMMON = dict(
    data=DATA, device=DEVICE, project=str(HERE / "runs"),
    seed=0, deterministic=True, exist_ok=True, val=True, plots=True,
    cache="ram", workers=8, pretrained=True,
)

# what the current best.pt was trained with
BASELINE_AUG = dict(
    epochs=80, patience=20, batch=32, imgsz=416, cos_lr=False, close_mosaic=10,
    degrees=0.0, flipud=0.0, fliplr=0.5, scale=0.5, translate=0.1,
    perspective=0.0, hsv_h=0.015, hsv_s=0.7, hsv_v=0.4, mosaic=1.0,
)

# proposed recipe
DRONE_AUG = dict(
    epochs=150, patience=40, batch=32, cos_lr=True, close_mosaic=25,
    lr0=0.01, lrf=0.01, warmup_epochs=5.0,
    degrees=180.0, flipud=0.5, fliplr=0.5, scale=0.8, translate=0.2,
    perspective=0.0005, hsv_h=0.015, hsv_s=0.7, hsv_v=0.5, mosaic=1.0,
)

RUNS = {
    "A_baseline_416": dict(BASELINE_AUG),
    "B_drone_320": dict(DRONE_AUG, imgsz=320),
    "C_drone_416": dict(DRONE_AUG, imgsz=416),
}


def main(names=None):
    for name in (names or RUNS):
        print(f"\n{'=' * 70}\n  {name}\n{'=' * 70}", flush=True)
        YOLO("yolo11n.pt").train(name=name, **COMMON, **RUNS[name])


if __name__ == "__main__":
    main(sys.argv[1:] or None)
