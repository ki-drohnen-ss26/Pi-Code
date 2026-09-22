"""
Run D: the drone recipe plus FPV capture-path degradation (see fpv_aug.py).

Split out from train.py so it can be run on its own once A/B/C are done.
Identical to run B in every other respect, so the comparison isolates the
degradation augmentation.
"""

import warnings
from pathlib import Path

warnings.filterwarnings("ignore")

from ultralytics import YOLO  # noqa: E402

import fpv_aug  # noqa: E402
from train import COMMON, DRONE_AUG  # noqa: E402

HERE = Path(__file__).parent


def main():
    model = fpv_aug.attach(YOLO("yolo11n.pt"))
    model.train(name="D_drone_fpv_320", **COMMON, **dict(DRONE_AUG, imgsz=320))


if __name__ == "__main__":
    main()
