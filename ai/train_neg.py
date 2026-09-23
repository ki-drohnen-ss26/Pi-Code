"""
Run F: run D's recipe, plus pad-free background images from the same rooms.

Run D reaches further than the old recipe but fires on pad-free scenes at up to
0.87 confidence (0.45 false detections per image at conf 0.25), because only 3
of its 175 training images contained no pad. Run F adds 76 hard negatives
harvested by negatives.py, taking the training set to 30% background.

Everything else matches run D, so the comparison isolates the negatives.
"""

import warnings
from pathlib import Path

warnings.filterwarnings("ignore")

from ultralytics import YOLO  # noqa: E402

import fpv_aug  # noqa: E402
from train import COMMON, DRONE_AUG  # noqa: E402

HERE = Path(__file__).parent


def main():
    cfg = dict(COMMON, data=str(HERE / "pad-dataset-neg" / "data.yaml"))
    model = fpv_aug.attach(YOLO("yolo11n.pt"))
    model.train(name="F_neg_fpv_320", **cfg, **dict(DRONE_AUG, imgsz=320))


if __name__ == "__main__":
    main()
