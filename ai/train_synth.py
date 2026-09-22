"""
Run E: the drone recipe plus FPV degradation, trained on the real images
*and* the composited distant pads from synth.py.

Identical to run D except for the dataset, so the comparison isolates what the
synthetic small pads buy.
"""

import warnings
from pathlib import Path

warnings.filterwarnings("ignore")

from ultralytics import YOLO  # noqa: E402

import fpv_aug  # noqa: E402
from train import COMMON, DRONE_AUG  # noqa: E402

HERE = Path(__file__).parent


def main():
    cfg = dict(COMMON, data=str(HERE / "pad-dataset-synth" / "data.yaml"))
    model = fpv_aug.attach(YOLO("yolo11n.pt"))
    model.train(name="E_synth_fpv_320", **cfg, **dict(DRONE_AUG, imgsz=320))


if __name__ == "__main__":
    main()
