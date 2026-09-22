"""
Export the chosen landing-pad model to TFLite INT8 for the Pi, then check that
quantisation did not cost accuracy.

The export size must equal the training size. The previous pipeline trained at
416 and exported at 320, so the network was run at a scale it had never been
optimised for; that mismatch is removed by taking imgsz from the checkpoint's
own training args.

Quantisation is calibrated on the training split, and the resulting .tflite is
validated on the test split with the same metric as the .pt, so the accuracy
lost to INT8 is measured rather than assumed.

    python3 export.py runs/<run>/weights/best.pt
"""

import shutil
import sys
import warnings
from pathlib import Path

warnings.filterwarnings("ignore")

import torch  # noqa: E402
from ultralytics import YOLO  # noqa: E402

HERE = Path(__file__).parent
DATA = str(HERE / "pad-dataset" / "data.yaml")


def main(weights):
    weights = Path(weights)
    ckpt = torch.load(weights, map_location="cpu", weights_only=False)
    imgsz = int(ckpt["train_args"]["imgsz"])
    print(f"checkpoint trained at imgsz={imgsz} -> exporting at the same size")

    model = YOLO(str(weights))
    pt = model.val(data=DATA, imgsz=imgsz, split="test", device="mps",
                   verbose=False, plots=False,
                   project=str(HERE / "eval"), name="export_pt", exist_ok=True)

    out = model.export(format="tflite", imgsz=imgsz, int8=True, data=DATA)
    out = Path(out)
    print("exported:", out)

    tfl = YOLO(str(out), task="detect").val(
        data=DATA, imgsz=imgsz, split="test", device="cpu", verbose=False,
        plots=False, project=str(HERE / "eval"), name="export_tflite",
        exist_ok=True)

    dest = HERE / f"pad_{imgsz}_int8.tflite"
    shutil.copy2(out, dest)

    print(f"\n{'=' * 62}")
    print(f"{'':<22}{'mAP50':>10}{'mAP50-95':>12}{'recall':>10}")
    print(f"{'PyTorch (.pt)':<22}{pt.box.map50:>10.4f}{pt.box.map:>12.4f}"
          f"{pt.box.mr:>10.4f}")
    print(f"{'TFLite INT8':<22}{tfl.box.map50:>10.4f}{tfl.box.map:>12.4f}"
          f"{tfl.box.mr:>10.4f}")
    print(f"{'delta':<22}{tfl.box.map50 - pt.box.map50:>10.4f}"
          f"{tfl.box.map - pt.box.map:>12.4f}{tfl.box.mr - pt.box.mr:>10.4f}")
    print(f"{'=' * 62}")
    print(f"\ncopy to the Pi:  {dest}  ({dest.stat().st_size / 1e6:.1f} MB)")
    print(f"set PAD_MODEL = \"{dest.name}\" in drone_pi.py")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "runs/B_drone_320/weights/best.pt")
