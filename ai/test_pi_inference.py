"""
Check the TFLite inference path in drone_pi.py against ground truth.

The rewrite changed preprocessing (letterbox instead of squash) and the box
decode (undoing the letterbox, vectorised NMS). Both are easy to get subtly
wrong in a way that still produces plausible-looking boxes, so this compares
the decoded boxes to the dataset labels on real images.

It also reports what the old squash-to-square preprocessing scores on the same
images, which is the accuracy that was being left on the table.

    ./pi_export_env/bin/python drone-ai/test_pi_inference.py
"""

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from drone_pi import Model  # noqa: E402

HERE = Path(__file__).parent
DEFAULT_MODEL = HERE / "pad_320_int8.tflite"
IMAGES = sorted((HERE / "pad-dataset" / "test" / "images").glob("*"))


def gt_boxes(label_path, w, h):
    out = []
    for line in label_path.read_text().split("\n"):
        p = line.split()
        if len(p) < 5:
            continue
        v = np.array(p[1:], dtype=float)
        if len(v) == 4:
            cx, cy, bw, bh = v
            b = [cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2]
        else:
            xy = v.reshape(-1, 2)
            b = [xy[:, 0].min(), xy[:, 1].min(), xy[:, 0].max(), xy[:, 1].max()]
        out.append(np.array(b) * np.array([w, h, w, h]))
    return out


def iou(a, b):
    lt, rb = np.maximum(a[:2], b[:2]), np.minimum(a[2:], b[2:])
    inter = np.prod(np.clip(rb - lt, 0, None))
    area = lambda x: max((x[2] - x[0]) * (x[3] - x[1]), 0.0)  # noqa: E731
    return inter / (area(a) + area(b) - inter + 1e-9)


class SquashModel(Model):
    """The old preprocessing: resize straight to SxS, ignoring aspect ratio."""

    def _preprocess(self, frame):
        S = self.size
        img = cv2.cvtColor(cv2.resize(frame, (S, S)), cv2.COLOR_BGR2RGB)
        x = (img / 255.0).astype(np.float32)
        if self.chw:
            x = x.transpose(2, 0, 1)
        # scale/pad chosen so boxes map back onto the un-squashed frame
        h, w = frame.shape[:2]
        self._sx, self._sy = S / w, S / h
        return np.expand_dims(x, 0), 1.0, 0.0, 0.0

    def _decode(self, raw, conf_thr, target_class, r, px, py, ow, oh):
        dets = super()._decode(raw, conf_thr, target_class, r, px, py,
                               self.size, self.size)
        return [(int(x1 / self._sx), int(y1 / self._sy),
                 int(x2 / self._sx), int(y2 / self._sy), c, k)
                for (x1, y1, x2, y2, c, k) in dets]


def score(model, conf=0.25):
    hits, ious, fp, gts = 0, [], 0, 0
    for f in IMAGES:
        img = cv2.imread(str(f))
        h, w = img.shape[:2]
        truth = gt_boxes(f.parent.parent / "labels" / f"{f.stem}.txt", w, h)
        pred = [np.array(d[:4], dtype=float) for d in model(img, conf)]
        gts += len(truth)
        used = set()
        for g in truth:
            best, bi = 0.0, -1
            for i, p in enumerate(pred):
                if i not in used and iou(g, p) > best:
                    best, bi = iou(g, p), i
            if best >= 0.5:
                hits += 1
                ious.append(best)
                used.add(bi)
        fp += len(pred) - len(used)
    return hits / max(gts, 1), (np.mean(ious) if ious else 0.0), fp / len(IMAGES)


def main():
    models = [Path(p) for p in sys.argv[1:]] or [DEFAULT_MODEL]
    print(f"{len(IMAGES)} test images\n")
    print(f"{'model':<26}{'preprocessing':<20}{'recall':>8}{'mean IoU':>10}{'FP/img':>9}")
    print("-" * 73)
    for path in models:
        if not path.exists():
            sys.exit(f"missing {path}")
        for label, cls in (("letterbox (fixed)", Model),
                           ("squash (old)", SquashModel)):
            r, i, f = score(cls(str(path)))
            print(f"{path.name:<26}{label:<20}{r:>8.3f}{i:>10.3f}{f:>9.2f}")


if __name__ == "__main__":
    main()
