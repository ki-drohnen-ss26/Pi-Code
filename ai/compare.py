"""
Compare trained landing-pad models.

Reports the usual mAP, but also breaks results down by how big the pad is in
the frame. That distinction is the whole game for a landing drone: a pad that
fills the frame is the last half-metre of the descent, while a pad 30 px across
is the approach, and only the second one is hard. A single mAP number hides
which of the two a model is good at.

Buckets are on sqrt(box area) as a fraction of the image side:
    far    < 0.15   pad is a small patch  -> approach / search
    mid    < 0.40
    near   >= 0.40  pad dominates frame   -> touchdown
"""

import sys
import warnings
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore")

from ultralytics import YOLO  # noqa: E402

HERE = Path(__file__).parent
DATA = str(HERE / "pad-dataset" / "data.yaml")
CONF = 0.25
IOU_MATCH = 0.5


def gt_boxes(label_path, w, h):
    """Roboflow polygon labels -> xyxy pixel boxes."""
    out = []
    for line in label_path.read_text().split("\n"):
        p = line.split()
        if len(p) < 5:
            continue
        v = np.array(p[1:], dtype=float)
        if len(v) == 4:
            cx, cy, bw, bh = v
            x1, y1, x2, y2 = cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2
        else:
            xy = v.reshape(-1, 2)
            x1, y1 = xy[:, 0].min(), xy[:, 1].min()
            x2, y2 = xy[:, 0].max(), xy[:, 1].max()
        out.append([x1 * w, y1 * h, x2 * w, y2 * h])
    return np.array(out).reshape(-1, 4)


def iou_matrix(a, b):
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    lt = np.maximum(a[:, None, :2], b[None, :, :2])
    rb = np.minimum(a[:, None, 2:], b[None, :, 2:])
    inter = np.prod(np.clip(rb - lt, 0, None), axis=2)
    area = lambda x: np.prod(x[:, 2:] - x[:, :2], axis=1)  # noqa: E731
    return inter / (area(a)[:, None] + area(b)[None, :] - inter + 1e-9)


SIZE_BUCKETS = ("far", "mid", "near")
# the three photo bursts; only two of them look anything like a drone's view
SCENES = {
    "cutout(dark bg)": "pad alone on black - not a view a drone ever gets",
    "hall(far)": "sports hall floor, pad small and oblique",
    "office(near)": "office / close-ups",
}


def bucket(box, w, h):
    frac = np.sqrt(abs((box[2] - box[0]) * (box[3] - box[1])) / (w * h))
    return "far" if frac < 0.15 else ("mid" if frac < 0.40 else "near")


def scene_of(name):
    stem = name.split(".rf.")[0]
    if stem.endswith("_jpeg"):
        return "hall(far)"
    if stem.endswith("_JPG"):
        return "office(near)"
    return "cutout(dark bg)"


def evaluate(model, split, imgsz):
    """Recall + localisation quality, bucketed by pad size and by scene."""
    img_dir = HERE / "pad-dataset" / split / "images"
    files = sorted(img_dir.glob("*"))
    blank = lambda keys: {k: {"n": 0, "hit": 0, "iou": []} for k in keys}  # noqa: E731
    stats, scenes = blank(SIZE_BUCKETS), blank(SCENES)
    fp = 0

    results = model.predict([str(f) for f in files], imgsz=imgsz, conf=CONF,
                            device="mps", verbose=False, stream=True)
    for f, r in zip(files, results):
        h, w = r.orig_shape
        gt = gt_boxes(f.parent.parent / "labels" / f"{f.stem}.txt", w, h)
        pred = r.boxes.xyxy.cpu().numpy() if r.boxes is not None else np.zeros((0, 4))
        m = iou_matrix(gt, pred)
        matched = set()
        sc = scene_of(f.name)
        for gi in range(len(gt)):
            cells = (stats[bucket(gt[gi], w, h)], scenes[sc])
            for c in cells:
                c["n"] += 1
            if m.shape[1]:
                pi = int(m[gi].argmax())
                if m[gi, pi] >= IOU_MATCH and pi not in matched:
                    matched.add(pi)
                    for c in cells:
                        c["hit"] += 1
                        c["iou"].append(float(m[gi, pi]))
        fp += len(pred) - len(matched)

    return stats, scenes, fp, len(files)


def main(weight_paths):
    rows = []
    for wp in weight_paths:
        wp = Path(wp)
        model = YOLO(str(wp))
        label = wp.parent.parent.name
        for imgsz in (320, 416):
            r_val = model.val(data=DATA, imgsz=imgsz, split="val", device="mps",
                              verbose=False, plots=False, project=str(HERE / "eval"),
                              name=f"{label}_{imgsz}_val", exist_ok=True)
            r_test = model.val(data=DATA, imgsz=imgsz, split="test", device="mps",
                               verbose=False, plots=False, project=str(HERE / "eval"),
                               name=f"{label}_{imgsz}_test", exist_ok=True)
            stats, scenes, fp, n = evaluate(model, "test", imgsz)
            rows.append((label, imgsz, r_val.box.map50, r_val.box.map,
                         r_test.box.map50, r_test.box.map, stats, scenes, fp, n))

    print(f"\n{'=' * 96}")
    print(f"{'run':<18}{'imgsz':>6} | {'val mAP50':>9}{'val 50-95':>10} | "
          f"{'test mAP50':>10}{'test 50-95':>11} | {'FP/img':>7}")
    print("-" * 96)
    for label, imgsz, v50, v, t50, t, _stats, _scenes, fp, n in rows:
        print(f"{label:<18}{imgsz:>6} | {v50:>9.3f}{v:>10.3f} | "
              f"{t50:>10.3f}{t:>11.3f} | {fp / n:>7.2f}")

    def table(title, key_index, keys, headers):
        print(f"\n{'=' * 96}\n{title}  (conf>={CONF:.2f}, IoU>={IOU_MATCH:.1f})")
        print(f"{'run':<18}{'imgsz':>6} | " + " | ".join(f"{h:^22}" for h in headers))
        print("-" * 96)
        for row in rows:
            label, imgsz, group = row[0], row[1], row[key_index]
            cells = []
            for k in keys:
                s = group[k]
                if s["n"] == 0:
                    cells.append(f"{'-':^22}")
                else:
                    miou = np.mean(s["iou"]) if s["iou"] else 0.0
                    cells.append(f"{s['hit']}/{s['n']} rec {s['hit'] / s['n']:.2f} "
                                 f"IoU {miou:.2f}".center(22))
            print(f"{label:<18}{imgsz:>6} | " + " | ".join(cells))

    table("Test recall by pad size in frame", 6, SIZE_BUCKETS,
          ("far (<15% of side)", "mid", "near (>40%)"))
    table("Test recall by scene", 7, list(SCENES), list(SCENES))


if __name__ == "__main__":
    main(sys.argv[1:])
