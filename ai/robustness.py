"""
Robustness probes along the two axes a landing drone actually moves in.

mAP on this dataset is close to saturated, and it cannot say much anyway: every
photo shows the same pad in one of three rooms, so a held-out frame is not a
held-out *scene*. These two probes ask questions the dataset can answer:

  rotation  A multirotor holds no particular heading, so the pad reaches the
            camera rotated by an arbitrary angle. Images are rotated about
            their centre and the polygon labels are rotated with them, giving
            exact ground truth rather than an inflated axis-aligned box.

  altitude  Higher means fewer pixels on the pad. Rather than synthesising
            pixels, the same image is run at a smaller inference size, which
            shrinks the pad in the network's view by exactly that ratio. The
            column heading gives the median pad width in pixels at that size.

Recall is reported at a fixed confidence, because for a landing controller a
missed pad is what matters, not the ranking quality that mAP measures.
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

HERE = Path(__file__).parent
SPLIT = "test"
CONF = 0.25
IOU_MATCH = 0.5
ANGLES = (0, 30, 45, 90, 135, 180)
ZOOMS = (1.0, 0.5, 0.3, 0.2, 0.12, 0.08)


def zoom_out(img, boxes, f):
    """Shrink the frame into a canvas of floor-coloured background.

    Simulates climbing: the pad keeps its appearance but occupies f times as
    much of the frame. The filler is the median colour of the image border,
    which for these uniform sports-hall and office floors is a fair stand-in
    for the surroundings that fall outside a standing-height photo. Reflection
    padding was rejected because it would mirror the pad itself into the
    filler, creating unlabelled duplicates.
    """
    if f >= 1.0:
        return img, boxes
    h, w = img.shape[:2]
    nw, nh = max(8, int(w * f)), max(8, int(h * f))
    small = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)

    border = np.concatenate([img[0], img[-1], img[:, 0], img[:, -1]])
    bg = np.median(border, axis=0)
    canvas = np.clip(np.full((h, w, 3), bg, np.float32)
                     + np.random.default_rng(0).normal(0, 4, (h, w, 3)),
                     0, 255).astype(np.uint8)
    ox, oy = (w - nw) // 2, (h - nh) // 2
    canvas[oy:oy + nh, ox:ox + nw] = small
    off = np.array([ox, oy, ox, oy], dtype=float)
    return canvas, [b * f + off for b in boxes]


def motion_blur(img, k):
    kern = np.zeros((k, k), np.float32)
    kern[k // 2, :] = 1.0 / k
    return cv2.filter2D(img, -1, kern)


def jpeg(img, q):
    return cv2.imdecode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])[1],
                        cv2.IMREAD_COLOR)


# capture-path degradations, in the units a 320px network input sees
DEGRADATIONS = {
    "clean": lambda im: im,
    "blur k5": lambda im: motion_blur(im, 5),
    "blur k11": lambda im: motion_blur(im, 11),
    "jpeg q30": lambda im: jpeg(im, 30),
    "noise s12": lambda im: np.clip(
        im.astype(np.float32) + np.random.default_rng(0).normal(0, 12, im.shape),
        0, 255).astype(np.uint8),
    "all three": lambda im: jpeg(np.clip(
        motion_blur(im, 9).astype(np.float32)
        + np.random.default_rng(0).normal(0, 10, im.shape), 0, 255
    ).astype(np.uint8), 35),
}


def polygons(label_path, w, h):
    out = []
    for line in label_path.read_text().split("\n"):
        p = line.split()
        if len(p) < 5:
            continue
        v = np.array(p[1:], dtype=float)
        if len(v) == 4:
            cx, cy, bw, bh = v
            xy = np.array([[cx - bw / 2, cy - bh / 2], [cx + bw / 2, cy - bh / 2],
                           [cx + bw / 2, cy + bh / 2], [cx - bw / 2, cy + bh / 2]])
        else:
            xy = v.reshape(-1, 2)
        out.append(xy * np.array([w, h]))
    return out


def iou(a, b):
    lt, rb = np.maximum(a[:2], b[:2]), np.minimum(a[2:], b[2:])
    inter = np.prod(np.clip(rb - lt, 0, None))
    area = lambda x: max((x[2] - x[0]) * (x[3] - x[1]), 0)  # noqa: E731
    return inter / (area(a) + area(b) - inter + 1e-9)


def rotate(img, polys, deg):
    """Rotate about centre onto a canvas big enough to keep every corner."""
    h, w = img.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2, h / 2), deg, 1.0)
    cos, sin = abs(M[0, 0]), abs(M[0, 1])
    nw, nh = int(h * sin + w * cos), int(h * cos + w * sin)
    M[0, 2] += nw / 2 - w / 2
    M[1, 2] += nh / 2 - h / 2
    out = cv2.warpAffine(img, M, (nw, nh), borderValue=(114, 114, 114))
    boxes = []
    for xy in polys:
        p = np.hstack([xy, np.ones((len(xy), 1))]) @ M.T
        boxes.append(np.array([p[:, 0].min(), p[:, 1].min(),
                               p[:, 0].max(), p[:, 1].max()]))
    return out, boxes


def recall(model, images, angle=0, imgsz=320, degrade=None, zoom=1.0):
    hit = tot = 0
    for f in images:
        img = cv2.imread(str(f))
        h, w = img.shape[:2]
        polys = polygons(f.parent.parent / "labels" / f"{f.stem}.txt", w, h)
        img, gts = rotate(img, polys, angle) if angle else (
            img, [np.array([xy[:, 0].min(), xy[:, 1].min(),
                            xy[:, 0].max(), xy[:, 1].max()]) for xy in polys])

        if zoom < 1.0:
            img, gts = zoom_out(img, gts, zoom)

        if degrade is not None:
            # degrade at the size the network sees, or a blur radius measured
            # in input pixels would be meaningless against a 2048px photo
            s = imgsz / max(img.shape[:2])
            img = cv2.resize(img, (max(1, int(img.shape[1] * s)),
                                   max(1, int(img.shape[0] * s))))
            gts = [g * s for g in gts]
            img = degrade(img)

        # Klasse 0 ist in Ein- wie Zwei-Klassen-Modellen das Pad; ohne Filter
        # zaehlten beim Zwei-Klassen-Modell auch Personen als Treffer mit.
        r = model.predict(img, imgsz=imgsz, conf=CONF, device=DEVICE,
                          classes=[0], verbose=False)[0]
        pred = r.boxes.xyxy.cpu().numpy() if r.boxes is not None else np.zeros((0, 4))
        for gt in gts:
            tot += 1
            if len(pred) and max(iou(gt, p) for p in pred) >= IOU_MATCH:
                hit += 1
    return hit / max(tot, 1), tot


def median_pad_px(images, zoom, imgsz=320):
    """Median pad width in input pixels after zooming out to `zoom`."""
    fr = []
    for f in images:
        img = cv2.imread(str(f))
        h, w = img.shape[:2]
        for xy in polygons(f.parent.parent / "labels" / f"{f.stem}.txt", w, h):
            side = max(xy[:, 0].max() - xy[:, 0].min(),
                       xy[:, 1].max() - xy[:, 1].min())
            fr.append(side / max(w, h))
    return np.median(fr) * imgsz * zoom


def main(weight_paths):
    images = sorted((HERE / "pad-dataset" / SPLIT / "images").glob("*"))
    models = [(Path(w).parent.parent.name, YOLO(w)) for w in weight_paths]

    def table(title, header, cols, call):
        print(f"\n{'=' * 84}\n{title}\n{'=' * 84}")
        print(f"{'run':<20}" + "".join(f"{c:>10}" for c in header))
        print("-" * 84)
        for name, m in models:
            cells = [call(m, c) for c in cols]
            print(f"{name:<20}" + "".join(f"{c:>10.2f}" for c in cells)
                  + f"   worst {min(cells):.2f}")

    print(f"\nRecall on {SPLIT} ({len(images)} images), conf>={CONF}, "
          f"IoU>={IOU_MATCH}, inference at 320px")

    table("A. ROTATION  (drone yaw is arbitrary)",
          [f"{a}deg" for a in ANGLES], ANGLES,
          lambda m, a: recall(m, images, angle=a)[0])

    print(f"\n{'=' * 84}\nB. ALTITUDE  (zoom out; pad shrinks in frame)\n{'=' * 84}")
    print(f"{'run':<20}" + "".join(f"{f'x{z}':>10}" for z in ZOOMS))
    print(f"{'median pad px':<20}"
          + "".join(f"{median_pad_px(images, z):>10.0f}" for z in ZOOMS))
    print("-" * 84)
    for name, m in models:
        cells = [recall(m, images, zoom=z)[0] for z in ZOOMS]
        print(f"{name:<20}" + "".join(f"{c:>10.2f}" for c in cells)
              + f"   worst {min(cells):.2f}")

    table("C. CAPTURE PATH  (blur / noise / JPEG at input resolution)",
          list(DEGRADATIONS), list(DEGRADATIONS.values()),
          lambda m, fn: recall(m, images, degrade=fn)[0])

    print(f"\n{'=' * 84}\nD. COMBINED  (what an actual approach looks like)\n{'=' * 84}")
    combos = [("nominal", 0, 1.0, None),
              ("yaw45 x0.3", 45, 0.3, None),
              ("yaw45 x0.3 blur", 45, 0.3, DEGRADATIONS["blur k5"]),
              ("yaw135 x0.2 all", 135, 0.2, DEGRADATIONS["all three"]),
              ("yaw90 x0.12 all", 90, 0.12, DEGRADATIONS["all three"])]
    print(f"{'run':<20}" + "".join(f"{c[0]:>18}" for c in combos))
    print("-" * 84)
    for name, m in models:
        cells = [recall(m, images, angle=a, degrade=d, zoom=z)[0]
                 for _, a, z, d in combos]
        print(f"{name:<20}" + "".join(f"{c:>18.2f}" for c in cells))


if __name__ == "__main__":
    main(sys.argv[1:])
