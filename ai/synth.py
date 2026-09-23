"""
Manufacture the training data the dataset is missing: distant pads.

The measured failure is altitude — recall halves once the pad is ~35 px across,
because the median training pad covers 47% of the image side and every photo was
taken by hand from standing height. Augmentation can only shrink what is
already in frame; it cannot invent a pad sitting on a distant patch of floor.

So: cut the pad out of the images that show it cleanly, using the polygon
labels as an exact mask, and paste it back onto floor regions of *other*
training images at small scale, arbitrary rotation and an oblique tilt.

Only train-split images are used as either pad source or background, so the
held-out splits stay clean.

    python3 synth.py [n_per_background]
"""

import random
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
SRC = HERE / "pad-dataset"
DST = HERE / "pad-dataset-synth"

TARGET_SIDE = (0.03, 0.14)   # pad side as a fraction of image side
MAX_TRIES = 40


def read_polys(path):
    out = []
    for line in path.read_text().split("\n"):
        p = line.split()
        if len(p) < 5:
            continue
        v = np.array(p[1:], dtype=float)
        if len(v) == 4:
            cx, cy, w, h = v
            out.append(np.array([[cx - w / 2, cy - h / 2], [cx + w / 2, cy - h / 2],
                                 [cx + w / 2, cy + h / 2], [cx - w / 2, cy + h / 2]]))
        else:
            out.append(v.reshape(-1, 2))
    return out


def write_polys(path, polys, w, h):
    lines = []
    for xy in polys:
        norm = (xy / np.array([w, h])).clip(0, 1).reshape(-1)
        lines.append("0 " + " ".join(f"{v:.6f}" for v in norm))
    path.write_text("\n".join(lines) + "\n")


def cut_pad(img, poly):
    """Return the pad's BGR crop and a soft alpha mask from its polygon."""
    h, w = img.shape[:2]
    pts = poly.astype(np.int32)
    mask = np.zeros((h, w), np.uint8)
    cv2.fillPoly(mask, [pts], 255)
    x, y, bw, bh = cv2.boundingRect(pts)
    if bw < 40 or bh < 40:
        return None, None
    # reject donors whose polygon is a thin sliver of its own box: those are
    # grazing-angle shots that cut mostly background, not pad
    if cv2.contourArea(pts) < 0.45 * bw * bh:
        return None, None
    return img[y:y + bh, x:x + bw].copy(), mask[y:y + bh, x:x + bw].copy()


def warp_pad(crop, mask, side, rng):
    """Scale to `side` px, rotate freely, and tilt as if seen from an angle."""
    h, w = crop.shape[:2]
    s = side / max(h, w)
    crop = cv2.resize(crop, (max(4, int(w * s)), max(4, int(h * s))))
    mask = cv2.resize(mask, (crop.shape[1], crop.shape[0]))

    # oblique view: squash one axis, then rotate in-plane. The floor is never
    # seen so close to edge-on that the pad becomes a line, and such slivers
    # are dominated by mask-edge artefacts rather than by the pad itself.
    squash = rng.uniform(0.6, 1.0)
    crop = cv2.resize(crop, (crop.shape[1], max(3, int(crop.shape[0] * squash))))
    mask = cv2.resize(mask, (crop.shape[1], crop.shape[0]))

    h, w = crop.shape[:2]
    ang = rng.uniform(0, 360)
    M = cv2.getRotationMatrix2D((w / 2, h / 2), ang, 1.0)
    cos, sin = abs(M[0, 0]), abs(M[0, 1])
    nw, nh = int(h * sin + w * cos) + 2, int(h * cos + w * sin) + 2
    M[0, 2] += nw / 2 - w / 2
    M[1, 2] += nh / 2 - h / 2
    crop = cv2.warpAffine(crop, M, (nw, nh))
    mask = cv2.warpAffine(mask, M, (nw, nh))
    return crop, mask


def paste(bg, crop, mask, x, y, rng):
    """Alpha-blend with a feathered edge and rough brightness match."""
    h, w = crop.shape[:2]
    region = bg[y:y + h, x:x + w]
    if region.shape[:2] != (h, w):
        return None

    # match the pad's overall exposure to this patch of floor
    m = mask > 127
    if m.sum() < 12:
        return None
    gain = (region.mean() + 1e-6) / (crop[m].mean() + 1e-6)
    gain = float(np.clip(gain, 0.55, 1.6)) * rng.uniform(0.9, 1.1)
    crop = np.clip(crop.astype(np.float32) * gain, 0, 255)

    alpha = cv2.GaussianBlur(mask, (5, 5), 0).astype(np.float32)[..., None] / 255.0
    bg[y:y + h, x:x + w] = (crop * alpha + region * (1 - alpha)).astype(np.uint8)
    return True


def main(n_per_bg=2, seed=0):
    rng = random.Random(seed)
    nprng = np.random.default_rng(seed)

    train_imgs = sorted((SRC / "train" / "images").glob("*"))
    lbl_of = lambda f: f.parent.parent / "labels" / f"{f.stem}.txt"  # noqa: E731

    # pad donors: the cut-out burst, where the pad is large and cleanly masked
    donors = []
    for f in train_imgs:
        if ".rf." in f.name and not f.name.split(".rf.")[0].startswith("IMG_"):
            img = cv2.imread(str(f))
            h, w = img.shape[:2]
            for poly in read_polys(lbl_of(f)):
                crop, mask = cut_pad(img, poly * np.array([w, h]))
                if crop is not None:
                    donors.append((crop, mask))
    # backgrounds: the hall burst, which has real floor with room around the pad
    backgrounds = [f for f in train_imgs
                   if f.name.split(".rf.")[0].endswith("_jpeg")]
    print(f"{len(donors)} pad cut-outs, {len(backgrounds)} background images")
    if not donors or not backgrounds:
        sys.exit("nothing to composite")

    if DST.exists():
        shutil.rmtree(DST)
    for split in ("train", "val", "test"):
        shutil.copytree(SRC / split, DST / split,
                        ignore=shutil.ignore_patterns("*.cache"))

    made = 0
    for bf in backgrounds:
        for k in range(n_per_bg):
            bg = cv2.imread(str(bf))
            h, w = bg.shape[:2]
            polys = [p * np.array([w, h]) for p in read_polys(lbl_of(bf))]
            existing = [cv2.boundingRect(p.astype(np.int32)) for p in polys]

            added = 0
            for _ in range(rng.randint(1, 3)):
                crop, mask = donors[rng.randrange(len(donors))]
                side = int(max(h, w) * nprng.uniform(*TARGET_SIDE))
                pc, pm = warp_pad(crop, mask, side, nprng)
                ph, pw = pc.shape[:2]
                if ph >= h or pw >= w or min(ph, pw) < 12:
                    continue
                for _ in range(MAX_TRIES):
                    x, y = rng.randrange(w - pw), rng.randrange(h - ph)
                    # keep clear of the real pad and of anything already pasted
                    if all(not (x < ex + ew and ex < x + pw
                                and y < ey + eh and ey < y + ph)
                           for ex, ey, ew, eh in existing):
                        break
                else:
                    continue
                if paste(bg, pc, pm, x, y, nprng) is None:
                    continue
                ys, xs = np.nonzero(pm > 127)
                poly = np.array([[x + xs.min(), y + ys.min()], [x + xs.max(), y + ys.min()],
                                 [x + xs.max(), y + ys.max()], [x + xs.min(), y + ys.max()]],
                                dtype=float)
                polys.append(poly)
                existing.append((x, y, pw, ph))
                added += 1

            if not added:
                continue
            name = f"synth_{made:04d}_{bf.stem[:14]}"
            cv2.imwrite(str(DST / "train" / "images" / f"{name}.jpg"), bg)
            write_polys(DST / "train" / "labels" / f"{name}.txt", polys, w, h)
            made += 1

    (DST / "data.yaml").write_text(
        f"path: {DST}\ntrain: train/images\nval: val/images\ntest: test/images\n"
        "\nnc: 1\nnames: ['landingPad']\n")
    n_train = len(list((DST / "train" / "images").glob("*")))
    print(f"wrote {made} composited images -> {n_train} training images total")
    print("->", DST / "data.yaml")


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 2)
