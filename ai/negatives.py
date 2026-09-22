"""
Hard negatives: pad-free crops from the same rooms.

Only 3 of 175 training images contain no pad, so "there is no pad here" was
never a supported answer and the detector always fires on the most pad-like
object in view — a rucksack, a poster, a chair back. That is the worse failure
for a landing controller: a missed pad delays the landing, a false pad puts the
aircraft on someone's desk.

The fix is background images. Rather than pull in an unrelated image corpus,
the crops come from the project's own photos, with any region overlapping a
labelled pad excluded. Same rooms, same camera, same lighting, same clutter —
which is exactly what makes them hard negatives rather than easy ones.

Train crops come from the train split and are added to training; val/test crops
come from the held-out splits and form a false-positive benchmark the model has
never seen.

    python3 negatives.py
"""

import shutil
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
SRC = HERE / "pad-dataset"
DST = HERE / "pad-dataset-neg"          # training set + negatives
BENCH = HERE / "pad-negatives-bench"    # held-out false-positive benchmark

CROPS_PER_TRAIN_IMAGE = 1
CROPS_PER_HELDOUT_IMAGE = 4
CROP_FRAC = (0.35, 0.75)                # crop side as a fraction of image side
MAX_TRIES = 60


def pad_boxes(label_path, w, h):
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


def crop_without_pad(img, boxes, rng, tries=MAX_TRIES):
    """A random window that does not touch any labelled pad."""
    h, w = img.shape[:2]
    for _ in range(tries):
        f = rng.uniform(*CROP_FRAC)
        cw, ch = int(w * f), int(h * f)
        if cw < 64 or ch < 64:
            continue
        x, y = rng.integers(0, w - cw + 1), rng.integers(0, h - ch + 1)
        rect = (x, y, x + cw, y + ch)
        # reject on any overlap at all: a sliver of pad would be an unlabelled
        # positive, which teaches exactly the wrong thing
        if all(not (rect[0] < b[2] and b[0] < rect[2]
                    and rect[1] < b[3] and b[1] < rect[3]) for b in boxes):
            return img[y:y + ch, x:x + cw]
    return None


def harvest(split, n_per_image, out_images, out_labels, rng, prefix):
    made = 0
    for f in sorted((SRC / split / "images").glob("*")):
        img = cv2.imread(str(f))
        if img is None:
            continue
        h, w = img.shape[:2]
        boxes = pad_boxes(f.parent.parent / "labels" / f"{f.stem}.txt", w, h)
        for k in range(n_per_image):
            crop = crop_without_pad(img, boxes, rng)
            if crop is None:
                continue
            name = f"{prefix}_{made:04d}"
            cv2.imwrite(str(out_images / f"{name}.jpg"), crop)
            (out_labels / f"{name}.txt").write_text("")   # background image
            made += 1
    return made


def main(seed=0):
    rng = np.random.default_rng(seed)

    if DST.exists():
        shutil.rmtree(DST)
    for split in ("train", "val", "test"):
        shutil.copytree(SRC / split, DST / split,
                        ignore=shutil.ignore_patterns("*.cache"))

    n_train = harvest("train", CROPS_PER_TRAIN_IMAGE,
                      DST / "train" / "images", DST / "train" / "labels",
                      rng, "neg")
    (DST / "data.yaml").write_text(
        f"path: {DST}\ntrain: train/images\nval: val/images\ntest: test/images\n"
        "\nnc: 1\nnames: ['landingPad']\n")
    total = len(list((DST / "train" / "images").glob("*")))
    print(f"training negatives: {n_train}  -> {total} train images "
          f"({n_train / total:.0%} background)")

    # held-out benchmark: crops from val+test, which no run has trained on
    if BENCH.exists():
        shutil.rmtree(BENCH)
    (BENCH / "images").mkdir(parents=True)
    (BENCH / "labels").mkdir(parents=True)
    n_bench = sum(harvest(s, CROPS_PER_HELDOUT_IMAGE,
                          BENCH / "images", BENCH / "labels", rng, f"bench_{s}")
                  for s in ("val", "test"))
    print(f"benchmark negatives: {n_bench}  -> {BENCH}")


if __name__ == "__main__":
    main()
