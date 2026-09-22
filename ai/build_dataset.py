"""
Rebuild the Landing-Pad dataset with a leak-free, scene-aware split.

Why this exists
---------------
The Roboflow export splits images randomly. But the 295 exported images come
from only 123 source photos, and those 123 photos are three near-continuous
bursts:

    A  1739_png     .. 1778_png       (38)  pad alone on a dark background
    B  IMG_1659_jpeg.. IMG_1705_jpeg  (46)  sports hall floor, pad far away
    C  IMG_1739_JPG .. IMG_1778_JPG   (39)  office / close-ups

Consecutive frames in a burst are near-identical. A random split therefore puts
frame N in train and frame N+1 in val, so the validation score measures
memorisation, not generalisation.

This script splits each burst into *contiguous blocks* instead, and drops the
frames immediately adjacent to a held-out block so no train frame is a
near-duplicate of a val/test frame.

Labels are Roboflow polygon (segment) format; they are copied through
unchanged. Ultralytics derives tight boxes from the polygons, which also means
rotation augmentation produces tight boxes rather than inflated ones.
"""

import re
import shutil
from collections import defaultdict
import os
from pathlib import Path

# The Roboflow export is not in this repository. Point ROBOFLOW_EXPORT at it,
# or drop it next to this script as "Landing-Pad-1".
SRC = Path(os.environ.get("ROBOFLOW_EXPORT", Path(__file__).parent / "Landing-Pad-1"))
DST = Path(__file__).parent / "pad-dataset"

TEST_FRAC = 0.12   # contiguous block at the start of each burst
VAL_FRAC = 0.20    # contiguous block at ~55% through each burst
BUFFER = 1         # train frames dropped on each side of a held-out block


def source_key(filename: str) -> str:
    """'IMG_1694_jpeg.rf.<hash>.jpg' -> 'IMG_1694_jpeg' (the original photo)."""
    return filename.split(".rf.")[0]


def sequence_and_frame(stem: str):
    """'IMG_1694_jpeg' -> ('IMG_jpeg', 1694). Groups a burst, orders within it."""
    m = re.match(r"^(?:(IMG)_)?(\d+)_(\w+)$", stem)
    prefix, number, ext = m.group(1) or "", m.group(2), m.group(3)
    return f"{prefix}_{ext}", int(number)


def main():
    # source photo -> every exported (augmented) copy of it
    copies = defaultdict(list)
    for split in ("train", "valid", "test"):
        for img in sorted((SRC / split / "images").glob("*")):
            copies[source_key(img.name)].append(img)

    # group source photos into bursts, ordered by frame number
    bursts = defaultdict(list)
    for stem in copies:
        seq, frame = sequence_and_frame(stem)
        bursts[seq].append((frame, stem))
    for seq in bursts:
        bursts[seq].sort()

    assignment = {}
    for seq, frames in sorted(bursts.items()):
        n = len(frames)
        n_test = max(2, round(n * TEST_FRAC))
        n_val = max(3, round(n * VAL_FRAC))
        val_start = round(n * 0.55)

        test_idx = set(range(0, n_test))
        val_idx = set(range(val_start, min(val_start + n_val, n)))

        held = test_idx | val_idx
        buffer_idx = {j for i in held for j in (i - BUFFER, i + BUFFER)} - held

        for i, (_, stem) in enumerate(frames):
            if i in test_idx:
                assignment[stem] = "test"
            elif i in val_idx:
                assignment[stem] = "val"
            elif i in buffer_idx:
                assignment[stem] = "drop"   # too close to a held-out frame
            else:
                assignment[stem] = "train"

        counts = {s: sum(1 for i, (_, st) in enumerate(frames)
                         if assignment[st] == s)
                  for s in ("train", "val", "test", "drop")}
        print(f"{seq:>10}  n={n:>3}  {counts}")

    if DST.exists():
        shutil.rmtree(DST)
    for split in ("train", "val", "test"):
        (DST / split / "images").mkdir(parents=True)
        (DST / split / "labels").mkdir(parents=True)

    written = defaultdict(int)
    for stem, split in assignment.items():
        if split == "drop":
            continue
        # val/test keep one clean copy; train keeps every Roboflow variant
        srcs = copies[stem] if split == "train" else copies[stem][:1]
        for img in srcs:
            lbl = img.parent.parent / "labels" / f"{img.stem}.txt"
            shutil.copy2(img, DST / split / "images" / img.name)
            shutil.copy2(lbl, DST / split / "labels" / lbl.name)
            written[split] += 1

    (DST / "data.yaml").write_text(
        f"path: {DST}\n"
        "train: train/images\n"
        "val: val/images\n"
        "test: test/images\n"
        "\n"
        "nc: 1\n"
        "names: ['landingPad']\n"
    )

    print()
    print("images written:", dict(written))
    n_src = sum(1 for s in assignment.values() if s != "drop")
    print(f"source photos used: {n_src}/{len(assignment)} "
          f"({sum(1 for s in assignment.values() if s == 'drop')} dropped as buffer)")
    print("->", DST / "data.yaml")


if __name__ == "__main__":
    main()
