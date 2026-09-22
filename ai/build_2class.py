"""
Zwei-Klassen-Datensatz: landingPad + person.

Warum ueberhaupt: der IMX500 der AI Camera haelt genau *ein* Netz auf dem
Sensor. Der bisherige Aufbau laesst pro Frame zwei Modelle laufen, das geht
dort nicht. Und auch auf der CPU des Pi Zero halbiert ein gemeinsames Netz die
Last — genau der Engpass, der zu SKIP_FRAMES = 2 gefuehrt hat.

Zwei Quellen:

  Pad-Bilder (pad-dataset-neg)   Pad-Labels als Klasse 0. Personen darin werden
                                 von YOLO11n vorgelabelt (Klasse 1) — sonst
                                 lernt das Netz, dass auf Pad-Bildern nie
                                 Personen sind, und die Buero-Serie enthaelt
                                 welche.
  people-recovered (760 Bilder)  echte Personen-Labels als Klasse 1. Diese
                                 Bilder enthalten keine Pads und wirken damit
                                 zugleich als Negativbeispiele fuer Klasse 0.

Die 66 vorgelabelten Personen der Pad-Bilder allein waeren viel zu duenn (und
im Test-Split kam keine einzige vor, das Ergebnis waere nicht messbar gewesen).
Erst die 2784 echten Personen-Instanzen machen die Klasse trainierbar.

Labelformat: die Pad-Labels sind Polygone. Ultralytics behandelt eine Datei als
Segment-Datei, sobald *eine* Zeile mehr als 6 Werte hat — 5-Werte-Boxen in
derselben Datei wuerden dann falsch gelesen. Deshalb werden die Personen-Boxen
als 4-Punkt-Polygone geschrieben statt die Pad-Polygone zu Boxen zu verflachen:
so bleiben die Pad-Boxen unter der 180-Grad-Rotation exakt.

    python3 build_2class.py
"""

import random
import shutil
import warnings
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore")

HERE = Path(__file__).parent
PADS = HERE / "pad-dataset-neg"
PEOPLE = HERE / "people-recovered" / "test"
DST = HERE / "pad-person-dataset"
PERSON_CONF = 0.5
SPLIT = (0.80, 0.10)          # train, val; Rest test


def parse(path):
    """-> Liste von (cls, np.ndarray Nx2 normalisierte Polygonpunkte)."""
    out = []
    for line in path.read_text().split("\n"):
        p = line.split()
        if len(p) < 5:
            continue
        c = int(p[0])
        v = np.array(p[1:], dtype=float)
        if len(v) == 4:
            cx, cy, w, h = v
            v = np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy - h / 2,
                          cx + w / 2, cy + h / 2, cx - w / 2, cy + h / 2])
        out.append((c, v.reshape(-1, 2)))
    return out


def write(path, rows):
    lines = []
    for c, xy in rows:
        flat = np.clip(xy, 0.0, 1.0).reshape(-1)
        lines.append(f"{c} " + " ".join(f"{v:.6f}" for v in flat))
    path.write_text("\n".join(lines) + ("\n" if lines else ""))


def main(seed=0):
    from ultralytics import YOLO

    if DST.exists():
        shutil.rmtree(DST)
    for s in ("train", "val", "test"):
        (DST / s / "images").mkdir(parents=True)
        (DST / s / "labels").mkdir(parents=True)

    # --- Pad-Bilder: Pad-Labels behalten, Personen dazu-vorlabeln ------------
    person_model = YOLO(str(HERE / "yolo11n.pt"))
    added = 0
    for split in ("train", "val", "test"):
        for img in sorted((PADS / split / "images").glob("*")):
            rows = parse(img.parent.parent / "labels" / f"{img.stem}.txt")
            r = person_model.predict(str(img), imgsz=640, conf=PERSON_CONF,
                                     classes=[0], device="mps", verbose=False)[0]
            h, w = r.orig_shape
            for b in (r.boxes.xyxy.cpu().numpy() if r.boxes is not None else []):
                x1, y1, x2, y2 = b / np.array([w, h, w, h])
                rows.append((1, np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])))
                added += 1
            shutil.copy2(img, DST / split / "images" / img.name)
            write(DST / split / "labels" / f"{img.stem}.txt", rows)
    print(f"Pad-Bilder uebernommen, {added} Personen vorgelabelt")

    # --- Personen-Bilder: Klasse 0 -> 1, eigener Split -----------------------
    files = sorted((PEOPLE / "images").glob("*"))
    random.Random(seed).shuffle(files)
    n_tr, n_va = int(len(files) * SPLIT[0]), int(len(files) * SPLIT[1])
    parts = {"train": files[:n_tr], "val": files[n_tr:n_tr + n_va],
             "test": files[n_tr + n_va:]}
    n_person = 0
    for split, group in parts.items():
        for img in group:
            lbl = PEOPLE / "labels" / f"{img.stem}.txt"
            if not lbl.exists():
                continue
            rows = [(1, xy) for _c, xy in parse(lbl)]   # Quelle hat nur Klasse 0
            n_person += len(rows)
            shutil.copy2(img, DST / split / "images" / img.name)
            write(DST / split / "labels" / f"{img.stem}.txt", rows)
    print(f"Personen-Bilder: { {k: len(v) for k, v in parts.items()} }, "
          f"{n_person} Instanzen")

    (DST / "data.yaml").write_text(
        f"path: {DST}\ntrain: train/images\nval: val/images\ntest: test/images\n"
        "\nnc: 2\nnames: ['landingPad', 'person']\n")

    print()
    for s in ("train", "val", "test"):
        imgs = list((DST / s / "images").glob("*"))
        c = {0: 0, 1: 0}
        for f in (DST / s / "labels").glob("*.txt"):
            for cls, _ in parse(f):
                c[cls] = c.get(cls, 0) + 1
        print(f"{s:<6} Bilder={len(imgs):<5} Pads={c[0]:<5} Personen={c[1]}")
    print("\n->", DST / "data.yaml")


if __name__ == "__main__":
    main()
