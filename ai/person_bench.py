"""
Taugt die Personen-Klasse des Zwei-Klassen-Modells als Ersatz für YOLO11n?

Das Zwei-Klassen-Modell soll beide bisherigen Detektoren ersetzen. Für das Pad
ist der Vergleich klar (gegen Run F). Für Personen ist die Messlatte das
Standardmodell, das heute auf dem Pi läuft — nicht ein abstrakter mAP-Wert.

Gemessen auf dem Test-Split des gemischten Datensatzes (233 Personen in 92
Bildern) bei fester Konfidenz, weil für die "ZONE BLOCKED"-Logik zählt, ob eine
Person gefunden wird, nicht wie gut sie gerankt ist.

    python3 person_bench.py
"""

import warnings
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore")

from ultralytics import YOLO  # noqa: E402

HERE = Path(__file__).parent
TEST = HERE / "pad-person-dataset" / "test"
THRESHOLDS = (0.25, 0.4, 0.5, 0.6)
IOU_MATCH = 0.5
PERSON_CLASS_IN = 1          # Klasse im gemischten Datensatz


def gt_person_boxes(label_path, w, h):
    out = []
    for line in label_path.read_text().split("\n"):
        p = line.split()
        if len(p) < 5 or int(p[0]) != PERSON_CLASS_IN:
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


def score(model, cls_id, imgsz):
    files = sorted((TEST / "images").glob("*"))
    hits = {t: 0 for t in THRESHOLDS}
    fp = {t: 0 for t in THRESHOLDS}
    total = 0
    for f in files:
        r = model.predict(str(f), imgsz=imgsz, conf=min(THRESHOLDS),
                          classes=[cls_id], device="mps", verbose=False)[0]
        boxes = r.boxes.xyxy.cpu().numpy() if r.boxes is not None else np.zeros((0, 4))
        confs = r.boxes.conf.cpu().numpy() if r.boxes is not None else np.array([])
        h, w = r.orig_shape
        gts = gt_person_boxes(TEST / "labels" / f"{f.stem}.txt", w, h)
        total += len(gts)
        for t in THRESHOLDS:
            keep = confs >= t
            bs = boxes[keep]
            used = set()
            for g in gts:
                best, bi = 0.0, -1
                for i, b in enumerate(bs):
                    if i not in used and iou(g, b) > best:
                        best, bi = iou(g, b), i
                if best >= IOU_MATCH:
                    hits[t] += 1
                    used.add(bi)
            fp[t] += len(bs) - len(used)
    return hits, fp, total, len(files)


def main():
    runs = [
        ("YOLO11n (Standard, heute im Einsatz)", HERE / "yolo11n.pt", 0, 640),
        ("YOLO11n @320 (Pi-Größe)", HERE / "yolo11n.pt", 0, 320),
        ("Run G, 2 Klassen, degrees=180",
         HERE / "runs" / "G_2class_320" / "weights" / "best.pt", 1, 320),
        ("Run H, 2 Klassen, degrees=30",
         HERE / "runs" / "H_2class_mild_320" / "weights" / "best.pt", 1, 320),
    ]
    print(f"Personen im Test-Split: siehe unten, IoU>={IOU_MATCH}\n")
    print(f"{'Modell':<38}{'conf':>6}{'Recall':>9}{'FP/Bild':>10}")
    print("-" * 64)
    for name, wp, cls_id, imgsz in runs:
        hits, fp, total, n = score(YOLO(str(wp)), cls_id, imgsz)
        for t in THRESHOLDS:
            label = name if t == THRESHOLDS[0] else ""
            print(f"{label:<38}{t:>6.2f}{hits[t] / max(total, 1):>9.2f}"
                  f"{fp[t] / n:>10.2f}")
        print()
    print(f"({total} Personen in {n} Bildern)")


if __name__ == "__main__":
    main()
