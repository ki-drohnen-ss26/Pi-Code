"""
Pad-Geometrie aus einer Detektion: Kontur, Mittelpunkt, Gierwinkel.

Der Detektor liefert einen achsparallelen Kasten. Fuer die Landung reicht das
nicht — der Regler braucht den Mittelpunkt genauer als eine Kastenmitte und die
Ausrichtung des Pads relativ zur Drohne.

Die rote Markierung des Pads zeichnet genau dessen Umriss nach. Innerhalb einer
bestaetigten Detektion ist diese Maske sehr sauber (die Hallenlinien stoeren
hier nicht, weil nur der Kasteninhalt betrachtet wird), also laesst sich daraus
ein rotiertes Rechteck fitten.

Wichtig: als *Pruefung*, ob ueberhaupt ein Pad vorliegt, taugt die Farbe nicht —
der Hallenboden hat rote, sich kreuzende Linien (siehe pattern_check.py). Hier
wird die Detektion vorausgesetzt und nur noch vermessen.

Das Pad ist quadratisch und traegt ein symmetrisches X, ist also 90-Grad-
symmetrisch. Der Gierwinkel ist damit nur modulo 90 Grad bestimmt. Fuer das
Ausrichten vor dem Aufsetzen genuegt das; eine eindeutige Richtung braeuchte
eine asymmetrische Markierung auf dem Pad.

    python3 pad_pose.py            # Demo, schreibt Bilder nach pose_demo/
"""

import warnings
from pathlib import Path

import cv2
import numpy as np

warnings.filterwarnings("ignore")

from pattern_check import red_mask  # noqa: E402

HERE = Path(__file__).parent
MARGIN = 0.08          # Kasten leicht aufweiten, der Rahmen liegt am Rand
MIN_RED_PIXELS = 60


def pad_pose(frame, box):
    """-> dict mit corners, center, angle_deg, side_px  oder None."""
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = (float(v) for v in box[:4])
    mx, my = (x2 - x1) * MARGIN, (y2 - y1) * MARGIN
    x1, y1 = max(int(x1 - mx), 0), max(int(y1 - my), 0)
    x2, y2 = min(int(x2 + mx), w), min(int(y2 + my), h)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None

    mask = red_mask(crop)
    ys, xs = np.nonzero(mask)
    if len(xs) < MIN_RED_PIXELS:
        return None

    pts = np.stack([xs, ys], 1).astype(np.float32)
    (cx, cy), (rw, rh), ang = cv2.minAreaRect(pts)
    if min(rw, rh) < 6:
        return None

    corners = cv2.boxPoints(((cx, cy), (rw, rh), ang)) + np.array([x1, y1])
    return {
        "corners": corners,                      # 4x2, Bildkoordinaten
        "center": (cx + x1, cy + y1),
        "angle_deg": float(ang % 90.0),          # 90-Grad-symmetrisch
        "side_px": float((rw + rh) / 2),
        "aspect": float(min(rw, rh) / max(rw, rh)),
    }


def draw_pose(frame, pose, color=(0, 220, 255)):
    cv2.polylines(frame, [pose["corners"].astype(np.int32)], True, color, 2)
    cx, cy = (int(v) for v in pose["center"])
    cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 18, 2)
    cv2.putText(frame, f"yaw {pose['angle_deg']:.0f}deg  {pose['side_px']:.0f}px",
                (cx - 70, cy - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return frame


def main():
    from ultralytics import YOLO

    out_dir = HERE / "pose_demo"
    out_dir.mkdir(exist_ok=True)
    model = YOLO("runs/D_drone_fpv_320/weights/best.pt")

    files = sorted((HERE / "pad-dataset" / "test" / "images").glob("*"))
    print(f"{'image':<22}{'yaw':>7}{'side px':>9}{'aspect':>8}")
    print("-" * 48)
    tiles = []
    for f in files[:8]:
        img = cv2.imread(str(f))
        r = model.predict(img, imgsz=320, conf=0.25, device="mps", verbose=False)[0]
        for b in r.boxes.xyxy.cpu().numpy():
            pose = pad_pose(img, b)
            if pose is None:
                continue
            cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])),
                          (0, 255, 0), 2)
            draw_pose(img, pose)
            print(f"{f.name.split('.rf.')[0]:<22}{pose['angle_deg']:>7.1f}"
                  f"{pose['side_px']:>9.0f}{pose['aspect']:>8.2f}")
        cv2.imwrite(str(out_dir / f"{f.name.split('.rf.')[0]}.jpg"), img)
        tiles.append(cv2.resize(img, (300, int(300 * img.shape[0] / img.shape[1]))))

    hmin = min(t.shape[0] for t in tiles)
    cv2.imwrite(str(out_dir / "_grid.jpg"),
                np.hstack([t[:hmin] for t in tiles]))
    print(f"\n-> {out_dir}/_grid.jpg")


if __name__ == "__main__":
    main()
