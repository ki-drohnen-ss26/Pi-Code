"""
Live-Demo auf dem Rechner (Webcam, Video oder Einzelbild).

Nutzt die PyTorch-Gewichte statt TFLite: auf dem Mac laeuft das ueber MPS
deutlich schneller, und TensorFlow wird nicht gebraucht.

Wichtig: imgsz muss der Trainingsgroesse entsprechen. Die alte live.py hat
keinen Wert uebergeben, also lief das Pad-Modell auf dem Ultralytics-Default
von 640 statt auf 320 — bei dieser Abweichung steigen die Fehlalarme von
0.00 auf 1.12 pro Bild.

    python3 live.py                    # Webcam
    python3 live.py --source video.mp4 # Videodatei
    python3 live.py --source bild.jpg  # Einzelbild, speichert das Ergebnis
    python3 live.py --conf 0.5
"""

import argparse
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO

HERE = Path(__file__).parent
PAD_WEIGHTS = HERE / "runs" / "F_neg_fpv_320" / "weights" / "best.pt"
# das quantisierte Modell, das tatsaechlich in der network.rpk steckt
PAD_WEIGHTS_IMX = HERE / "runs" / "F_neg_fpv_320" / "weights" / "best_imx_model"
PERSON_WEIGHTS = HERE / "yolo11n.pt"
PAD_IMGSZ = 320      # Trainingsgroesse von Run F — nicht veraendern
PERSON_IMGSZ = 640   # COCO-Standard fuer das Personenmodell
PERSON_CLASS = 0

GREEN, RED, WHITE = (0, 255, 0), (0, 0, 255), (255, 255, 255)


def boxes_overlap(b1, b2):
    return (max(b1[0], b2[0]) < min(b1[2], b2[2])
            and max(b1[1], b2[1]) < min(b1[3], b2[3]))


def detect(model, frame, imgsz, conf, device, classes=None):
    r = model(frame, imgsz=imgsz, conf=conf, device=device, verbose=False,
              classes=classes)[0]
    out = []
    for box in r.boxes:
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
        out.append((x1, y1, x2, y2, float(box.conf[0])))
    return out


def draw(frame, boxes, color, label):
    for (x1, y1, x2, y2, conf) in boxes:
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        cv2.putText(frame, f"{label} {conf:.2f}", (x1, max(y1 - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)


def annotate(frame, pad_model, person_model, conf, device):
    pads = detect(pad_model, frame, PAD_IMGSZ, conf, device)
    persons = detect(person_model, frame, PERSON_IMGSZ, conf, device,
                     classes=[PERSON_CLASS])
    draw(frame, pads, GREEN, "PAD")
    draw(frame, persons, RED, "PERSON")

    cv2.putText(frame, f"Pads:{len(pads)}  Persons:{len(persons)}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, WHITE, 2)
    if any(boxes_overlap(p[:4], q[:4]) for p in pads for q in persons):
        cv2.putText(frame, "!! ZONE BLOCKED !!", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, RED, 3)
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0",
                    help="Webcam-Index, Videodatei oder Bilddatei")
    # 0.4 aus fp_bench.py: 0.02 Fehlalarme/Bild bei Recall 1.00 und 0.94
    # auf x0.3 verkleinerten Pads
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--weights", default=None)
    ap.add_argument("--imx", action="store_true",
                    help="das quantisierte Modell aus der .rpk statt PyTorch")
    args = ap.parse_args()

    # Das quantisierte IMX-Modell ist ONNX und laeuft nur auf der CPU.
    # Passende Schwelle: die Quantisierung verschiebt die Konfidenzen nach
    # unten, gemessen 0.3 statt 0.4 (bei 0.5+ gehen ferne Pads verloren).
    if args.imx:
        args.weights = args.weights or str(PAD_WEIGHTS_IMX)
        device = "cpu"
        if args.conf == 0.4:
            args.conf = 0.3
    else:
        args.weights = args.weights or str(PAD_WEIGHTS)
        device = "mps" if torch.backends.mps.is_available() else (
            "cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")
    print(f"Pad-Modell: {Path(args.weights).name} @ {PAD_IMGSZ}px, conf={args.conf}")

    pad_model = YOLO(args.weights, task="detect")
    person_model = YOLO(str(PERSON_WEIGHTS))

    # Einzelbild: annotieren, speichern, fertig
    if Path(args.source).suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}:
        frame = cv2.imread(args.source)
        if frame is None:
            raise SystemExit(f"Bild nicht lesbar: {args.source}")
        out = Path(args.source).with_name(Path(args.source).stem + "_out.jpg")
        cv2.imwrite(str(out), annotate(frame, pad_model, person_model,
                                       args.conf, device))
        print(f"Gespeichert: {out}")
        return

    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit(
            "Kamera/Video konnte nicht geoeffnet werden.\n"
            "macOS: Systemeinstellungen -> Datenschutz & Sicherheit -> Kamera "
            "-> Terminal (bzw. Python) erlauben.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Druecke 'q' zum Beenden")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            cv2.imshow("Drone AI",
                       annotate(frame, pad_model, person_model, args.conf, device))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        for _ in range(5):
            cv2.waitKey(1)


if __name__ == "__main__":
    main()
