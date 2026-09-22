"""
Landing-Pad-Erkennung auf der Raspberry Pi AI Camera (IMX500).

Das Netz laeuft auf dem Sensor, nicht auf der CPU des Pi. Damit ist der Pi
Zero frei fuer Flugsteuerung — genau der Engpass, der auf dem CPU-Weg zu
SKIP_FRAMES = 2 gefuehrt hat.

Zwei Dinge, die vom CPU-Weg (drone_pi.py) abweichen:

  Eingabedatei   modlib nimmt packerOut.zip, nicht network.rpk. Es paketiert
                 intern selbst. Die .rpk brauchst du nur, wenn du stattdessen
                 rpicam-apps benutzt — dessen eingebaute Parser passen aber
                 nicht zum YOLO-Ausgabeformat von Ultralytics.
  Schwelle       0.3 statt 0.4. Die Quantisierung verschiebt die Konfidenzen
                 nach unten; gemessen faellt die Erkennung ferner Pads ab
                 0.5 deutlich ab (Recall bei x0.3 Zoom: 0.94 -> 0.75).

Voraussetzungen auf dem Pi:
    sudo apt update && sudo apt full-upgrade
    sudo apt install imx500-all
    sudo reboot
    pip install git+https://github.com/SonySemiconductorSolutions/aitrios-rpi-application-module-library.git

Aufruf:
    python3 pi_aicam.py
    python3 pi_aicam.py --conf 0.4 --no-display
"""

import argparse
from pathlib import Path

import numpy as np
from modlib.apps import Annotator
from modlib.devices import AiCamera
from modlib.models import COLOR_FORMAT, MODEL_TYPE, Model
from modlib.models.post_processors import pp_od_yolo_ultralytics

HERE = Path(__file__).parent
PACKER = HERE / "packerOut.zip"
LABELS = HERE / "labels.txt"


class LandingPad(Model):
    """Run F, eine Klasse: landingPad."""

    def __init__(self):
        super().__init__(
            model_file=str(PACKER),
            model_type=MODEL_TYPE.CONVERTED,
            color_format=COLOR_FORMAT.RGB,
            preserve_aspect_ratio=False,
        )
        self.labels = np.genfromtxt(str(LABELS), dtype=str, delimiter="\n",
                                    ndmin=1)

    def post_process(self, output_tensors):
        return pp_od_yolo_ultralytics(output_tensors)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conf", type=float, default=0.3,
                    help="Konfidenzschwelle (gemessen: 0.3 fuer das "
                         "quantisierte Modell)")
    ap.add_argument("--fps", type=int, default=16)
    ap.add_argument("--no-display", action="store_true",
                    help="ohne Bildausgabe, nur Zahlen — fuer Betrieb ohne Monitor")
    args = ap.parse_args()

    for f in (PACKER, LABELS):
        if not f.exists():
            raise SystemExit(f"fehlt: {f}")

    device = AiCamera(frame_rate=args.fps)
    model = LandingPad()
    device.deploy(model)
    annotator = Annotator()

    print(f"laeuft — Schwelle {args.conf}, Strg+C zum Beenden")
    with device as stream:
        for frame in stream:
            dets = frame.detections[frame.detections.confidence > args.conf]

            for bbox, score, class_id, _ in dets:
                # bbox ist normiert (0..1): x1, y1, x2, y2
                x1, y1, x2, y2 = bbox
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                w, h = x2 - x1, y2 - y1
                # cx, cy: 0.5/0.5 = Pad genau mittig -> Regelabweichung
                # w, h:   groesser = naeher am Boden
                print(f"PAD  Mitte {cx:.3f},{cy:.3f}  Groesse {w:.3f}x{h:.3f}"
                      f"  conf {score:.2f}")

            if not args.no_display:
                labels = [f"{model.labels[int(c)]}: {s:0.2f}"
                          for _, s, c, _ in dets]
                annotator.annotate_boxes(frame, dets, labels=labels,
                                         alpha=0.3, corner_radius=10)
                frame.display()


if __name__ == "__main__":
    main()
