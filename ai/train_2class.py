"""
Run G: ein Netz, zwei Klassen (landingPad + person).

Ersetzt die beiden getrennten Detektoren. Pflicht fuer die AI Camera (der
IMX500 haelt nur ein Netz), und auch auf der CPU des Pi Zero halbiert es die
Last pro Frame.

Rezept wie Run F (Drohnen-Augmentierung + FPV-Degradation), damit der Vergleich
der Pad-Leistung gegen F aussagekraeftig bleibt. Weniger Epochen, weil der
Datensatz mit 859 statt 251 Bildern gut dreimal so gross ist.

Zur Rotation: degrees=180 und flipud drehen auch die Personen-Bilder auf den
Kopf. Fuer allgemeine Fotos waere das unsinnig, fuer eine Drohnenkamera nicht —
von oben und in Schraeglage erscheinen Personen in jeder Orientierung.

WICHTIG — mit dem Interpreter aus pi_export_env starten:

    ../pi_export_env/bin/python train_2class.py

Das System-Python hat ultralytics 8.4.50 mit torch 2.11, und dort bricht das
Training mit einem MPS-Bug in der Label-Zuordnung ab ("shape mismatch ... in
tal.py"). Der tritt erst auf, wenn viele Objekte pro Bild vorkommen — Run F
hatte ~1 Pad je Bild, hier sind es bis zu 48 Personen. Mit ultralytics 8.4.90
und torch 2.12.1 in pi_export_env laeuft es durch.
"""

import warnings
from pathlib import Path

warnings.filterwarnings("ignore")

from ultralytics import YOLO  # noqa: E402

import fpv_aug  # noqa: E402
from train import COMMON, DRONE_AUG  # noqa: E402

HERE = Path(__file__).parent


VARIANTS = {
    # Run G: Rezept unveraendert von Run F uebernommen
    "G_2class_320": dict(),
    # Run H: milde Rotation. Begruendung aus der Messung zu Run B — degrees=180
    # brachte fuer das Pad nichts (es ist 90-Grad-symmetrisch, die Invarianz war
    # ohnehin gratis), kostet aber bei Personen viel: 608 Bilder reichen nicht,
    # um jede Orientierung zu lernen, und aufrecht ist der Normalfall.
    "H_2class_mild_320": dict(degrees=30.0, flipud=0.0),
}


def main(name="H_2class_mild_320"):
    cfg = dict(COMMON, data=str(HERE / "pad-person-dataset" / "data.yaml"))
    aug = dict(DRONE_AUG, imgsz=320, epochs=120, patience=30, **VARIANTS[name])
    model = fpv_aug.attach(YOLO("yolo11n.pt"))
    model.train(name=name, **cfg, **aug)


if __name__ == "__main__":
    import sys
    main(sys.argv[1] if len(sys.argv) > 1 else "H_2class_mild_320")
