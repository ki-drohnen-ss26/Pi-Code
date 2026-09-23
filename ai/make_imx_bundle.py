"""
Baut ein kleines Upload-Paket für die IMX500-Konvertierung in Google Colab.

Der IMX500-Export braucht Linux, das gibt es auf dem Mac nicht. Colab ist ein
kostenloses x86-Linux und erledigt die beiden rechenintensiven Schritte
(MCT-Quantisierung und Konvertierung). Nur das Paketieren zur .rpk muss laut
Raspberry-Pi-Doku zwingend auf dem Pi selbst laufen.

Für die Quantisierung wird ein Repräsentativdatensatz gebraucht, aber nur in
Netz-Eingangsgroesse. Die Originalbilder sind bis zu 2048 px gross; auf 320 px
herunterskaliert schrumpft das Paket von ~200 MB auf wenige MB, ohne dass die
Kalibrierung etwas verliert — das Netz sieht die Bilder ohnehin nur bei 320 px.
YOLO-Labels sind normalisiert und bleiben beim Skalieren gültig.

    python3 make_imx_bundle.py      ->  imx_bundle.zip
"""

import shutil
import zipfile
from pathlib import Path

import cv2

HERE = Path(__file__).parent
WEIGHTS = HERE / "runs" / "F_neg_fpv_320" / "weights" / "best.pt"
SRC = HERE / "pad-dataset-neg"
STAGE = HERE / ".imx_bundle"
OUT = HERE / "imx_bundle.zip"
IMGSZ = 320


def main():
    if not WEIGHTS.exists():
        raise SystemExit(f"fehlt: {WEIGHTS}")

    if STAGE.exists():
        shutil.rmtree(STAGE)
    STAGE.mkdir()
    shutil.copy2(WEIGHTS, STAGE / "best.pt")

    for split in ("train", "val"):
        (STAGE / "data" / split / "images").mkdir(parents=True)
        (STAGE / "data" / split / "labels").mkdir(parents=True)
        for img_path in sorted((SRC / split / "images").glob("*")):
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            h, w = img.shape[:2]
            s = IMGSZ / max(h, w)
            if s < 1.0:
                img = cv2.resize(img, (max(1, int(w * s)), max(1, int(h * s))),
                                 interpolation=cv2.INTER_AREA)
            cv2.imwrite(str(STAGE / "data" / split / "images" / f"{img_path.stem}.jpg"),
                        img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            lbl = img_path.parent.parent / "labels" / f"{img_path.stem}.txt"
            if lbl.exists():
                shutil.copy2(lbl, STAGE / "data" / split / "labels" / lbl.name)

    # Kalibrierset: Ultralytics zieht die INT8-Kalibrierbilder aus dem
    # *val*-Split. Mit nur 25 Bildern warnt der Export (">300 images
    # recommended") und quantisiert schlechter. Deshalb ein zweites yaml, das
    # als val den zusammengelegten Satz aus train+val zeigt.
    calib = STAGE / "data" / "calib" / "images"
    calib_lbl = STAGE / "data" / "calib" / "labels"
    calib.mkdir(parents=True); calib_lbl.mkdir(parents=True)
    n_calib = 0
    for split in ("train", "val"):
        for f in (STAGE / "data" / split / "images").glob("*"):
            shutil.copy2(f, calib / f.name)
            lbl = STAGE / "data" / split / "labels" / f"{f.stem}.txt"
            if lbl.exists():
                shutil.copy2(lbl, calib_lbl / lbl.name)
            n_calib += 1

    # relative Pfade, damit es in Colab ohne Anpassung laeuft
    (STAGE / "data" / "data.yaml").write_text(
        "path: /content/imx/data\n"
        "train: train/images\n"
        "val: val/images\n"
        "\nnc: 1\nnames: ['landingPad']\n")
    # nur fuer den Export: val zeigt auf das Kalibrierset
    (STAGE / "data" / "calib.yaml").write_text(
        "path: /content/imx/data\n"
        "train: calib/images\n"
        "val: calib/images\n"
        "\nnc: 1\nnames: ['landingPad']\n")
    print(f"Kalibrierbilder: {n_calib}")

    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(STAGE.rglob("*")):
            if f.is_file():
                z.write(f, f.relative_to(STAGE))
    shutil.rmtree(STAGE)

    n = len(list((SRC / "train" / "images").glob("*")))
    print(f"{OUT.name}  ({OUT.stat().st_size / 1e6:.1f} MB, {n} Kalibrierbilder)")
    print("\nIn Colab hochladen, dann diese Zelle ausführen:\n")
    print(COLAB_CELL)


COLAB_CELL = r"""
!mkdir -p /content/imx && cd /content/imx && unzip -oq /content/imx_bundle.zip
!apt-get -qq update && apt-get -qq install -y default-jre >/dev/null
!pip -q install ultralytics

from ultralytics import YOLO
model = YOLO("/content/imx/best.pt")
model.export(format="imx", imgsz=320, int8=True,
             data="/content/imx/data/data.yaml")

# packerOut.zip herunterladen und auf den Pi kopieren
from google.colab import files
files.download("/content/imx/best_imx_model/packerOut.zip")
"""


if __name__ == "__main__":
    main()
