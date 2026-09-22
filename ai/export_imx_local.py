"""
IMX500-Export (Schritt 1+2) lokal auf dem Mac — ohne Colab.

Ultralytics blockiert den IMX-Export mit `assert LINUX`. Die Pruefung ist
vorsichtig, aber technisch nicht noetig: die gesamte Toolchain ist
plattformunabhaengig. Nachgesehen statt vermutet:

  imx500-converter   reines Python (py3-none-any), ruft nur weiter
  sdspconv           reines Java — Scala/Kotlin-JARs, keine .so/.dylib/.dll
  model-compression-toolkit, edge-mdt-*   reines Python
  ortools            hat macosx_11_0_arm64-Wheels

Getestet: `imxconv-pt --version` laeuft auf macOS. Deshalb wird die Sperre
hier gezielt aufgehoben.

Zwei Fallstricke, die den Colab-Lauf zerlegt hatten und hier vermieden sind:

  TensorFlow    darf nicht installiert sein. Sonys Konverter verlangt
                protobuf 4.25.5, TF braucht 5.x; model_compression_toolkit
                importiert TF nur, weil es da ist. Fuer den PyTorch-Weg
                unnoetig — die venv hier hat es nicht.
  Kalibrierung  Ultralytics zieht die INT8-Kalibrierbilder aus dem *val*-
                Eintrag. Der echte val-Split hat 25 Bilder, der Export warnt
                dann selbst (">300 images recommended"). calib.yaml unten
                zeigt auf train+val zusammen.

Aufruf (die venv und JAVA_HOME setzt run_imx_local.sh):
    python export_imx_local.py
"""

import os
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).parent
WEIGHTS = HERE / "runs" / "F_neg_fpv_320" / "weights" / "best.pt"
DATASET = HERE / "pad-dataset-neg"
CALIB = HERE / "imx-calib"
IMGSZ = 320


def build_calib():
    """val-Eintrag auf train+val zeigen lassen, per Symlink statt Kopie."""
    if CALIB.exists():
        shutil.rmtree(CALIB)
    (CALIB / "images").mkdir(parents=True)
    (CALIB / "labels").mkdir(parents=True)
    # Bilder OHNE Pad bleiben draussen. MCTs Mixed-Precision-Suche vergleicht
    # die Ausgaben von Float- und quantisiertem Modell und normiert dabei ueber
    # deren Norm. Auf einem pad-freien Bild ist die Konfidenz-Ausgabe nahezu
    # null, die Normierung teilt durch ~0, und die Sensitivitaet wird inf/NaN —
    # der Solver bricht dann mit "Cannot multiply variables with NaN/inf values"
    # ab. Fuer das Training sind die Negativbilder wichtig, fuer die
    # Kalibrierung schaedlich.
    n = skipped = 0
    for split in ("train", "val"):
        for img in sorted((DATASET / split / "images").glob("*")):
            lbl = DATASET / split / "labels" / f"{img.stem}.txt"
            if not lbl.exists() or not lbl.read_text().strip():
                skipped += 1
                continue
            (CALIB / "images" / img.name).symlink_to(img.resolve())
            (CALIB / "labels" / lbl.name).symlink_to(lbl.resolve())
            n += 1
    print(f"pad-freie Bilder uebersprungen: {skipped}")
    yaml = HERE / "calib.yaml"
    yaml.write_text(
        f"path: {CALIB}\ntrain: images\nval: images\n\nnc: 1\nnames: ['landingPad']\n")
    print(f"Kalibrierbilder: {n}")
    return yaml


def main():
    if not WEIGHTS.exists():
        sys.exit(f"fehlt: {WEIGHTS}")
    if "JAVA_HOME" not in os.environ:
        print("WARNUNG: JAVA_HOME nicht gesetzt — der Konverter braucht Java 17+")

    calib = build_calib()

    # Sperre aufheben: siehe Modul-Docstring. Muss vor dem Export passieren.
    from ultralytics.engine import exporter
    exporter.LINUX = True

    from ultralytics import YOLO

    model = YOLO(str(WEIGHTS))
    print("Klassen:", model.names)

    out = Path(model.export(format="imx", imgsz=IMGSZ, int8=True,
                            data=str(calib), device="cpu"))
    print("\nExport-Verzeichnis:", out)
    for f in sorted(out.iterdir()):
        print(f"   {f.name}  ({f.stat().st_size / 1e6:.2f} MB)")

    packer = out / "packerOut.zip"
    if packer.exists():
        dest = HERE / "packerOut.zip"
        shutil.copy2(packer, dest)
        print(f"\n-> {dest}")
        print("Schritt 3 (aarch64): imx500-package -i packerOut.zip -o .")
    else:
        sys.exit("packerOut.zip wurde nicht erzeugt")


if __name__ == "__main__":
    main()
