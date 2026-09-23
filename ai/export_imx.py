"""
Export für die Raspberry Pi AI Camera (Sony IMX500) -> .rpk

MUSS AUF LINUX LAUFEN. Ultralytics erzwingt `assert LINUX`, und Sonys Konverter
(`imxconv-pt`) wird nicht für macOS ausgeliefert. Auf dem Mac bricht das hier
sofort ab. Geeignet sind: der Raspberry Pi selbst (Bookworm), ein x86-Ubuntu,
oder Docker mit --platform linux/amd64.

Der Weg besteht aus zwei Schritten, und nur der erste steckt in diesem Skript:

  1. hier: .pt -> MCT-Quantisierung -> ONNX -> imxconv-pt -> packerOut.zip
  2. danach: imx500-package -i packerOut.zip -o .   -> network.rpk

Schritt 2 kommt aus dem apt-Paket `imx500-tools` und ist nicht Teil von
Ultralytics.

Die Quantisierung ist hier keine simple INT8-Konvertierung wie beim TFLite-
Export, sondern gradientenbasiertes Post-Training-Quantisieren über einen
Repräsentativdatensatz — deshalb wird `data` gebraucht und deshalb dauert es
einige Minuten.

    python3 export_imx.py                       # Run F
    python3 export_imx.py runs/X/weights/best.pt
"""

import platform
import sys
import warnings
from pathlib import Path

warnings.filterwarnings("ignore")

HERE = Path(__file__).parent
DEFAULT_WEIGHTS = HERE / "runs" / "F_neg_fpv_320" / "weights" / "best.pt"
DATA = HERE / "pad-dataset-neg" / "data.yaml"
IMGSZ = 320


def main(weights):
    if platform.system() != "Linux":
        sys.exit(
            f"IMX500-Export braucht Linux (hier: {platform.system()}).\n"
            "Auf dem Pi:      sudo apt install -y imx500-tools openjdk-17-jre\n"
            "oder per Docker: docker run --rm --platform linux/amd64 -v $PWD:/w "
            "-w /w python:3.11 bash -c \\\n"
            "                 'apt-get update && apt-get install -y "
            "default-jre libgl1 libglib2.0-0 && \\\n"
            "                  pip install ultralytics && python3 export_imx.py'"
        )

    from ultralytics import YOLO

    weights = Path(weights)
    print(f"Modell: {weights}\nDaten:  {DATA}\nimgsz:  {IMGSZ}\n")

    model = YOLO(str(weights))
    # int8=True ist für IMX Pflicht; Ultralytics installiert
    # model-compression-toolkit und imx500-converter bei Bedarf selbst nach.
    out = Path(model.export(format="imx", imgsz=IMGSZ, int8=True, data=str(DATA)))

    packer = out / "packerOut.zip"
    print(f"\nfertig: {out}")
    for f in sorted(out.iterdir()):
        print(f"   {f.name}  ({f.stat().st_size / 1e6:.1f} MB)")

    if packer.exists():
        print("\nJetzt noch zur .rpk paketieren (braucht apt-Paket imx500-tools):")
        print(f"   imx500-package -i {packer} -o {out}")
        print(f"   -> {out / 'network.rpk'}")
    else:
        print("\nWARNUNG: keine packerOut.zip erzeugt — Konverter-Schritt "
              "fehlgeschlagen.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_WEIGHTS)
