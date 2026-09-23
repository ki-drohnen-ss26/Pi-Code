#!/usr/bin/env bash
# Webcam-Test mit dem *quantisierten* Modell — also genau dem, was in der
# network.rpk steckt und spaeter auf dem Kamerasensor laeuft.
#
# Braucht eine eigene venv, weil das IMX/ONNX-Modell andere Pakete verlangt
# als das System-Python. Falls sie fehlt, unten die Neuanlage.
set -e
VENV="/private/tmp/claude-501/-Users-amirebrahimi-Downloads/3904a2a6-873c-4429-974e-82a67fb0eb69/scratchpad/imxenv"

if [ ! -x "$VENV/bin/python" ]; then
  cat <<'MSG'
Die venv fuer das quantisierte Modell existiert nicht mehr
(sie lag in einem temporaeren Ordner). Neu anlegen mit:

  python3.12 -m venv ~/imxenv
  ~/imxenv/bin/pip install ultralytics onnxruntime

danach in dieser Datei VENV auf ~/imxenv aendern.

Der normale Test funktioniert davon unabhaengig:  python3 live.py
MSG
  exit 1
fi

exec "$VENV/bin/python" "$(dirname "$0")/live.py" --imx "$@"
