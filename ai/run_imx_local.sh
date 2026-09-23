#!/usr/bin/env bash
# Startet den lokalen IMX500-Export in der dafuer gebauten venv.
# Getrennte venv, weil der Sony-Konverter protobuf herunterstuft und damit
# das TensorFlow in pi_export_env unbrauchbar machen wuerde.
set -e
export JAVA_HOME=/opt/homebrew/opt/openjdk@17
export PATH="$JAVA_HOME/bin:$PATH"
VENV="/private/tmp/claude-501/-Users-amirebrahimi-Downloads/3904a2a6-873c-4429-974e-82a67fb0eb69/scratchpad/imxenv"
exec "$VENV/bin/python" "$(dirname "$0")/export_imx_local.py" "$@"
