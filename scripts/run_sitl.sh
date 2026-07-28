#!/usr/bin/env bash
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$PWD/../ardupilot}"
if [[ ! -x "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]]; then
  echo "Set ARDUPILOT_DIR to a valid ArduPilot checkout." >&2
  exit 2
fi

exec "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
  -v ArduCopter \
  --console \
  --map \
  --out=udp:127.0.0.1:14550 \
  "$@"

