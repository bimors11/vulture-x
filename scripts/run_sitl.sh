#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ARDUPILOT_DIR:-}" ]]; then
  if [[ -d "$HOME/ArduSITL/ardupilot" ]]; then
    ARDUPILOT_DIR="$HOME/ArduSITL/ardupilot"
  else
    ARDUPILOT_DIR="$HOME/ardupilot"
  fi
fi
if [[ ! -x "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]]; then
  echo "Set ARDUPILOT_DIR to a valid ArduPilot checkout; expected $ARDUPILOT_DIR." >&2
  exit 2
fi

cd "$ARDUPILOT_DIR"

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "$PATH" | awk -v RS=: -v ORS=: '$0 != ENVIRON["VIRTUAL_ENV"] "/bin" {print}' | sed 's/:$//')"
  unset VIRTUAL_ENV
fi

args=(
  -v ArduCopter
  -f gazebo-iris
  --model JSON
  --out=udp:127.0.0.1:14550
  --out=udp:127.0.0.1:14551
)

if [[ -z "${VULTURE_X_SITL_INTERACTIVE:-}" ]]; then
  if [[ -t 1 ]]; then
    VULTURE_X_SITL_INTERACTIVE=1
  else
    VULTURE_X_SITL_INTERACTIVE=0
  fi
fi

if [[ "$VULTURE_X_SITL_INTERACTIVE" == "1" ]]; then
  args+=(--console --map)
else
  args+=(--mavproxy-args=--daemon --mavproxy-args=--non-interactive)
fi

exec "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" "${args[@]}" "$@"
