#!/usr/bin/env bash
set -euo pipefail

VEHICLE_MODE="${VULTURE_X_VEHICLE:-quad}"
extra_args=()
while (($# > 0)); do
  case "$1" in
    -quad|--quad)
      VEHICLE_MODE="quad"
      shift
      ;;
    -plane|--plane)
      VEHICLE_MODE="plane"
      shift
      ;;
    *)
      extra_args+=("$1")
      shift
      ;;
  esac
done

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

pkill -TERM -f 'mavproxy.py .*--master tcp:127\.0\.0\.1:5760' >/dev/null 2>&1 || true
sleep 0.5
pkill -KILL -f 'mavproxy.py .*--master tcp:127\.0\.0\.1:5760' >/dev/null 2>&1 || true

case "$VEHICLE_MODE" in
  quad)
    vehicle="ArduCopter"
    frame="gazebo-iris"
    ;;
  plane)
    vehicle="ArduPlane"
    frame="gazebo-zephyr"
    ;;
  *)
    echo "sitl_status=failed reason=invalid_vehicle_mode value=$VEHICLE_MODE" >&2
    exit 2
    ;;
esac

args=(
  -v "$vehicle"
  -f "$frame"
  --model JSON
  --out=udp:127.0.0.1:14550
  --out=udp:127.0.0.1:14551
  --out=udp:127.0.0.1:14552
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

exec "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" "${args[@]}" "${extra_args[@]}"
