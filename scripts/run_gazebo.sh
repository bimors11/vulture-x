#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
VEHICLE_MODE="${VULTURE_X_VEHICLE:-quad}"
WORLD=""
HEADLESS="${VULTURE_X_GAZEBO_HEADLESS:-0}"

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
      WORLD="$1"
      shift
      ;;
  esac
done

case "$VEHICLE_MODE" in
  quad)
    WORLD="${WORLD:-$REPO_ROOT/simulation/worlds/vulture_x_test.sdf}"
    ;;
  plane)
    WORLD="${WORLD:-$REPO_ROOT/simulation/worlds/vulture_x_plane.sdf}"
    ;;
  *)
    echo "gazebo_status=failed reason=invalid_vehicle_mode value=$VEHICLE_MODE" >&2
    exit 2
    ;;
esac

if ! command -v gz >/dev/null 2>&1; then
  echo "Gazebo Sim command 'gz' was not found. Run scripts/setup_environment.sh first." >&2
  exit 2
fi

if ! gz sim -h >/dev/null 2>&1; then
  echo "'gz sim' is not available. Gazebo Classic alone is not enough for the current ArduPilot plugin." >&2
  exit 2
fi

if [[ ! -d "$ARDUPILOT_GAZEBO_DIR/build" ]]; then
  echo "ArduPilot Gazebo plugin build not found at $ARDUPILOT_GAZEBO_DIR/build." >&2
  echo "Run scripts/setup_environment.sh first." >&2
  exit 2
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/simulation/worlds:$REPO_ROOT/simulation/models:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# OpenCV wheels can place Qt plugin paths in the virtualenv. Gazebo GUI must use
# the system Qt plugin path, while the server path should avoid Qt entirely.
unset QT_PLUGIN_PATH
unset QT_QPA_PLATFORM_PLUGIN_PATH

cleanup_stale_server() {
  pkill -TERM -f '^gz sim( |$)' >/dev/null 2>&1 || true
  sleep 1
  pkill -KILL -f '^gz sim( |$)' >/dev/null 2>&1 || true
}

if [[ "$HEADLESS" == "1" ]]; then
  trap cleanup_stale_server EXIT
  gz sim -v4 -s -r "$WORLD"
  exit $?
fi

trap cleanup_stale_server EXIT
gz sim -v4 -r "$WORLD"
