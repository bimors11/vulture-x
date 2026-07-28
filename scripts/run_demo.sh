#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$REPO_ROOT/logs/demo"
CAMERA_FRAME_DIR="$LOG_DIR/camera_frames"
MAVLINK_ENDPOINT="${VULTURE_X_MAVLINK:-udpin:0.0.0.0:14550}"

mkdir -p "$LOG_DIR"
mkdir -p "$CAMERA_FRAME_DIR"

if [[ ! -x "$REPO_ROOT/.venv/bin/python" ]]; then
  echo "Missing .venv. Run scripts/setup_environment.sh first." >&2
  exit 2
fi

source "$REPO_ROOT/.venv/bin/activate"

pids=()
cleanup() {
  local status=$?
  trap - INT TERM EXIT
  if ((${#pids[@]} > 0)); then
    echo "Stopping demo processes..."
    kill "${pids[@]}" >/dev/null 2>&1 || true
    wait "${pids[@]}" >/dev/null 2>&1 || true
  fi
  exit "$status"
}
trap cleanup INT TERM EXIT

echo "Starting Gazebo..."
"$REPO_ROOT/scripts/run_gazebo.sh" >"$LOG_DIR/gazebo.log" 2>&1 &
pids+=("$!")
echo "Gazebo PID: ${pids[-1]} (log: $LOG_DIR/gazebo.log)"
sleep 8

if ! kill -0 "${pids[0]}" >/dev/null 2>&1; then
  echo "Gazebo exited before SITL start. Last Gazebo log lines:" >&2
  tail -n 80 "$LOG_DIR/gazebo.log" >&2 || true
  exit 1
fi

if command -v gz >/dev/null 2>&1; then
  streaming_topic="$(gz topic -l 2>/dev/null | grep -m1 'enable_streaming' || true)"
  if [[ -n "$streaming_topic" ]]; then
    gz topic -t "$streaming_topic" -m gz.msgs.Boolean -p "data: true" >/dev/null 2>&1 || true
    echo "Requested Gazebo camera streaming on $streaming_topic"
  else
    echo "No Gazebo camera streaming topic found yet; camera verification may fail." >&2
  fi
fi

echo "Starting ArduPilot Copter SITL with sim_vehicle.py..."
"$REPO_ROOT/scripts/run_sitl.sh" >"$LOG_DIR/sitl.log" 2>&1 &
pids+=("$!")
echo "SITL PID: ${pids[-1]} (log: $LOG_DIR/sitl.log)"

echo "Waiting for MAVLink heartbeat on $MAVLINK_ENDPOINT..."
if ! python "$REPO_ROOT/tools/verify_environment.py" \
  --checks python,opencv,pymavlink,mavlink \
  --mavlink "$MAVLINK_ENDPOINT" \
  --heartbeat-timeout-s 45; then
  echo "SITL did not produce a verified heartbeat. Last SITL log lines:" >&2
  tail -n 120 "$LOG_DIR/sitl.log" >&2 || true
  exit 1
fi

echo "MAVLink connection status: heartbeat received on $MAVLINK_ENDPOINT"

if [[ "${VULTURE_X_ALLOW_SITL_ARM:-0}" == "1" ]]; then
  echo "Operator SITL arm flag detected. Sending GUIDED arm/takeoff to local SITL only..."
  python "$REPO_ROOT/tools/sitl_arm_takeoff.py" \
    --mavlink "$MAVLINK_ENDPOINT" \
    --altitude-m "${VULTURE_X_TAKEOFF_ALT_M:-5}"
else
  echo "SITL arm/takeoff disabled. Set VULTURE_X_ALLOW_SITL_ARM=1 to fly the simulated drone."
fi

echo "Starting camera bridge on UDP 5600..."
gst-launch-1.0 -q \
  udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
  ! rtph264depay \
  ! avdec_h264 \
  ! videoconvert \
  ! jpegenc \
  ! multifilesink location="$CAMERA_FRAME_DIR/frame-%06d.jpg" max-files=30 \
  >"$LOG_DIR/camera_bridge.log" 2>&1 &
pids+=("$!")

python "$REPO_ROOT/tools/verify_environment.py" \
  --checks camera \
  --camera-dir "$CAMERA_FRAME_DIR" \
  --camera-timeout-s 20

echo "Starting OpenCV camera viewer. Press Ctrl+C or q/ESC in the viewer to stop."
if [[ "${VULTURE_X_HEADLESS:-0}" == "1" ]]; then
  python "$REPO_ROOT/tools/camera_viewer.py" \
    --camera-dir "$CAMERA_FRAME_DIR" \
    --headless \
    --timeout-s "${VULTURE_X_HEADLESS_TIMEOUT_S:-10}"
else
  python "$REPO_ROOT/tools/camera_viewer.py" --camera-dir "$CAMERA_FRAME_DIR"
fi
