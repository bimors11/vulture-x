#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${ARDUPILOT_DIR:-}" ]]; then
  if [[ -d "$HOME/ArduSITL/ardupilot" ]]; then
    ARDUPILOT_DIR="$HOME/ArduSITL/ardupilot"
  else
    ARDUPILOT_DIR="$HOME/ardupilot"
  fi
fi
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
GZ_VERSION="${GZ_VERSION:-harmonic}"

source /etc/os-release
UBUNTU_BASE="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
if [[ "${ID:-}" != "ubuntu" && "${ID_LIKE:-}" != *"ubuntu"* ]]; then
  echo "Unsupported OS: ${PRETTY_NAME:-unknown}. Ubuntu-compatible Linux is required." >&2
  exit 2
fi
if [[ "$UBUNTU_BASE" != "jammy" ]]; then
  echo "Unsupported Ubuntu base '$UBUNTU_BASE'. This script is pinned to Jammy/Mint 21.x with Gazebo Harmonic." >&2
  exit 2
fi

have_sudo() {
  command -v sudo >/dev/null 2>&1 && sudo -n true >/dev/null 2>&1
}

install_linux_packages() {
  if ! have_sudo; then
    echo "sudo is required for Linux package installation. Re-run after granting sudo credentials." >&2
    exit 2
  fi

  sudo apt-get update
  sudo apt-get install -y curl lsb-release gnupg git build-essential cmake pkg-config \
    python3-dev python3-pip python3-venv

  if ! gz sim -h >/dev/null 2>&1; then
    "$REPO_ROOT/scripts/reinstall_gazebo.sh"
  else
    sudo apt-get install -y rapidjson-dev libgz-sim8-dev \
      libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
      gstreamer1.0-gl
  fi
}

setup_ardupilot() {
  if [[ -d "$ARDUPILOT_DIR/.git" ]]; then
    echo "Using existing ArduPilot checkout: $ARDUPILOT_DIR"
  else
    echo "Cloning ArduPilot into $ARDUPILOT_DIR"
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
  fi

  git -C "$ARDUPILOT_DIR" submodule update --init --recursive

  if [[ "${SKIP_ARDUPILOT_PREREQS:-0}" != "1" ]]; then
    "$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh" -y
  fi

  (cd "$ARDUPILOT_DIR" && ./waf configure --board sitl && ./waf copter)
}

setup_ardupilot_gazebo() {
  if [[ -d "$ARDUPILOT_GAZEBO_DIR/.git" ]]; then
    echo "Using existing ArduPilot Gazebo plugin checkout: $ARDUPILOT_GAZEBO_DIR"
  else
    git clone https://github.com/ArduPilot/ardupilot_gazebo "$ARDUPILOT_GAZEBO_DIR"
  fi

  export GZ_VERSION
  cmake -S "$ARDUPILOT_GAZEBO_DIR" -B "$ARDUPILOT_GAZEBO_DIR/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
  cmake --build "$ARDUPILOT_GAZEBO_DIR/build" --parallel "$(nproc)"
}

setup_python() {
  cd "$REPO_ROOT"
  if [[ -x .venv/bin/python ]]; then
    version="$("$REPO_ROOT/.venv/bin/python" -c 'import sys; print(".".join(map(str, sys.version_info[:3])))')"
    echo "Using existing .venv with Python $version"
  elif command -v python3.11 >/dev/null 2>&1; then
    python3.11 -m venv .venv
  elif command -v uv >/dev/null 2>&1; then
    uv venv --python 3.11 .venv
  else
    echo "Python 3.11+ was not found. Install python3.11 or uv, then re-run." >&2
    exit 2
  fi

  if "$REPO_ROOT/.venv/bin/python" -m pip --version >/dev/null 2>&1; then
    "$REPO_ROOT/.venv/bin/python" -m pip install --upgrade pip
    "$REPO_ROOT/.venv/bin/python" -m pip install -r "$REPO_ROOT/requirements.txt"
  elif command -v uv >/dev/null 2>&1; then
    uv pip install --python "$REPO_ROOT/.venv/bin/python" -r "$REPO_ROOT/requirements.txt"
  else
    echo "pip is unavailable in .venv and uv is not installed." >&2
    exit 2
  fi
}

install_linux_packages
setup_ardupilot
setup_ardupilot_gazebo
setup_python

cat <<EOF
Environment setup complete.

Add these exports to your shell if you run Gazebo manually:
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/simulation/worlds:$REPO_ROOT/simulation/models:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:\${GZ_SIM_RESOURCE_PATH:-}"

Verify:
  source .venv/bin/activate
  python tools/verify_environment.py
EOF
