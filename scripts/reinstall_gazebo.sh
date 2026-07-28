#!/usr/bin/env bash
set -euo pipefail

source /etc/os-release
UBUNTU_BASE="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"

if [[ "${ID:-}" != "ubuntu" && "${ID_LIKE:-}" != *"ubuntu"* ]]; then
  echo "Unsupported OS: ${PRETTY_NAME:-unknown}. Ubuntu-compatible Linux is required." >&2
  exit 2
fi

if [[ "$UBUNTU_BASE" != "jammy" ]]; then
  echo "Unsupported Ubuntu base '$UBUNTU_BASE'. Install script expects Jammy/Mint 21.x." >&2
  exit 2
fi

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required to remove/install Gazebo packages." >&2
  exit 2
fi

echo "Removing Gazebo Classic packages if present..."
sudo apt-get remove --purge -y \
  gazebo \
  gazebo-common \
  gazebo-doc \
  gazebo-plugin-base \
  libgazebo11 \
  libgazebo-dev || true

sudo apt-get autoremove -y

echo "Installing OSRF Gazebo package repository for $UBUNTU_BASE..."
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $UBUNTU_BASE main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

echo "Installing Gazebo Harmonic and ArduPilot Gazebo build dependencies..."
sudo apt-get update
sudo apt-get install -y \
  gz-harmonic \
  libgz-sim8-dev \
  rapidjson-dev \
  libopencv-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-gl

if ! gz sim -h >/dev/null 2>&1; then
  echo "Gazebo Harmonic install finished, but 'gz sim' is still unavailable." >&2
  exit 1
fi

echo "Gazebo reinstall complete."
gz sim --versions || true
