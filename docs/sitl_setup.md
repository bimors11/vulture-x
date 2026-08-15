# Linux Mint and ArduPilot SITL Setup

First reproduce the Python baseline exactly as described in `README.md`. Do not
install or launch ArduPilot until those checks pass.

Linux Mint is Ubuntu-based, but verify its base release:

```bash
cat /etc/os-release
```

Use the current official documentation:

- [Linux build environment](https://ardupilot.org/dev/docs/building-setup-linux.html)
- [SITL setup on Linux](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
- [Using SITL](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)

Keep ArduPilot beside this repository, not inside it. A typical official-source
flow is:

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  --console \
  --map \
  -w \
  --out=udp:127.0.0.1:14550
```

Inspect the current official instructions and installer before executing them.
Do not force the Ubuntu prerequisite script on an unsupported Mint base.

The Vulture-X SITL configuration listens at `udpin:0.0.0.0:14550` for
tracking/command helpers, exports `udp:127.0.0.1:14551` for QGroundControl, and
exports `udp:127.0.0.1:14552` for the UI heartbeat/status monitor. In
QGroundControl, add a UDP comm link that listens on port `14551` if
auto-detection does not connect. Record exact ports because MAVProxy/SITL
options can change them.

Vulture-X launch scripts keep the original quadcopter path as the default and
add explicit startup profile flags:

```bash
scripts/run_ui.sh -quad
scripts/run_ui.sh -plane
scripts/run_gazebo.sh -plane
scripts/run_sitl.sh -plane
```

`-plane` uses the same ArduPilot checkout, `ArduPlane`, the `gazebo-zephyr`
frame, and `simulation/worlds/vulture_x_plane.sdf`. It is a simulation
environment profile only. The UI includes a SITL-only fixed-wing steering
helper that switches an already-armed ArduPlane instance to `FBWA` and uses
bounded RC roll/pitch overrides plus an airspeed-based throttle governor.
Fixed-wing guidance is still not
implemented in the main Vulture-X package.

The current Vulture-X milestone cannot connect to this endpoint. The next Codex
agent must first implement and mock-test heartbeat, identity validation,
telemetry, ACK correlation, timeouts, and clean shutdown. Its first real SITL
connection must be read-only.
