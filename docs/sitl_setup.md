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
scripts/run_ui.sh -manual
scripts/run_gazebo.sh -plane
scripts/run_sitl.sh -plane
```

`-manual` starts only the browser UI and leaves Gazebo, SITL/MAVLink, camera
bridging, and target motion disconnected until the operator starts or connects
them from the panel.

`-plane` uses the same ArduPilot checkout, `ArduPlane`, the `gazebo-zephyr`
frame, and `simulation/worlds/vulture_x_plane.sdf`. The fixed-wing steering
helper requires `FBWA` and uses bounded `RC_CHANNELS_OVERRIDE` roll, pitch,
throttle, and neutral yaw commands. In simulator mode it may switch an
already-armed SITL plane to `FBWA` as a convenience. In manual/hardware mode it
blocks unless the aircraft is already armed and already in `FBWA`.

The plane takeoff helper is simulator-only. The WebUI must not run
`tools/sitl_arm_takeoff.py` in manual/hardware mode, and no hardware arming or
takeoff replacement is provided.

Fixed-wing steering fails safe by releasing RC override and stopping, without
switching modes or commanding RTL/LOITER, on stale video, heartbeat timeout,
disarm, flight-mode change, sustained target loss after `plane_loss_hold_s`,
below-minimum tracking altitude when altitude telemetry is valid, or persistent
low airspeed when aircraft minimum airspeed is known.
