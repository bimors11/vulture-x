# Vulture-X

Vulture-X is a ground-based visual tracking and UAV-guidance research system.
The planned aircraft carries an ArduPilot flight controller and analog FPV
camera but no companion computer. Video processing and high-level guidance run
on a Linux ground station.

The project is simulation-first and limited to cooperative, non-contact testing.
It does not implement collision, impact guidance, RF interference, spoofing,
jamming, payload deployment, or autonomous engagement of real aircraft.

## First-milestone status

Implemented:

- strict YAML configuration and safety cross-validation;
- immutable tracking, vehicle, guidance, and safety models;
- deterministic synthetic OpenCV video with motion, noise, disappearance,
  sudden-movement, and delay controls;
- CSRT/KCF/template tracking with normalized image-plane results;
- bounded image-error guidance and expiring commands;
- speed, vertical-speed, acceleration, and yaw-rate limiting;
- tracking timeout, heartbeat timeout, and navigation-health checks;
- transport abstraction and a mock-only MAVLink vehicle;
- explicit mission state machine through `TRACK`;
- JSONL events and CSV telemetry logging;
- unit and mocked integration tests.
- UI launcher selection for the existing quadcopter profile and a fixed-wing
  ArduPlane/Gazebo Zephyr profile.

Not implemented:

- a real `pymavlink` UDP client;
- ArduPilot SITL command integration;
- fixed-wing visual guidance commands in the main Vulture-X package;
- `GUIDED` flight, takeoff, LOITER, RTL, or LAND against SITL as part of the
  main Vulture-X package;
- V4L2/video-file capture and real analog hardware;
- ELRS transport;
- mission behavior after `TRACK`.

The mock vehicle never communicates with hardware. Do not interpret this
milestone as SITL, HIL, or flight readiness.

## Linux setup

Python 3.11 or newer is required:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

Verify the first milestone:

```bash
pytest
ruff check .
mypy
vulture-x --config configs/default.yaml --check-config
```

The normal bootstrap is safe and has no vehicle command authority:

```bash
vulture-x --config configs/default.yaml
```

## SITL and Gazebo test environment

The environment scripts are for local simulation testing only. They do not arm,
take off, or send Vulture-X guidance commands. The current Vulture-X milestone
still has only a mock vehicle client; use the SITL/Gazebo pieces to validate the
external test environment before implementing the guarded MAVLink client.

This machine is expected to use the existing ArduPilot checkout at
`~/ArduSITL/ardupilot`. If that directory is missing, the scripts fall back to
`~/ardupilot`. The path may always be overridden with `ARDUPILOT_DIR=/path`.
The setup script is pinned to Ubuntu Jammy-compatible systems, including Linux
Mint 21.x, and exits with a clear error on unsupported bases.

The Gazebo path follows the current ArduPilot Gazebo Sim integration:
Gazebo Harmonic on Jammy plus the `ArduPilot/ardupilot_gazebo` plugin. ROS is
not installed by these scripts.

Setup:

```bash
scripts/setup_environment.sh
```

To replace an existing Gazebo Classic install with the compatible Gazebo
Harmonic/Gazebo Sim stack first:

```bash
scripts/reinstall_gazebo.sh
```

The setup script performs these idempotent steps:

- installs required apt packages;
- reuses or creates `~/ardupilot`;
- initializes ArduPilot submodules;
- runs ArduPilot prerequisite installation;
- builds ArduCopter SITL;
- can launch ArduPlane SITL from the same installed ArduPilot checkout;
- reuses or creates `~/ardupilot_gazebo`;
- builds the Gazebo Sim plugin;
- creates `.venv` with Python 3.11+;
- installs `requirements.txt`.

Run the pieces separately:

```bash
scripts/run_gazebo.sh
scripts/run_sitl.sh
```

The default simulation profile remains the original quadcopter/Iris profile.
To select explicitly:

```bash
scripts/run_gazebo.sh -quad
scripts/run_sitl.sh -quad
scripts/run_gazebo.sh -plane
scripts/run_sitl.sh -plane
```

`scripts/run_sitl.sh` exports MAVLink to `udp:127.0.0.1:14550` for Vulture-X
tracking/command helpers, `udp:127.0.0.1:14551` for QGroundControl, and
`udp:127.0.0.1:14552` for the UI status monitor. Keeping UI status on its own
port avoids intermittent heartbeat loss while tracking is also connected.

For a visible SITL/MAVProxy terminal:

```bash
scripts/run_sitl_terminal.sh
```

Or run this in a second terminal yourself:

```bash
VULTURE_X_SITL_INTERACTIVE=1 scripts/run_sitl.sh
```

Run the combined demo:

```bash
scripts/run_demo.sh
```

`run_demo.sh` activates `.venv`, starts Gazebo, starts ArduPilot Copter SITL,
waits for a MAVLink heartbeat on `udpin:0.0.0.0:14550`, requests Gazebo camera
streaming when a streaming topic is available, starts an OpenCV camera viewer,
prints connection status, and stops child processes on `Ctrl+C`.

Verify the environment:

```bash
source .venv/bin/activate
python tools/verify_environment.py
```

Useful partial checks:

```bash
python tools/verify_environment.py --checks python,opencv,pymavlink,gazebo,ardupilot
python tools/verify_environment.py --checks mavlink --mavlink udpin:0.0.0.0:14550
python tools/verify_environment.py --checks camera
```

For QGroundControl, add a UDP comm link listening on port `14551` if the local
SITL stream is not auto-detected.

To verify that the Gazebo camera can see the stationary red target without
sending any vehicle commands:

```bash
python tools/track_camera_target.py --headless --timeout-s 8
```

To run a short SITL-only visual steering pass after the drone is already armed
and in `GUIDED`:

```bash
python tools/sitl_track_target.py \
  --enable-guidance \
  --camera-dir /tmp/vulture-x-target-camera-ertummun \
  --timeout-s 20 \
  --forward-mps 3.0
```

For a guarded local SITL fixed-wing takeoff, start the `-plane` environment,
wait for MAVLink to show connected, then run:

```bash
VULTURE_X_ALLOW_SITL_ARM=1 python tools/sitl_arm_takeoff.py \
  --vehicle plane \
  --altitude-m 50
```

The plane helper follows the official ArduPilot Gazebo Zephyr demo pattern:
switch to `FBWA`, arm, apply `RC3=1800` throttle, then switch to `CIRCLE` after
the model starts rolling. It is not a vertical takeoff. It is simulation-only
and still requires the explicit `VULTURE_X_ALLOW_SITL_ARM=1` operator flag.
The helper sends MAVLink from system id `255` by default because ArduPilot only
accepts RC override from a system id allowed by `MAV_GCS_SYSID`. If your SITL
parameters use a different GCS id, pass `--source-system` to match it.

For a browser control panel that wraps Gazebo, SITL, camera status, target view,
and timed steering:

```bash
scripts/run_ui.sh
```

The UI accepts the same startup profile flags:

```bash
scripts/run_ui.sh -quad
scripts/run_ui.sh -plane
```

By default the UI binds to `0.0.0.0`, prints both localhost and LAN URLs, opens
Gazebo GUI and SITL in separate terminals when a terminal emulator is available,
starts the camera bridge, and starts bounded random motion for the red Gazebo
target. If no supported terminal emulator is found, Gazebo and SITL still start
with logs under `logs/ui/`.

The UI reads heartbeat status from `udpin:0.0.0.0:14552` by default. Override it
with `VULTURE_X_UI_MAVLINK=udpin:0.0.0.0:PORT` only when your SITL/MAVProxy
launch exports a separate status port. If an older SITL instance was already
running with only port `14550`, restart it through `scripts/run_ui.sh` or
`scripts/run_sitl.sh` so the UI receives the dedicated status stream.

The Gazebo launcher clears Python/OpenCV Qt plugin paths before starting Gazebo
so the GUI can use the system Qt plugins. To run Gazebo server-only instead:

```bash
VULTURE_X_GAZEBO_HEADLESS=1 scripts/run_ui.sh
```

Use `scripts/run_ui.sh --no-auto-start` to open only the web UI. Open the
printed `vulture_x_ui_lan_url` from another device on the same network. Anyone
who can reach that URL can operate the local SITL panel.

The UI target mover changes only the simulated `target_marker` pose inside the
existing test area. Steering still requires the drone to already be armed; the
panel does not arm or take off. The steering section lets the operator choose
orange-target tracking or a custom UI-selected target, set forward speed,
vertical climb/descent speed, vertical image-error gain, and command rate for
the timed SITL steering helper. For a custom target, drag a box on the camera
frame, switch tracking mode to custom selection, and start steering. The camera
view has a stream refresh FPS control so the browser can request frames faster
than the once-per-second status poll.

The quad steering panel clamps forward speed to `5 m/s`, vertical speed to
`5 m/s`, and vertical gain to `8.0`. The fixed-wing SITL steering path clamps
ArduPlane guided airspeed to `25 m/s`, altitude slew rate to `10 m/s`, and
vertical gain to `10.0`. These controls are aggressive simulation-only tuning
knobs; the helper still requires the vehicle to already be armed, switches
fixed-wing steering to `GUIDED`, and stops pursuit commands on target loss.

The default quadcopter test world is at `simulation/worlds/vulture_x_test.sdf`. It contains
an ArduPilot-controlled Iris-with-gimbal model from the ArduPilot Gazebo plugin,
a forward camera provided by that model, a visible red target marker, and a
simple open test area. The ground includes visual-only grid lines, colored
patches, simple houses, a shed, and static vehicles so manual UI selections have
trackable corners and texture instead of a flat same-color surface. The camera
stream uses the plugin's GStreamer UDP path on `127.0.0.1:5600`.

The fixed-wing test world is `simulation/worlds/vulture_x_plane.sdf`. It reuses
the same test area and target marker, swaps the vehicle include to
the local `zephyr_with_camera` model, and streams camera video on the same UDP
port for the UI bridge.

## Continuing toward SITL

Read [AGENTS.md](AGENTS.md), [PROJECT_SPEC.md](PROJECT_SPEC.md), and
[docs/sitl_setup.md](docs/sitl_setup.md) before continuing. The next step is a
fully mocked asynchronous `pymavlink` client, followed by read-only validation
against ArduPilot Copter SITL. Arming must always require an explicit operator
CLI flag.
