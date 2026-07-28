# Linux Mint and ArduPilot SITL Handoff

This document is for the Codex agent and operator continuing Vulture-X on a
Linux Mint development machine.

## Current repository state

Vulture-X is at Milestone 0. It has no MAVLink client and no flight-command
authority. The first job on the Linux machine is to reproduce the baseline,
not to start a vehicle.

## 1. Inspect and verify the baseline

```bash
git clone https://github.com/bimors11/vulture-x.git
cd vulture-x
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
pytest
ruff check .
mypy
vulture-x --config configs/default.yaml --check-config
```

Python 3.11 or newer is required. Stop and diagnose any failure before adding
features. Do not weaken validation or tests to make the baseline pass.

## 2. Prepare ArduPilot separately

Linux Mint is Ubuntu-based, but its release and Ubuntu base must be checked
before running ArduPilot's prerequisite installer:

```bash
cat /etc/os-release
```

Use the current official instructions rather than assuming commands in this
handoff are permanently current:

- [ArduPilot Linux build environment](https://ardupilot.org/dev/docs/building-setup-linux.html)
- [ArduPilot SITL setup on Linux](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
- [Using ArduPilot SITL](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)

Keep the ArduPilot checkout beside, not inside, the Vulture-X checkout:

```text
development/
├── ardupilot/
└── vulture-x/
```

A typical official-source setup begins with:

```bash
cd ..
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Before executing the installer, the agent must compare it with the current
official documentation and inspect what it will change. If the Linux Mint base
release is unsupported, stop and use an officially supported environment
instead of forcing the installer.

## 3. Reproduce a standalone Copter SITL

From the ArduPilot checkout, the usual development command is:

```bash
Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  --console \
  --map \
  -w \
  --out=udp:127.0.0.1:14550
```

The `-w` option resets simulated parameters. Use it deliberately, not on every
run once a deterministic scenario has been established. Confirm heartbeat and
basic simulator operation in MAVProxy before involving Vulture-X.

The Vulture-X default endpoint is:

```yaml
vehicle:
  connection: "udpin:0.0.0.0:14550"
```

Port assignments can vary with SITL and MAVProxy options. Record the exact
launch command and observed endpoints in test evidence.

## 4. Continue implementation in specification order

The next code milestone is Task 2, the asynchronous MAVLink vehicle interface.
The Codex agent must:

1. Add the selected MAVLink dependency explicitly to `pyproject.toml`.
2. Implement heartbeat receipt without blocking the main event loop.
3. Validate system/component IDs, ArduPilot identity, vehicle type, and MAVLink
   version before setting the client connected.
4. Add a bounded telemetry cache and clean shutdown.
5. Track `COMMAND_ACK` by command and timeout.
6. Test success, rejection, mismatched identity, missing heartbeat, ACK failure,
   timeout, and shutdown with a mocked FCU.
7. Keep arm, takeoff, and movement-command paths unavailable until their
   specification tasks and explicit operator gate are implemented.

Only after mocked integration tests pass should the client be connected
read-only to Copter SITL. Initial SITL work should receive and log telemetry;
it should not arm or change mode.

Then proceed through Tasks 3–8 in `PROJECT_SPEC.md`, preserving their order and
acceptance criteria.

## 5. SITL evidence required

For each automated scenario, record:

- Vulture-X commit and configuration;
- ArduPilot commit and launch command;
- Python and dependency versions;
- start/end time and random seed, when applicable;
- state transitions, received telemetry, safety decisions, and commands;
- machine-readable pass/fail metrics;
- logs needed to reproduce the decision sequence.

Before claiming Version 0.1 acceptance, automate every mandatory scenario in
section 17.3 of `PROJECT_SPEC.md`. In particular, prove stale-target,
heartbeat-loss, GPS/EKF failure, battery-return, operator-abort,
acknowledgement-timeout, restart, geofence, and minimum-separation behavior.

## 6. Safety boundary

SITL success is simulation evidence only. Do not connect propulsion, perform
HIL, or attempt outdoor flight under this handoff. Those stages require the
separate gates, safety case, test cards, and approvals defined in the
specification.

