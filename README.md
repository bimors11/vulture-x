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
- CSRT/KCF tracking with normalized image-plane results;
- bounded image-error guidance and expiring commands;
- speed, vertical-speed, acceleration, and yaw-rate limiting;
- tracking timeout, heartbeat timeout, and navigation-health checks;
- transport abstraction and a mock-only MAVLink vehicle;
- explicit mission state machine through `TRACK`;
- JSONL events and CSV telemetry logging;
- unit and mocked integration tests.

Not implemented:

- a real `pymavlink` UDP client;
- ArduPilot SITL command integration;
- `GUIDED` flight, takeoff, LOITER, RTL, or LAND against SITL;
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

## Continuing toward SITL

Read [AGENTS.md](AGENTS.md), [PROJECT_SPEC.md](PROJECT_SPEC.md), and
[docs/sitl_setup.md](docs/sitl_setup.md) before continuing. The next step is a
fully mocked asynchronous `pymavlink` client, followed by read-only validation
against ArduPilot Copter SITL. Arming must always require an explicit operator
CLI flag.

