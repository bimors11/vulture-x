# Codex Agent Instructions

These instructions apply to the entire Vulture-X repository.

## Authority and current boundary

`PROJECT_SPEC.md` is the current authoritative prompt. Read it, this file,
`README.md`, `docs/architecture.md`, `docs/safety.md`, and
`docs/sitl_setup.md` before changing code.

This repository implements the first milestone only. It has synthetic video,
OpenCV tracking, bounded image guidance, safety policies, a mock vehicle, and
mission state transitions through `TRACK`. It has no functioning SITL or
hardware MAVLink client. Do not claim otherwise.

## Product rules

- Keep the product/repository name `vulture-x` and Python package `vulture_x`.
- The ground laptop performs vision and high-level guidance; there is no
  onboard companion computer in the planned architecture.
- Use ArduPilot `GUIDED`, never PX4 `OFFBOARD` assumptions.
- Keep MAVLink transport independent of guidance, tracking, and mission code.
- Keep video sources independent of tracking code.
- Use Python 3.11+, asyncio, bounded queues, strict types, YAML configuration,
  monotonic timing, and structured logs.
- Never auto-arm. Any future arm path requires an explicit CLI enable flag and
  verified FCU identity, health, mode, and operator intent.
- Commands must be bounded and expire. Expired commands must never be sent.
- Tracking loss must zero pursuit, then cause HOLD, then ABORT according to
  configured monotonic timeouts.
- Safety decisions override guidance and mission decisions.
- Incomplete safety-critical functions must raise clearly or remain absent;
  never add a hidden successful fallback.
- Do not add collision/contact, terminal attack, RF interference, spoofing,
  jamming, payload deployment, or real-aircraft engagement behavior.
- Maintain configurable minimum separation. Image size alone is not verified
  physical range; real separation enforcement remains incomplete until a
  validated range source exists.

## Required checks

Use an isolated Python environment:

```bash
python -m pip install -e ".[dev]"
pytest
ruff check .
mypy
vulture-x --config configs/default.yaml --check-config
```

Every change needs tests, documentation, machine-readable failure reasons, and
a report of assumptions and incomplete components.

## Next implementation order

1. Add an asynchronous `pymavlink` UDP transport/client with mocked heartbeat,
   identity, telemetry cache, ACK correlation, timeout, and clean-shutdown
   tests.
2. Connect read-only to Copter SITL and record exact ArduPilot commit, launch
   command, ports, and logs.
3. Add command encoding tests without sending commands.
4. Add explicit operator-enable CLI gating.
5. Exercise mode/arm/takeoff only in SITL after all guards and ACK handling
   pass.
6. Integrate synthetic tracking and bounded guidance with SITL.
7. Automate target-loss HOLD and ABORT/LOITER-or-RTL evidence.
8. Implement video-file and V4L2 sources only after the synthetic path is
   stable.
9. Add ELRS only after transport-independent SITL behavior is proven.

Do not connect propulsion, perform HIL, or attempt outdoor flight under these
instructions.

