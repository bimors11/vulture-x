# Vulture-X

Vulture-X is a safety-oriented research platform for cooperative, non-contact
UAV stand-off experiments using ArduPilot, MAVLink, constrained guidance, and
deterministic simulation.

This initial release contains the Milestone 0 foundation:

- immutable data models and mission/safety enums;
- strict YAML configuration with cross-field safety validation;
- structured console and JSON Lines event logging;
- a command-line bootstrap and configuration check;
- unit tests, linting, and strict type-checking configuration.

It does **not** connect to a flight controller, arm a vehicle, or transmit
movement commands. Those capabilities belong to later, separately tested
milestones.

## Setup

Python 3.11 or newer is required.

```bash
python -m venv .venv
python -m pip install -e ".[dev]"
```

Validate the default configuration:

```bash
vulture-x --config configs/default.yaml --check-config
```

Run the safe bootstrap:

```bash
vulture-x --config configs/default.yaml
```

Run development checks:

```bash
pytest
ruff check .
mypy
```

See [PROJECT_SPEC.md](PROJECT_SPEC.md) for the engineering baseline and
[docs/safety_case.md](docs/safety_case.md) for current operational limits.
Linux Mint and ArduPilot continuation instructions are in
[docs/LINUX_MINT_SITL_HANDOFF.md](docs/LINUX_MINT_SITL_HANDOFF.md), with
repository-wide Codex rules in [AGENTS.md](AGENTS.md).
