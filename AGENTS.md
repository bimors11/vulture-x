# Codex Agent Instructions

These instructions apply to the entire Vulture-X repository.

## Start here

Read, in order:

1. `PROJECT_SPEC.md`
2. `README.md`
3. `docs/safety_case.md`
4. `docs/LINUX_MINT_SITL_HANDOFF.md`
5. `docs/test_plan.md`

The repository currently implements only Milestone 0 / Task 1. Configuration,
shared data models, structured logging, a safe CLI bootstrap, and unit tests
exist. MAVLink connectivity, vehicle commands, target ingestion, guidance,
mission supervision, and SITL integration do not yet exist.

## Development rules

- Keep the product name `Vulture-X`, distribution/repository name `vulture-x`,
  Python package name `vulture_x`, and target protocol name
  `vulture-target-v1`.
- Implement the tasks in section 26 of `PROJECT_SPEC.md` in order.
- Complete one milestone at a time. Do not skip directly to flight behavior.
- Preserve ArduPilot flight-control authority and internal failsafes.
- Never arm automatically at process startup.
- Require explicit operator enable before any future arm or takeoff path.
- Never transmit movement setpoints until FCU identity, mode, health, position,
  home, target freshness/frame, and safety guards are validated.
- Every transmitted command must have ACK or telemetry confirmation, a bounded
  retry policy, an expiry/timeout, and structured logs.
- Use monotonic time for freshness, timeout, and command-expiration decisions.
- Reject stale, malformed, unsupported-frame, or low-confidence target data.
- Safety decisions override mission and guidance decisions.
- Do not add collision/contact, jamming, spoofing, denial-of-service,
  weaponization, or non-cooperative-aircraft control behavior.
- Incomplete safety-critical functions must raise `NotImplementedError`; they
  must never silently succeed.

## Required checks for every change

From an activated Python 3.11+ virtual environment:

```bash
python -m pip install -e ".[dev]"
pytest
ruff check .
mypy
vulture-x --config configs/default.yaml --check-config
```

Add or update tests and documentation with every implementation change. Report
the commands run, results, assumptions, and unresolved safety risks.

## SITL gate

Use ArduPilot Copter SITL only after the existing checks pass. Keep ArduPilot in
a separate checkout; do not vendor it into this repository. Follow
`docs/LINUX_MINT_SITL_HANDOFF.md`, verify Linux Mint/Ubuntu-base compatibility,
and remain in SITL until all mandatory simulated failure cases for the active
milestone pass.

Do not interpret successful unit tests or a successful SITL build as approval
for HIL, propulsion-connected testing, or outdoor flight.

