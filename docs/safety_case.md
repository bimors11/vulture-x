# Safety Case

## Current claim

The Milestone 0 program cannot command a vehicle. It loads a strict
configuration, initializes structured logging, reports that command authority
is disabled, and exits.

## Operational limitation

This repository is not flight-ready. It must not be used to arm, navigate, or
control an aircraft. ArduPilot SITL integration, vehicle identity validation,
health monitoring, geofencing, separation enforcement, and abort behavior are
future milestones requiring their own evidence.

## Current controls

- Unknown configuration fields fail validation.
- Unsafe threshold ordering fails validation.
- Unsupported frames and autopilot identities fail validation.
- Data-age configuration uses monotonic-time semantics.
- The default configuration targets SITL.
- There are no command endpoints or automatic arming paths.

## Open hazards

All runtime flight hazards in `PROJECT_SPEC.md` remain open until their owning
milestones are implemented and tested. Passing Milestone 0 tests is not flight
approval.

