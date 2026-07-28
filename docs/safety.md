# Safety

## Current boundary

The first milestone uses a mock vehicle only. It cannot connect to ArduPilot,
ELRS, a serial radio, or real hardware.

## Implemented controls

- Strict configuration rejects unknown or inconsistent settings.
- Tracking invalidity produces WARNING, then HOLD_REQUIRED, then
  ABORT_REQUIRED using monotonic time.
- Heartbeat loss and unhealthy navigation produce machine-readable abort
  results.
- Guidance outputs have a short validity interval.
- Expired commands are rejected both by command safety and the mock vehicle.
- Forward, lateral, vertical, acceleration, and yaw-rate limits are enforced.
- Mock arming requires explicit operator enable and validated mock identity.
- ABORT requires explicit operator reset.

## Important unresolved constraint

Target apparent size is only a simulated range indicator. It does not establish
physical separation in metres. Before real guidance can be enabled, a validated
range/separation source and conservative uncertainty handling must enforce
`minimum_separation_m`.

## Prohibited behavior

Do not implement intentional collision, impact or terminal attack guidance, RF
interference, spoofing, jamming, payload deployment, autonomous engagement of
real aircraft, raw PWM, direct actuator control, or unrestricted command output.

