# Test Plan

The first milestone verifies:

- configuration acceptance and fail-closed validation;
- deterministic synthetic frames and disappearance scenarios;
- bounding-box normalization and target-loss output;
- OpenCV synthetic-video tracking integration;
- guidance deadbands, mapping, expiry, speed limiting, and acceleration limiting;
- tracking warning/HOLD/ABORT timeout boundaries;
- heartbeat timeout boundaries;
- mission transitions through TRACK, invalid transitions, ABORT, and reset;
- mock identity, explicit arm enable, GUIDED requirement, and expired-command
  rejection;
- JSONL event and CSV telemetry output.

No current test connects to ArduPilot. SITL tests remain empty until a real
asynchronous MAVLink client exists and read-only connectivity is demonstrated.

