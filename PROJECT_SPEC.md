You are developing a new repository named `vulture-x`.

Current repository note: the package-level first milestone remains
simulation-first, while local helper tools now include an ArduPlane fixed-wing
tracking path that uses `FBWA` plus bounded `RC_CHANNELS_OVERRIDE`. Do not move
that fixed-wing path back to `GUIDED_CHANGE_*` commands.

The project is a ground-based visual tracking and UAV guidance research system. It must be tested first using ArduPilot SITL before any real hardware integration.

## System Architecture

The final physical architecture will be:

```text
Drone:
- 5-inch to 7-inch FPV quadcopter
- ArduPilot-compatible flight controller
- GPS
- ELRS receiver
- Analog FPV camera
- Analog VTX
- No onboard companion computer

Ground:
- Analog video receiver
- USB analog video capture device
- Linux laptop
- OpenCV tracking application
- MAVLink guidance application
- ELRS transmitter module
```

Data flow:

```text
Analog camera
    ↓
Analog VTX
    ↓
Ground video receiver
    ↓
USB video capture
    ↓
OpenCV target tracker
    ↓
Guidance controller
    ↓
MAVLink velocity setpoints
    ↓
ELRS MAVLink link
    ↓
ArduPilot GUIDED mode
```

For the first development phase, replace the real camera, VTX, video receiver, and ELRS link with simulation interfaces.

The initial system shall run entirely with:

* ArduPilot Copter SITL
* Python 3.11 or newer
* pymavlink
* OpenCV
* asyncio
* YAML configuration
* pytest
* ruff
* mypy

Do not use ROS.

## Safety Boundary

The software is intended for controlled visual tracking and non-contact flight testing.

Do not implement:

* intentional collision
* impact guidance
* terminal attack logic
* RF interference
* spoofing
* jamming
* payload deployment
* autonomous engagement of real aircraft

The interceptor must maintain a configurable minimum separation distance during simulation and flight testing.

## First Development Objective

Create a working SITL prototype where:

1. ArduPilot Copter SITL represents the interceptor drone.
2. A simulated video source represents the analog camera feed.
3. A visible synthetic target moves across the video frame.
4. OpenCV tracks the target.
5. The tracker outputs:

   * normalized horizontal image error
   * normalized vertical image error
   * target bounding-box size
   * tracking confidence
   * target lost status
6. A guidance controller converts image tracking error into limited navigation commands.
7. Navigation commands are sent to ArduPilot SITL using MAVLink in `GUIDED` mode.
8. The system enters a safe state when:

   * the target is lost
   * tracking confidence is too low
   * MAVLink communication is lost
   * SITL reports unhealthy navigation
   * command limits are exceeded
9. All telemetry, tracking outputs, guidance commands, and state transitions are logged.
10. The complete simulation can be launched with one command.

## Repository Structure

Create this structure:

```text
vulture-x/
├── README.md
├── PROJECT_SPEC.md
├── pyproject.toml
├── .gitignore
├── configs/
│   ├── default.yaml
│   └── sitl.yaml
├── src/
│   └── vulture_x/
│       ├── __init__.py
│       ├── main.py
│       ├── config.py
│       ├── models.py
│       ├── enums.py
│       ├── vehicle/
│       │   ├── mavlink_client.py
│       │   ├── telemetry.py
│       │   ├── commands.py
│       │   └── health.py
│       ├── vision/
│       │   ├── video_source.py
│       │   ├── synthetic_video.py
│       │   ├── tracker.py
│       │   └── target_state.py
│       ├── guidance/
│       │   ├── controller.py
│       │   ├── image_guidance.py
│       │   └── limiters.py
│       ├── mission/
│       │   ├── supervisor.py
│       │   ├── state_machine.py
│       │   └── guards.py
│       ├── safety/
│       │   ├── supervisor.py
│       │   ├── tracking_safety.py
│       │   ├── command_safety.py
│       │   └── abort_policy.py
│       └── logging/
│           ├── event_logger.py
│           └── telemetry_logger.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── sitl/
├── scripts/
│   ├── install_ardupilot_sitl.sh
│   ├── run_sitl.sh
│   ├── run_vulture_x.sh
│   └── run_demo.sh
└── docs/
    ├── architecture.md
    ├── sitl_setup.md
    ├── tracking.md
    └── safety.md
```

## Mission State Machine

Implement:

```text
BOOT
  ↓
SELF_TEST
  ↓
WAIT_FCU
  ↓
READY
  ↓
ARMED
  ↓
TAKEOFF
  ↓
SEARCH
  ↓
TRACK
  ↓
GUIDANCE
  ↓
HOLD
  ↓
RETURN
  ↓
LAND

Any active state
  ↓
ABORT
```

State behavior:

### SEARCH

* Display the video feed.
* Search for a target.
* Do not send pursuit commands.
* Allow manual target selection.

### TRACK

* Run the selected OpenCV tracker.
* Calculate image-plane errors.
* Validate confidence.
* Do not send unrestricted commands.

### GUIDANCE

* Convert image error into limited MAVLink navigation setpoints.
* Continuously validate tracking confidence.
* Apply speed, acceleration, and yaw-rate limits.
* Abort when target data becomes invalid.

### HOLD

* Stop pursuit commands.
* Command zero horizontal velocity or switch to `LOITER`.
* Wait for target reacquisition or operator instruction.

### RETURN

* Command ArduPilot `RTL`.
* Stop sending tracking guidance.

### ABORT

* Stop all guidance commands.
* Command `LOITER` or `RTL`.
* Log the abort reason.
* Require explicit operator reset.

## Tracking Interface

Create this model:

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class TrackingResult:
    timestamp_monotonic_s: float
    detected: bool
    confidence: float
    center_x_normalized: float
    center_y_normalized: float
    width_normalized: float
    height_normalized: float
    horizontal_error: float
    vertical_error: float
```

Normalized image coordinates:

```text
horizontal_error:
-1.0 = far left
 0.0 = image center
+1.0 = far right

vertical_error:
-1.0 = top
 0.0 = image center
+1.0 = bottom
```

## Synthetic Video Source

Implement a synthetic video generator using OpenCV.

Requirements:

* configurable resolution
* configurable frame rate
* moving target rectangle or circle
* configurable target speed
* configurable target size
* configurable background
* optional noise
* optional temporary target disappearance
* optional sudden target movement
* optional frame delay

Default:

```yaml
vision:
  source: "synthetic"
  width: 640
  height: 480
  fps: 30
  tracker: "CSRT"
```

The synthetic target must move through repeatable scenarios using a deterministic random seed.

## Tracker

Start with OpenCV CSRT or KCF.

The tracker must support:

* manual initial target selection
* synthetic automatic initialization
* confidence estimation
* target-lost detection
* tracker reset
* bounding-box validation

Do not add neural-network detection in the first milestone.

## Guidance Strategy

Use image-based guidance.

Initial mapping:

```text
horizontal image error
    ↓
yaw-rate command or lateral velocity command

vertical image error
    ↓
vertical velocity command

target apparent size
    ↓
forward velocity command
```

For the initial SITL implementation:

* use horizontal image error for yaw or lateral correction
* use vertical image error for climb or descent correction
* use target bounding-box size only as a simulated range indicator
* apply strict command limits

Create:

```python
@dataclass(frozen=True)
class GuidanceCommand:
    timestamp_monotonic_s: float
    velocity_forward_mps: float
    velocity_right_mps: float
    velocity_down_mps: float
    yaw_rate_deg_s: float
    valid_until_monotonic_s: float
    reason: str
```

All guidance commands must expire.

Expired commands must never be sent.

## Initial Guidance Limits

Use conservative SITL defaults:

```yaml
guidance:
  update_rate_hz: 10
  max_forward_speed_mps: 5.0
  max_lateral_speed_mps: 3.0
  max_vertical_speed_mps: 2.0
  max_acceleration_mps2: 1.5
  max_yaw_rate_deg_s: 30.0
  horizontal_deadband: 0.05
  vertical_deadband: 0.05
  target_size_setpoint: 0.12
```

## Tracking Safety

Use:

```yaml
safety:
  minimum_tracking_confidence: 0.6
  tracking_warning_timeout_s: 0.3
  tracking_abort_timeout_s: 1.0
  mavlink_heartbeat_timeout_s: 2.0
  command_expiration_s: 0.25
  minimum_separation_m: 10.0
  maximum_test_speed_mps: 5.0
```

Behavior:

```text
Tracking valid:
    send limited guidance commands

Tracking briefly invalid:
    command zero pursuit velocity

Tracking invalid beyond warning timeout:
    enter HOLD

Tracking invalid beyond abort timeout:
    enter ABORT and command LOITER or RTL
```

## MAVLink Integration

Use `pymavlink`.

Required telemetry:

* `HEARTBEAT`
* `GLOBAL_POSITION_INT`
* `LOCAL_POSITION_NED`
* `ATTITUDE`
* `VFR_HUD`
* `SYS_STATUS`
* `GPS_RAW_INT`
* `EKF_STATUS_REPORT`
* `COMMAND_ACK`

Required commands:

* mode change
* arm
* takeoff
* velocity setpoint
* yaw or yaw-rate setpoint
* loiter
* RTL
* land

Use ArduPilot `GUIDED` mode.

Do not use PX4 `OFFBOARD` assumptions.

Do not send:

* raw motor PWM
* direct actuator commands
* unrestricted attitude-rate commands
* commands without expiration
* commands before vehicle identity is validated

## ELRS Abstraction

The first SITL version shall not depend directly on ELRS hardware.

Create a generic MAVLink transport interface:

```python
class MavlinkTransport:
    async def connect(self) -> None:
        ...

    async def receive(self):
        ...

    async def send(self, message) -> None:
        ...

    async def close(self) -> None:
        ...
```

Initial implementation:

```text
UDP SITL transport
```

Future implementation:

```text
ELRS MAVLink UDP or serial transport
```

The guidance, tracking, and mission modules must not depend on whether MAVLink is transported through:

* SITL UDP
* serial telemetry
* ELRS
* another supported transport

## Analog Video Abstraction

Create a video source interface:

```python
class VideoSource:
    async def open(self) -> None:
        ...

    async def read(self):
        ...

    async def close(self) -> None:
        ...
```

Implement:

1. Synthetic video source.
2. Video-file source.
3. V4L2 USB capture source.

The real analog system will appear to Linux as a USB capture device such as:

```text
/dev/video0
```

The tracking code must not depend on whether frames come from synthetic video, a file, or an analog capture card.

## Configuration Example

Create `configs/sitl.yaml`:

```yaml
project:
  name: "vulture-x"
  environment: "sitl"

vehicle:
  connection: "udpin:0.0.0.0:14550"
  source_system: 191
  source_component: 191
  guided_mode: "GUIDED"
  hold_mode: "LOITER"
  recovery_mode: "RTL"
  takeoff_altitude_m: 10.0

vision:
  source: "synthetic"
  width: 640
  height: 480
  fps: 30
  tracker: "CSRT"
  show_window: true

guidance:
  update_rate_hz: 10
  max_forward_speed_mps: 5.0
  max_lateral_speed_mps: 3.0
  max_vertical_speed_mps: 2.0
  max_acceleration_mps2: 1.5
  max_yaw_rate_deg_s: 30.0
  target_size_setpoint: 0.12

safety:
  minimum_tracking_confidence: 0.6
  tracking_warning_timeout_s: 0.3
  tracking_abort_timeout_s: 1.0
  mavlink_heartbeat_timeout_s: 2.0
  command_expiration_s: 0.25
  minimum_separation_m: 10.0

logging:
  level: "INFO"
  directory: "./logs"
  event_jsonl: true
  telemetry_csv: true
```

## Logging

Log:

* mission state
* ArduPilot mode
* armed state
* vehicle position
* vehicle attitude
* tracking bounding box
* tracking confidence
* horizontal and vertical image error
* raw guidance command
* limited guidance command
* command timestamp
* command expiry
* safety flags
* abort reason
* MAVLink acknowledgement

Use:

* JSONL for events
* CSV for time-series telemetry

## Tests

Implement unit tests for:

* configuration validation
* tracker output normalization
* target-lost detection
* guidance deadband
* velocity limiting
* acceleration limiting
* command expiration
* tracking timeout
* MAVLink heartbeat timeout
* mission state transitions
* invalid transition rejection
* abort behavior

Implement integration tests for:

* synthetic video to tracker
* tracker to guidance
* guidance to limiter
* MAVLink command generation
* mission supervisor with mocked vehicle

Implement SITL tests for:

1. Successful MAVLink connection.
2. Arm and takeoff.
3. Enter `GUIDED`.
4. Track synthetic target.
5. Generate bounded guidance commands.
6. Target disappears temporarily.
7. System enters `HOLD`.
8. Target remains missing.
9. System enters `ABORT`.
10. ArduPilot switches to `LOITER` or `RTL`.
11. All events are logged.

## Scripts

Create:

```text
scripts/run_sitl.sh
scripts/run_vulture_x.sh
scripts/run_demo.sh
```

`run_demo.sh` must:

1. Start ArduPilot Copter SITL.
2. Wait for SITL readiness.
3. Start Vulture-X.
4. Load `configs/sitl.yaml`.
5. Start the synthetic video source.
6. Display tracking status.
7. Stop all processes cleanly on exit.

## Coding Rules

* Use Python 3.11 or newer.
* Use type hints.
* Use asyncio.
* Use monotonic time for timeouts and command expiration.
* Use bounded queues.
* No global mutable state.
* No silent exception handling.
* No hard-coded addresses outside configuration.
* No automatic arming without explicit CLI enable flag.
* No unrestricted control outputs.
* No hidden fallback behavior.
* Every safety-related failure must have a machine-readable reason code.

## First Implementation Milestone

Implement only the following in the first pass:

1. Repository structure.
2. Configuration loader and validation.
3. Data models and enums.
4. Structured logging.
5. Synthetic video generator.
6. OpenCV tracker.
7. Basic image-error calculation.
8. Guidance command model.
9. Command limiters.
10. Mock MAVLink vehicle interface.
11. Mission state machine through `TRACK`.
12. Unit tests.
13. README with setup instructions.

Do not implement real ELRS hardware support yet.

Do not implement real analog capture until the synthetic video path works.

After the first milestone:

* run `pytest`;
* run `ruff`;
* run `mypy`;
* report all generated files;
* report assumptions;
* report incomplete components;
* do not claim SITL integration works unless it has been executed successfully.
