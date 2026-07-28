# Vulture-X

## Engineering Specification and Development Baseline

**Document status:** Initial development baseline  
**Intended use:** Cooperative, non-contact UAV interception research, simulation, HIL testing, and controlled flight testing  
**Primary autopilot:** ArduPilot  
**Companion environment:** Linux, Python 3.11+, optional ROS 2  
**Primary transport:** MAVLink 2 over UDP or serial  
**Repository type:** Safety-oriented research software

---

## 1. Project Objective

Vulture-X is a modular software stack for a cooperative UAV interceptor that:

1. Connects to an ArduPilot vehicle through MAVLink.
2. Receives target state from an authorized cooperative source.
3. Estimates relative target motion.
4. Generates constrained approach commands.
5. Maintains a configurable stand-off distance.
6. Aborts safely when target confidence, vehicle health, link quality, or geofence conditions become invalid.
7. Supports Software-in-the-Loop, Hardware-in-the-Loop, and controlled outdoor testing.

This project shall not implement collision, kinetic engagement, unauthorized radio interference, GNSS spoofing, denial-of-service, or autonomous harmful action.

The initial product definition is:

> An autonomous chase-and-stand-off demonstrator for cooperative targets.

---

## 2. Scope

### 2.1 In scope

- ArduPilot SITL integration.
- MAVLink vehicle connection.
- Vehicle health monitoring.
- Cooperative target telemetry ingestion.
- Relative-state estimation.
- Position and velocity guidance.
- Stand-off orbit or trailing behavior.
- State-machine-based mission supervision.
- Geofence and separation enforcement.
- Link-loss and stale-data handling.
- Logging and replay.
- Deterministic simulation scenarios.
- Unit, integration, SITL, and HIL tests.
- Optional ROS 2 adapters.

### 2.2 Out of scope

- RF spoofing.
- RF jamming.
- GNSS manipulation.
- Unauthorized target exploitation.
- Physical collision.
- Payload release.
- Weaponization.
- Facial recognition.
- Autonomous identification of persons.
- Control of non-cooperative aircraft.
- Operation outside segregated test areas.

---

## 3. Design Principles

The system shall follow these principles:

1. **ArduPilot retains flight-control authority.**  
   The companion computer sends only high-level setpoints. Stabilization, EKF, actuator control, and core failsafes remain in the autopilot.

2. **Safety constraints override mission objectives.**  
   Separation, geofence, battery reserve, data age, and vehicle health have higher priority than interception.

3. **All mission behavior is state-machine controlled.**  
   Commands shall not be issued through unstructured callback logic.

4. **No command is trusted without feedback.**  
   Mode changes, arming, takeoff, and mission commands must be verified through telemetry or `COMMAND_ACK`.

5. **Target data must be time-valid and frame-valid.**  
   Stale, unframed, or low-confidence target data shall not drive the vehicle.

6. **Simulation precedes flight.**  
   Every behavior must pass unit tests and SITL scenarios before HIL or outdoor testing.

7. **The system must fail predictably.**  
   Loss of companion, loss of target, bad GPS, bad EKF, or link degradation must result in a defined safe state.

---

## 4. Top-Level Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                     Mission Supervisor                      │
│ state machine, guards, abort logic, operator authority      │
├───────────────────────────────┬─────────────────────────────┤
│ Target Ingestion              │ Vehicle Interface           │
│ cooperative target telemetry  │ MAVLink connection          │
│ timestamp/frame validation    │ health, modes, ACK handling  │
├───────────────────────────────┼─────────────────────────────┤
│ Target Estimator              │ Safety Supervisor           │
│ filtering and prediction      │ geofence, separation, limits │
├───────────────────────────────┼─────────────────────────────┤
│ Guidance Manager              │ Logging and Replay          │
│ pursuit, lead, orbit, trail   │ events, telemetry, commands  │
├───────────────────────────────┴─────────────────────────────┤
│                 ArduPilot SITL or Real Autopilot            │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. Recommended Repository Structure

```text
vulture-x/
├── README.md
├── PROJECT_SPEC.md
├── LICENSE
├── pyproject.toml
├── .env.example
├── .gitignore
├── configs/
│   ├── default.yaml
│   ├── sitl.yaml
│   ├── hil.yaml
│   └── flight_test.yaml
├── docs/
│   ├── architecture.md
│   ├── safety_case.md
│   ├── mavlink_interface.md
│   ├── coordinate_frames.md
│   ├── test_plan.md
│   └── flight_test_cards.md
├── src/
│   └── vulture_x/
│       ├── __init__.py
│       ├── main.py
│       ├── config.py
│       ├── models.py
│       ├── enums.py
│       ├── clock.py
│       ├── mission/
│       │   ├── supervisor.py
│       │   ├── state_machine.py
│       │   ├── guards.py
│       │   └── transitions.py
│       ├── vehicle/
│       │   ├── mavlink_client.py
│       │   ├── telemetry.py
│       │   ├── commands.py
│       │   ├── health.py
│       │   └── mode_manager.py
│       ├── target/
│       │   ├── source.py
│       │   ├── cooperative_udp.py
│       │   ├── mavlink_target.py
│       │   ├── validator.py
│       │   └── estimator.py
│       ├── guidance/
│       │   ├── manager.py
│       │   ├── pursuit.py
│       │   ├── lead.py
│       │   ├── trail.py
│       │   ├── standoff.py
│       │   └── limiters.py
│       ├── safety/
│       │   ├── supervisor.py
│       │   ├── geofence.py
│       │   ├── separation.py
│       │   ├── data_freshness.py
│       │   └── abort_policy.py
│       ├── logging/
│       │   ├── event_logger.py
│       │   ├── telemetry_logger.py
│       │   └── replay.py
│       └── api/
│           ├── status_server.py
│           └── schemas.py
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── sitl/
│   └── replay/
├── scripts/
│   ├── run_sitl.sh
│   ├── run_vulture.sh
│   ├── run_target_sim.sh
│   └── analyze_logs.py
├── docker/
│   ├── Dockerfile
│   └── compose.yaml
└── .github/
    └── workflows/
        ├── test.yaml
        └── lint.yaml
```

---

## 6. Operating Modes

The initial implementation shall support:

### 6.1 Observe

The system receives target and vehicle telemetry but sends no movement commands.

Use this mode for:

- telemetry validation;
- timestamp validation;
- frame validation;
- estimator testing;
- dry-run logging.

### 6.2 Follow

The interceptor follows the cooperative target while preserving:

- minimum horizontal separation;
- minimum vertical separation;
- maximum closure rate;
- maximum speed;
- maximum acceleration;
- geofence constraints.

### 6.3 Stand-off

The interceptor approaches until the configured stand-off radius is reached, then:

- holds position;
- trails the target;
- or orbits around a predicted target position.

### 6.4 Return

The mission supervisor commands RTL or another configured recovery behavior.

### 6.5 Abort

Abort is a high-priority state entered when any critical safety condition is violated.

---

## 7. Mission State Machine

```text
BOOT
  ↓
SELF_TEST
  ↓
WAIT_FCU
  ↓
WAIT_HOME
  ↓
READY
  ↓
ARMED_STANDBY
  ↓
TAKEOFF
  ↓
OBSERVE
  ↓
TRACK
  ↓
APPROACH
  ↓
STANDOFF
  ↓
RETURN
  ↓
LAND
  ↓
COMPLETE

Any active state
  └──────────────→ ABORT
```

### 7.1 State definitions

#### BOOT

Actions:

- load configuration;
- initialize logging;
- initialize event bus;
- validate software version.

Exit criteria:

- configuration valid;
- required modules initialized.

#### SELF_TEST

Actions:

- check configuration ranges;
- check coordinate-frame settings;
- check local clock;
- check target-source configuration;
- verify command limiter initialization.

Exit criteria:

- no critical self-test failures.

#### WAIT_FCU

Actions:

- connect to MAVLink endpoint;
- wait for heartbeat;
- identify autopilot and vehicle type;
- confirm MAVLink protocol version.

Exit criteria:

- valid FCU heartbeat received;
- system ID and component ID validated.

#### WAIT_HOME

Actions:

- wait for global position;
- wait for valid home position;
- wait for EKF health.

Exit criteria:

- home position valid;
- position estimate valid;
- geofence reference established.

#### READY

Actions:

- remain disarmed;
- publish readiness status;
- wait for operator enable.

Exit criteria:

- explicit operator authorization;
- all pre-arm mission guards pass.

#### ARMED_STANDBY

Actions:

- arm through verified command path;
- confirm armed telemetry state.

Exit criteria:

- vehicle armed;
- flight mode confirmed.

#### TAKEOFF

Actions:

- command takeoff to configured altitude;
- monitor ascent rate and altitude.

Exit criteria:

- altitude reached within tolerance;
- vehicle health remains valid.

#### OBSERVE

Actions:

- receive target state;
- run target validation;
- run estimator;
- do not issue chase commands.

Exit criteria:

- target confidence above threshold;
- operator enables tracking.

#### TRACK

Actions:

- maintain target estimate;
- assess intercept feasibility;
- calculate safe approach corridor.

Exit criteria:

- valid guidance solution;
- all safety constraints satisfied.

#### APPROACH

Actions:

- generate constrained setpoints;
- limit closure rate;
- continuously evaluate abort guards.

Exit criteria:

- stand-off condition reached;
- or mission timeout;
- or safety violation.

#### STANDOFF

Actions:

- maintain configured relative geometry;
- optionally trail or orbit;
- prevent entry into minimum separation volume.

Exit criteria:

- mission complete;
- target lost;
- return requested;
- safety violation.

#### RETURN

Actions:

- stop target pursuit;
- command configured recovery mode.

Exit criteria:

- recovery mode accepted;
- landing sequence begins.

#### LAND

Actions:

- supervise landing telemetry;
- stop companion guidance commands.

Exit criteria:

- vehicle disarmed.

#### ABORT

Actions:

- stop all mission guidance;
- command configured safe mode;
- log the triggering condition;
- require explicit reset before resuming.

---

## 8. Safety Constraints

The following parameters shall exist in configuration:

```yaml
safety:
  min_horizontal_separation_m: 30.0
  min_vertical_separation_m: 15.0
  preferred_standoff_m: 50.0
  max_closure_rate_mps: 5.0
  max_groundspeed_mps: 15.0
  max_vertical_speed_mps: 3.0
  max_acceleration_mps2: 2.0
  max_yaw_rate_deg_s: 30.0
  target_warning_age_s: 0.5
  target_abort_age_s: 2.0
  vehicle_heartbeat_timeout_s: 2.0
  command_ack_timeout_s: 1.0
  mission_timeout_s: 600
  min_battery_remaining_pct: 35
  return_battery_remaining_pct: 45
```

### 8.1 Mandatory abort triggers

The mission shall abort when any of the following occurs:

- FCU heartbeat timeout.
- Target data age exceeds abort threshold.
- Target coordinate frame is unknown.
- Vehicle EKF becomes unhealthy.
- Vehicle position becomes invalid.
- Geofence boundary is violated or predicted to be violated.
- Horizontal or vertical separation falls below the hard minimum.
- Battery falls below the configured return threshold.
- Command acknowledgements repeatedly fail.
- Companion process detects internal exception in a safety-critical module.
- Operator activates manual abort.
- Guidance output exceeds configured limits.
- Target estimator covariance exceeds configured threshold.

### 8.2 Command authority

Priority order:

1. RC pilot or independent safety pilot.
2. Autopilot internal failsafe.
3. Safety supervisor.
4. Mission supervisor.
5. Guidance manager.
6. Operator mission request.

The guidance manager shall never override a safety decision.

---

## 9. Coordinate Frames

The project shall use explicit frame definitions.

### 9.1 Internal navigation frame

Recommended internal frame:

- local NED for guidance and relative motion;
- WGS84 latitude, longitude, altitude for global telemetry;
- monotonic timestamps for timing;
- UTC timestamps only for logging and correlation.

### 9.2 Required target fields

```python
@dataclass(frozen=True)
class TargetState:
    source_id: str
    timestamp_monotonic_s: float
    latitude_deg: float
    longitude_deg: float
    altitude_msl_m: float
    velocity_n_mps: float
    velocity_e_mps: float
    velocity_d_mps: float
    heading_deg: float | None
    position_accuracy_m: float | None
    velocity_accuracy_mps: float | None
    confidence: float
```

Every target state must carry:

- source identifier;
- valid timestamp;
- coordinate frame;
- position;
- velocity when available;
- confidence or accuracy estimate.

Target messages without a timestamp or frame identifier shall be rejected.

---

## 10. Vehicle Interface

### 10.1 Connection

Initial supported connection strings:

```text
udpin:0.0.0.0:14550
udpout:127.0.0.1:14551
serial:/dev/ttyUSB0:57600
```

Connection configuration:

```yaml
vehicle:
  connection: "udpin:0.0.0.0:14550"
  source_system: 191
  source_component: 191
  expected_autopilot: "ARDUPILOTMEGA"
  expected_vehicle_type: "QUADROTOR"
  heartbeat_hz_min: 0.5
```

### 10.2 MAVLink messages to consume

Minimum telemetry set:

- `HEARTBEAT`
- `SYS_STATUS`
- `BATTERY_STATUS`
- `GLOBAL_POSITION_INT`
- `LOCAL_POSITION_NED`
- `ATTITUDE`
- `VFR_HUD`
- `EKF_STATUS_REPORT`
- `GPS_RAW_INT`
- `HOME_POSITION`
- `EXTENDED_SYS_STATE`
- `COMMAND_ACK`
- `STATUSTEXT`

### 10.3 Commands

Initial command set:

- set mode;
- arm;
- disarm when safe;
- takeoff;
- land;
- RTL;
- guided position target;
- guided velocity target.

Every command shall:

1. be assigned an internal command ID;
2. record send timestamp;
3. await telemetry confirmation or `COMMAND_ACK`;
4. retry only according to explicit retry policy;
5. log success or failure;
6. fail closed after retry exhaustion.

### 10.4 ArduPilot mode policy

Recommended modes:

- `GUIDED` for companion-generated setpoints;
- `LOITER` for temporary hold;
- `RTL` for recovery;
- `LAND` for immediate landing when appropriate.

Do not use PX4-specific `OFFBOARD` assumptions in ArduPilot code.

---

## 11. Target Estimation

The first implementation shall use a constant-velocity estimator.

State vector:

\[
x =
\begin{bmatrix}
p_N & p_E & p_D & v_N & v_E & v_D
\end{bmatrix}^{T}
\]

The estimator shall provide:

- filtered position;
- filtered velocity;
- prediction at configurable look-ahead time;
- covariance or confidence;
- data age;
- source validity.

### 11.1 Initial implementation stages

#### Stage A: Pass-through estimator

Use validated target telemetry directly.

#### Stage B: Alpha-beta filter

Use a lightweight filter for SITL and early tests.

#### Stage C: Kalman filter

Use a six-state constant-velocity model.

#### Stage D: Multi-source fusion

Optional later development for cooperative GNSS plus vision bearing.

No advanced estimator should be added before deterministic test cases exist.

---

## 12. Guidance

### 12.1 Initial guidance laws

Implement in this order:

1. Position hold at a fixed stand-off waypoint.
2. Pure pursuit with closure-rate limiting.
3. Lead pursuit using target velocity prediction.
4. Trailing-point guidance.
5. Stand-off orbit.

### 12.2 Pure pursuit

The desired direction is based on the line from interceptor position to the predicted target position.

The generated command must be constrained by:

- maximum groundspeed;
- maximum acceleration;
- maximum closure rate;
- geofence;
- separation volume;
- altitude corridor.

### 12.3 Lead pursuit

Predicted target position:

\[
p_{target,predicted} =
p_{target} + v_{target} t_{lookahead}
\]

The look-ahead time shall be bounded:

```yaml
guidance:
  lookahead_time_s: 2.0
  lookahead_time_min_s: 0.5
  lookahead_time_max_s: 5.0
```

### 12.4 Stand-off control

Let:

\[
r = p_{target} - p_{interceptor}
\]

The controller shall reduce commanded approach speed as the range approaches the preferred stand-off radius.

Example behavior:

```text
range > 2 × stand-off:
    normal approach speed

stand-off < range ≤ 2 × stand-off:
    progressively reduce closure rate

minimum separation < range ≤ stand-off:
    hold or move outward

range ≤ minimum separation:
    abort immediately
```

### 12.5 Guidance output

Initial output type:

```python
@dataclass(frozen=True)
class GuidanceCommand:
    timestamp_monotonic_s: float
    velocity_n_mps: float
    velocity_e_mps: float
    velocity_d_mps: float
    yaw_deg: float | None
    valid_until_monotonic_s: float
    reason: str
```

Every command must expire. Expired setpoints must not be transmitted.

---

## 13. Command Limiters

All guidance commands shall pass through a limiter chain:

```text
raw guidance
    ↓
speed limiter
    ↓
acceleration limiter
    ↓
vertical-speed limiter
    ↓
yaw-rate limiter
    ↓
closure-rate limiter
    ↓
geofence limiter
    ↓
separation limiter
    ↓
validated command
```

A limiter shall be able to return:

- accepted;
- modified;
- rejected;
- abort required.

Limiter decisions must be logged.

---

## 14. Configuration

Use YAML configuration with schema validation.

Example:

```yaml
project:
  name: "vulture-x"
  environment: "sitl"

vehicle:
  connection: "udpin:0.0.0.0:14550"
  source_system: 191
  source_component: 191
  guided_mode: "GUIDED"
  recovery_mode: "RTL"

target:
  source: "cooperative_udp"
  listen_host: "0.0.0.0"
  listen_port: 15550
  expected_frame: "WGS84_MSL"
  minimum_confidence: 0.8

guidance:
  type: "lead_pursuit"
  update_rate_hz: 10
  preferred_standoff_m: 50.0
  lookahead_time_s: 2.0

safety:
  min_horizontal_separation_m: 30.0
  min_vertical_separation_m: 15.0
  max_closure_rate_mps: 5.0
  max_groundspeed_mps: 15.0
  max_acceleration_mps2: 2.0
  target_abort_age_s: 2.0
  return_battery_remaining_pct: 45

logging:
  level: "INFO"
  directory: "./logs"
  telemetry_csv: true
  event_jsonl: true
```

Configuration shall be rejected at startup when:

- minimum separation exceeds preferred stand-off;
- warning age exceeds abort age;
- return battery threshold is below abort battery threshold;
- speed or acceleration values are non-positive;
- connection string is invalid;
- coordinate frame is unsupported.

---

## 15. Logging

The system shall write:

### 15.1 Event log

JSON Lines format:

```json
{
  "timestamp_utc": "2026-07-28T12:00:00.000Z",
  "timestamp_monotonic_s": 12345.67,
  "level": "WARNING",
  "module": "safety.supervisor",
  "event": "TARGET_DATA_STALE",
  "details": {
    "age_s": 0.83,
    "warning_threshold_s": 0.5
  }
}
```

### 15.2 Telemetry log

CSV or Parquet fields:

- vehicle position;
- vehicle velocity;
- vehicle attitude;
- flight mode;
- armed state;
- battery;
- target position;
- target velocity;
- target confidence;
- relative range;
- horizontal separation;
- vertical separation;
- closure rate;
- raw guidance output;
- limited guidance output;
- active mission state;
- active safety flags.

### 15.3 Command log

Record:

- command type;
- parameters;
- send timestamp;
- target system/component;
- retry count;
- acknowledgement result;
- telemetry confirmation;
- final status.

---

## 16. Status API

A read-only local HTTP API may expose:

```text
GET /health
GET /status
GET /vehicle
GET /target
GET /mission
GET /safety
GET /metrics
```

Example status response:

```json
{
  "state": "STANDOFF",
  "vehicle_connected": true,
  "vehicle_mode": "GUIDED",
  "armed": true,
  "target_valid": true,
  "target_age_s": 0.12,
  "range_m": 51.8,
  "closure_rate_mps": 0.4,
  "safety_status": "NORMAL"
}
```

The API shall not expose command endpoints in the first version.

---

## 17. Test Strategy

### 17.1 Unit tests

Required unit-test areas:

- configuration validation;
- coordinate conversion;
- timestamp validation;
- target stale-data detection;
- state transition guards;
- speed limiting;
- acceleration limiting;
- closure-rate calculation;
- separation checks;
- guidance command expiration;
- command retry logic;
- target estimator update and prediction.

### 17.2 Integration tests

Test combinations:

- MAVLink client plus mocked FCU;
- target source plus estimator;
- estimator plus guidance;
- guidance plus safety limiter;
- mission supervisor plus mocked vehicle interface.

### 17.3 SITL tests

Minimum scenarios:

1. Nominal takeoff and stand-off.
2. Moving target with constant velocity.
3. Target turns 90 degrees.
4. Target stops.
5. Target data freezes.
6. Target telemetry becomes noisy.
7. Target jumps outside geofence.
8. FCU heartbeat stops.
9. GPS becomes unhealthy.
10. EKF reports failure.
11. Battery crosses return threshold.
12. Operator abort.
13. Command acknowledgement timeout.
14. Companion restart.
15. Interceptor approaches minimum separation.

### 17.4 HIL tests

HIL shall use:

- real autopilot;
- motors disconnected or propulsion made safe;
- simulated target;
- real companion computer;
- real MAVLink transport.

Inject:

- serial disconnection;
- UDP packet loss;
- delayed target packets;
- duplicate packets;
- out-of-order packets;
- invalid frames;
- target timestamp rollback;
- companion CPU overload;
- process crash and restart.

### 17.5 Outdoor test gates

Outdoor test is permitted only after:

- all unit tests pass;
- all mandatory SITL tests pass;
- HIL abort tests pass;
- safety pilot procedure is approved;
- geofence is loaded and verified;
- stand-off distance is physically marked in test plan;
- target is cooperative;
- test area is segregated;
- propulsion and battery margins are documented.

---

## 18. Acceptance Criteria

Version 0.1 is accepted when:

- the application connects to ArduPilot SITL;
- vehicle health is reported correctly;
- a simulated cooperative target is received;
- stale target data is detected;
- the interceptor reaches a 50 m stand-off radius without violating 30 m minimum separation;
- closure rate never exceeds configured limit;
- geofence violation prediction triggers abort;
- heartbeat loss triggers RTL or configured recovery;
- all state transitions are logged;
- all mandatory tests pass in CI.

Version 0.2 is accepted when:

- target estimator supports prediction;
- lead pursuit works in SITL;
- replay tests reproduce guidance decisions;
- HIL fault injection passes;
- companion restart returns to a safe state.

Version 1.0 is accepted only after controlled flight validation.

---

## 19. Development Milestones

### Milestone 0: Repository foundation

Deliverables:

- package structure;
- configuration model;
- logging;
- type definitions;
- CI;
- linting;
- unit-test framework.

### Milestone 1: Vehicle interface

Deliverables:

- MAVLink connection;
- heartbeat monitoring;
- telemetry cache;
- mode management;
- arm and takeoff command verification;
- ACK handling.

### Milestone 2: Target ingestion

Deliverables:

- cooperative UDP target protocol;
- target validation;
- data-age monitoring;
- replay source.

### Milestone 3: Safety supervisor

Deliverables:

- geofence checks;
- separation checks;
- speed and acceleration limits;
- abort policy;
- battery and health guards.

### Milestone 4: Mission supervisor

Deliverables:

- full state machine;
- transition guards;
- operator enable and abort;
- deterministic recovery behavior.

### Milestone 5: Guidance

Deliverables:

- fixed stand-off waypoint;
- pure pursuit;
- lead pursuit;
- trailing mode;
- orbit mode.

### Milestone 6: SITL automation

Deliverables:

- target simulator;
- repeatable scenarios;
- automatic pass/fail metrics;
- CI SITL test job.

### Milestone 7: HIL

Deliverables:

- real Pixhawk connection;
- fault injection;
- recovery verification;
- test report.

### Milestone 8: Controlled flight test

Deliverables:

- test cards;
- risk assessment;
- preflight checklist;
- flight logs;
- engineering findings.

---

## 20. Coding Standards

### 20.1 Python

- Python 3.11 or newer.
- Type hints required.
- `mypy` strict mode preferred.
- `ruff` for linting.
- `black` for formatting.
- `pytest` for tests.
- `pydantic` or equivalent for configuration validation.
- No global mutable state.
- No blocking MAVLink reads on the main control loop.
- Use monotonic time for data age and command expiration.
- Use structured logging.

### 20.2 Error handling

Forbidden:

```python
except Exception:
    pass
```

Required:

- catch specific exceptions where possible;
- log the error;
- classify severity;
- transition to a safe state when safety-relevant;
- preserve traceback for debugging.

### 20.3 Concurrency

Preferred initial design:

- one asyncio event loop;
- separate tasks for MAVLink receive, target receive, mission loop, and logging;
- bounded queues;
- explicit timeouts;
- no unbounded thread creation.

### 20.4 Control-loop rates

Recommended initial rates:

```yaml
rates:
  vehicle_receive_hz: 20
  target_receive_hz: 10
  estimator_hz: 20
  guidance_hz: 10
  safety_hz: 20
  status_hz: 2
```

Control-loop overruns shall be logged.

---

## 21. Initial Data Models

```python
from dataclasses import dataclass
from enum import Enum, auto


class MissionState(Enum):
    BOOT = auto()
    SELF_TEST = auto()
    WAIT_FCU = auto()
    WAIT_HOME = auto()
    READY = auto()
    ARMED_STANDBY = auto()
    TAKEOFF = auto()
    OBSERVE = auto()
    TRACK = auto()
    APPROACH = auto()
    STANDOFF = auto()
    RETURN = auto()
    LAND = auto()
    ABORT = auto()
    COMPLETE = auto()


@dataclass(frozen=True)
class VehicleState:
    timestamp_monotonic_s: float
    connected: bool
    armed: bool
    mode: str
    latitude_deg: float | None
    longitude_deg: float | None
    altitude_msl_m: float | None
    velocity_n_mps: float | None
    velocity_e_mps: float | None
    velocity_d_mps: float | None
    battery_remaining_pct: float | None
    ekf_healthy: bool
    gps_healthy: bool
    home_valid: bool


@dataclass(frozen=True)
class RelativeState:
    timestamp_monotonic_s: float
    north_m: float
    east_m: float
    down_m: float
    velocity_n_mps: float
    velocity_e_mps: float
    velocity_d_mps: float
    horizontal_range_m: float
    vertical_separation_m: float
    range_3d_m: float
    closure_rate_mps: float
    confidence: float
```

---

## 22. Cooperative Target Protocol

Initial transport: UDP JSON.

Example:

```json
{
  "protocol": "vulture-target-v1",
  "source_id": "target-sitl-01",
  "sequence": 1042,
  "timestamp_utc": "2026-07-28T12:00:00.000Z",
  "frame": "WGS84_MSL",
  "position": {
    "latitude_deg": -6.914744,
    "longitude_deg": 107.609810,
    "altitude_msl_m": 760.0
  },
  "velocity": {
    "north_mps": 8.0,
    "east_mps": 1.0,
    "down_mps": 0.0
  },
  "accuracy": {
    "position_m": 2.5,
    "velocity_mps": 0.5
  },
  "confidence": 0.95
}
```

Validation requirements:

- protocol string must match;
- source must be allow-listed;
- sequence must not roll backward unexpectedly;
- timestamp must be valid;
- coordinates must be in valid ranges;
- velocity must be bounded;
- confidence must be in `[0, 1]`;
- frame must be supported;
- packet size must be bounded.

Optional later enhancement:

- message authentication;
- signed messages;
- replay protection;
- source-specific keys.

---

## 23. Safety Case Outline

The repository shall include `docs/safety_case.md` covering:

1. System description.
2. Intended operating environment.
3. Hazard identification.
4. Hazard severity and likelihood.
5. Risk controls.
6. Verification evidence.
7. Residual risk.
8. Operational limitations.
9. Emergency procedures.
10. Approval status.

Minimum hazards:

- loss of target data;
- wrong coordinate frame;
- stale target position;
- estimator divergence;
- command saturation;
- geofence breach;
- loss of MAVLink;
- companion reboot;
- RC takeover failure;
- battery exhaustion;
- unexpected target maneuver;
- navigation sensor degradation;
- software deadlock;
- excessive closure rate;
- loss of visual separation.

---

## 24. Flight-Test Philosophy

Flight-test progression:

```text
desktop unit test
    ↓
simulation
    ↓
SITL multi-vehicle
    ↓
HIL with motors safe
    ↓
tethered or restrained test where applicable
    ↓
single-UAV observe mode
    ↓
dual-UAV cooperative follow
    ↓
stand-off approach
    ↓
moving-target stand-off
```

No test phase may be skipped.

Each test card shall include:

- objective;
- configuration;
- software commit;
- autopilot firmware;
- test area;
- weather limits;
- battery limits;
- abort criteria;
- safety pilot action;
- expected result;
- recorded metrics.

---

## 25. Codex Development Instructions

Use the following rules when generating code for this repository.

### 25.1 General instruction

Implement one milestone at a time. Do not generate placeholder functions that silently succeed. Every incomplete function must raise `NotImplementedError` or be clearly marked as pending.

### 25.2 Required behavior

For every feature:

1. Add or update data models.
2. Add configuration schema.
3. Add implementation.
4. Add unit tests.
5. Add integration tests where applicable.
6. Update documentation.
7. Run linting and tests.
8. Report assumptions and unresolved risks.

### 25.3 Prohibited behavior

Do not:

- bypass safety checks;
- suppress exceptions without logging;
- hard-code IP addresses in application code;
- use wall-clock time for data-age calculations;
- send unbounded velocity commands;
- arm automatically at startup;
- ignore `COMMAND_ACK`;
- continue guidance with stale target data;
- use unsupported coordinate frames;
- issue commands before FCU identity is validated;
- mix PX4 OFFBOARD behavior into ArduPilot GUIDED logic;
- create RF interference features;
- add collision or contact behavior.

### 25.4 Pull-request checklist

Every pull request shall answer:

- What safety behavior changed?
- What new failure modes were introduced?
- What tests prove the behavior?
- What configuration keys were added?
- What telemetry or log fields were added?
- How does the system fail when the new module stops responding?
- Does the change affect flight-test approval?

---

## 26. Initial Codex Task Sequence

Use these tasks in order.

### Task 1

Create the Python package, `pyproject.toml`, configuration loader, data models, enums, logging, and unit-test skeleton.

Acceptance:

- package imports successfully;
- invalid configuration is rejected;
- `pytest`, `ruff`, and `mypy` pass.

### Task 2

Implement an asynchronous MAVLink client with:

- heartbeat detection;
- telemetry cache;
- connection timeout;
- FCU identity validation;
- command-ack tracking;
- clean shutdown.

Acceptance:

- mocked heartbeat connects;
- missing heartbeat times out;
- mismatched autopilot identity is rejected;
- ACK success and failure are tested.

### Task 3

Implement cooperative UDP target ingestion.

Acceptance:

- valid packets are accepted;
- stale, malformed, oversized, unsupported-frame, and low-confidence packets are rejected;
- packet sequence handling is tested.

### Task 4

Implement target age and vehicle-health safety checks.

Acceptance:

- warning and abort thresholds work;
- safety results include machine-readable reason codes;
- tests cover boundary conditions.

### Task 5

Implement mission state machine through `OBSERVE`.

Acceptance:

- all transitions use explicit guards;
- invalid transitions are rejected;
- abort can be entered from all active states.

### Task 6

Implement coordinate conversion and relative-state calculation.

Acceptance:

- known coordinate test vectors pass;
- horizontal, vertical, 3D range, and closure rate are tested.

### Task 7

Implement fixed stand-off guidance with command limiting.

Acceptance:

- command goes toward stand-off point;
- speed and acceleration limits are enforced;
- guidance does not enter minimum separation;
- command expiration works.

### Task 8

Integrate with ArduPilot SITL.

Acceptance:

- connect;
- arm with explicit operator enable;
- take off;
- reach observe state;
- follow a simulated target;
- stop at stand-off;
- abort on stale target;
- command RTL.

---

## 27. Definition of Done

A feature is done only when:

- code is typed;
- tests pass;
- failure behavior is defined;
- logs are sufficient for post-flight analysis;
- configuration is documented;
- no safety-critical TODO remains hidden;
- simulation evidence exists;
- review confirms ArduPilot compatibility.

---

## 28. Initial README Summary

Suggested repository description:

> Vulture-X is a safety-oriented research platform for cooperative, non-contact UAV interception using ArduPilot, MAVLink, constrained guidance, and deterministic simulation. It is designed for SITL, HIL, and controlled flight testing with explicit geofence, stand-off, command-limiting, and abort behavior.

---

## 29. License and Compliance

Recommended:

- choose a clear open-source license;
- document third-party dependencies;
- maintain a software bill of materials;
- comply with local aviation, radio, privacy, and test-range requirements;
- restrict real-world operation to authorized cooperative testing.

---

## 30. First Release Target

The first release shall demonstrate:

```text
ArduPilot SITL interceptor
        +
cooperative target simulator
        +
validated target state
        +
relative-state estimator
        +
constrained stand-off guidance
        +
safety supervisor
        +
automatic RTL on fault
        +
replayable logs
```

This is the minimum coherent baseline. Advanced tracking, ROS 2 integration, vision, multi-vehicle coordination, and outdoor operation shall be added only after this baseline is stable.

