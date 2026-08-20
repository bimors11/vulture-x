# Codex Handoff

This document records the current repository state as observed on this machine
before the next Codex session. It is intentionally descriptive only. Do not use
it as permission to start the next feature.

## 1. Current Repository State

- Current branch: `main`
- Upstream status: `## main...origin/main`
- Latest local commit: `c4c92fe Add NanoTrack YuNet vision runtime support`
- Push status: the latest local commit has not been pushed from this machine;
  HTTPS push failed for lack of credentials and SSH push failed with
  `Permission denied (publickey)`.

Recorded before creating this handoff file:

```text
$ git status --short
```

No output.

```text
$ git diff --stat
```

No output.

Working tree after this handoff is created:

- Modified files: none expected.
- New files: `docs/CODEX_HANDOFF.md`
- Deleted files: none expected.
- Uncommitted changes: this handoff file only.

Do not commit this handoff unless the user explicitly asks.

## 2. What Has Actually Been Implemented

| Item | Status | Current implementation |
| --- | --- | --- |
| performance instrumentation | PARTIAL | `tools/sitl_track_target.py` has `RuntimeMetrics` for rolling timing/rate/count samples: capture, vision, guidance, MAVLink TX, control period, jitter, deadline misses, dropped frames, tracker failures. It is surfaced in demand JSON and WebUI runtime panel when the tracking process is writing demand state. It is not a full worker-wide tracing system. |
| TrackerNano | IMPLEMENTED | `src/vulture_x/vision/tracker.py` and `tools/sitl_track_target.py` define `NanoTracker`. SITL helper supports `--tracker-engine nano` and NanoTrack model path arguments. WebUI defaults manual/head tracking toward Nano when appropriate. |
| TemplateMatching fallback | IMPLEMENTED | `TemplateMatchingTracker` remains available. SITL helper supports `--tracker-engine template`. Red/banner detector-backed tracking continues to use the template path. |
| YuNet | PARTIAL | `tools/vulture_x_ui.py` uses OpenCV `FaceDetectorYN_create` in `detect_heads_yunet()` for head candidate acquisition when the model/API is available. It falls back to Haar head detection. The tracking process validates `--yunet-model-path` for `head`/`person`, but does not run YuNet reacquisition internally. |
| head acquisition | PARTIAL | WebUI can acquire head candidates using YuNet fallback chain and stores operator-selected normalized bbox. Tracking still depends on `--selection-file`; there is no autonomous target choice. |
| head reacquisition | NOT IMPLEMENTED | No YuNet/Nano association loop reacquires a lost head after tracking degrades or fails. |
| TrackingBackend abstraction | PARTIAL | There is a `BboxTracker` protocol for OpenCV-style trackers and `TrackingObservation` data shapes, but no complete shared backend abstraction across runtime/UI/package. |
| TrackingObservation | PARTIAL | Defined in `src/vulture_x/vision/tracker.py` and in `tools/sitl_track_target.py`. It is not the main runtime contract for all control decisions. |
| TrackingResult | PARTIAL | Package-level `vulture_x.models.TrackingResult` and `tracking_result_from_bbox()` still exist. The SITL helper mostly uses bbox/demand dictionaries instead of a new unified runtime result object. |
| confidence propagation | PARTIAL | Nano confidence is read from OpenCV `getTrackingScore()` when available and written as `tracker_confidence`/`last_confidence`. Template/classic trackers either return `None` or older local scoring. Confidence is not yet a complete safety input. |
| bbox jump validation | PARTIAL | Target-relative bbox transition gates exist in package and SITL helper before accepting large bbox changes. It is not a complete motion model. |
| bbox smoothing | IMPLEMENTED | Existing bbox smoothing remains in `TemplateMatchingTracker` and SITL helper stabilization paths. |
| vision states: ACQUIRE, TRACK, DEGRADED, LOST | PARTIAL | WebUI HUD maps demand/target state into labels. Demand JSON writes `vision_state: "TRACK"` in the detected plane path. There is no full internal ACQUIRE/TRACK/DEGRADED/LOST time-state machine. |
| time-based tracking validity | PARTIAL | Existing monotonic safety timeouts exist for frame age, heartbeat age, and fixed-wing target loss. Tracker internals still rely partly on miss counters. |
| Capture Worker | NOT IMPLEMENTED | No dedicated capture worker/buffer architecture. |
| Vision Worker | NOT IMPLEMENTED | No dedicated vision worker. |
| Control Worker | NOT IMPLEMENTED | Control remains in the main tracking loop. |
| MAVLink RX Worker | NOT IMPLEMENTED | Tracking loop still calls `recv_match`; WebUI has a separate status monitor thread only. |
| LatestFrameBuffer | NOT IMPLEMENTED | No shared latest-frame buffer class exists. |
| LatestTrackingResult | NOT IMPLEMENTED | No shared latest-tracking-result buffer class exists. |
| LatestVehicleState | NOT IMPLEMENTED | No shared latest-vehicle-state buffer class exists. |
| fixed 30 Hz control scheduler | PARTIAL | The SITL helper defaults to a rate-based loop and records period/jitter/deadline metrics. It is not a dedicated fixed-deadline scheduler. |
| worker watchdogs | NOT IMPLEMENTED | No worker supervision architecture exists. Heartbeat/video/target-loss safety timeouts exist in the control loop. |
| Operational HUD | PARTIAL | `tools/vulture_x_ui.py` has an operational HUD renderer with top bar, bottom strip, target brackets, reticle, command cue, mode labels, and HUD toggle. It is presentation-layer only. |
| Diagnostic HUD | PARTIAL | Diagnostic HUD overlays tracker, PWM, guidance, and performance fields. Raw bbox/motion-gate style diagnostic toggles are not complete. |
| runtime performance panel | PARTIAL | WebUI shows runtime performance from demand JSON when the tracking process provides metrics. It is not an always-on runtime profiler. |
| GStreamer/appsink changes if any | NOT IMPLEMENTED | RTSP/GStreamer flow remains file/JPEG bridge based; there is no direct appsink capture backend. |

## 3. Current Fixed-Wing Operational Flow

Current behavior is implemented mainly in `tools/sitl_track_target.py` and
`tools/vulture_x_ui.py`.

- Aircraft can start in any flight mode: IMPLEMENTED for fixed-wing. `verify_connection()` does not require FBWA before start for `--vehicle plane`.
- Operator selects target: IMPLEMENTED for `custom`/`head` through WebUI selection file before starting tracking.
- START TRACKING validates prerequisites: IMPLEMENTED. It validates connection, camera frames, model paths, target selection, parameter acceptance, heartbeat/armed state, altitude, and airspeed where available.
- START TRACKING requests FBWA: IMPLEMENTED. `request_plane_fbwa()` requests FBWA and waits up to `PLANE_FBWA_TRANSITION_TIMEOUT_S = 3.0`.
- START TRACKING waits for FBWA confirmation before RC override: IMPLEMENTED. RC override is not enabled until target and FBWA checks pass.
- TRACKING requires FBWA to remain active: IMPLEMENTED. If heartbeat mode changes away from FBWA after tracking starts, the process fails safe.
- Pilot/external mode change away from FBWA: IMPLEMENTED. The code immediately releases RC override, aborts tracking, leaves the new mode unchanged, does not reassert FBWA, and does not automatically request AUTO.
- NORMAL STOP stops commands and releases RC override: IMPLEMENTED through process termination/finally release path.
- NORMAL STOP requests AUTO: PARTIAL. WebUI `stop_steering()` calls `set_plane_auto()`, which commands AUTO for simulator-like endpoints. It does not wait for AUTO confirmation in the normal stop path.
- Restore previous flight mode: NOT IMPLEMENTED and intentionally not desired.

Current difference from intended flow: normal STOP requests AUTO but does not
wait for AUTO confirmation.

## 4. Safety Behavior

| Case | Releases RC override | Requests AUTO | Mode change requested? | Aborts tracking | Actual behavior |
| --- | --- | --- | --- | --- | --- |
| target lost | Yes | No | No | Yes | After `plane_loss_hold_s`, writes failsafe demand and exits tracking. |
| vision degraded | No separate behavior | No | No | Not directly | No real DEGRADED safety state. Miss holding and target-loss timeout handle loss. |
| video stale | Yes | No | No | Yes | If frame age exceeds threshold, writes `stale_video` failsafe and exits. |
| MAVLink heartbeat timeout | Yes | No | No | Yes | If active tracking and heartbeat age exceeds timeout, releases override and exits. |
| low altitude | Yes | No | No | Yes | Unknown altitude blocks start unless `--surface-test`; below minimum during tracking fails safe. |
| low airspeed | Yes | No | No | Yes | If configured min airspeed is known and low airspeed persists, fails safe. Unknown airspeed blocks start when a min airspeed is known. |
| disarm | Yes | No | No | Yes | Heartbeat disarmed during active plane tracking fails safe unless `--surface-test`. |
| pilot mode takeover | Yes | No | No | Yes | Any active FBWA tracking mode change away from FBWA releases override and exits. |
| internal exception | Usually | No | No | Usually | Main process `finally` releases override if active. Some setup failures return before active override. |
| worker failure | Not applicable | Not applicable | Not applicable | Not applicable | Dedicated workers do not exist yet. |

## 5. Current Aircraft Parameter Reading

All ArduPlane parameter reads currently come from
`PLANE_RESPONSE_PARAM_NAMES` in `tools/sitl_track_target.py`. They are requested
by `read_plane_parameters()`, stored in a local `dict[str, float]`, optionally
cached with `save_plane_param_cache()`, and copied into
`PlaneResponseModel.raw_params` by `plane_response_model_from_params()`.

| Parameter | Read? | Used? | Purpose | File/function |
| --- | --- | --- | --- | --- |
| `ROLL_LIMIT_DEG` | Yes | Yes | Caps effective max roll in degrees. | `tools/sitl_track_target.py::plane_response_model_from_params` |
| `PTCH_LIM_MAX_DEG` | Yes | Yes | Caps effective pitch-up limit. | `plane_response_model_from_params` |
| `PTCH_LIM_MIN_DEG` | Yes | Yes | Caps effective pitch-down limit by absolute value. | `plane_response_model_from_params` |
| `LIM_ROLL_CD` | Yes | Yes | Legacy fallback roll limit, centidegrees. | `plane_response_model_from_params` |
| `LIM_PITCH_MAX` | Yes | Yes | Legacy fallback pitch-up limit, centidegrees. | `plane_response_model_from_params` |
| `LIM_PITCH_MIN` | Yes | Yes | Legacy fallback pitch-down limit, centidegrees. | `plane_response_model_from_params` |
| `ARSPD_FBW_MIN` | Yes | Yes | Clamps target airspeed lower bound and provides low-airspeed safety threshold. | `plane_response_model_from_params`, main tracking loop |
| `ARSPD_FBW_MAX` | Yes | Yes | Clamps target airspeed upper bound when positive. | `plane_response_model_from_params` |
| `TRIM_THROTTLE` | Yes | Yes | Sets cruise throttle unless UI fixed-throttle mode is active. | `plane_response_model_from_params` |
| `THR_MIN` | Yes | Yes | Sets min throttle unless fixed-throttle mode is active. | `plane_response_model_from_params` |
| `THR_MAX` | Yes | Yes | Sets max throttle unless fixed-throttle mode is active. | `plane_response_model_from_params` |
| `RCMAP_ROLL` | Yes | Yes | Maps roll override channel. | `plane_response_model_from_params` |
| `RCMAP_PITCH` | Yes | Yes | Maps pitch override channel. | `plane_response_model_from_params` |
| `RCMAP_THROTTLE` | Yes | Yes | Maps throttle override channel. | `plane_response_model_from_params` |
| `RCMAP_YAW` | Yes | Yes | Maps yaw channel for calibration model; yaw override is not actively commanded. | `plane_response_model_from_params` |
| `RC1_MIN`..`RC8_MIN` | Yes | Yes | RC calibration lower PWM per channel. | `rc_calibration_from_params`, `attitude_to_plane_rc_pwm` |
| `RC1_TRIM`..`RC8_TRIM` | Yes | Yes | RC calibration trim PWM per channel. | `rc_calibration_from_params`, `attitude_to_plane_rc_pwm` |
| `RC1_MAX`..`RC8_MAX` | Yes | Yes | RC calibration upper PWM per channel. | `rc_calibration_from_params`, `attitude_to_plane_rc_pwm` |
| `RC1_REVERSED`..`RC4_REVERSED` | Yes | Yes | RC reversal for mapped channels 1-4. | `plane_response_model_from_params`, `rc_calibration_from_params` |
| `RC5_REVERSED`..`RC8_REVERSED` | No | Partially if present | Code can consume these if manually present in values, but live parameter request list does not request them. | `plane_response_model_from_params` |
| `RC_OPTIONS` | Yes | Yes | Blocks start when MAVLink RC override is disabled by option bit. | `validate_rc_override_acceptance` |
| `RC_OVERRIDE_TIME` | Yes | Yes | Blocks if `<= 0`; warns if missing or long; stored in response model. | `validate_rc_override_acceptance`, `plane_response_model_from_params` |
| `MAV_GCS_SYSID` | Yes | Yes | Validates source-system when MAVLink sysid enforcement is active. | `validate_rc_override_acceptance` |
| `MAV_GCS_SYSID_HI` | Yes | Yes | Validates allowed source-system range. | `validate_rc_override_acceptance` |
| `MAV_OPTIONS` | Yes | Yes | Detects whether GCS sysid enforcement applies. | `validate_rc_override_acceptance` |
| `MIS_RESTART` | Yes | Warning only | Warns if mission restart behavior is nonzero. Does not change tracking control. | main parameter warning path |
| `SERVO1_FUNCTION`..`SERVO8_FUNCTION` | Yes | Display only | Printed/formatted for operator context. Does not change runtime behavior. | `format_servo_functions` |
| `RLL2SRV_TCONST` | Yes | No | Read but not currently applied to roll response. | `PLANE_RESPONSE_PARAM_NAMES` only |
| `PTCH2SRV_TCONST` | Yes | Yes | Reduces pitch filter alpha and pitch step limit. | `plane_response_model_from_params` |
| `RLL2SRV_P`, `RLL2SRV_I`, `RLL2SRV_D` | Yes | No | Read but effectively unused. | `PLANE_RESPONSE_PARAM_NAMES` only |
| `PTCH2SRV_P`, `PTCH2SRV_I`, `PTCH2SRV_D` | Yes | No | Read but effectively unused. | `PLANE_RESPONSE_PARAM_NAMES` only |
| `NAVL1_PERIOD` | Yes | No | Read but effectively unused. | `PLANE_RESPONSE_PARAM_NAMES` only |
| `NAVL1_DAMPING` | Yes | No | Read but effectively unused. | `PLANE_RESPONSE_PARAM_NAMES` only |

## 6. Current Aircraft Adaptation Logic

- `PlaneResponseModel`: stores effective roll/pitch/throttle/airspeed limits,
  mapped RC channels, RC calibration, min airspeed, override timeout, and raw
  params.
- `plane_response_model_from_params()`: computes effective roll limit, pitch-up
  and pitch-down limits, throttle limits, target airspeed clamp, pitch filter
  alpha, pitch step, RC channel mapping, RC reversal, and RC calibration.
- `channel_from_param()`: validates `RCMAP_*` values into channels 1-8 with
  fallback defaults.
- `rc_calibration_from_params()`: accepts sane `RCx_MIN/TRIM/MAX` values in
  800..2200 order, otherwise falls back to 1000/1500/2000 and preserves reversal.
- `attitude_to_plane_rc_pwm()`: converts roll/pitch/throttle commands into PWM
  on mapped/calibrated channels.
- `rc_override()`: sends `RC_CHANNELS_OVERRIDE` and clamps absolute PWM to
  800..2200.
- `validate_rc_override_acceptance()`: blocks if `RC_OPTIONS` disables override,
  `RC_OVERRIDE_TIME <= 0`, or MAVLink source-system is outside enforced
  `MAV_GCS_SYSID`/`MAV_GCS_SYSID_HI`.
- `format_servo_functions()`: displays `SERVOx_FUNCTION`; it does not alter
  control behavior.

Read but effectively unused: `RLL2SRV_TCONST`, `RLL2SRV_P/I/D`,
`PTCH2SRV_P/I/D`, `NAVL1_PERIOD`, `NAVL1_DAMPING`, and `SERVOx_FUNCTION`
except for display.

## 7. Current Unknown / Invalid Telemetry Handling

- Relative altitude: `float | None`; initialized as `None`. No fake
  `relative_altitude = 50.0` default remains in current source.
- Airspeed: `float | None`; initialized as `None`.
- Flight mode: `str | None` from heartbeat. Unknown mode is not treated as FBWA.
- Armed state: taken from heartbeat. `--surface-test` can permit unarmed SITL
  checks; normal tracking requires armed state before start.
- Heartbeat: active tracking uses monotonic heartbeat age and fails safe on
  timeout.
- WebUI MAVLink monitor defaults to disconnected/offline, not connected.

## 8. Current Tracking Tuning

These are the current fixed-wing source-of-truth defaults. Preserve them unless
there is an explicit, tested reason to change them.

| Setting | Current default |
| --- | ---: |
| `plane_airspeed_mps` | 20.0 |
| `vertical_gain` | 52.0 |
| `plane_centering_gain` | 1.55 |
| `plane_near_centering_gain` | 2.65 |
| `plane_roll_gain_scale` | 1.75 |
| `plane_pitch_gain_scale` | 1.20 |
| `plane_pitch_near_gain_scale` | 1.60 |
| `plane_error_deadband` | 0.015 |
| `plane_lead_s` | 0.0 |
| `plane_damping_gain` | 0.14 |
| `plane_near_damping_gain` | 0.30 |
| `plane_pitch_filter_alpha` | 0.45 |
| `plane_max_pitch_step_deg` | 2.0 |
| `plane_max_roll_step_deg` | 6.0 |
| `max_plane_roll_deg` | 35.0 |
| `max_plane_pitch_deg` | 40.0 |
| `plane_near_pitch_down_limit_deg` | 40.0 |
| `plane_far_control_scale` | 0.72 |
| `plane_near_control_scale` | 1.0 |
| `plane_camera_hfov_deg` | 70.0 |
| `plane_proximity_far_size` | 0.025 |
| `plane_proximity_near_size` | 0.16 |
| `plane_throttle` | 0.55 |
| `plane_throttle_airspeed_gain` | 0.04 |
| `plane_min_throttle` | 0.25 |
| `plane_max_throttle` | 0.80 |
| `plane_near_throttle_reduction` | 0.0 |
| `plane_pitch_below_center_boost` | 0.25 |
| `plane_loss_hold_s` | 1.5 |
| `min_tracking_alt_m` | 15.0 |
| `airspeed_low_persistence_s` | 2.0 |

WebUI simulator mode currently forces fixed-wing throttle/min/max throttle to
`0.80` and preserves at least the tuned simulator gain floors in
`AppState.start_steering()`.

## 9. Current Runtime Architecture

Current implementation:

```text
Camera / RTSP / Gazebo UDP
        |
        v
GStreamer-or-tool bridge writes JPEG frames
        |
        v
FrameDirectoryReader / cv2 capture path
        |
        v
DetectorBackedTracker or CustomSelectionTracker
        |
        v
bbox validation + smoothing + demand JSON
        |
        v
single control loop in tools/sitl_track_target.py
        |
        v
MAVLink RC_CHANNELS_OVERRIDE for ArduPlane FBWA
or MAVLink body velocity for quad GUIDED
        |
        v
ArduPilot SITL / connected vehicle

Browser WebUI
        |
        v
/api/frame.jpg reads latest JPEG + demand JSON
        |
        v
Operational HUD / Diagnostic HUD
```

Planned but not implemented:

```text
Camera -> Capture Worker -> LatestFrameBuffer
LatestFrameBuffer -> Vision Worker -> LatestTrackingResult
MAVLink RX Worker -> LatestVehicleState
LatestTrackingResult + LatestVehicleState -> fixed 30 Hz Control Worker
Control Worker -> MAVLink -> ArduPlane
```

## 10. Important Files and Responsibilities

- `tools/sitl_track_target.py`: local SITL/manual visual steering helper. Owns
  CLI parsing, model path validation, frame reads, trackers, fixed-wing guidance
  loop, ArduPlane parameter reads/adaptation, failsafes, demand JSON, and
  `RC_CHANNELS_OVERRIDE`.
- `tools/vulture_x_ui.py`: WebUI server, camera bridge process control, MAVLink
  status monitor, live tuning JSON, target selection, YuNet head candidates,
  HUD rendering, start/stop steering, and normal stop AUTO command.
- `tools/mavlink_endpoint.py`: MAVLink endpoint parsing/opening, including
  `udpcl:` normalization to `udpout:`.
- `tools/ensure_vision_models.py`: verifies/downloads OpenCV NanoTrack and
  YuNet ONNX models by expected path/hash.
- `tools/sitl_arm_takeoff.py`: guarded SITL-only arm/takeoff helper. Includes
  explicit environment gate and separate quad/plane takeoff paths.
- `tools/track_camera_target.py`: non-commanding camera target/head detection
  utilities, including Haar fallback helpers.
- `src/vulture_x/vision/tracker.py`: package-level tracking helpers,
  `TemplateMatchingTracker`, `NanoTracker`, `OpenCvTracker`, bbox smoothing and
  transition plausibility.
- `src/vulture_x/vision/target_state.py`: converts bbox observations to package
  `TrackingResult`.
- `src/vulture_x/models.py`: immutable package models for tracking, guidance,
  vehicle state, commands, and safety.
- `src/vulture_x/guidance/*`: package guidance and bounded command limiting for
  the milestone path; not the local fixed-wing FBWA helper.
- `src/vulture_x/safety/*`: package safety checks/supervisor/abort policy.
- `src/vulture_x/vehicle/*`: package vehicle abstractions and mock transport;
  real async pymavlink client is not implemented yet.
- `src/vulture_x/mission/*`: package mission state machine through `TRACK`.
- `configs/*.yaml`: package runtime configuration used by `vulture-x`.
- `models/opencv/*`: local ONNX model assets for NanoTrack/YuNet.

## 11. Public Interfaces That Must Not Be Broken

- WebUI HTTP endpoints used by browser/tests:
  `/api/status`, `/api/frame.jpg`, `/api/start_steering`,
  `/api/stop_steering`, `/api/tracking_tuning`, `/api/selection`,
  `/api/selection/head`, `/api/start_bridge`, `/api/connect_mavlink`.
- WebUI LAN behavior: do not make the server localhost-only.
- Runtime demand JSON keys used by WebUI/HUD/tests: `detected`, `mode`,
  `vehicle`, `bbox`, `center_x`, `center_y`, `guided_error_x`,
  `guided_error_y`, `roll_deg`, `pitch_deg`, `roll_pwm`, `pitch_pwm`,
  `throttle_pwm`, `max_roll_deg`, `max_pitch_deg`, `rate_hz`,
  `frame_age_ms`, `tracking_result_age_ms`, `command_age_ms`,
  `tracker_confidence`, `tracker_engine`, `vision_state`,
  `control_authority`, `rc_override_active`, `target_proximity`, `throttle`,
  `airspeed_mps`, `effective_max_roll_deg`, `effective_max_pitch_deg`,
  `tracking_tuning_revision`, `runtime_metrics`, `failsafe`,
  `failsafe_reason`.
- Live tuning JSON shape in `logs/ui/tracking_tuning.json`:
  `{"ok": true, "revision": int, "values": {...}}`.
- Selection JSON shape in `logs/ui/custom_selection.json`: normalized
  `x`, `y`, `width`, `height`, plus selection mode metadata.
- SITL helper CLI arguments, especially: `--enable-guidance`, `--mavlink`,
  `--vehicle`, `--camera-dir`, `--pipeline`, `--tracking-mode`,
  `--selection-file`, `--tracker`, `--tracker-engine`,
  `--tracker-nano-backbone`, `--tracker-nano-neckhead`,
  `--yunet-model-path`, `--demand-state-file`, `--tuning-file`,
  `--max-frame-age-ms`, `--mavlink-heartbeat-timeout-s`, `--surface-test`,
  `--source-system`, `--source-component`, `--read-plane-params`,
  `--plane-param-cache-file`, and all `--plane-*` tuning flags.
- `tools.mavlink_endpoint.parse_mavlink_endpoint()` behavior, including
  `udpcl:` to `udpout:` conversion.
- Package tracker signatures and names: `OpenCvTracker`, `TemplateMatchingTracker`,
  `NanoTracker`, `TrackingObservation`, `tracking_result_from_bbox()`.
- Test-covered helper signatures: `request_plane_fbwa()`, `verify_connection()`,
  `validate_rc_override_acceptance()`, `plane_response_model_from_params()`,
  `attitude_to_plane_rc_pwm()`, `rc_override()`.
- Quad path remains ArduPilot `GUIDED` body velocity; fixed-wing tracking remains
  ArduPlane `FBWA` with bounded `RC_CHANNELS_OVERRIDE`.

## 12. Tests

Checks run on this machine after the current implementation:

```text
$ pytest
157 passed in 0.96s
```

```text
$ python -m compileall tools src tests
completed successfully
```

```text
$ ruff check .
All checks passed!
```

```text
$ mypy
Success: no issues found in 33 source files
```

```text
$ vulture-x --config configs/default.yaml --check-config
Configuration valid: configs/default.yaml
```

JavaScript check: not applicable as a separate command. No standalone changed
`.js` files exist; browser JavaScript is embedded in `tools/vulture_x_ui.py`.

## 13. Known Problems / Remaining Work

- Manual tracker jumping may still exist. Nano and bbox plausibility help, but
  there is no long-run validation against real camera motion.
- Head tracker performance is improved for acquisition, but head reacquisition
  after loss/degradation is not implemented.
- Model availability is better because ONNX assets and verifier exist, but the
  workflow still depends on binary model files being present/correct.
- Runtime latency is only partially measured. Frame mtime is not true sensor or
  RTSP glass-to-glass latency.
- True RTSP capture latency is not measured because the current bridge is still
  file/JPEG based, not direct appsink.
- Thread safety for the planned worker architecture is not addressed because
  workers/buffers do not exist yet.
- HUD has no screenshot regression tests. Diagnostic raw bbox/motion-gate
  toggles are not complete.
- Aircraft portability remains incomplete. There is dynamic RCMAP, RC
  calibration, and some parameter-based clamping, but no Aircraft Compatibility
  & Adaptation Layer yet.
- Parameter issue: `RC5_REVERSED`..`RC8_REVERSED` are consumed if present but
  not requested from live aircraft.
- Parameter assumptions around `MAV_OPTIONS` and `RC_OPTIONS` bits should be
  validated against the target ArduPilot version before relying on them broadly.
- Normal STOP currently commands AUTO in WebUI but does not wait for AUTO
  confirmation.
- No MAVLink RX worker exists; telemetry is read in the same process/loop.
- No complete time-based ACQUIRE/TRACK/DEGRADED/LOST state machine exists.

## 14. Explicitly Preserve These Decisions

The next session must NOT:

- rewrite the project
- migrate to C++
- replace `RC_CHANNELS_OVERRIDE` for fixed-wing
- use `GUIDED_CHANGE_SPEED` for active fixed-wing tracking
- use `GUIDED_CHANGE_ALTITUDE` for active fixed-wing tracking
- use `GUIDED_CHANGE_HEADING` for active fixed-wing tracking
- use `SET_ATTITUDE_TARGET` for active fixed-wing tracking
- use `SET_POSITION_TARGET` for active fixed-wing tracking
- change the tuned fixed-wing controller equations
- change tuned gains without explicit reason
- require AUTO before START TRACKING
- reassert FBWA after pilot changes mode
- restore previous flight mode after normal STOP
- auto-arm real hardware
- make WebUI localhost-only

WebUI is intentionally usable on the closed LAN.

## 15. Next Task

NEXT TASK:
Implement Aircraft Compatibility & Adaptation Layer.

Intended next architecture:

```text
Connected ArduPlane
        ↓
Read aircraft parameters
        ↓
AircraftCapabilityModel
        ↓
CompatibilityEvaluator
        ↓
Aircraft Profile
        +
Payload Profile
        ↓
EffectiveTrackingEnvelope
        ↓
Tracking prerequisite validation
        ↓
READY / WARNING / BLOCKED
```

Important principle:

```text
ArduPilot parameters
=
aircraft capability and safety limits

Vulture-X profile
=
tracking behavior

Effective configuration
=
safe intersection of both
```

Do NOT automatically derive Vulture-X tracking gains directly from ArduPlane
PID values.

## 16. Final Response

After this file is created, respond only with:

```text
Handoff created:
docs/CODEX_HANDOFF.md

Current tests:
...

Working tree:
...

Recommended next session:
Aircraft Compatibility & Adaptation Layer
```
