**\*\*\\\*\\\*# Codex Handoff\\\*\\\*\*\***

This document records the current repository state as observed on this machine

before the next Codex session. It is intentionally descriptive only. Do not use

it as permission to start the next feature.

**\*\*\\\*\\\*## 1. Current Repository State\\\*\\\*\*\***

\\\\- Current branch: \\\\\\\`main\\\\\\\`

\\\\- Upstream status: \\\\\\\`## main...origin/main\\\\\\\`

\\\\- Current-machine baseline verification required before implementation:

\\\\\\\`\\\\\\\`\\\\\\\`bash

git status --short
git log -1 --oneline
git rev-parse HEAD
git rev-parse origin/main

\\\\\\\`\\\\\\\`\\\\\\\`

Expected baseline for this handoff:

\\\\\\\`\\\\\\\`\\\\\\\`text

12ccdf6885441313616a1b72e2a4af62b1b589b6

\\\\\\\`\\\\\\\`\\\\\\\`

If HEAD or repository behavior differs, inspect the current source and update assumptions before changing code.


\\\\- Baseline commit expected for this handoff: \\\\\\\`12ccdf6885441313616a1b72e2a4af62b1b589b6\\\\\\\`

\\\\- Repository source: baseline commit \\\\\\\`12ccdf6885441313616a1b72e2a4af62b1b589b6\\\\\\\` is available on \\\\\\\`origin/main\\\\\\\`.

\\\\- IMPORTANT: before editing code, re-run \\\\\\\`git status --short\\\\\\\`, \\\\\\\`git log -1 --oneline\\\\\\\`, and \\\\\\\`git rev-parse HEAD\\\\\\\`. If the current repository differs from this handoff, the repository is the source of truth.

Historical handoff snapshot below may be stale. Re-verify before implementation:

\\\\\\\`\\\\\\\`\\\\\\\`text

$ git status --short

\\\\\\\`\\\\\\\`\\\\\\\`

No output.

\\\\\\\`\\\\\\\`\\\\\\\`text

$ git diff --stat

\\\\\\\`\\\\\\\`\\\\\\\`

No output.

Working tree after this handoff is created:

\\\\- Modified files: none expected.

\\\\- New files: \\\\\\\`docs/CODEX\\\\\\\_HANDOFF.md\\\\\\\`

\\\\- Deleted files: none expected.

\\\\- Uncommitted changes: this handoff file only.

Do not commit this handoff unless the user explicitly asks.

**\*\*\\\*\\\*## 2. What Has Actually Been Implemented\\\*\\\*\*\***

\\\\| Item | Status | Current implementation |

\\\\| --- | --- | --- |

\\\\| performance instrumentation | PARTIAL | \\\\\\\`tools/sitl\\\\\\\_track\\\\\\\_target.py\\\\\\\` has \\\\\\\`RuntimeMetrics\\\\\\\` for rolling timing/rate/count samples: capture, vision, guidance, MAVLink TX, control period, jitter, deadline misses, dropped frames, tracker failures. It is surfaced in demand JSON and WebUI runtime panel when the tracking process is writing demand state. It is not a full worker-wide tracing system. |

\\\\| TrackerNano | IMPLEMENTED | \\\\\\\`src/vulture\\\\\\\_x/vision/tracker.py\\\\\\\` and \\\\\\\`tools/sitl\\\\\\\_track\\\\\\\_target.py\\\\\\\` define \\\\\\\`NanoTracker\\\\\\\`. SITL helper supports \\\\\\\`--tracker-engine nano\\\\\\\` and NanoTrack model path arguments. WebUI defaults manual/head tracking toward Nano when appropriate. |

\\\\| TemplateMatching fallback | IMPLEMENTED | \\\\\\\`TemplateMatchingTracker\\\\\\\` remains available. SITL helper supports \\\\\\\`--tracker-engine template\\\\\\\`. Red/banner detector-backed tracking continues to use the template path. |

\\\\| YuNet | PARTIAL | \\\\\\\`tools/vulture\\\\\\\_x\\\\\\\_ui.py\\\\\\\` uses OpenCV \\\\\\\`FaceDetectorYN\\\\\\\_create\\\\\\\` in \\\\\\\`detect\\\\\\\_heads\\\\\\\_yunet()\\\\\\\` for head candidate acquisition when the model/API is available. It falls back to Haar head detection. The tracking process validates \\\\\\\`--yunet-model-path\\\\\\\` for \\\\\\\`head\\\\\\\`/\\\\\\\`person\\\\\\\`, but does not run YuNet reacquisition internally. |

\\\\| head acquisition | PARTIAL | WebUI can acquire head candidates using YuNet fallback chain and stores operator-selected normalized bbox. Tracking still depends on \\\\\\\`--selection-file\\\\\\\`; there is no autonomous target choice. |

\\\\| head reacquisition | NOT IMPLEMENTED | No YuNet/Nano association loop reacquires a lost head after tracking degrades or fails. |

\\\\| TrackingBackend abstraction | PARTIAL | There is a \\\\\\\`BboxTracker\\\\\\\` protocol for OpenCV-style trackers and \\\\\\\`TrackingObservation\\\\\\\` data shapes, but no complete shared backend abstraction across runtime/UI/package. |

\\\\| TrackingObservation | PARTIAL | Defined in \\\\\\\`src/vulture\\\\\\\_x/vision/tracker.py\\\\\\\` and in \\\\\\\`tools/sitl\\\\\\\_track\\\\\\\_target.py\\\\\\\`. It is not the main runtime contract for all control decisions. |

\\\\| TrackingResult | PARTIAL | Package-level \\\\\\\`vulture\\\\\\\_x.models.TrackingResult\\\\\\\` and \\\\\\\`tracking\\\\\\\_result\\\\\\\_from\\\\\\\_bbox()\\\\\\\` still exist. The SITL helper mostly uses bbox/demand dictionaries instead of a new unified runtime result object. |

\\\\| confidence propagation | PARTIAL | Nano confidence is read from OpenCV \\\\\\\`getTrackingScore()\\\\\\\` when available and written as \\\\\\\`tracker\\\\\\\_confidence\\\\\\\`/\\\\\\\`last\\\\\\\_confidence\\\\\\\`. Template/classic trackers either return \\\\\\\`None\\\\\\\` or older local scoring. Confidence is not yet a complete safety input. |

\\\\| bbox jump validation | PARTIAL | Target-relative bbox transition gates exist in package and SITL helper before accepting large bbox changes. It is not a complete motion model. |

\\\\| bbox smoothing | IMPLEMENTED | Existing bbox smoothing remains in \\\\\\\`TemplateMatchingTracker\\\\\\\` and SITL helper stabilization paths. |

\\\\| vision states: ACQUIRE, TRACK, DEGRADED, LOST | PARTIAL | WebUI HUD maps demand/target state into labels. Demand JSON writes \\\\\\\`vision\\\\\\\_state: "TRACK"\\\\\\\` in the detected plane path. There is no full internal ACQUIRE/TRACK/DEGRADED/LOST time-state machine. |

\\\\| time-based tracking validity | PARTIAL | Existing monotonic safety timeouts exist for frame age, heartbeat age, and fixed-wing target loss. Tracker internals still rely partly on miss counters. |

\\\\| Capture Worker | NOT IMPLEMENTED | No dedicated capture worker/buffer architecture. |

\\\\| Vision Worker | NOT IMPLEMENTED | No dedicated vision worker. |

\\\\| Control Worker | NOT IMPLEMENTED | Control remains in the main tracking loop. |

\\\\| MAVLink RX Worker | NOT IMPLEMENTED | Tracking loop still calls \\\\\\\`recv\\\\\\\_match\\\\\\\`; WebUI has a separate status monitor thread only. |

\\\\| LatestFrameBuffer | NOT IMPLEMENTED | No shared latest-frame buffer class exists. |

\\\\| LatestTrackingResult | NOT IMPLEMENTED | No shared latest-tracking-result buffer class exists. |

\\\\| LatestVehicleState | NOT IMPLEMENTED | No shared latest-vehicle-state buffer class exists. |

\\\\| fixed 30 Hz control scheduler | PARTIAL | The SITL helper defaults to a rate-based loop and records period/jitter/deadline metrics. It is not a dedicated fixed-deadline scheduler. |

\\\\| worker watchdogs | NOT IMPLEMENTED | No worker supervision architecture exists. Heartbeat/video/target-loss safety timeouts exist in the control loop. |

\\\\| Operational HUD | PARTIAL | \\\\\\\`tools/vulture\\\\\\\_x\\\\\\\_ui.py\\\\\\\` has an operational HUD renderer with top bar, bottom strip, target brackets, reticle, command cue, mode labels, and HUD toggle. It is presentation-layer only. |

\\\\| Diagnostic HUD | PARTIAL | Diagnostic HUD overlays tracker, PWM, guidance, and performance fields. Raw bbox/motion-gate style diagnostic toggles are not complete. |

\\\\| runtime performance panel | PARTIAL | WebUI shows runtime performance from demand JSON when the tracking process provides metrics. It is not an always-on runtime profiler. |

\\\\| GStreamer/appsink changes if any | NOT IMPLEMENTED | RTSP/GStreamer flow remains file/JPEG bridge based; there is no direct appsink capture backend. |

**\*\*\\\*\\\*## 3. Current Fixed-Wing Operational Flow\\\*\\\*\*\***

Current behavior is implemented mainly in \\\\\\\`tools/sitl\\\\\\\_track\\\\\\\_target.py\\\\\\\` and

\\\\\\\`tools/vulture\\\\\\\_x\\\\\\\_ui.py\\\\\\\`.

\\\\- Aircraft can start in any flight mode: IMPLEMENTED for fixed-wing. \\\\\\\`verify\\\\\\\_connection()\\\\\\\` does not require FBWA before start for \\\\\\\`--vehicle plane\\\\\\\`.

\\\\- Operator selects target: IMPLEMENTED for \\\\\\\`custom\\\\\\\`/\\\\\\\`head\\\\\\\` through WebUI selection file before starting tracking.

\\\\- START TRACKING validates prerequisites: IMPLEMENTED. It validates connection, camera frames, model paths, target selection, parameter acceptance, heartbeat/armed state, altitude, and airspeed where available.

\\\\- START TRACKING requests FBWA: IMPLEMENTED. \\\\\\\`request\\\\\\\_plane\\\\\\\_fbwa()\\\\\\\` requests FBWA and waits up to \\\\\\\`PLANE\\\\\\\_FBWA\\\\\\\_TRANSITION\\\\\\\_TIMEOUT\\\\\\\_S = 3.0\\\\\\\`.

\\\\- START TRACKING waits for FBWA confirmation before RC override: IMPLEMENTED. RC override is not enabled until target and FBWA checks pass.

\\\\- TRACKING requires FBWA to remain active: IMPLEMENTED. If heartbeat mode changes away from FBWA after tracking starts, the process fails safe.

\\\\- Pilot/external mode change away from FBWA: IMPLEMENTED. The code immediately releases RC override, aborts tracking, leaves the new mode unchanged, does not reassert FBWA, and does not automatically request AUTO.

\\\\- NORMAL STOP stops commands and releases RC override: IMPLEMENTED through process termination/finally release path.

\\\\- NORMAL STOP requests AUTO: PARTIAL. WebUI \\\\\\\`stop\\\\\\\_steering()\\\\\\\` calls \\\\\\\`set\\\\\\\_plane\\\\\\\_auto()\\\\\\\`, which commands AUTO for simulator-like endpoints. It does not wait for AUTO confirmation in the normal stop path.

\\\\- Restore previous flight mode: NOT IMPLEMENTED and intentionally not desired.

Current difference from intended flow: normal STOP requests AUTO but does not

wait for AUTO confirmation.

**\*\*\\\*\\\*## 4. Safety Behavior\\\*\\\*\*\***

\\\\| Case | Releases RC override | Requests AUTO | Mode change requested? | Aborts tracking | Actual behavior |

\\\\| --- | --- | --- | --- | --- | --- |

\\\\| target lost | Yes | No | No | Yes | After \\\\\\\`plane\\\\\\\_loss\\\\\\\_hold\\\\\\\_s\\\\\\\`, writes failsafe demand and exits tracking. |

\\\\| vision degraded | No separate behavior | No | No | Not directly | No real DEGRADED safety state. Miss holding and target-loss timeout handle loss. |

\\\\| video stale | Yes | No | No | Yes | If frame age exceeds threshold, writes \\\\\\\`stale\\\\\\\_video\\\\\\\` failsafe and exits. |

\\\\| MAVLink heartbeat timeout | Yes | No | No | Yes | If active tracking and heartbeat age exceeds timeout, releases override and exits. |

\\\\| low altitude | Yes | No | No | Yes | Unknown altitude blocks start unless \\\\\\\`--surface-test\\\\\\\`; below minimum during tracking fails safe. |

\\\\| low airspeed | Yes | No | No | Yes | If configured min airspeed is known and low airspeed persists, fails safe. Unknown airspeed blocks start when a min airspeed is known. |

\\\\| disarm | Yes | No | No | Yes | Heartbeat disarmed during active plane tracking fails safe unless \\\\\\\`--surface-test\\\\\\\`. |

\\\\| pilot mode takeover | Yes | No | No | Yes | Any active FBWA tracking mode change away from FBWA releases override and exits. |

\\\\| internal exception | Usually | No | No | Usually | Main process \\\\\\\`finally\\\\\\\` releases override if active. Some setup failures return before active override. |

\\\\| worker failure | Not applicable | Not applicable | Not applicable | Not applicable | Dedicated workers do not exist yet. |

**\*\*\\\*\\\*## 5. Current Aircraft Parameter Reading\\\*\\\*\*\***

All ArduPlane parameter reads currently come from

\\\\\\\`PLANE\\\\\\\_RESPONSE\\\\\\\_PARAM\\\\\\\_NAMES\\\\\\\` in \\\\\\\`tools/sitl\\\\\\\_track\\\\\\\_target.py\\\\\\\`. They are requested

by \\\\\\\`read\\\\\\\_plane\\\\\\\_parameters()\\\\\\\`, stored in a local \\\\\\\`dict[str, float]\\\\\\\`, optionally

cached with \\\\\\\`save\\\\\\\_plane\\\\\\\_param\\\\\\\_cache()\\\\\\\`, and copied into

\\\\\\\`PlaneResponseModel.raw\\\\\\\_params\\\\\\\` by \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params()\\\\\\\`.

\\\\| Parameter | Read? | Used? | Purpose | File/function |

\\\\| --- | --- | --- | --- | --- |

\\\\| \\\\\\\`ROLL\\\\\\\_LIMIT\\\\\\\_DEG\\\\\\\` | Yes | Yes | Caps effective max roll in degrees. | \\\\\\\`tools/sitl\\\\\\\_track\\\\\\\_target.py::plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`PTCH\\\\\\\_LIM\\\\\\\_MAX\\\\\\\_DEG\\\\\\\` | Yes | Yes | Caps effective pitch-up limit. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`PTCH\\\\\\\_LIM\\\\\\\_MIN\\\\\\\_DEG\\\\\\\` | Yes | Yes | Caps effective pitch-down limit by absolute value. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`LIM\\\\\\\_ROLL\\\\\\\_CD\\\\\\\` | Yes | Yes | Legacy fallback roll limit, centidegrees. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`LIM\\\\\\\_PITCH\\\\\\\_MAX\\\\\\\` | Yes | Yes | Legacy fallback pitch-up limit, centidegrees. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`LIM\\\\\\\_PITCH\\\\\\\_MIN\\\\\\\` | Yes | Yes | Legacy fallback pitch-down limit, centidegrees. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`ARSPD\\\\\\\_FBW\\\\\\\_MIN\\\\\\\` | Yes | Yes | Clamps target airspeed lower bound and provides low-airspeed safety threshold. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\`, main tracking loop |

\\\\| \\\\\\\`ARSPD\\\\\\\_FBW\\\\\\\_MAX\\\\\\\` | Yes | Yes | Clamps target airspeed upper bound when positive. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`TRIM\\\\\\\_THROTTLE\\\\\\\` | Yes | Yes | Sets cruise throttle unless UI fixed-throttle mode is active. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`THR\\\\\\\_MIN\\\\\\\` | Yes | Yes | Sets min throttle unless fixed-throttle mode is active. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`THR\\\\\\\_MAX\\\\\\\` | Yes | Yes | Sets max throttle unless fixed-throttle mode is active. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`RCMAP\\\\\\\_ROLL\\\\\\\` | Yes | Yes | Maps roll override channel. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`RCMAP\\\\\\\_PITCH\\\\\\\` | Yes | Yes | Maps pitch override channel. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`RCMAP\\\\\\\_THROTTLE\\\\\\\` | Yes | Yes | Maps throttle override channel. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`RCMAP\\\\\\\_YAW\\\\\\\` | Yes | Yes | Maps yaw channel for calibration model; yaw override is not actively commanded. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`RC1\\\\\\\_MIN\\\\\\\`..\\\\\\\`RC8\\\\\\\_MIN\\\\\\\` | Yes | Yes | RC calibration lower PWM per channel. | \\\\\\\`rc\\\\\\\_calibration\\\\\\\_from\\\\\\\_params\\\\\\\`, \\\\\\\`attitude\\\\\\\_to\\\\\\\_plane\\\\\\\_rc\\\\\\\_pwm\\\\\\\` |

\\\\| \\\\\\\`RC1\\\\\\\_TRIM\\\\\\\`..\\\\\\\`RC8\\\\\\\_TRIM\\\\\\\` | Yes | Yes | RC calibration trim PWM per channel. | \\\\\\\`rc\\\\\\\_calibration\\\\\\\_from\\\\\\\_params\\\\\\\`, \\\\\\\`attitude\\\\\\\_to\\\\\\\_plane\\\\\\\_rc\\\\\\\_pwm\\\\\\\` |

\\\\| \\\\\\\`RC1\\\\\\\_MAX\\\\\\\`..\\\\\\\`RC8\\\\\\\_MAX\\\\\\\` | Yes | Yes | RC calibration upper PWM per channel. | \\\\\\\`rc\\\\\\\_calibration\\\\\\\_from\\\\\\\_params\\\\\\\`, \\\\\\\`attitude\\\\\\\_to\\\\\\\_plane\\\\\\\_rc\\\\\\\_pwm\\\\\\\` |

\\\\| \\\\\\\`RC1\\\\\\\_REVERSED\\\\\\\`..\\\\\\\`RC4\\\\\\\_REVERSED\\\\\\\` | Yes | Yes | RC reversal for mapped channels 1-4. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\`, \\\\\\\`rc\\\\\\\_calibration\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`RC5\\\\\\\_REVERSED\\\\\\\`..\\\\\\\`RC8\\\\\\\_REVERSED\\\\\\\` | No | Partially if present | Code can consume these if manually present in values, but live parameter request list does not request them. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`RC\\\\\\\_OPTIONS\\\\\\\` | Yes | Yes | Blocks start when MAVLink RC override is disabled by option bit. | \\\\\\\`validate\\\\\\\_rc\\\\\\\_override\\\\\\\_acceptance\\\\\\\` |

\\\\| \\\\\\\`RC\\\\\\\_OVERRIDE\\\\\\\_TIME\\\\\\\` | Yes | Yes | Blocks if \\\\\\\`<= 0\\\\\\\`; warns if missing or long; stored in response model. | \\\\\\\`validate\\\\\\\_rc\\\\\\\_override\\\\\\\_acceptance\\\\\\\`, \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`MAV\\\\\\\_GCS\\\\\\\_SYSID\\\\\\\` | Yes | Yes | Validates source-system when MAVLink sysid enforcement is active. | \\\\\\\`validate\\\\\\\_rc\\\\\\\_override\\\\\\\_acceptance\\\\\\\` |

\\\\| \\\\\\\`MAV\\\\\\\_GCS\\\\\\\_SYSID\\\\\\\_HI\\\\\\\` | Yes | Yes | Validates allowed source-system range. | \\\\\\\`validate\\\\\\\_rc\\\\\\\_override\\\\\\\_acceptance\\\\\\\` |

\\\\| \\\\\\\`MAV\\\\\\\_OPTIONS\\\\\\\` | Yes | Yes | Detects whether GCS sysid enforcement applies. | \\\\\\\`validate\\\\\\\_rc\\\\\\\_override\\\\\\\_acceptance\\\\\\\` |

\\\\| \\\\\\\`MIS\\\\\\\_RESTART\\\\\\\` | Yes | Warning only | Warns if mission restart behavior is nonzero. Does not change tracking control. | main parameter warning path |

\\\\| \\\\\\\`SERVO1\\\\\\\_FUNCTION\\\\\\\`..\\\\\\\`SERVO8\\\\\\\_FUNCTION\\\\\\\` | Yes | Display only | Printed/formatted for operator context. Does not change runtime behavior. | \\\\\\\`format\\\\\\\_servo\\\\\\\_functions\\\\\\\` |

\\\\| \\\\\\\`RLL2SRV\\\\\\\_TCONST\\\\\\\` | Yes | No | Read but not currently applied to roll response. | \\\\\\\`PLANE\\\\\\\_RESPONSE\\\\\\\_PARAM\\\\\\\_NAMES\\\\\\\` only |

\\\\| \\\\\\\`PTCH2SRV\\\\\\\_TCONST\\\\\\\` | Yes | Yes | Reduces pitch filter alpha and pitch step limit. | \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params\\\\\\\` |

\\\\| \\\\\\\`RLL2SRV\\\\\\\_P\\\\\\\`, \\\\\\\`RLL2SRV\\\\\\\_I\\\\\\\`, \\\\\\\`RLL2SRV\\\\\\\_D\\\\\\\` | Yes | No | Read but effectively unused. | \\\\\\\`PLANE\\\\\\\_RESPONSE\\\\\\\_PARAM\\\\\\\_NAMES\\\\\\\` only |

\\\\| \\\\\\\`PTCH2SRV\\\\\\\_P\\\\\\\`, \\\\\\\`PTCH2SRV\\\\\\\_I\\\\\\\`, \\\\\\\`PTCH2SRV\\\\\\\_D\\\\\\\` | Yes | No | Read but effectively unused. | \\\\\\\`PLANE\\\\\\\_RESPONSE\\\\\\\_PARAM\\\\\\\_NAMES\\\\\\\` only |

\\\\| \\\\\\\`NAVL1\\\\\\\_PERIOD\\\\\\\` | Yes | No | Read but effectively unused. | \\\\\\\`PLANE\\\\\\\_RESPONSE\\\\\\\_PARAM\\\\\\\_NAMES\\\\\\\` only |

\\\\| \\\\\\\`NAVL1\\\\\\\_DAMPING\\\\\\\` | Yes | No | Read but effectively unused. | \\\\\\\`PLANE\\\\\\\_RESPONSE\\\\\\\_PARAM\\\\\\\_NAMES\\\\\\\` only |

**\*\*\\\*\\\*## 6. Current Aircraft Adaptation Logic\\\*\\\*\*\***

\\\\- \\\\\\\`PlaneResponseModel\\\\\\\`: stores effective roll/pitch/throttle/airspeed limits,

  mapped RC channels, RC calibration, min airspeed, override timeout, and raw

  params.

\\\\- \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params()\\\\\\\`: computes effective roll limit, pitch-up

  and pitch-down limits, throttle limits, target airspeed clamp, pitch filter

  alpha, pitch step, RC channel mapping, RC reversal, and RC calibration.

\\\\- \\\\\\\`channel\\\\\\\_from\\\\\\\_param()\\\\\\\`: validates \\\\\\\`RCMAP\\\\\\\_\\\\\\\*\\\\\\\` values into channels 1-8 with

  fallback defaults.

\\\\- \\\\\\\`rc\\\\\\\_calibration\\\\\\\_from\\\\\\\_params()\\\\\\\`: accepts sane \\\\\\\`RCx\\\\\\\_MIN/TRIM/MAX\\\\\\\` values in

  800..2200 order, otherwise falls back to 1000/1500/2000 and preserves reversal.

\\\\- \\\\\\\`attitude\\\\\\\_to\\\\\\\_plane\\\\\\\_rc\\\\\\\_pwm()\\\\\\\`: converts roll/pitch/throttle commands into PWM

  on mapped/calibrated channels.

\\\\- \\\\\\\`rc\\\\\\\_override()\\\\\\\`: sends \\\\\\\`RC\\\\\\\_CHANNELS\\\\\\\_OVERRIDE\\\\\\\` and clamps absolute PWM to

  800..2200.

\\\\- \\\\\\\`validate\\\\\\\_rc\\\\\\\_override\\\\\\\_acceptance()\\\\\\\`: blocks if \\\\\\\`RC\\\\\\\_OPTIONS\\\\\\\` disables override,

  \\\\\\\`RC\\\\\\\_OVERRIDE\\\\\\\_TIME <= 0\\\\\\\`, or MAVLink source-system is outside enforced

  \\\\\\\`MAV\\\\\\\_GCS\\\\\\\_SYSID\\\\\\\`/\\\\\\\`MAV\\\\\\\_GCS\\\\\\\_SYSID\\\\\\\_HI\\\\\\\`.

\\\\- \\\\\\\`format\\\\\\\_servo\\\\\\\_functions()\\\\\\\`: displays \\\\\\\`SERVOx\\\\\\\_FUNCTION\\\\\\\`; it does not alter

  control behavior.

Read but effectively unused: \\\\\\\`RLL2SRV\\\\\\\_TCONST\\\\\\\`, \\\\\\\`RLL2SRV\\\\\\\_P/I/D\\\\\\\`,

\\\\\\\`PTCH2SRV\\\\\\\_P/I/D\\\\\\\`, \\\\\\\`NAVL1\\\\\\\_PERIOD\\\\\\\`, \\\\\\\`NAVL1\\\\\\\_DAMPING\\\\\\\`, and \\\\\\\`SERVOx\\\\\\\_FUNCTION\\\\\\\`

except for display.

**\*\*\\\*\\\*## 7. Current Unknown / Invalid Telemetry Handling\\\*\\\*\*\***

\\\\- Relative altitude: \\\\\\\`float | None\\\\\\\`; initialized as \\\\\\\`None\\\\\\\`. No fake

  \\\\\\\`relative\\\\\\\_altitude = 50.0\\\\\\\` default remains in current source.

\\\\- Airspeed: \\\\\\\`float | None\\\\\\\`; initialized as \\\\\\\`None\\\\\\\`.

\\\\- Flight mode: \\\\\\\`str | None\\\\\\\` from heartbeat. Unknown mode is not treated as FBWA.

\\\\- Armed state: taken from heartbeat. \\\\\\\`--surface-test\\\\\\\` can permit unarmed SITL

  checks; normal tracking requires armed state before start.

\\\\- Heartbeat: active tracking uses monotonic heartbeat age and fails safe on

  timeout.

\\\\- WebUI MAVLink monitor defaults to disconnected/offline, not connected.

**\*\*\\\*\\\*## 8. Current Tracking Tuning\\\*\\\*\*\***

These are the current fixed-wing source-of-truth defaults. Preserve them unless

there is an explicit, tested reason to change them.

\\\\| Setting | Current default |

\\\\| --- | ---: |

\\\\| \\\\\\\`plane\\\\\\\_airspeed\\\\\\\_mps\\\\\\\` | 20.0 |

\\\\| \\\\\\\`vertical\\\\\\\_gain\\\\\\\` | 52.0 |

\\\\| \\\\\\\`plane\\\\\\\_centering\\\\\\\_gain\\\\\\\` | 1.55 |

\\\\| \\\\\\\`plane\\\\\\\_near\\\\\\\_centering\\\\\\\_gain\\\\\\\` | 2.65 |

\\\\| \\\\\\\`plane\\\\\\\_roll\\\\\\\_gain\\\\\\\_scale\\\\\\\` | 1.75 |

\\\\| \\\\\\\`plane\\\\\\\_pitch\\\\\\\_gain\\\\\\\_scale\\\\\\\` | 1.20 |

\\\\| \\\\\\\`plane\\\\\\\_pitch\\\\\\\_near\\\\\\\_gain\\\\\\\_scale\\\\\\\` | 1.60 |

\\\\| \\\\\\\`plane\\\\\\\_error\\\\\\\_deadband\\\\\\\` | 0.015 |

\\\\| \\\\\\\`plane\\\\\\\_lead\\\\\\\_s\\\\\\\` | 0.0 |

\\\\| \\\\\\\`plane\\\\\\\_damping\\\\\\\_gain\\\\\\\` | 0.14 |

\\\\| \\\\\\\`plane\\\\\\\_near\\\\\\\_damping\\\\\\\_gain\\\\\\\` | 0.30 |

\\\\| \\\\\\\`plane\\\\\\\_pitch\\\\\\\_filter\\\\\\\_alpha\\\\\\\` | 0.45 |

\\\\| \\\\\\\`plane\\\\\\\_max\\\\\\\_pitch\\\\\\\_step\\\\\\\_deg\\\\\\\` | 2.0 |

\\\\| \\\\\\\`plane\\\\\\\_max\\\\\\\_roll\\\\\\\_step\\\\\\\_deg\\\\\\\` | 6.0 |

\\\\| \\\\\\\`max\\\\\\\_plane\\\\\\\_roll\\\\\\\_deg\\\\\\\` | 35.0 |

\\\\| \\\\\\\`max\\\\\\\_plane\\\\\\\_pitch\\\\\\\_deg\\\\\\\` | 40.0 |

\\\\| \\\\\\\`plane\\\\\\\_near\\\\\\\_pitch\\\\\\\_down\\\\\\\_limit\\\\\\\_deg\\\\\\\` | 40.0 |

\\\\| \\\\\\\`plane\\\\\\\_far\\\\\\\_control\\\\\\\_scale\\\\\\\` | 0.72 |

\\\\| \\\\\\\`plane\\\\\\\_near\\\\\\\_control\\\\\\\_scale\\\\\\\` | 1.0 |

\\\\| \\\\\\\`plane\\\\\\\_camera\\\\\\\_hfov\\\\\\\_deg\\\\\\\` | 70.0 |

\\\\| \\\\\\\`plane\\\\\\\_proximity\\\\\\\_far\\\\\\\_size\\\\\\\` | 0.025 |

\\\\| \\\\\\\`plane\\\\\\\_proximity\\\\\\\_near\\\\\\\_size\\\\\\\` | 0.16 |

\\\\| \\\\\\\`plane\\\\\\\_throttle\\\\\\\` | 0.55 |

\\\\| \\\\\\\`plane\\\\\\\_throttle\\\\\\\_airspeed\\\\\\\_gain\\\\\\\` | 0.04 |

\\\\| \\\\\\\`plane\\\\\\\_min\\\\\\\_throttle\\\\\\\` | 0.25 |

\\\\| \\\\\\\`plane\\\\\\\_max\\\\\\\_throttle\\\\\\\` | 0.80 |

\\\\| \\\\\\\`plane\\\\\\\_near\\\\\\\_throttle\\\\\\\_reduction\\\\\\\` | 0.0 |

\\\\| \\\\\\\`plane\\\\\\\_pitch\\\\\\\_below\\\\\\\_center\\\\\\\_boost\\\\\\\` | 0.25 |

\\\\| \\\\\\\`plane\\\\\\\_loss\\\\\\\_hold\\\\\\\_s\\\\\\\` | 1.5 |

\\\\| \\\\\\\`min\\\\\\\_tracking\\\\\\\_alt\\\\\\\_m\\\\\\\` | 15.0 |

\\\\| \\\\\\\`airspeed\\\\\\\_low\\\\\\\_persistence\\\\\\\_s\\\\\\\` | 2.0 |

WebUI simulator mode currently forces fixed-wing throttle/min/max throttle to

\\\\\\\`0.80\\\\\\\` and preserves at least the tuned simulator gain floors in

\\\\\\\`AppState.start\\\\\\\_steering()\\\\\\\`.

IMPORTANT tuning rule for the Runtime Refactor:

\\\\\\\`\\\\\\\`\\\\\\\`text

Do not change, normalize, or "clean up" tuning values during runtime work.

Use the CURRENT repository/config/runtime values as source of truth.

Live WebUI tuning discovered during simulator testing is a separate tuning activity.

In particular, the recent simulator experiment where Overall Response was reduced from 70 toward 40 improved roll oscillation, but that experiment MUST NOT be silently written into source defaults by this runtime refactor.

\\\\\\\`\\\\\\\`\\\\\\\`

The Runtime Refactor must preserve whatever values are actually active in the current repository/config unless the user explicitly authorizes a tuning change.


**\*\*\\\*\\\*## 9. Current Runtime Architecture\\\*\\\*\*\***

Current implementation:

\\\\\\\`\\\\\\\`\\\\\\\`text

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

single control loop in tools/sitl\\\\\\\_track\\\\\\\_target.py

        |

        v

MAVLink RC\\\\\\\_CHANNELS\\\\\\\_OVERRIDE for ArduPlane FBWA

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

\\\\\\\`\\\\\\\`\\\\\\\`

Planned but not implemented:

\\\\\\\`\\\\\\\`\\\\\\\`text

Camera -> Capture Worker -> LatestFrameBuffer

LatestFrameBuffer -> Vision Worker -> LatestTrackingResult

MAVLink RX Worker -> LatestVehicleState

LatestTrackingResult + LatestVehicleState -> fixed 30 Hz Control Worker

Control Worker -> MAVLink -> ArduPlane

\\\\\\\`\\\\\\\`\\\\\\\`

**\*\*\\\*\\\*## 10. Important Files and Responsibilities\\\*\\\*\*\***

\\\\- \\\\\\\`tools/sitl\\\\\\\_track\\\\\\\_target.py\\\\\\\`: local SITL/manual visual steering helper. Owns

  CLI parsing, model path validation, frame reads, trackers, fixed-wing guidance

  loop, ArduPlane parameter reads/adaptation, failsafes, demand JSON, and

  \\\\\\\`RC\\\\\\\_CHANNELS\\\\\\\_OVERRIDE\\\\\\\`.

\\\\- \\\\\\\`tools/vulture\\\\\\\_x\\\\\\\_ui.py\\\\\\\`: WebUI server, camera bridge process control, MAVLink

  status monitor, live tuning JSON, target selection, YuNet head candidates,

  HUD rendering, start/stop steering, and normal stop AUTO command.

\\\\- \\\\\\\`tools/mavlink\\\\\\\_endpoint.py\\\\\\\`: MAVLink endpoint parsing/opening, including

  \\\\\\\`udpcl:\\\\\\\` normalization to \\\\\\\`udpout:\\\\\\\`.

\\\\- \\\\\\\`tools/ensure\\\\\\\_vision\\\\\\\_models.py\\\\\\\`: verifies/downloads OpenCV NanoTrack and

  YuNet ONNX models by expected path/hash.

\\\\- \\\\\\\`tools/sitl\\\\\\\_arm\\\\\\\_takeoff.py\\\\\\\`: guarded SITL-only arm/takeoff helper. Includes

  explicit environment gate and separate quad/plane takeoff paths.

\\\\- \\\\\\\`tools/track\\\\\\\_camera\\\\\\\_target.py\\\\\\\`: non-commanding camera target/head detection

  utilities, including Haar fallback helpers.

\\\\- \\\\\\\`src/vulture\\\\\\\_x/vision/tracker.py\\\\\\\`: package-level tracking helpers,

  \\\\\\\`TemplateMatchingTracker\\\\\\\`, \\\\\\\`NanoTracker\\\\\\\`, \\\\\\\`OpenCvTracker\\\\\\\`, bbox smoothing and

  transition plausibility.

\\\\- \\\\\\\`src/vulture\\\\\\\_x/vision/target\\\\\\\_state.py\\\\\\\`: converts bbox observations to package

  \\\\\\\`TrackingResult\\\\\\\`.

\\\\- \\\\\\\`src/vulture\\\\\\\_x/models.py\\\\\\\`: immutable package models for tracking, guidance,

  vehicle state, commands, and safety.

\\\\- \\\\\\\`src/vulture\\\\\\\_x/guidance/\\\\\\\*\\\\\\\`: package guidance and bounded command limiting for

  the milestone path; not the local fixed-wing FBWA helper.

\\\\- \\\\\\\`src/vulture\\\\\\\_x/safety/\\\\\\\*\\\\\\\`: package safety checks/supervisor/abort policy.

\\\\- \\\\\\\`src/vulture\\\\\\\_x/vehicle/\\\\\\\*\\\\\\\`: package vehicle abstractions and mock transport;

  real async pymavlink client is not implemented yet.

\\\\- \\\\\\\`src/vulture\\\\\\\_x/mission/\\\\\\\*\\\\\\\`: package mission state machine through \\\\\\\`TRACK\\\\\\\`.

\\\\- \\\\\\\`configs/\\\\\\\*.yaml\\\\\\\`: package runtime configuration used by \\\\\\\`vulture-x\\\\\\\`.

\\\\- \\\\\\\`models/opencv/\\\\\\\*\\\\\\\`: local ONNX model assets for NanoTrack/YuNet.

**\*\*\\\*\\\*## 11. Public Interfaces That Must Not Be Broken\\\*\\\*\*\***

\\\\- WebUI HTTP endpoints used by browser/tests:

  \\\\\\\`/api/status\\\\\\\`, \\\\\\\`/api/frame.jpg\\\\\\\`, \\\\\\\`/api/start\\\\\\\_steering\\\\\\\`,

  \\\\\\\`/api/stop\\\\\\\_steering\\\\\\\`, \\\\\\\`/api/tracking\\\\\\\_tuning\\\\\\\`, \\\\\\\`/api/selection\\\\\\\`,

  \\\\\\\`/api/selection/head\\\\\\\`, \\\\\\\`/api/start\\\\\\\_bridge\\\\\\\`, \\\\\\\`/api/connect\\\\\\\_mavlink\\\\\\\`.

\\\\- WebUI LAN behavior: do not make the server localhost-only.

\\\\- Runtime demand JSON keys used by WebUI/HUD/tests: \\\\\\\`detected\\\\\\\`, \\\\\\\`mode\\\\\\\`,

  \\\\\\\`vehicle\\\\\\\`, \\\\\\\`bbox\\\\\\\`, \\\\\\\`center\\\\\\\_x\\\\\\\`, \\\\\\\`center\\\\\\\_y\\\\\\\`, \\\\\\\`guided\\\\\\\_error\\\\\\\_x\\\\\\\`,

  \\\\\\\`guided\\\\\\\_error\\\\\\\_y\\\\\\\`, \\\\\\\`roll\\\\\\\_deg\\\\\\\`, \\\\\\\`pitch\\\\\\\_deg\\\\\\\`, \\\\\\\`roll\\\\\\\_pwm\\\\\\\`, \\\\\\\`pitch\\\\\\\_pwm\\\\\\\`,

  \\\\\\\`throttle\\\\\\\_pwm\\\\\\\`, \\\\\\\`max\\\\\\\_roll\\\\\\\_deg\\\\\\\`, \\\\\\\`max\\\\\\\_pitch\\\\\\\_deg\\\\\\\`, \\\\\\\`rate\\\\\\\_hz\\\\\\\`,

  \\\\\\\`frame\\\\\\\_age\\\\\\\_ms\\\\\\\`, \\\\\\\`tracking\\\\\\\_result\\\\\\\_age\\\\\\\_ms\\\\\\\`, \\\\\\\`command\\\\\\\_age\\\\\\\_ms\\\\\\\`,

  \\\\\\\`tracker\\\\\\\_confidence\\\\\\\`, \\\\\\\`tracker\\\\\\\_engine\\\\\\\`, \\\\\\\`vision\\\\\\\_state\\\\\\\`,

  \\\\\\\`control\\\\\\\_authority\\\\\\\`, \\\\\\\`rc\\\\\\\_override\\\\\\\_active\\\\\\\`, \\\\\\\`target\\\\\\\_proximity\\\\\\\`, \\\\\\\`throttle\\\\\\\`,

  \\\\\\\`airspeed\\\\\\\_mps\\\\\\\`, \\\\\\\`effective\\\\\\\_max\\\\\\\_roll\\\\\\\_deg\\\\\\\`, \\\\\\\`effective\\\\\\\_max\\\\\\\_pitch\\\\\\\_deg\\\\\\\`,

  \\\\\\\`tracking\\\\\\\_tuning\\\\\\\_revision\\\\\\\`, \\\\\\\`runtime\\\\\\\_metrics\\\\\\\`, \\\\\\\`failsafe\\\\\\\`,

  \\\\\\\`failsafe\\\\\\\_reason\\\\\\\`.

\\\\- Live tuning JSON shape in \\\\\\\`logs/ui/tracking\\\\\\\_tuning.json\\\\\\\`:

  \\\\\\\`{"ok": true, "revision": int, "values": {...}}\\\\\\\`.

\\\\- Selection JSON shape in \\\\\\\`logs/ui/custom\\\\\\\_selection.json\\\\\\\`: normalized

  \\\\\\\`x\\\\\\\`, \\\\\\\`y\\\\\\\`, \\\\\\\`width\\\\\\\`, \\\\\\\`height\\\\\\\`, plus selection mode metadata.

\\\\- SITL helper CLI arguments, especially: \\\\\\\`--enable-guidance\\\\\\\`, \\\\\\\`--mavlink\\\\\\\`,

  \\\\\\\`--vehicle\\\\\\\`, \\\\\\\`--camera-dir\\\\\\\`, \\\\\\\`--pipeline\\\\\\\`, \\\\\\\`--tracking-mode\\\\\\\`,

  \\\\\\\`--selection-file\\\\\\\`, \\\\\\\`--tracker\\\\\\\`, \\\\\\\`--tracker-engine\\\\\\\`,

  \\\\\\\`--tracker-nano-backbone\\\\\\\`, \\\\\\\`--tracker-nano-neckhead\\\\\\\`,

  \\\\\\\`--yunet-model-path\\\\\\\`, \\\\\\\`--demand-state-file\\\\\\\`, \\\\\\\`--tuning-file\\\\\\\`,

  \\\\\\\`--max-frame-age-ms\\\\\\\`, \\\\\\\`--mavlink-heartbeat-timeout-s\\\\\\\`, \\\\\\\`--surface-test\\\\\\\`,

  \\\\\\\`--source-system\\\\\\\`, \\\\\\\`--source-component\\\\\\\`, \\\\\\\`--read-plane-params\\\\\\\`,

  \\\\\\\`--plane-param-cache-file\\\\\\\`, and all \\\\\\\`--plane-\\\\\\\*\\\\\\\` tuning flags.

\\\\- \\\\\\\`tools.mavlink\\\\\\\_endpoint.parse\\\\\\\_mavlink\\\\\\\_endpoint()\\\\\\\` behavior, including

  \\\\\\\`udpcl:\\\\\\\` to \\\\\\\`udpout:\\\\\\\` conversion.

\\\\- Package tracker signatures and names: \\\\\\\`OpenCvTracker\\\\\\\`, \\\\\\\`TemplateMatchingTracker\\\\\\\`,

  \\\\\\\`NanoTracker\\\\\\\`, \\\\\\\`TrackingObservation\\\\\\\`, \\\\\\\`tracking\\\\\\\_result\\\\\\\_from\\\\\\\_bbox()\\\\\\\`.

\\\\- Test-covered helper signatures: \\\\\\\`request\\\\\\\_plane\\\\\\\_fbwa()\\\\\\\`, \\\\\\\`verify\\\\\\\_connection()\\\\\\\`,

  \\\\\\\`validate\\\\\\\_rc\\\\\\\_override\\\\\\\_acceptance()\\\\\\\`, \\\\\\\`plane\\\\\\\_response\\\\\\\_model\\\\\\\_from\\\\\\\_params()\\\\\\\`,

  \\\\\\\`attitude\\\\\\\_to\\\\\\\_plane\\\\\\\_rc\\\\\\\_pwm()\\\\\\\`, \\\\\\\`rc\\\\\\\_override()\\\\\\\`.

\\\\- Quad path remains ArduPilot \\\\\\\`GUIDED\\\\\\\` body velocity; fixed-wing tracking remains

  ArduPlane \\\\\\\`FBWA\\\\\\\` with bounded \\\\\\\`RC\\\\\\\_CHANNELS\\\\\\\_OVERRIDE\\\\\\\`.

**\*\*\\\*\\\*## 12. Tests\\\*\\\*\*\***

Historical test counts in older handoffs must NOT be treated as current truth.

On the current updated machine/repository, re-run the complete baseline before implementation:

\\\\\\\`\\\\\\\`\\\\\\\`bash

python -m pytest
ruff check .
mypy
python -m compileall tools src tests
vulture-x --config configs/default.yaml --check-config

\\\\\\\`\\\\\\\`\\\\\\\`

Also record:

\\\\\\\`\\\\\\\`\\\\\\\`bash

python --version
python -c "import cv2, numpy; print('cv2', cv2.__version__); print('numpy', numpy.__version__)"
git status --short
git log -1 --oneline

\\\\\\\`\\\\\\\`\\\\\\\`

The user has already confirmed that the current test suite passes on the updated machine, but Codex must record the exact current pass count and tool results at session start rather than reusing the historical \\\\\\\`157 passed\\\\\\\` figure.

JavaScript check remains only applicable if browser JavaScript is changed. Browser JavaScript is currently embedded in \\\\\\\`tools/vulture\\\\\\\_x\\\\\\\_ui.py\\\\\\\`.

\\\*\\\*## 13. Known Problems / Remaining Work\\\*\\\*

\\\\- Manual tracker jumping may still exist. Nano and bbox plausibility help, but

  there is no long-run validation against real camera motion.

\\\\- Head tracker performance is improved for acquisition, but head reacquisition

  after loss/degradation is not implemented.

\\\\- Model availability is better because ONNX assets and verifier exist, but the

  workflow still depends on binary model files being present/correct.

\\\\- Runtime latency is only partially measured. Frame mtime is not true sensor or

  RTSP glass-to-glass latency.

\\\\- True RTSP capture latency is not measured because the current bridge is still

  file/JPEG based, not direct appsink.

\\\\- Thread safety for the planned worker architecture is not addressed because

  workers/buffers do not exist yet.

\\\\- HUD has no screenshot regression tests. Diagnostic raw bbox/motion-gate

  toggles are not complete.

\\\\- Aircraft portability remains incomplete. There is dynamic RCMAP, RC

  calibration, and some parameter-based clamping, but no Aircraft Compatibility

  & Adaptation Layer yet.

\\\\- Parameter issue: \\\\\\\`RC5\\\\\\\_REVERSED\\\\\\\`..\\\\\\\`RC8\\\\\\\_REVERSED\\\\\\\` are consumed if present but

  not requested from live aircraft.

\\\\- Parameter assumptions around \\\\\\\`MAV\\\\\\\_OPTIONS\\\\\\\` and \\\\\\\`RC\\\\\\\_OPTIONS\\\\\\\` bits should be

  validated against the target ArduPilot version before relying on them broadly.

\\\\- Normal STOP currently commands AUTO in WebUI but does not wait for AUTO

  confirmation.

\\\\- No MAVLink RX worker exists; telemetry is read in the same process/loop.

\\\\- No complete time-based ACQUIRE/TRACK/DEGRADED/LOST state machine exists.

\\\*\\\*## 14. Explicitly Preserve These Decisions\\\*\\\*

The next session must NOT:

\\\\- rewrite the project

\\\\- migrate to C++

\\\\- replace \\\\\\\`RC\\\\\\\_CHANNELS\\\\\\\_OVERRIDE\\\\\\\` for fixed-wing

\\\\- use \\\\\\\`GUIDED\\\\\\\_CHANGE\\\\\\\_SPEED\\\\\\\` for active fixed-wing tracking

\\\\- use \\\\\\\`GUIDED\\\\\\\_CHANGE\\\\\\\_ALTITUDE\\\\\\\` for active fixed-wing tracking

\\\\- use \\\\\\\`GUIDED\\\\\\\_CHANGE\\\\\\\_HEADING\\\\\\\` for active fixed-wing tracking

\\\\- use \\\\\\\`SET\\\\\\\_ATTITUDE\\\\\\\_TARGET\\\\\\\` for active fixed-wing tracking

\\\\- use \\\\\\\`SET\\\\\\\_POSITION\\\\\\\_TARGET\\\\\\\` for active fixed-wing tracking

\\\\- change the tuned fixed-wing controller equations

\\\\- change tuned gains without explicit reason

\\\\- require AUTO before START TRACKING

\\\\- reassert FBWA after pilot changes mode

\\\\- restore previous flight mode after normal STOP

\\\\- auto-arm real hardware

\\\\- make WebUI localhost-only

WebUI is intentionally usable on the closed LAN.

\*\*## 15. Next Task - Runtime Architecture Refactor\*\*


### Pre-Execution Gate

Before changing any source code, Codex MUST:

\\\`\\\`\\\`text

1. Verify HEAD and origin/main.
2. Confirm the baseline is 12ccdf6885441313616a1b72e2a4af62b1b589b6, or document why it differs.
3. Run and record the current full baseline tests.
4. Inspect the current WebUI -> tracking subprocess relationship.
5. Check actual GStreamer/OpenCV/appsink capability on this machine.
6. Record current controller/tuning values without modifying them.
7. Only then begin the incremental runtime refactor.

\\\`\\\`\\\`

If any of these checks materially contradict this handoff, update the implementation plan from the repository facts rather than forcing the handoff assumptions.



The immediate next Codex task is to convert the CURRENT Vulture-X implementation into an efficient runtime-based architecture before continuing the tracking roadmap.

Only this runtime refactor is authorized now.

Do NOT automatically start Tracking Phase 1, Tracking Phase 2, Tracking Phase 3, or Aircraft Compatibility after finishing this task.

\### Objective

Change Vulture-X from the current mostly sequential/file-oriented flow:

\`\`\`text

Camera / RTSP

    ↓

GStreamer

    ↓

JPEG file

    ↓

cv2.imread()

    ↓

Tracking

    ↓

Guidance

    ↓

MAVLink

\`\`\`

into:

\`\`\`text

                    CAMERA

                       ↓

                Capture Worker

                       ↓

                  LatestFrame

                       ↓

                 Vision Worker

                       ↓

            LatestTrackingResult

                       │

                       │

AUTOPILOT              ▼

    ↓             Control Worker

MAVLink RX Worker      fixed 30 Hz

    ↓                  │

LatestVehicleState     ↓

    └──────────────→ MAVLink TX

                       ↓

                    ArduPlane



LatestFrame

\+

LatestTrackingResult

\+

LatestVehicleState

\+

RuntimeMetrics

        ↓

Preview / HUD / WebUI

\`\`\`

The purpose is lower latency, no frame backlog, deterministic control timing, and a runtime suitable for small Linux SBCs such as Orange Pi Zero 2W-class hardware.

This is a runtime refactor, NOT a controller redesign.

---

### 15.0 Runtime Migration Rules

The runtime refactor MUST follow these rules before implementation details below.

#### A. Prefer One Long-Lived Vulture-X Process

The current WebUI may launch \\\`tools/sitl_track_target.py\\\` as a subprocess. Do not preserve that subprocess boundary by inventing complex IPC.

Preferred target:

\\\`\\\`\\\`text

ONE Vulture-X PROCESS

├── Capture Worker
├── Vision Worker
├── MAVLink RX Worker
├── Control Worker
└── WebUI / Preview

\\\`\\\`\\\`

Shared runtime data should be exchanged with in-memory latest-value snapshots inside that process.

Do NOT introduce shared-memory frameworks, message brokers, multiprocessing queues, sockets, or other complex IPC merely to preserve the current WebUI -> tracking subprocess architecture.

If the existing subprocess launcher must temporarily remain for compatibility, keep it as a transitional/legacy path. It must NOT become the architecture of the new runtime.

The long-term direction is one long-lived runtime process with internal workers and a simple WebUI interface.

#### B. Refactor Incrementally, No Big-Bang Rewrite

Implement the runtime in small steps and keep the repository runnable after each step.

Preferred migration order:

\\\`\\\`\\\`text

1. runtime types + LatestValue
2. in-memory capture backend
3. Capture Worker
4. Vision Worker
5. MAVLink RX Worker + LatestVehicleState
6. fixed-deadline Control Worker
7. WebUI/HUD preview separation
8. worker health + metrics
9. benchmark / cleanup

\\\`\\\`\\\`

Do NOT rewrite \\\`tools/sitl_track_target.py\\\` in one large replacement.

Prefer compatibility adapters and progressive extraction.

After each meaningful step, run the relevant unit/regression tests before continuing.

#### C. Capability-Check GStreamer Before Choosing the AppSink Integration

Before implementing the appsink backend, inspect the actual updated machine:

\\\`\\\`\\\`bash

python - <<'PY'
import cv2
print(cv2.getBuildInformation())
PY

gst-launch-1.0 --version
gst-inspect-1.0 appsink

\\\`\\\`\\\`

Also check whether Python GStreamer bindings are already available if they are being considered.

Determine:

\\\`\\\`\\\`text

- whether the installed OpenCV build has GStreamer support
- whether system GStreamer is installed
- whether appsink is available
- whether Python GStreamer bindings are already present
\\\`\\\`\\\`

Do NOT redesign or replace the Python environment just to obtain one preferred integration method.

Choose the simplest in-memory GStreamer path actually supported by the current machine.

If OpenCV's GStreamer backend is unavailable but system GStreamer/appsink is available, use an appropriate lightweight binding/path rather than forcing a new OpenCV build.

Always keep the legacy file backend available during migration.

#### D. Preview Quality Is an Explicit Refactor Goal

The current preview/HUD has shown visible JPEG degradation.

For the NEW preview path, target:

\\\`\\\`\\\`text

latest in-memory camera frame
        ↓
HUD render at source/native frame resolution
        ↓
ONE browser JPEG encode
        ↓
WebUI

\\\`\\\`\\\`

Do NOT:

\\\`\\\`\\\`text

JPEG frame
→ decode
→ draw HUD
→ JPEG encode again

\\\`\\\`\\\`

merely to serve the normal new preview path.

Preserve the camera's native aspect ratio.

Avoid unnecessary upscaling before browser delivery.

The browser may scale for display, but the server should not repeatedly resample or recompress the frame without need.

This preview-quality requirement is part of the Runtime Refactor completion criteria.


\---

\### 15.1 Inspect Current Source First

Before editing, inspect the CURRENT repository, especially:

\`\`\`text

tools/sitl\_track\_target.py

tools/vulture\_x\_ui.py

tools/mavlink\_endpoint.py

src/vulture\_x/models.py

src/vulture\_x/vision/

src/vulture\_x/guidance/

src/vulture\_x/safety/

src/vulture\_x/vehicle/

tests/

\`\`\`

Identify current frame capture, JPEG bridge, \`FrameDirectoryReader\`, \`cv2.imread\`, GStreamer launch, tracking loop, \`recv\_match\`, MAVLink TX, guidance calculation, demand JSON, HUD rendering, \`/api/frame.jpg\`, runtime metrics, failsafe paths, and shutdown behavior.

Repository code is the source of truth.

\---

\### 15.2 Preserve the Legacy Capture Path

Do NOT delete the current file/JPEG path yet.

Provide two capture backends, or equivalent configuration:

\`\`\`text

file

appsink

\`\`\`

Intent:

\`\`\`text

file = legacy fallback / debug / compatibility

appsink = preferred runtime path

\`\`\`

Existing SITL and tests that depend on the file path must continue working.

\---

\### 15.3 Add an In-Memory VideoFrame Contract

Create a canonical frame type, for example:

\`\`\`python

@dataclass(frozen=True)

class VideoFrame:

    sequence: int

    image: np.ndarray

    received\_timestamp\_ns: int

    source: str | None = None

\`\`\`

Use monotonic timestamps for freshness calculations.

\`received\_timestamp\_ns\` is the best available LOCAL frame receive/acquisition time. Do NOT describe it as true camera sensor latency unless the source genuinely provides a sensor timestamp.

\---

\### 15.4 Add FrameSource

Create a capture abstraction such as:

\`\`\`python

class FrameSource(Protocol):

    def read(self) -> VideoFrame | None:

        ...

    def close(self) -> None:

        ...

\`\`\`

Adapt the old file capture and the new appsink capture to the same interface where practical.

\---

\### 15.5 Implement GStreamer AppSink Capture


Before implementation, complete the capability check from section 15.0C.

Do not assume \`cv2.VideoCapture(..., cv2.CAP_GSTREAMER)\` is available simply because OpenCV is installed from \`opencv-contrib-python\`.

Use the simplest supported in-memory path on the actual machine.

New preferred path:

\`\`\`text

RTSP

→ GStreamer

→ H264 decode

→ BGR/GRAY frame

→ appsink

→ NumPy/OpenCV frame in RAM

→ Vision Worker

\`\`\`

Remove JPEG encode, filesystem write, \`cv2.imread\`, and JPEG decode from the NEW critical tracking path.

Use bounded/latest-frame-oriented GStreamer buffering. Where supported use behavior equivalent to:

\`\`\`text

max-buffers=1

drop=true

sync=false

\`\`\`

Do not create an unbounded GStreamer or Python frame queue. Do not claim zero-copy unless it is actually implemented.

\---

\### 15.6 Implement Latest-Value Buffers

Create a small thread-safe latest-value primitive.

\`\`\`python

class LatestValue(Generic[T]):

    def publish(self, value: T) -> None:

        ...

    def get\_latest(self) -> T | None:

        ...

\`\`\`

Use it for:

\`\`\`text

LatestFrame

LatestTrackingResult

LatestVehicleState

\`\`\`

Important behavior:

\`\`\`text

camera produces: 101, 102, 103, 104

vision finishes 101

vision reads 104

\`\`\`

Frames 102 and 103 may be skipped. Do NOT use a FIFO frame backlog.

Prefer immutable snapshots and short lock duration. Do not hold locks while doing expensive OpenCV, GStreamer, MAVLink, or HUD work.

\---

\### 15.7 Capture Worker

Add a dedicated Capture Worker.

Responsibilities only:

\`\`\`text

read/decode frame

assign sequence

timestamp

publish LatestFrame

capture metrics

worker heartbeat

\`\`\`

It must NOT track targets, detect heads, render HUD, read MAVLink, calculate guidance, send RC override, or write tracking frames to persistent storage.

Capture must continue even if vision is slower.

\---

\### 15.8 Vision Worker

Add a dedicated Vision Worker.

Responsibilities:

\`\`\`text

read newest LatestFrame

ignore already-processed sequence

run existing tracking path

publish LatestTrackingResult

vision metrics

worker heartbeat

\`\`\`

Do not create backlog.

Preserve existing behavior for NanoTracker, TemplateMatchingTracker, manual/custom tracking, head tracking/acquisition, red mode, banner mode, bbox validation, bbox smoothing, and tracker confidence.

Do NOT redesign tracking algorithms during this runtime task.

\---

\### 15.9 MAVLink RX Worker

Move continuous telemetry reception out of the control loop.

\`\`\`text

Autopilot

    ↓

MAVLink RX Worker

    ↓

LatestVehicleState

\`\`\`

Use non-blocking or short-bounded receive/drain behavior.

The MAVLink RX worker owns telemetry RECEIVE only. It must NOT send tracking RC override commands.

Preserve telemetry currently required for mode, armed state, heartbeat, relative altitude, airspeed, safety, and current aircraft parameter behavior.

Unknown values remain \`None\`. Do not restore fake altitude or airspeed defaults.

\---

\### 15.10 LatestVehicleState

Create or reuse a coherent immutable vehicle snapshot.

At minimum preserve current required fields:

\`\`\`text

mode

armed

relative\_altitude\_m

airspeed\_mps

heartbeat\_timestamp\_ns

\`\`\`

plus other state currently required by guidance/safety.

The Control Worker should read a snapshot instead of calling blocking MAVLink receive.

\---

\### 15.11 Fixed 30 Hz Control Worker

Create a dedicated fixed-deadline Control Worker.

Target:

\`\`\`text

30 Hz

33.3 ms period

\`\`\`

Do NOT use \`work(); time.sleep(1 / 30)\` because work time would reduce the real rate.

Use monotonic deadline scheduling and handle overruns without building timing backlog.

Record:

\`\`\`text

control period

control jitter

deadline misses

\`\`\`

\---

\### 15.12 One Owner for Active Tracking Commands

During active tracking:

\`\`\`text

Control Worker = owner of tracking RC override / tracking command TX

\`\`\`

Capture Worker, Vision Worker, HUD renderer, and Preview path must not send active tracking commands.

Mode transition helpers may remain coordinated by the runtime/state owner, but RC override command ownership must be clear.

\---

\### 15.13 Preserve Fixed-Wing Controller Exactly

Do NOT change existing fixed-wing guidance equations.

Preserve current functions/logic including the current equivalents of:

\`\`\`text

plane\_pitch\_command()

plane\_throttle\_for\_airspeed()

scheduled\_gain()

adaptive\_pitch\_gain()

damped\_axis\_error()

damp\_pitch\_command()

near\_target\_control\_scale()

far\_target\_control\_scale()

perspective\_correct\_error()

\`\`\`

For equivalent valid target and vehicle inputs, the refactored runtime must produce equivalent roll demand, pitch demand, throttle demand, roll PWM, pitch PWM, and throttle PWM within normal floating-point tolerance.

This runtime work must not be used to retune the aircraft.

\---

\### 15.14 Preserve Current Fixed-Wing Tuning

Preserve the complete source-of-truth tuning values from section 8.

Especially do not reset current values such as:

\`\`\`text

plane\_centering\_gain = 1.55

plane\_near\_centering\_gain = 2.65

plane\_roll\_gain\_scale = 1.75

plane\_pitch\_gain\_scale = 1.20

plane\_pitch\_near\_gain\_scale = 1.60

plane\_damping\_gain = 0.14

plane\_pitch\_filter\_alpha = 0.45

plane\_max\_roll\_step\_deg = 6.0

\`\`\`

If actual current repository values differ, current repository values win. Do not restore older historical defaults.

\---

\### 15.15 Preserve Fixed-Wing Operational Behavior

Do not change:

\`\`\`text

aircraft may start in ANY flight mode

operator selects target

START TRACKING

→ validate prerequisites

→ request FBWA

→ wait until actual mode confirms FBWA

→ only then enable control authority / RC override

\`\`\`

During active tracking, actual mode must remain FBWA.

If pilot/external source changes mode away from FBWA:

\`\`\`text

stop tracking command generation

release RC override

control\_authority = false

abort tracking

preserve the newly selected mode

do not reassert FBWA

do not force AUTO

\`\`\`

Normal STOP keeps the existing intended behavior: stop tracking command output, release RC override, request AUTO.

Do not change flight-mode semantics in this task.

\---

\### 15.16 Preserve Quad Path

Do NOT change the quad control strategy.

\`\`\`text

Quad = ArduPilot GUIDED body velocity

Fixed-wing = ArduPlane FBWA + RC\_CHANNELS\_OVERRIDE

\`\`\`

Runtime architecture may be shared, but control strategies remain separate.

\---

\### 15.17 Freshness at the Control Boundary

The Control Worker must use the latest available data but must also know its age.

Track at least:

\`\`\`text

frame\_age\_ms

tracking\_result\_age\_ms

heartbeat\_age\_ms

\`\`\`

Do not invent new final safety thresholds without measurement.

Initially preserve current stale-video, heartbeat, target-loss, altitude, and airspeed safety semantics.

Never use an indefinitely old tracking result merely because it is still the newest object in memory.

\---

\### 15.18 Separate Preview/HUD from Flight-Critical Runtime

Target:

\`\`\`text

LatestFrame

\+

LatestTrackingResult

\+

LatestVehicleState

\+

RuntimeMetrics

        ↓

Preview Renderer

        ↓

Operational / Diagnostic HUD

        ↓

JPEG for browser

        ↓

WebUI

\`\`\`

Rules:

\`\`\`text

browser request must NOT run tracking

browser request must NOT run detector

browser request must NOT read MAVLink synchronously

HUD failure must NOT stop tracking

WebUI disconnect must NOT stop tracking

\`\`\`

\`/api/frame.jpg\` should use already-available runtime snapshots.

The architecture must allow independent rates, for example camera 30 FPS, vision 25-30 Hz, control 30 Hz, preview 10-15 FPS. Do not hardcode these exact preview values if current tests/config require something else.



Additional preview-quality rule:

\\\`\\\`\\\`text

latest raw/in-memory frame
→ HUD/OSD render
→ ONE JPEG encode for browser

\\\`\\\`\\\`

Do not decode an already-compressed preview JPEG and re-encode it only to draw the operational HUD.

Keep native aspect ratio and avoid unnecessary server-side upscaling.

\---

\### 15.19 Remove Persistent Frame I/O from the New Critical Path

The appsink path must keep tracking frames in RAM.

Do not write every camera frame to microSD/SSD as part of tracking.

Legacy file mode may still do so for compatibility.

Demand JSON may remain for WebUI/test compatibility, but internal communication between workers must use in-memory state.

\---

\### 15.20 Demand JSON and WebUI Compatibility

Do not break existing demand JSON fields listed in section 11.

The new runtime may generate demand JSON as a compatibility/output adapter.

Demand JSON must no longer be the primary communication path between vision, control, and MAVLink.

Do not break current WebUI HTTP endpoints. WebUI remains accessible on the LAN. Do not add authentication or make it localhost-only in this task.

\---

\### 15.21 Use Threads First

Preferred initial model:

\`\`\`text

main Vulture-X process

├── Capture Thread

├── Vision Thread

├── Control Thread

├── MAVLink RX Thread

└── WebUI / Preview path

\`\`\`

Do NOT migrate to multiprocessing simply because Python has a GIL.

OpenCV, GStreamer, codecs, and DNN operations are largely native workloads, and threading avoids unnecessary IPC, pickle, frame copies, and deployment complexity.

Only consider multiprocessing later if profiling proves a real Python/GIL bottleneck.

\---

\### 15.22 Worker Heartbeats and Supervision

Track monotonic worker heartbeat timestamps:

\`\`\`text

capture\_worker\_heartbeat\_ns

vision\_worker\_heartbeat\_ns

control\_worker\_heartbeat\_ns

mavlink\_rx\_worker\_heartbeat\_ns

\`\`\`

The runtime owner must detect stalled critical workers.

Keep supervision simple and in-process.

Failure behavior must preserve current safety:

\`\`\`text

Capture stall → frame stale → stale-video safety

Vision stall → tracking result stale → target/vision safety

MAVLink RX stall → heartbeat stale → release control / abort locally

Control failure → active command output stops → runtime fault

\`\`\`

Preserve \`RC\_OVERRIDE\_TIME\` as autopilot-side protection if Vulture-X stops sending overrides.

\---

\### 15.23 Runtime Owner

Create or evolve one runtime coordinator, conceptually:

\`\`\`python

class TrackingRuntime:

    ...

\`\`\`

Responsibilities may include constructing buffers, starting/stopping workers, coordinating clean shutdown, owning shared operational runtime state, coordinating MAVLink command authority, collecting worker health, and exposing snapshots to WebUI.

It must NOT duplicate tracker algorithms, guidance equations, or HUD drawing logic.

Long term \`tools/sitl\_track\_target.py\` should become more launcher-like, but do not rewrite everything only to make the file small.

\---

\### 15.24 Clean Shutdown Must Be Centralized

Shutdown/release behavior must be idempotent.

Preserve safe release on:

\`\`\`text

normal stop

SIGINT

SIGTERM

target loss

video stale

MAVLink timeout

low altitude

persistent low airspeed

disarm

pilot mode takeover

worker fault

internal exception

\`\`\`

For fixed-wing when override is active:

\`\`\`text

release RC override

control\_authority = false

\`\`\`

Do not let several threads issue contradictory mode commands.

\---

\### 15.25 Runtime Metrics

Extend the existing \`RuntimeMetrics\` where practical.

Measure:

\`\`\`text

capture\_ms

vision\_ms

guidance\_ms

mavlink\_tx\_ms

capture\_fps

vision\_fps

control\_hz

mavlink\_rx\_hz

mavlink\_tx\_hz

frame\_age\_ms

tracking\_result\_age\_ms

command\_age\_ms

control\_period\_ms

control\_jitter\_ms

dropped\_frames

deadline\_misses

tracker\_failures

\`\`\`

Where practical report mean, P50, P95, P99, and max.

Do not spam hot-loop logs. Publish summaries around 1-2 Hz.

Operational HUD remains minimal. Diagnostic HUD/WebUI may expose engineering metrics.

\---

\### 15.26 Optional Embedded System Metrics

If simple and portable on Linux, add optional diagnostics for:

\`\`\`text

CPU usage

RAM usage

temperature

CPU frequency

\`\`\`

Failure to obtain them must not affect tracking. Do not add a heavy monitoring framework.

\---

\### 15.27 Dependency Policy

Keep the runtime based on current lightweight dependencies:

\`\`\`text

Python

OpenCV

NumPy

GStreamer

pymavlink

\`\`\`

Do NOT add ROS, PyTorch, TensorFlow, Ultralytics, or a new GUI framework as part of this refactor.

Do NOT migrate to C++.

Optimize only after measuring real bottlenecks.

\---

\### 15.28 Suggested Runtime Structure

Use current repository structure where practical.

Possible direction:

\`\`\`text

src/vulture\_x/runtime/

├── buffers.py

├── capture.py

├── scheduler.py

├── workers.py

├── metrics.py

└── state.py

\`\`\`

and if appropriate:

\`\`\`text

src/vulture\_x/vehicle/mavlink\_runtime.py

\`\`\`

This is a suggestion, not a requirement. Avoid unnecessary file proliferation.

\---

\### 15.29 Regression Tests

Add tests for at least:

\`\`\`text

LatestValue overwrite behavior

LatestFrame sequence behavior

Capture Worker startup/shutdown

Vision Worker startup/shutdown

MAVLink RX Worker snapshot update

Control Worker startup/shutdown

Vision Worker skips obsolete/intermediate frames

worker heartbeat updates

worker stall/fault handling

fixed-deadline control scheduling

control overrun/deadline miss accounting

stale tracking result handling

clean shutdown

legacy file capture compatibility

demand JSON compatibility

WebUI compatibility

\`\`\`

Use fake clocks where practical. Avoid real sleeps in scheduler/state unit tests.

\---

\### 15.30 Mandatory Controller Regression

Create regression coverage showing that equivalent inputs produce equivalent fixed-wing commands before and after the runtime refactor.

Verify:

\`\`\`text

roll command

pitch command

throttle command

roll PWM

pitch PWM

throttle PWM

\`\`\`

The runtime refactor must not change controller behavior.

\---

\### 15.31 Mandatory Safety Regression

Verify that the refactor does not weaken target lost, video stale, MAVLink heartbeat timeout, low altitude, low airspeed, disarm, pilot takeover, or internal exception behavior.

Pilot takeover regression is critical:

\`\`\`text

active tracking

\+

actual mode changes away from FBWA

→ release RC override

→ abort tracking

→ leave new mode unchanged

→ do not reassert FBWA

→ do not force AUTO

\`\`\`

\---

\### 15.32 Preserve Existing Aircraft Parameter Logic

Do not implement Aircraft Compatibility yet.

Preserve current parameter reading/adaptation documented in sections 5 and 6, including current roll/pitch limits, airspeed limits, throttle limits, RCMAP, RC calibration, reversal, override validation, and MIS\_RESTART warning behavior.

\---

\### 15.33 No AI Expansion During Runtime Refactor

Do NOT implement yet:

\`\`\`text

generic AI Object mode

new object detector

TargetAssociator

AI reacquisition

ReID

Model Registry

Tracking Profiles

\`\`\`

Existing NanoTrack and YuNet behavior must continue working.

\---

\### 15.34 Runtime Benchmark

Add or extend a benchmark mode/tool that can compare:

\`\`\`text

legacy file/JPEG capture

vs

appsink in-memory capture

\`\`\`

Measure where possible:

\`\`\`text

capture FPS

vision FPS

control Hz

vision latency P50/P95/P99

control jitter P50/P95/P99

frame age P50/P95/P99

tracking result age P50/P95/P99

dropped frames

deadline misses

CPU

RAM

\`\`\`

Clearly state tested hardware. Do not present desktop benchmark numbers as Orange Pi results.

Engineering goals, not hard pass/fail requirements:

\`\`\`text

camera               30 FPS when source supports it

NanoTrack vision     target >=25 Hz

control              30 Hz

MAVLink TX           20-30 Hz, preferably 30

control jitter P95   target <3 ms

latest-only buffering

\`\`\`

Report actual measured results even when they miss the goals.

\---

\### 15.35 Validation Commands

Run:

\`\`\`bash

pytest

ruff check .

mypy

python -m compileall tools src tests

vulture-x --config configs/default.yaml --check-config

\`\`\`

Also inspect:

\`\`\`bash

git diff --check

git diff --stat

git diff

\`\`\`

If the environment supports the appsink backend, perform a non-flight smoke test.

Do not install unrelated dependencies only to force validation green. Do not commit unless explicitly requested.

\---

\### 15.36 Completion Criteria

The Runtime Refactor is complete when all of these are true:

\`\`\`text

appsink/in-memory capture exists

legacy file capture still works as fallback

latest-frame-only buffering exists

Capture Worker exists

Vision Worker exists

LatestTrackingResult exists

MAVLink RX Worker exists

LatestVehicleState exists

Control Worker runs with fixed-deadline 30 Hz scheduling

WebUI/HUD is outside the flight-critical vision/control path

runtime worker health is observable

runtime timing/freshness metrics are observable

clean safe shutdown is preserved

fixed-wing controller output regression passes

fixed-wing operational safety regression passes

quad path remains functional

existing WebUI/API compatibility remains functional



preferred one-process runtime exists or any remaining subprocess boundary is explicitly documented as transitional

no complex IPC was introduced merely to preserve the old subprocess architecture

GStreamer/appsink capability was checked on the actual machine before backend selection

runtime refactor was implemented incrementally rather than as a big-bang rewrite

new WebUI preview path renders HUD from the in-memory/native frame and performs only one normal browser JPEG encode

native video aspect ratio is preserved without unnecessary server-side upscaling\`\`\`

\---

\*\*## 16. Roadmap After Runtime Refactor\*\*

After this task is complete, STOP.

The remaining tracking roadmap stays limited to THREE phases:

\`\`\`text

PHASE 1

Tracking Core + Robust Single Target

PHASE 2

AI Acquisition + Reacquisition

PHASE 3

Model Platform + Validation

\`\`\`

These are future tasks and require explicit user authorization.

Future Phase 1 includes canonical TrackingBackend, DetectorBackend interface, TrackingObservation, TargetTrack, TrackingStateManager, TrackingResult V2, \`command\_usable\`, freshness contract, time-based TRACK/DEGRADED/LOST, manual tracking robustness, and optional lightweight Kalman/motion estimation.

Future Phase 2 includes YuNet through DetectorBackend, generic OpenCV DNN ONNX detector, AI Object mode, operator candidate selection, TargetAssociator, detector + TrackerNano hybrid, reacquisition, identity protection, and adaptive detector scheduling.

Future Phase 3 includes Model Registry, model metadata, SHA256 validation, runtime readiness, Tracking Profiles, AI Models WebUI, evaluation dataset, benchmark tooling, and tracking regression metrics.

Aircraft Compatibility & Adaptation Layer remains a separate major task.

Recommended project order:

\`\`\`text

Runtime Refactor

    ↓

STOP / test / benchmark

    ↓

Tracking Phase 1

    ↓

Tracking Phase 2

    ↓

Tracking Phase 3

    ↓

Aircraft Compatibility

\`\`\`

Aircraft Compatibility may be scheduled earlier only by explicit user instruction.

\---

\*\*## 17. Final Response\*\*

After completing ONLY the Runtime Architecture Refactor, respond with:

\`\`\`text

Vulture-X Runtime Refactor completed.

Implemented:

\- appsink in-memory capture:

\- legacy capture fallback:

\- LatestFrame:

\- Capture Worker:

\- Vision Worker:

\- LatestTrackingResult:

\- MAVLink RX Worker:

\- LatestVehicleState:

\- fixed 30 Hz Control Worker:

\- Preview/HUD separation:

\- worker health:

\- runtime metrics:

\- one-process runtime status / transitional subprocess boundary:

\- GStreamer capability check:

\- preview single-encode path:

\- native aspect-ratio handling:

\- incremental migration steps completed:

Behavior preserved:

\- fixed-wing guidance:

\- tuned gains:

\- RC override:

\- FBWA flow:

\- pilot takeover:

\- normal STOP:

\- quad path:

\- tracking modes:

\- WebUI/API:

Performance:

\- legacy backend:

\- appsink backend:

\- capture FPS:

\- vision FPS:

\- control Hz:

\- control jitter P95:

\- frame age P95:

\- tracking result age P95:

\- CPU:

\- RAM:

Tests:

\- pytest:

\- ruff:

\- mypy:

\- compileall:

\- config validation:

Files changed:

\- ...

Known remaining limitations:

\- ...

Next work:

\- Tracking Phase 1

\- Tracking Phase 2

\- Tracking Phase 3

\- Aircraft Compatibility Layer

No later task was started.

\`\`\`

Do not start another task automatically.