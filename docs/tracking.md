# Tracking

The synthetic source produces deterministic BGR frames using a configurable
seed, resolution, frame rate, target shape/size/velocity, background, Gaussian
noise, disappearance interval, sudden movement, and extra delay.

`OpenCvTracker` wraps OpenCV CSRT, KCF, or the `TEMPLATE` matcher adapted from
the fixed-wing visual-nav prototype. It supports explicit initialization,
update, loss reporting, bounding-box validation, and reset. Synthetic tests use
the generator's known initial bounding box; the SITL UI can provide manual ROI
selection for local simulation helpers.

The quad tracking path also includes the RDV-style point-to-ROI initialization
method. A click point can be resolved against externally supplied detections by
first choosing the smallest detection that contains the click, then the nearest
detection inside a bounded search radius, then a small clamped fallback square.
Selected detection boxes are padded before tracker initialization. This keeps
the RDV interaction pattern without adding RDV's DepthAI/YOLO runtime dependency
to the first-milestone package.

After a tracking loss, callers that have an independent detector can ask
`OpenCvTracker.relock_from_detections` to reinitialize from the highest
confidence detection. No hardware camera, YOLO model, or fixed-wing/plane
guidance path is added by this package-level support.

The `TEMPLATE` matcher searches near the last bounding box with OpenCV template
matching, rejects matches below a configured score, and slowly updates its
template after successful locks. It searches several nearby scales and blends
the resulting center and width/height so a manually selected small far-target
ROI can grow or shrink as apparent target size changes without command spikes.
It rejects implausible center jumps and area changes before updating the lock.
It uses the same local-search behavior as the prototype, with an added
low-variance path for solid-color synthetic markers.

The SITL helper's default detector mode uses the Gazebo tracking banner rather
than a small color blob. It searches for bright-magenta square-ish regions with a
dark center crosshair/X mark, then initializes the local template tracker from
that detected ROI. This makes the initial fixed-wing lock more stable while the
aircraft is moving and avoids selecting unrelated runway markings or plain
magenta patches. Banner candidates touching the image edge are rejected because
their cropped bbox center is not the real banner center. During banner tracking,
the helper revalidates with the detector on each frame and accepts only
plausible near-previous bbox transitions, which reduces jumps to unrelated
magenta regions.

For manual custom selections in SITL, the plane world includes dense visual-only
ground grass feature strips around the tracking area. These low static visuals
add corners and color variation for template/CSRT/KCF lock without changing
ground collision or vehicle dynamics.

The fixed-wing SITL helper applies a small image-error deadband before roll and
pitch correction, and its RC pitch mapping follows the ArduPlane convention that
lower pitch-channel PWM commands nose-down and higher PWM commands nose-up. This
helper remains simulation-only. Both banner tracking and custom ROI selection
share the same controller: far targets are acquired with softened roll/pitch
authority, while near targets keep enough pitch-down authority to dive toward
the selected center.

For fixed-wing SITL steering, the controller is explicitly centered on the
camera crosshair. It applies scheduled proportional gain to both roll and pitch:
`plane_centering_gain` while the target is far, blending toward
`plane_near_centering_gain` as the bounding box grows. A far-target control
scale keeps initial acquisition smooth but still leaves enough authority to keep
the target pulled toward the crosshair throughout the approach. Roll/pitch step
limits prevent one-frame command jumps. Near the target, the default taper is
released so the plane can continue to pitch down toward the target center
instead of floating above it. A small scheduled damping term opposes image-error
rate without masking the proportional correction, so the controller stays
assertive when a near banner or custom selection is still off center. If the
fixed-wing tracker briefly loses the
target, the helper holds the last roll/pitch/throttle command only for the
bounded `plane_loss_hold_s` window. After that timeout it releases RC override,
reports `tracking_failsafe=active reason=target_lost`, and stops steering so
the pilot or ArduPilot owns the aircraft again. Commands are bounded with
read-only
ArduPlane response parameters when available. At startup the helper requests
common roll, pitch, throttle, airspeed, L1, and servo PID/time-constant
parameters such as `ROLL_LIMIT_DEG`, `PTCH_LIM_MAX_DEG`,
`PTCH_LIM_MIN_DEG`, legacy `LIM_*` fallbacks, `TRIM_THROTTLE`,
`ARSPD_FBW_MIN/MAX`, `RCMAP_*`, `RC*_MIN/TRIM/MAX`, `RLL2SRV_*`, and
`PTCH2SRV_*`. These values are used only to adapt command limits, calibrated RC
mapping, and smoothing; the helper does not write parameters.

The browser UI keeps quad and plane steering controls in separate panels. Quad
mode exposes only body-velocity image-guidance fields. Plane mode exposes the
fixed-wing crosshair-centering, pitch, throttle, camera geometry, safety, and
response-model fields. Plane tuning is applied through
`logs/ui/tracking_tuning.json`; invalid live tuning updates are ignored and the
last valid values remain active.

Manual box and head/face tracking use OpenCV NanoTrack by default. Head
candidate acquisition uses OpenCV YuNet at a small input size, and the selected
candidate is then tracked by NanoTrack. The legacy template tracker is kept as
`--tracker-engine template` for diagnostics and A/B comparison. The required
ONNX files live in `models/opencv/`; runtime startup validates explicit paths
and does not download models implicitly. Use `python tools/ensure_vision_models.py`
to verify hashes, or `python tools/ensure_vision_models.py --download` to restore
missing model files on a second development machine.

NanoTrack exposes `getTrackingScore()`, and Vulture-X reports that value as
tracker confidence for Nano-backed manual/head tracking. OpenCV's classic
tracker API and the local template fallback do not provide a calibrated
confidence score; those paths still use an explicit detected/lost heuristic
rather than a probabilistic estimate.

Normalized errors use `-1` at the left/top edge, `0` at image center, and `+1`
at the right/bottom edge.

Machine-readable tracking failure reasons used by the SITL/UI helper path
include `missing_custom_selection_file`, `target_lost`, `stale_video`,
`mavlink_heartbeat_timeout`, `vehicle_disarmed`, `pilot_mode_change`,
`altitude_unknown`, `below_tracking_altitude`, `airspeed_unknown`, and
`low_airspeed`. The unresolved safety constraint remains unchanged: image size
and template lock do not prove physical range or separation.
