# Runtime Architecture

The runtime refactor introduces in-process latest-value primitives and worker
classes under `src/vulture_x/runtime/`. The new path is designed around
dropping stale intermediate frames instead of queueing a backlog:

```text
FrameSource -> CaptureWorker -> LatestFrame
LatestFrame -> VisionWorker -> LatestTrackingResult
MAVLink RX Worker -> LatestVehicleState
LatestFrame + LatestTrackingResult + LatestVehicleState -> ControlWorker
Runtime snapshots -> Preview/HUD
```

Current machine capability check:

- OpenCV `4.11.0` reports `GStreamer: NO`.
- System GStreamer is available as `gst-launch-1.0 1.24.2`.
- `gst-inspect-1.0 appsink` reports the `appsink` element from
  `gst-plugins-base`.
- Python GI/GStreamer bindings are available.

Because OpenCV's GStreamer backend is unavailable on this machine, the appsink
backend uses Python GStreamer bindings directly. The legacy file/JPEG directory
reader remains available as `FileFrameSource` for compatibility and debugging.

The new preview helper renders HUD overlays onto the latest native-size
in-memory frame and then performs one browser JPEG encode. It does not decode a
previous preview JPEG simply to draw the HUD.

The WebUI plane START TRACKING path now creates an in-process
`RuntimeSteeringSession` by default. That session opens MAVLink, validates the
plane, reads the existing response parameters, constructs `TrackingRuntime`, and
uses these runtime workers for live fixed-wing tracking:

```text
GstAppSinkFrameSource/FileFrameSource -> CaptureWorker -> LatestFrame
LatestFrame -> VisionWorker -> LatestTrackingResult
MavlinkTelemetryReceiver -> MAVLink RX Worker -> LatestVehicleState
LatestTrackingResult + LatestVehicleState -> 30 Hz ControlWorker -> RC override
```

The Control Worker is the only active owner of fixed-wing tracking
`RC_CHANNELS_OVERRIDE` transmission in the new plane path. Capture, vision, HUD,
and preview code do not send tracking commands. Pilot takeover remains a
failsafe: when active tracking sees a mode other than `FBWA`, the runtime
releases RC override and does not reassert `FBWA` or command `AUTO`.

The old WebUI-to-`tools/sitl_track_target.py` subprocess path remains available
as a transitional compatibility path by setting `VULTURE_X_LEGACY_STEERING=1`;
quad steering also remains on the legacy helper path. The legacy path is no
longer the default fixed-wing WebUI tracking path.

`tools/runtime_benchmark.py` is a non-flight smoke/benchmark helper. It does not
open MAVLink and does not send commands. It can measure the legacy file source
or attempt to construct and read from the appsink sources:

```bash
python tools/runtime_benchmark.py --backend file --camera-dir logs/ui/camera --duration-s 5
python tools/runtime_benchmark.py --backend appsink-udp --duration-s 5
python tools/runtime_benchmark.py --backend appsink-rtsp --rtsp-url rtsp://HOST/PATH --duration-s 5
```

The runtime foundation does not change fixed-wing guidance equations, tuned
gains, FBWA mode semantics, RC override mapping, quad `GUIDED` body-velocity
behavior, or the existing safety thresholds.
