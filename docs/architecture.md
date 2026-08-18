# Architecture

Vulture-X processes simulated or captured analog video on a Linux ground
station. Tracking outputs normalized image error and apparent target size.
Guidance converts those signals into short-lived commands. The quadcopter path
uses body-frame velocity and yaw-rate requests in ArduPilot `GUIDED`. The
fixed-wing helper path uses ArduPlane `FBWA` with bounded
`RC_CHANNELS_OVERRIDE` roll, pitch, throttle, and neutral yaw commands. Every
request passes through safety and limiter layers before the MAVLink boundary.

```text
VideoSource -> OpenCV tracker -> TrackingResult -> tracking safety
            -> image guidance -> command limiters -> command safety
            -> mission supervisor -> MAVLink vehicle boundary -> ArduPilot
```

The package-level first milestone still ends at a mock vehicle boundary. Local
tools now include SITL/manual MAVLink helpers for the working quad and
fixed-wing test flows; keep those helpers separate from package guidance code.

The future physical video path is analog camera/VTX, ground receiver, and USB
capture. The future control link may be SITL UDP, serial telemetry, or ELRS.
Neither tracking nor guidance may depend on which concrete source or transport
is selected.

Fixed-wing live tuning is mediated by `logs/ui/tracking_tuning.json`. The WebUI
writes the file atomically and the tracking process reloads it only when
`st_mtime_ns` changes, retaining the last valid values on invalid JSON or bad
fields.
