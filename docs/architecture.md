# Architecture

Vulture-X processes simulated or captured analog video on a Linux ground
station. Tracking outputs normalized image error and apparent target size.
Guidance converts those signals into short-lived body-frame velocity and
yaw-rate requests. Every request passes through safety and limiter layers before
a transport-independent MAVLink boundary.

```text
VideoSource -> OpenCV tracker -> TrackingResult -> tracking safety
            -> image guidance -> command limiters -> command safety
            -> mission supervisor -> MAVLink vehicle boundary -> ArduPilot
```

The first milestone ends at a mock vehicle boundary. The mock records commands
for tests but never opens UDP or serial connections.

The future physical video path is analog camera/VTX, ground receiver, and USB
capture. The future control link may be SITL UDP, serial telemetry, or ELRS.
Neither tracking nor guidance may depend on which concrete source or transport
is selected.

