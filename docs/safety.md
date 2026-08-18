# Safety

Vulture-X is for controlled visual tracking and non-contact testing. It must not
implement collision, terminal attack, RF interference, spoofing, jamming,
payload deployment, or autonomous engagement behavior.

The fixed-wing helper uses ArduPlane `FBWA` with bounded
`RC_CHANNELS_OVERRIDE`. It never auto-arms and never performs hardware takeoff.
The WebUI takeoff action is simulator-only.

Fixed-wing steering releases RC override and stops without switching flight
modes or commanding RTL/LOITER when any of these conditions occurs:

- target loss persists beyond `plane_loss_hold_s`;
- video frames become stale beyond `max_frame_age_ms`;
- MAVLink heartbeat becomes stale;
- the aircraft disarms;
- the aircraft leaves `FBWA`;
- valid altitude telemetry is below `min_tracking_alt_m`;
- valid airspeed stays below the aircraft minimum for the configured persistence
  period.

Image bounding-box size is only apparent image size. It is not a validated
physical range source and cannot prove minimum separation.
