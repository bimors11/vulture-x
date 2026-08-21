"""Read-only MAVLink runtime telemetry adapter."""

from __future__ import annotations

import time
from typing import Any

from pymavlink import mavutil  # type: ignore[import-untyped]

from vulture_x.runtime.state import VehicleSnapshot


class MavlinkTelemetryReceiver:
    """Drain bounded MAVLink telemetry into immutable vehicle snapshots."""

    def __init__(self, connection: mavutil.mavfile) -> None:
        self._connection = connection
        self._mode: str | None = None
        self._armed: bool | None = None
        self._relative_altitude_m: float | None = None
        self._airspeed_mps: float | None = None
        self._heading_deg: float | None = None
        self._heartbeat_timestamp_ns: int | None = None

    def receive(self) -> VehicleSnapshot | None:
        updated = False
        deadline_ns = time.monotonic_ns() + 5_000_000
        while time.monotonic_ns() < deadline_ns:
            try:
                message = self._connection.recv_match(
                    type=["HEARTBEAT", "VFR_HUD", "GLOBAL_POSITION_INT"],
                    blocking=False,
                    timeout=0,
                )
            except (TypeError, OSError):
                break
            if message is None:
                break
            updated = True
            self._apply_message(message)
        if not updated:
            return None
        now_ns = time.monotonic_ns()
        return VehicleSnapshot(
            timestamp_ns=now_ns,
            heartbeat_timestamp_ns=self._heartbeat_timestamp_ns,
            mode=self._mode,
            armed=self._armed,
            relative_altitude_m=self._relative_altitude_m,
            airspeed_mps=self._airspeed_mps,
            heading_deg=self._heading_deg,
            connected=self._heartbeat_timestamp_ns is not None,
        )

    def _apply_message(self, message: Any) -> None:
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            self._heartbeat_timestamp_ns = time.monotonic_ns()
            self._mode = mavutil.mode_string_v10(message)
            base_mode = int(getattr(message, "base_mode", 0))
            self._armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif message_type == "VFR_HUD":
            self._heading_deg = float(message.heading)
            self._airspeed_mps = float(message.airspeed)
        elif message_type == "GLOBAL_POSITION_INT":
            self._relative_altitude_m = float(message.relative_alt) / 1000.0
