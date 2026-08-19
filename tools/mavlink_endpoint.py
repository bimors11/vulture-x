"""MAVLink endpoint parsing helpers shared by local SITL tools."""

from __future__ import annotations

from dataclasses import dataclass

from pymavlink import mavutil


@dataclass(frozen=True)
class MavlinkEndpoint:
    device: str
    baud: int = 115200


def parse_mavlink_endpoint(endpoint: str) -> MavlinkEndpoint:
    value = endpoint.strip()
    if not value:
        raise ValueError("empty_mavlink_endpoint")

    if value.startswith("serial:"):
        parts = value.split(":", 2)
        if len(parts) != 3 or not parts[1] or not parts[2]:
            raise ValueError("serial endpoint must use serial:DEVICE:BAUD")
        try:
            baud = int(parts[2])
        except ValueError as exc:
            raise ValueError("serial endpoint baud must be an integer") from exc
        if baud <= 0:
            raise ValueError("serial endpoint baud must be positive")
        return MavlinkEndpoint(parts[1], baud)

    allowed_prefixes = (
        "udpin:",
        "udpout:",
        "udpcl:",
        "udp:",
        "tcp:",
        "tcpin:",
        "udpbcast:",
        "mcast:",
    )
    if value.startswith(allowed_prefixes):
        if value.startswith("udpcl:"):
            # Mission Planner-style "udpcl" is a remote UDP client. pymavlink treats a
            # client socket as a non-binding udpout connection; binding to the remote IP
            # would attempt to assign the aircraft address as a local interface and raises
            # OSError 99. Preserve the aircraft endpoint while keeping the socket outbound.
            return MavlinkEndpoint("udpout:" + value.removeprefix("udpcl:"))
        return MavlinkEndpoint(value)

    if value.startswith("/dev/"):
        return MavlinkEndpoint(value)

    raise ValueError(
        "mavlink endpoint must be udpin:HOST:PORT, udpout:HOST:PORT, "
        "udpcl:HOST:PORT, tcp:HOST:PORT, or serial:DEVICE:BAUD"
    )


def open_mavlink_connection(
    endpoint: str,
    *,
    source_system: int,
    source_component: int,
    autoreconnect: bool = False,
) -> mavutil.mavfile:
    parsed = parse_mavlink_endpoint(endpoint)
    connection = mavutil.mavlink_connection(
        parsed.device,
        baud=parsed.baud,
        source_system=source_system,
        source_component=source_component,
        autoreconnect=autoreconnect,
    )
    if parsed.device.startswith("udpout:"):
        remote = parsed.device.removeprefix("udpout:")
        host, port_text = remote.rsplit(":", 1)
        try:
            connection.port.connect((host, int(port_text)))
        except OSError:
            # Some environments reject explicit connect() for outbound UDP sockets,
            # but keep the remote peer set so the socket still sends to the aircraft.
            connection.destination_addr = (host, int(port_text))
    return connection
