"""MAVLink transport contracts and mock vehicle implementation."""

from vulture_x.vehicle.mavlink_client import MockMavlinkVehicle
from vulture_x.vehicle.transport import MavlinkTransport

__all__ = ["MavlinkTransport", "MockMavlinkVehicle"]

