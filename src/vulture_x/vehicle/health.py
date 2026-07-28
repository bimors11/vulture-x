"""Vehicle health guard contract."""

from vulture_x.enums import SafetyReason, SafetyStatus
from vulture_x.models import SafetyResult, VehicleState


def evaluate_navigation_health(vehicle: VehicleState) -> SafetyResult:
    if not vehicle.navigation_healthy or not vehicle.gps_healthy:
        return SafetyResult(
            SafetyStatus.ABORT_REQUIRED,
            SafetyReason.VEHICLE_NAVIGATION_UNHEALTHY,
            {
                "navigation_healthy": vehicle.navigation_healthy,
                "gps_healthy": vehicle.gps_healthy,
            },
        )
    return SafetyResult(SafetyStatus.NORMAL, SafetyReason.NONE, {})

