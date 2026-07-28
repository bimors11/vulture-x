"""Guidance controller interface."""

from typing import Protocol

from vulture_x.models import GuidanceCommand, TrackingResult


class GuidanceController(Protocol):
    def calculate(
        self,
        tracking: TrackingResult,
        *,
        now_monotonic_s: float,
    ) -> GuidanceCommand:
        """Convert a validated tracking result into an expiring command."""
        ...

