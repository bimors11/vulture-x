"""Video-source and visual-tracking components."""

from vulture_x.vision.synthetic_video import SyntheticVideoSource
from vulture_x.vision.tracker import (
    InitialTargetSelection,
    NanoTracker,
    OpenCvTracker,
    TargetDetection,
    TrackingObservation,
    choose_initial_bbox_from_point,
)
from vulture_x.vision.video_source import BoundingBox, VideoFrame, VideoSource

__all__ = [
    "BoundingBox",
    "InitialTargetSelection",
    "NanoTracker",
    "OpenCvTracker",
    "SyntheticVideoSource",
    "TargetDetection",
    "TrackingObservation",
    "VideoFrame",
    "VideoSource",
    "choose_initial_bbox_from_point",
]
