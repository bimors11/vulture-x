"""Video-source and visual-tracking components."""

from vulture_x.vision.synthetic_video import SyntheticVideoSource
from vulture_x.vision.tracker import (
    InitialTargetSelection,
    OpenCvTracker,
    TargetDetection,
    choose_initial_bbox_from_point,
)
from vulture_x.vision.video_source import BoundingBox, VideoFrame, VideoSource

__all__ = [
    "BoundingBox",
    "InitialTargetSelection",
    "OpenCvTracker",
    "SyntheticVideoSource",
    "TargetDetection",
    "VideoFrame",
    "VideoSource",
    "choose_initial_bbox_from_point",
]
