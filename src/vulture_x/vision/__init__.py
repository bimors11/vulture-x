"""Video-source and visual-tracking components."""

from vulture_x.vision.synthetic_video import SyntheticVideoSource
from vulture_x.vision.tracker import OpenCvTracker
from vulture_x.vision.video_source import BoundingBox, VideoFrame, VideoSource

__all__ = [
    "BoundingBox",
    "OpenCvTracker",
    "SyntheticVideoSource",
    "VideoFrame",
    "VideoSource",
]

