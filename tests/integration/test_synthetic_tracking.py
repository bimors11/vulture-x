import asyncio

from vulture_x.config import VisionConfig
from vulture_x.vision.synthetic_video import SyntheticVideoSource
from vulture_x.vision.tracker import OpenCvTracker


def test_synthetic_video_to_opencv_tracker() -> None:
    async def scenario() -> None:
        source = SyntheticVideoSource(VisionConfig(tracker="CSRT"), pace=False)
        await source.open()
        first = await source.read()
        bbox = source.current_target_bbox
        assert bbox is not None
        tracker = OpenCvTracker("CSRT")
        tracker.initialize(first, bbox)
        second = await source.read()
        result = tracker.update(second)
        assert result.detected
        assert result.confidence >= 0.6
        assert -1 <= result.horizontal_error <= 1
        await source.close()

    asyncio.run(scenario())

