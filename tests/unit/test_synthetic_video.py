import asyncio

import numpy as np

from vulture_x.config import VisionConfig
from vulture_x.vision.synthetic_video import SyntheticVideoSource


def test_synthetic_source_is_deterministic() -> None:
    async def scenario() -> None:
        config = VisionConfig(noise_stddev=3.0, random_seed=7)
        first = SyntheticVideoSource(config, pace=False)
        second = SyntheticVideoSource(config, pace=False)
        await first.open()
        await second.open()
        for _ in range(3):
            frame_a = await first.read()
            frame_b = await second.read()
            assert np.array_equal(frame_a.image, frame_b.image)
            assert first.current_target_bbox == second.current_target_bbox
        await first.close()
        await second.close()

    asyncio.run(scenario())


def test_synthetic_source_can_hide_target() -> None:
    async def scenario() -> None:
        source = SyntheticVideoSource(
            VisionConfig(disappear_after_frame=1, disappear_duration_frames=2),
            pace=False,
        )
        await source.open()
        await source.read()
        assert source.current_target_bbox is not None
        await source.read()
        assert source.current_target_bbox is None
        await source.read()
        assert source.current_target_bbox is None
        await source.read()
        assert source.current_target_bbox is not None
        await source.close()

    asyncio.run(scenario())

