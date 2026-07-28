"""Deterministic synthetic video source for tracking tests."""

import asyncio
from time import monotonic

import cv2
import numpy as np

from vulture_x.config import VisionConfig
from vulture_x.vision.video_source import BoundingBox, ImageFrame, VideoFrame


class SyntheticVideoSource:
    def __init__(self, config: VisionConfig, *, pace: bool = True) -> None:
        self._config = config
        self._pace = pace
        self._opened = False
        self._sequence = 0
        self._rng = np.random.default_rng(config.random_seed)
        self._x = float(config.width - config.target_width_px) * 0.2
        self._y = float(config.height - config.target_height_px) * 0.35
        self._velocity_x = config.target_speed_x_px_s
        self._velocity_y = config.target_speed_y_px_s
        self._current_bbox: BoundingBox | None = None

    @property
    def current_target_bbox(self) -> BoundingBox | None:
        return self._current_bbox

    async def open(self) -> None:
        if self._opened:
            raise RuntimeError("synthetic video source is already open")
        self._opened = True

    async def read(self) -> VideoFrame:
        if not self._opened:
            raise RuntimeError("synthetic video source is not open")
        if self._pace:
            await asyncio.sleep((1.0 / self._config.fps) + self._config.frame_delay_s)

        self._apply_scenario_events()
        self._advance_target()
        image = np.empty(
            (self._config.height, self._config.width, 3),
            dtype=np.uint8,
        )
        image[:, :] = self._config.background_bgr

        if self._target_visible():
            bbox = (
                self._x,
                self._y,
                float(self._config.target_width_px),
                float(self._config.target_height_px),
            )
            self._draw_target(image, bbox)
            self._current_bbox = bbox
        else:
            self._current_bbox = None

        if self._config.noise_stddev > 0:
            noise = self._rng.normal(0, self._config.noise_stddev, image.shape)
            image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        frame = VideoFrame(monotonic(), self._sequence, image)
        self._sequence += 1
        return frame

    async def close(self) -> None:
        self._opened = False
        self._current_bbox = None

    def _target_visible(self) -> bool:
        start = self._config.disappear_after_frame
        if start is None:
            return True
        return not start <= self._sequence < start + self._config.disappear_duration_frames

    def _apply_scenario_events(self) -> None:
        if self._config.sudden_move_frame == self._sequence:
            max_x = self._config.width - self._config.target_width_px
            max_y = self._config.height - self._config.target_height_px
            self._x = float(self._rng.uniform(0, max_x))
            self._y = float(self._rng.uniform(0, max_y))

    def _advance_target(self) -> None:
        delta_t = 1.0 / self._config.fps
        self._x += self._velocity_x * delta_t
        self._y += self._velocity_y * delta_t
        max_x = float(self._config.width - self._config.target_width_px)
        max_y = float(self._config.height - self._config.target_height_px)
        if self._x <= 0 or self._x >= max_x:
            self._x = min(max(self._x, 0.0), max_x)
            self._velocity_x *= -1
        if self._y <= 0 or self._y >= max_y:
            self._y = min(max(self._y, 0.0), max_y)
            self._velocity_y *= -1

    def _draw_target(self, image: ImageFrame, bbox: BoundingBox) -> None:
        x, y, width, height = (round(value) for value in bbox)
        color = self._config.target_bgr
        if self._config.target_shape == "circle":
            center = (x + width // 2, y + height // 2)
            cv2.circle(image, center, min(width, height) // 2, color, thickness=-1)
        else:
            cv2.rectangle(image, (x, y), (x + width, y + height), color, thickness=-1)
