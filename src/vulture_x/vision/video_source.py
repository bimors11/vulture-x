"""Transport-independent video source contract."""

from dataclasses import dataclass
from typing import Protocol, TypeAlias

import numpy as np
from numpy.typing import NDArray

BoundingBox: TypeAlias = tuple[float, float, float, float]
ImageFrame: TypeAlias = NDArray[np.uint8]


@dataclass(frozen=True, slots=True)
class VideoFrame:
    timestamp_monotonic_s: float
    sequence: int
    image: ImageFrame


class VideoSource(Protocol):
    async def open(self) -> None:
        """Acquire source resources."""
        ...

    async def read(self) -> VideoFrame:
        """Return the next frame."""
        ...

    async def close(self) -> None:
        """Release source resources."""
        ...

