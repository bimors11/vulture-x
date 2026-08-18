import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import cv2
import numpy as np


def load_camera_target_module() -> ModuleType:
    tools_dir = Path("tools").resolve()
    if str(tools_dir) not in sys.path:
        sys.path.insert(0, str(tools_dir))
    spec = importlib.util.spec_from_file_location(
        "track_camera_target_test_module",
        tools_dir / "track_camera_target.py",
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def draw_banner(frame: np.ndarray, top_left: tuple[int, int], size: int) -> None:
    x, y = top_left
    cv2.rectangle(frame, (x, y), (x + size, y + size), (210, 0, 255), -1)
    center = (x + size // 2, y + size // 2)
    cv2.line(frame, (x + 8, center[1]), (x + size - 8, center[1]), (10, 10, 10), 4)
    cv2.line(frame, (center[0], y + 8), (center[0], y + size - 8), (10, 10, 10), 4)
    cv2.line(frame, (x + 10, y + 10), (x + size - 10, y + size - 10), (10, 10, 10), 3)
    cv2.line(frame, (x + size - 10, y + 10), (x + 10, y + size - 10), (10, 10, 10), 3)


def test_banner_crosshair_is_detected_as_target() -> None:
    module = load_camera_target_module()
    frame = np.zeros((160, 220, 3), dtype=np.uint8)

    draw_banner(frame, (70, 44), 76)

    bbox = module.detect_red_target(frame, min_area=80.0)

    assert bbox is not None
    x, y, width, height = bbox
    assert x <= 72
    assert y <= 46
    assert x + width >= 144
    assert y + height >= 118


def test_plain_magenta_patch_without_crosshair_has_no_target() -> None:
    module = load_camera_target_module()
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    frame[20:28, 40:48] = (210, 0, 255)

    assert module.detect_red_target(frame, min_area=25.0) is None


def test_small_magenta_target_blob_is_detected() -> None:
    module = load_camera_target_module()
    frame = np.zeros((220, 300, 3), dtype=np.uint8)
    cv2.rectangle(frame, (142, 130), (158, 154), (210, 0, 245), -1)
    cv2.line(frame, (150, 133), (150, 151), (20, 20, 20), 2)

    bbox = module.detect_red_target(frame, min_area=25.0)

    assert bbox is not None
    x, y, width, height = bbox
    assert x <= 142
    assert y <= 130
    assert x + width >= 158
    assert y + height >= 154


def test_wide_ground_colored_region_is_not_target() -> None:
    module = load_camera_target_module()
    frame = np.zeros((220, 300, 3), dtype=np.uint8)
    frame[150:220, :] = (120, 140, 170)

    assert module.detect_red_target(frame, min_area=25.0) is None


def test_small_far_banner_is_detected() -> None:
    module = load_camera_target_module()
    frame = np.zeros((120, 160, 3), dtype=np.uint8)

    draw_banner(frame, (70, 50), 18)

    bbox = module.detect_red_target(frame, min_area=25.0)

    assert bbox is not None


def test_dim_magenta_banner_is_detected() -> None:
    module = load_camera_target_module()
    frame = np.zeros((160, 220, 3), dtype=np.uint8)
    x, y, size = 72, 44, 70
    cv2.rectangle(frame, (x, y), (x + size, y + size), (145, 25, 170), -1)
    center = (x + size // 2, y + size // 2)
    cv2.line(frame, (x + 8, center[1]), (x + size - 8, center[1]), (20, 20, 20), 4)
    cv2.line(frame, (center[0], y + 8), (center[0], y + size - 8), (20, 20, 20), 4)

    assert module.detect_red_target(frame, min_area=80.0) is not None


def test_partially_visible_edge_banner_is_rejected() -> None:
    module = load_camera_target_module()
    frame = np.zeros((160, 220, 3), dtype=np.uint8)

    draw_banner(frame, (190, 20), 70)

    assert module.detect_red_target(frame, min_area=25.0) is None


def test_partially_visible_edge_colored_target_is_detected() -> None:
    module = load_camera_target_module()
    frame = np.zeros((180, 260, 3), dtype=np.uint8)
    cv2.rectangle(frame, (0, 58), (38, 122), (0, 0, 230), -1)

    bbox = module.detect_colored_target(frame, min_area=80.0)

    assert bbox is not None
    x, y, width, height = bbox
    assert x == 0
    assert y <= 58
    assert x + width >= 38
    assert y + height >= 122
