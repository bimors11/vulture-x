#!/usr/bin/env python3
"""Verify or fetch local OpenCV DNN models used by Vulture-X vision."""

from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.request
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True, slots=True)
class VisionModel:
    name: str
    path: Path
    sha256: str
    url: str


MODELS = (
    VisionModel(
        name="nanotrack_backbone",
        path=REPO_ROOT / "models" / "opencv" / "nanotrack" / "nanotrack_backbone_sim.onnx",
        sha256="530bdd0cd00f19afab79a863e71ba71e3312395a5dc9151af675082bdaaa2fc4",
        url=(
            "https://github.com/HonglinChu/SiamTrackers/raw/master/"
            "NanoTrack/models/nanotrackv2/nanotrack_backbone_sim.onnx"
        ),
    ),
    VisionModel(
        name="nanotrack_head",
        path=REPO_ROOT / "models" / "opencv" / "nanotrack" / "nanotrack_head_sim.onnx",
        sha256="0d8c0637be849f092cc7236cae02e55c8b9455ebe37ba50601d6115db4247cd9",
        url=(
            "https://github.com/HonglinChu/SiamTrackers/raw/master/"
            "NanoTrack/models/nanotrackv2/nanotrack_head_sim.onnx"
        ),
    ),
    VisionModel(
        name="yunet_face_detector",
        path=REPO_ROOT / "models" / "opencv" / "yunet" / "face_detection_yunet_2023mar.onnx",
        sha256="8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4",
        url=(
            "https://github.com/opencv/opencv_zoo/raw/main/"
            "models/face_detection_yunet/face_detection_yunet_2023mar.onnx"
        ),
    ),
)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as model_file:
        for chunk in iter(lambda: model_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download_model(model: VisionModel) -> None:
    model.path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = model.path.with_suffix(model.path.suffix + ".tmp")
    with urllib.request.urlopen(model.url, timeout=60) as response:
        tmp_path.write_bytes(response.read())
    actual = file_sha256(tmp_path)
    if actual != model.sha256:
        tmp_path.unlink(missing_ok=True)
        raise RuntimeError(
            f"model_hash_mismatch name={model.name} expected={model.sha256} actual={actual}"
        )
    tmp_path.replace(model.path)


def verify_model(model: VisionModel) -> tuple[bool, str]:
    if not model.path.exists():
        return False, f"missing path={model.path}"
    actual = file_sha256(model.path)
    if actual != model.sha256:
        return False, f"hash_mismatch path={model.path} actual={actual}"
    return True, f"ok path={model.path}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--download",
        action="store_true",
        help="Download missing or invalid model files, then verify SHA-256 hashes.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    failed = False
    for model in MODELS:
        ok, detail = verify_model(model)
        if not ok and args.download:
            print(f"vision_model_status=downloading name={model.name} reason={detail}")
            try:
                download_model(model)
            except (OSError, RuntimeError) as exc:
                print(f"vision_model_status=failed name={model.name} reason={exc}")
                failed = True
                continue
            ok, detail = verify_model(model)
        print(
            f"vision_model_status={'ok' if ok else 'failed'} name={model.name} {detail}"
        )
        failed = failed or not ok
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
