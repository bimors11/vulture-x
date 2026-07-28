"""Strict YAML configuration loading and cross-field safety validation."""

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class ProjectConfig(StrictModel):
    name: Literal["vulture-x"] = "vulture-x"
    environment: Literal["development", "sitl", "hil", "flight_test"] = "development"


class VehicleConfig(StrictModel):
    connection: str
    source_system: int = Field(default=191, ge=1, le=255)
    source_component: int = Field(default=191, ge=1, le=255)
    expected_autopilot: Literal["ARDUPILOTMEGA"] = "ARDUPILOTMEGA"
    expected_vehicle_type: Literal["QUADROTOR"] = "QUADROTOR"
    guided_mode: Literal["GUIDED"] = "GUIDED"
    hold_mode: Literal["LOITER"] = "LOITER"
    recovery_mode: Literal["RTL", "LAND"] = "RTL"
    takeoff_altitude_m: float = Field(default=10.0, gt=0, le=120)

    @model_validator(mode="after")
    def validate_connection(self) -> "VehicleConfig":
        parts = self.connection.split(":")
        valid = False
        if len(parts) == 3 and parts[0] in {"udpin", "udpout"}:
            try:
                valid = bool(parts[1]) and 1 <= int(parts[2]) <= 65535
            except ValueError:
                valid = False
        elif len(parts) == 3 and parts[0] == "serial":
            try:
                valid = bool(parts[1]) and int(parts[2]) > 0
            except ValueError:
                valid = False
        if not valid:
            raise ValueError(
                "connection must be udpin:HOST:PORT, udpout:HOST:PORT, "
                "or serial:DEVICE:BAUD"
            )
        return self


class VisionConfig(StrictModel):
    source: Literal["synthetic"] = "synthetic"
    width: int = Field(default=640, ge=64, le=7680)
    height: int = Field(default=480, ge=64, le=4320)
    fps: float = Field(default=30.0, gt=0, le=240)
    tracker: Literal["CSRT", "KCF"] = "CSRT"
    show_window: bool = False
    random_seed: int = 42
    target_shape: Literal["rectangle", "circle"] = "rectangle"
    target_width_px: int = Field(default=72, gt=2)
    target_height_px: int = Field(default=54, gt=2)
    target_speed_x_px_s: float = 90.0
    target_speed_y_px_s: float = 45.0
    background_bgr: tuple[int, int, int] = (24, 24, 24)
    target_bgr: tuple[int, int, int] = (40, 220, 255)
    noise_stddev: float = Field(default=0.0, ge=0, le=100)
    disappear_after_frame: int | None = Field(default=None, ge=0)
    disappear_duration_frames: int = Field(default=0, ge=0)
    sudden_move_frame: int | None = Field(default=None, ge=0)
    frame_delay_s: float = Field(default=0.0, ge=0, le=5)

    @model_validator(mode="after")
    def validate_target_geometry(self) -> "VisionConfig":
        if self.target_width_px >= self.width or self.target_height_px >= self.height:
            raise ValueError("synthetic target dimensions must be smaller than the video frame")
        for color in (*self.background_bgr, *self.target_bgr):
            if not 0 <= color <= 255:
                raise ValueError("BGR color channels must be in [0, 255]")
        return self


class GuidanceConfig(StrictModel):
    update_rate_hz: float = Field(default=10.0, gt=0)
    max_forward_speed_mps: float = Field(default=5.0, gt=0)
    max_lateral_speed_mps: float = Field(default=3.0, gt=0)
    max_vertical_speed_mps: float = Field(default=2.0, gt=0)
    max_acceleration_mps2: float = Field(default=1.5, gt=0)
    max_yaw_rate_deg_s: float = Field(default=30.0, gt=0, le=90)
    horizontal_deadband: float = Field(default=0.05, ge=0, lt=1)
    vertical_deadband: float = Field(default=0.05, ge=0, lt=1)
    target_size_setpoint: float = Field(default=0.12, gt=0, lt=1)


class SafetyConfig(StrictModel):
    minimum_tracking_confidence: float = Field(default=0.6, ge=0, le=1)
    tracking_warning_timeout_s: float = Field(default=0.3, ge=0)
    tracking_abort_timeout_s: float = Field(default=1.0, gt=0)
    mavlink_heartbeat_timeout_s: float = Field(default=2.0, gt=0)
    command_expiration_s: float = Field(default=0.25, gt=0)
    minimum_separation_m: float = Field(default=10.0, gt=0)
    maximum_test_speed_mps: float = Field(default=5.0, gt=0)

    @model_validator(mode="after")
    def validate_timeout_order(self) -> "SafetyConfig":
        if self.tracking_warning_timeout_s >= self.tracking_abort_timeout_s:
            raise ValueError("tracking warning timeout must be below tracking abort timeout")
        if self.command_expiration_s >= self.mavlink_heartbeat_timeout_s:
            raise ValueError("command expiration must be below MAVLink heartbeat timeout")
        return self


class LoggingConfig(StrictModel):
    level: Literal["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"] = "INFO"
    directory: Path = Path("./logs")
    event_jsonl: bool = True
    telemetry_csv: bool = True


class AppConfig(StrictModel):
    project: ProjectConfig
    vehicle: VehicleConfig
    vision: VisionConfig
    guidance: GuidanceConfig
    safety: SafetyConfig
    logging: LoggingConfig = LoggingConfig()

    @model_validator(mode="after")
    def validate_command_envelope(self) -> "AppConfig":
        if self.safety.maximum_test_speed_mps > self.guidance.max_forward_speed_mps:
            raise ValueError("maximum test speed must not exceed maximum forward speed")
        return self


def load_config(path: Path) -> AppConfig:
    """Load and validate a UTF-8 YAML configuration file."""
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"cannot read configuration {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML in configuration {path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError(f"configuration {path} must contain a YAML mapping")
    return AppConfig.model_validate(raw)
