"""Strict YAML configuration loading and safety validation."""

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class ProjectConfig(StrictModel):
    name: Literal["vulture-x"] = "vulture-x"
    environment: Literal["sitl", "hil", "flight_test"] = "sitl"


class VehicleConfig(StrictModel):
    connection: str
    source_system: int = Field(default=191, ge=1, le=255)
    source_component: int = Field(default=191, ge=1, le=255)
    expected_autopilot: Literal["ARDUPILOTMEGA"] = "ARDUPILOTMEGA"
    expected_vehicle_type: Literal["QUADROTOR"] = "QUADROTOR"
    guided_mode: Literal["GUIDED"] = "GUIDED"
    recovery_mode: Literal["RTL", "LAND", "LOITER"] = "RTL"
    heartbeat_hz_min: float = Field(default=0.5, gt=0)

    @model_validator(mode="after")
    def validate_connection(self) -> "VehicleConfig":
        parts = self.connection.split(":")
        valid = False
        if len(parts) == 3 and parts[0] in {"udpin", "udpout"}:
            try:
                port = int(parts[2])
                valid = bool(parts[1]) and 1 <= port <= 65535
            except ValueError:
                valid = False
        elif len(parts) == 3 and parts[0] == "serial":
            try:
                baud = int(parts[2])
                valid = bool(parts[1]) and baud > 0
            except ValueError:
                valid = False
        if not valid:
            raise ValueError(
                "connection must be udpin:HOST:PORT, udpout:HOST:PORT, "
                "or serial:DEVICE:BAUD"
            )
        return self


class TargetConfig(StrictModel):
    source: Literal["cooperative_udp"] = "cooperative_udp"
    listen_host: str = "0.0.0.0"
    listen_port: int = Field(default=15550, ge=1, le=65535)
    expected_frame: Literal["WGS84_MSL"] = "WGS84_MSL"
    minimum_confidence: float = Field(default=0.8, ge=0, le=1)
    allowed_source_ids: tuple[str, ...] = ()


class GuidanceConfig(StrictModel):
    type: Literal["fixed_standoff", "pure_pursuit", "lead_pursuit"] = "lead_pursuit"
    update_rate_hz: float = Field(default=10.0, gt=0)
    preferred_standoff_m: float = Field(default=50.0, gt=0)
    lookahead_time_s: float = Field(default=2.0, gt=0)
    lookahead_time_min_s: float = Field(default=0.5, gt=0)
    lookahead_time_max_s: float = Field(default=5.0, gt=0)

    @model_validator(mode="after")
    def validate_lookahead(self) -> "GuidanceConfig":
        if not self.lookahead_time_min_s <= self.lookahead_time_s <= self.lookahead_time_max_s:
            raise ValueError("lookahead_time_s must be within its configured min/max range")
        return self


class SafetyConfig(StrictModel):
    min_horizontal_separation_m: float = Field(default=30.0, gt=0)
    min_vertical_separation_m: float = Field(default=15.0, gt=0)
    max_closure_rate_mps: float = Field(default=5.0, gt=0)
    max_groundspeed_mps: float = Field(default=15.0, gt=0)
    max_vertical_speed_mps: float = Field(default=3.0, gt=0)
    max_acceleration_mps2: float = Field(default=2.0, gt=0)
    max_yaw_rate_deg_s: float = Field(default=30.0, gt=0)
    target_warning_age_s: float = Field(default=0.5, ge=0)
    target_abort_age_s: float = Field(default=2.0, gt=0)
    vehicle_heartbeat_timeout_s: float = Field(default=2.0, gt=0)
    command_ack_timeout_s: float = Field(default=1.0, gt=0)
    mission_timeout_s: float = Field(default=600.0, gt=0)
    min_battery_remaining_pct: float = Field(default=35.0, ge=0, le=100)
    return_battery_remaining_pct: float = Field(default=45.0, ge=0, le=100)

    @model_validator(mode="after")
    def validate_threshold_order(self) -> "SafetyConfig":
        if self.target_warning_age_s > self.target_abort_age_s:
            raise ValueError("target warning age must not exceed target abort age")
        if self.return_battery_remaining_pct < self.min_battery_remaining_pct:
            raise ValueError("return battery threshold must not be below minimum battery threshold")
        return self


class LoggingConfig(StrictModel):
    level: Literal["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"] = "INFO"
    directory: Path = Path("./logs")
    telemetry_csv: bool = True
    event_jsonl: bool = True


class RatesConfig(StrictModel):
    vehicle_receive_hz: float = Field(default=20.0, gt=0)
    target_receive_hz: float = Field(default=10.0, gt=0)
    estimator_hz: float = Field(default=20.0, gt=0)
    guidance_hz: float = Field(default=10.0, gt=0)
    safety_hz: float = Field(default=20.0, gt=0)
    status_hz: float = Field(default=2.0, gt=0)


class AppConfig(StrictModel):
    project: ProjectConfig
    vehicle: VehicleConfig
    target: TargetConfig
    guidance: GuidanceConfig
    safety: SafetyConfig
    logging: LoggingConfig = LoggingConfig()
    rates: RatesConfig = RatesConfig()

    @model_validator(mode="after")
    def validate_safety_geometry(self) -> "AppConfig":
        if self.safety.min_horizontal_separation_m > self.guidance.preferred_standoff_m:
            raise ValueError(
                "minimum horizontal separation must not exceed preferred stand-off distance"
            )
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

