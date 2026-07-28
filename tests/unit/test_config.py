from pathlib import Path

import pytest
from pydantic import ValidationError

from vulture_x.config import AppConfig, load_config

DEFAULT_CONFIG = Path("configs/default.yaml")


def test_default_configuration_loads() -> None:
    config = load_config(DEFAULT_CONFIG)
    assert config.project.name == "vulture-x"
    assert config.vehicle.expected_autopilot == "ARDUPILOTMEGA"
    assert config.safety.min_horizontal_separation_m == 30.0


def test_unknown_configuration_key_is_rejected() -> None:
    config = load_config(DEFAULT_CONFIG).model_dump(mode="python")
    config["safety"]["unsafe_override"] = True
    with pytest.raises(ValidationError, match="Extra inputs are not permitted"):
        AppConfig.model_validate(config)


@pytest.mark.parametrize(
    ("section", "field", "value", "message"),
    [
        ("safety", "max_groundspeed_mps", 0, "greater than 0"),
        (
            "safety",
            "target_warning_age_s",
            3.0,
            "warning age must not exceed",
        ),
        (
            "safety",
            "return_battery_remaining_pct",
            20.0,
            "return battery threshold",
        ),
        (
            "safety",
            "min_horizontal_separation_m",
            60.0,
            "must not exceed preferred",
        ),
    ],
)
def test_invalid_safety_configuration_is_rejected(
    section: str, field: str, value: object, message: str
) -> None:
    config = load_config(DEFAULT_CONFIG).model_dump(mode="python")
    config[section][field] = value
    with pytest.raises(ValidationError, match=message):
        AppConfig.model_validate(config)


@pytest.mark.parametrize(
    "connection",
    ["udp:localhost", "udpin::14550", "udpin:localhost:70000", "serial:/dev/ttyUSB0:nope"],
)
def test_invalid_connection_string_is_rejected(connection: str) -> None:
    config = load_config(DEFAULT_CONFIG).model_dump(mode="python")
    config["vehicle"]["connection"] = connection
    with pytest.raises(ValidationError, match="connection must be"):
        AppConfig.model_validate(config)
