from pathlib import Path

import pytest
from pydantic import ValidationError

from vulture_x.config import AppConfig, load_config

DEFAULT_CONFIG = Path("configs/default.yaml")


def config_dict() -> dict[str, object]:
    return load_config(DEFAULT_CONFIG).model_dump(mode="python")


def test_default_and_sitl_configurations_load() -> None:
    default = load_config(DEFAULT_CONFIG)
    sitl = load_config(Path("configs/sitl.yaml"))
    assert default.project.name == "vulture-x"
    assert default.vision.source == "synthetic"
    assert sitl.project.environment == "sitl"


def test_unknown_configuration_key_is_rejected() -> None:
    raw = config_dict()
    safety = raw["safety"]
    assert isinstance(safety, dict)
    safety["unsafe_override"] = True
    with pytest.raises(ValidationError, match="Extra inputs are not permitted"):
        AppConfig.model_validate(raw)


@pytest.mark.parametrize(
    ("section", "field", "value", "message"),
    [
        ("guidance", "max_forward_speed_mps", 0, "greater than 0"),
        ("safety", "tracking_warning_timeout_s", 2.0, "warning timeout"),
        ("safety", "command_expiration_s", 3.0, "heartbeat timeout"),
        ("safety", "maximum_test_speed_mps", 6.0, "maximum test speed"),
        ("vision", "target_width_px", 700, "smaller than the video frame"),
        ("vision", "tracker", "MOSSE", "CSRT"),
    ],
)
def test_invalid_configuration_is_rejected(
    section: str,
    field: str,
    value: object,
    message: str,
) -> None:
    raw = config_dict()
    values = raw[section]
    assert isinstance(values, dict)
    values[field] = value
    with pytest.raises(ValidationError, match=message):
        AppConfig.model_validate(raw)


@pytest.mark.parametrize(
    "connection",
    ["udp:localhost", "udpin::14550", "udpin:localhost:70000", "serial:/dev/ttyUSB0:nope"],
)
def test_invalid_connection_string_is_rejected(connection: str) -> None:
    raw = config_dict()
    vehicle = raw["vehicle"]
    assert isinstance(vehicle, dict)
    vehicle["connection"] = connection
    with pytest.raises(ValidationError, match="connection must be"):
        AppConfig.model_validate(raw)

