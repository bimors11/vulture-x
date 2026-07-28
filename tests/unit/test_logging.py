import json
import logging
from pathlib import Path

from vulture_x.config import LoggingConfig
from vulture_x.logging import TelemetryCsvLogger, configure_logging, log_event


def test_event_logger_writes_json_lines(tmp_path: Path) -> None:
    logger = configure_logging(LoggingConfig(directory=tmp_path))
    log_event(logger, logging.WARNING, "TRACKING_LOST", {"age_s": 0.4})
    for handler in logger.handlers:
        handler.flush()
    event = json.loads((tmp_path / "events.jsonl").read_text(encoding="utf-8"))
    assert event["event"] == "TRACKING_LOST"
    assert event["details"] == {"age_s": 0.4}


def test_telemetry_logger_writes_csv(tmp_path: Path) -> None:
    path = tmp_path / "telemetry.csv"
    with TelemetryCsvLogger(path, ("time", "confidence")) as logger:
        logger.write({"time": 1.0, "confidence": 0.8})
    assert path.read_text(encoding="utf-8").splitlines() == [
        "time,confidence",
        "1.0,0.8",
    ]

