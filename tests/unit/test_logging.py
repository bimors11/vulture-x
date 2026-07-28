import json
import logging
from pathlib import Path

from vulture_x.config import LoggingConfig
from vulture_x.logging import configure_logging, log_event


def test_event_logger_writes_json_lines(tmp_path: Path) -> None:
    logger = configure_logging(
        LoggingConfig(level="INFO", directory=tmp_path, event_jsonl=True)
    )
    log_event(logger, logging.WARNING, "TARGET_DATA_STALE", {"age_s": 0.83})
    for handler in logger.handlers:
        handler.flush()

    event = json.loads((tmp_path / "events.jsonl").read_text(encoding="utf-8"))
    assert event["event"] == "TARGET_DATA_STALE"
    assert event["details"] == {"age_s": 0.83}
    assert event["level"] == "WARNING"
    assert event["timestamp_monotonic_s"] > 0

