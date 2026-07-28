"""JSON Lines event logging with monotonic correlation timestamps."""

import json
import logging
from datetime import UTC, datetime
from time import monotonic
from typing import Any

from vulture_x.config import LoggingConfig


class JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        event: dict[str, Any] = {
            "timestamp_utc": datetime.now(UTC).isoformat(timespec="milliseconds"),
            "timestamp_monotonic_s": monotonic(),
            "level": record.levelname,
            "module": record.name,
            "event": getattr(record, "event", record.getMessage()),
            "details": getattr(record, "details", {}),
        }
        if record.exc_info:
            event["exception"] = self.formatException(record.exc_info)
        return json.dumps(event, separators=(",", ":"), default=str)


def configure_logging(config: LoggingConfig) -> logging.Logger:
    """Configure and return the application logger."""
    logger = logging.getLogger("vulture_x")
    logger.setLevel(config.level)
    logger.handlers.clear()
    logger.propagate = False

    console = logging.StreamHandler()
    console.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)s %(name)s %(message)s")
    )
    logger.addHandler(console)

    if config.event_jsonl:
        config.directory.mkdir(parents=True, exist_ok=True)
        event_file = logging.FileHandler(config.directory / "events.jsonl", encoding="utf-8")
        event_file.setFormatter(JsonFormatter())
        logger.addHandler(event_file)
    return logger


def log_event(
    logger: logging.Logger,
    level: int,
    event: str,
    details: dict[str, object] | None = None,
) -> None:
    """Emit a machine-readable event through configured handlers."""
    logger.log(level, event, extra={"event": event, "details": details or {}})
