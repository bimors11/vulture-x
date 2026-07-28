"""Command-line bootstrap for the Vulture-X foundation release."""

import argparse
import logging
from collections.abc import Sequence
from pathlib import Path

from pydantic import ValidationError

from vulture_x import __version__
from vulture_x.config import load_config
from vulture_x.logging import configure_logging, log_event


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="vulture-x")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/default.yaml"),
        help="path to the YAML configuration",
    )
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="validate configuration and exit without starting subsystems",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_config(args.config)
    except (ValueError, ValidationError) as exc:
        print(f"Configuration error: {exc}")
        return 2

    if args.check_config:
        print(f"Configuration valid: {args.config}")
        return 0

    logger = configure_logging(config.logging)
    log_event(
        logger,
        logging.INFO,
        "APPLICATION_INITIALIZED",
        {
            "version": __version__,
            "environment": config.project.environment,
            "command_authority": "disabled",
        },
    )
    logger.info(
        "Foundation initialized; vehicle connectivity and command authority are not implemented."
    )
    return 0


def cli() -> None:
    raise SystemExit(main())


if __name__ == "__main__":
    cli()
