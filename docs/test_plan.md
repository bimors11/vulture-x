# Test Plan

Milestone 0 verification consists of:

- importing the installed package;
- accepting the repository default configuration;
- rejecting unknown fields, invalid endpoints, unsafe separation geometry,
  reversed data-age thresholds, and reversed battery thresholds;
- verifying immutable data models and command-expiration boundaries;
- verifying parseable JSON Lines event output;
- running `pytest`, `ruff check .`, and strict `mypy`.

SITL and HIL tests are intentionally deferred until their integration
milestones. Empty test-suite directories reserve their eventual locations
without implying test coverage that does not exist.

