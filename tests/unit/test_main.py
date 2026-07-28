from vulture_x.main import main


def test_check_default_configuration(capsys: object) -> None:
    result = main(["--config", "configs/default.yaml", "--check-config"])
    assert result == 0


def test_missing_configuration_fails_closed(capsys: object) -> None:
    result = main(["--config", "does-not-exist.yaml"])
    assert result == 2

