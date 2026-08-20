import importlib.util
import sys
from pathlib import Path
from types import ModuleType


def load_model_module() -> ModuleType:
    tools_dir = Path("tools").resolve()
    spec = importlib.util.spec_from_file_location(
        "ensure_vision_models_test_module",
        tools_dir / "ensure_vision_models.py",
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_vision_model_manifest_files_are_present_and_hashed() -> None:
    module = load_model_module()

    assert len(module.MODELS) == 3
    for model in module.MODELS:
        ok, detail = module.verify_model(model)
        assert ok, detail
