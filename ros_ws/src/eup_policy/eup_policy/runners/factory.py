"""Factory for creating policy runners from manifest metadata."""

from .dummy_runner import DummyRunner
from .mmn_runner import MmnRunner
from .onnx_runner import OnnxRunner
from .torch_runner import TorchRunner


def create_runner(manifest):
    runner = manifest.runner.lower()
    # Keep runner selection centralized so BodyPolicy and ArmPolicy share the
    # same model-format behavior.
    if runner == "dummy":
        return DummyRunner(manifest)
    if runner == "onnx":
        return OnnxRunner(manifest)
    if runner in {"pth", "torch"}:
        return TorchRunner(manifest)
    if runner == "mmn":
        return MmnRunner(manifest)
    raise ValueError(f"Unsupported policy runner: {manifest.runner}")
