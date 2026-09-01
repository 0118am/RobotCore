"""Factory for creating policy runners from manifest metadata."""

from .dummy_runner import DummyRunner
from .onnx_runner import OnnxRunner
from .tensorrt_runner import TensorRtRunner


def create_runner(manifest):
    runner = manifest.runner.lower()
    # Keep runner selection centralized so BodyPolicy and ArmPolicy share the
    # same model-format behavior.
    if runner == "dummy":
        return DummyRunner(manifest)
    if runner == "onnx":
        return OnnxRunner(manifest)
    if runner in {"tensorrt", "trt"}:
        return TensorRtRunner(manifest)
    raise ValueError(f"Unsupported policy runner: {manifest.runner}")
