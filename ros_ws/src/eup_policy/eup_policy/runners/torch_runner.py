"""PyTorch runner placeholder.

This runner is blocked on the target Jetson PyTorch wheel/container decision.
"""

from .base import PolicyRunner


class TorchRunner(PolicyRunner):
    def __init__(self, manifest):
        super().__init__(manifest)
        raise RuntimeError(
            "PTH runner is a Phase 0/3 integration point. Validate the NVIDIA "
            "PyTorch wheel or container on the target Jetson first."
        )
