"""MMN runner placeholder until the model format is specified."""

from .base import PolicyRunner


class MmnRunner(PolicyRunner):
    def __init__(self, manifest):
        super().__init__(manifest)
        raise RuntimeError(
            "MMN runner is reserved until the model format and runtime contract "
            "are confirmed."
        )
