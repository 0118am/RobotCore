"""Deterministic policy runner for integration tests."""

from .base import PolicyRunner


class DummyRunner(PolicyRunner):
    """Deterministic runner used for graph and logging validation."""

    def run(self, observation):
        # Keep outputs neutral so integration tests validate plumbing without
        # accidentally moving simulated or real actuators.
        return [0.0] * 8 if self.manifest.role == "body" else []
