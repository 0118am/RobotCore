"""Deterministic policy runner used before real model artifacts exist."""

from .base import PolicyRunner


class DummyRunner(PolicyRunner):
    """Deterministic runner used for graph and logging validation."""

    def run(self, observation):
        # Keep outputs neutral so integration tests validate plumbing without
        # accidentally moving simulated or real actuators.
        if self.manifest.role == "body":
            return [0.0] * 8
        if self.manifest.role == "arm":
            joint_count = int(self.manifest.output_schema.get("joint_count", 6))
            return [0.0] * joint_count
        return []
