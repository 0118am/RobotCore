"""Base class for policy runner adapters."""

class PolicyRunner:
    def __init__(self, manifest):
        # The manifest carries runner type, model path, and I/O schema without
        # forcing every runner to parse YAML itself.
        self.manifest = manifest

    def run(self, observation):
        raise NotImplementedError
