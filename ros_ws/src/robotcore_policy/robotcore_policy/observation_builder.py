"""Observation freshness tracking for policy inputs."""

class ObservationBuilder:
    """Tracks input freshness and returns missing input names."""

    def __init__(self, required_inputs, max_age_ns):
        self.required_inputs = list(required_inputs)
        self.max_age_ns = int(max_age_ns)
        self.last_seen_ns = {name: None for name in self.required_inputs}
        self.values = {}

    def update(self, name, stamp_ns, value):
        if name in self.last_seen_ns:
            self.last_seen_ns[name] = int(stamp_ns)
        self.values[name] = value

    def missing_inputs(self, now_ns):
        # Report names of missing/stale topics instead of hiding readiness
        # failures behind a generic policy-not-ready state.
        missing = []
        for name, stamp_ns in self.last_seen_ns.items():
            if stamp_ns is None or now_ns - stamp_ns > self.max_age_ns:
                missing.append(name)
        return missing

    def build(self):
        return dict(self.values)
