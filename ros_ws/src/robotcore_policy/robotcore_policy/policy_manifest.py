"""Policy manifest loading.

Each policy package is described by policy.yaml so the runtime can switch model
formats without hardcoding runner settings in ROS nodes.
"""

from dataclasses import dataclass, field
from pathlib import Path

import yaml


@dataclass
class PolicyManifest:
    name: str
    role: str
    runner: str = "dummy"
    model_path: str = ""
    input_schema: list[str] = field(default_factory=list)
    output_schema: dict = field(default_factory=dict)
    isaac_contract: dict = field(default_factory=dict)


def load_policy_manifest(path: str, default_name: str, role: str) -> PolicyManifest:
    # Missing manifests fall back to a dummy-style descriptor. This keeps Phase 1
    # launches useful before real model artifacts are selected.
    if not path:
        return PolicyManifest(name=default_name, role=role)

    manifest_path = Path(path)
    if manifest_path.suffix.lower() == ".onnx":
        return PolicyManifest(
            name=default_name,
            role=role,
            runner="onnx",
            model_path=str(manifest_path),
            input_schema=[
                "/robot/body_state",
                "/runtime/trajectory_target",
            ],
        )
    if not manifest_path.exists():
        return PolicyManifest(name=default_name, role=role, model_path=path)

    data = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    # Preserve unknown manifest extensions by ignoring them here; schema-specific
    # validation can be added once real policies arrive.
    return PolicyManifest(
        name=str(data.get("name", default_name)),
        role=str(data.get("role", role)),
        runner=str(data.get("runner", "dummy")),
        model_path=str(data.get("model_path", "")),
        input_schema=list(data.get("input_schema", [])),
        output_schema=dict(data.get("output_schema", {})),
        isaac_contract=dict(data.get("isaac_contract", {})),
    )
