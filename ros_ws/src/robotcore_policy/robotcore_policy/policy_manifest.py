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
    deployment_validation: dict = field(default_factory=dict)


def load_policy_manifest(path: str, default_name: str, role: str) -> PolicyManifest:
    path_value = str(path).strip()
    if not path_value:
        raise ValueError("policy manifest path is required")

    manifest_path = Path(path_value)
    if manifest_path.suffix.lower() != ".yaml":
        raise ValueError("policy_path must reference a policy.yaml manifest")
    if not manifest_path.is_file():
        raise FileNotFoundError(f"policy manifest does not exist: {manifest_path}")

    data = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"policy manifest must contain a mapping: {manifest_path}")

    required_fields = (
        "name",
        "role",
        "runner",
        "model_path",
        "input_schema",
        "output_schema",
    )
    missing_fields = [field for field in required_fields if field not in data]
    if missing_fields:
        requested_name = str(default_name).strip() or manifest_path.stem
        raise ValueError(
            f"policy manifest for {requested_name} is missing required fields: "
            + ", ".join(missing_fields)
        )

    name = str(data["name"]).strip()
    manifest_role = str(data["role"]).strip()
    runner = str(data["runner"]).strip()
    raw_model_path = str(data["model_path"]).strip()
    if not name or not manifest_role or not runner or not raw_model_path:
        raise ValueError("policy manifest identity, runner, and model_path must be non-empty")
    if manifest_role != role:
        raise ValueError(
            f"policy role {manifest_role!r} does not match requested role {role!r}"
        )

    input_schema = data["input_schema"]
    output_schema = data["output_schema"]
    if not isinstance(input_schema, list) or not all(
        isinstance(value, str) and value for value in input_schema
    ):
        raise ValueError("policy input_schema must be a list of non-empty strings")
    if not isinstance(output_schema, dict):
        raise ValueError("policy output_schema must be a mapping")

    optional_mappings = {}
    for field_name in ("isaac_contract", "deployment_validation"):
        value = data.get(field_name, {})
        if not isinstance(value, dict):
            raise ValueError(f"policy {field_name} must be a mapping")
        optional_mappings[field_name] = dict(value)

    model_path = raw_model_path
    if raw_model_path and not Path(raw_model_path).is_absolute():
        colocated_model = manifest_path.parent / raw_model_path
        if colocated_model.exists():
            model_path = str(colocated_model.resolve())

    return PolicyManifest(
        name=name,
        role=manifest_role,
        runner=runner,
        model_path=model_path,
        input_schema=list(input_schema),
        output_schema=dict(output_schema),
        isaac_contract=optional_mappings["isaac_contract"],
        deployment_validation=optional_mappings["deployment_validation"],
    )
