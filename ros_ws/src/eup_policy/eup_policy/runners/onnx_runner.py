"""ONNX policy runner.

The first real use case is an IsaacLab RSL-RL WarpAUV policy exported as ONNX.
Its observation contract is recorded in policy.yaml so the ROS observation
builder and this runner can stay explicit about sim-to-sim assumptions.
"""

from .base import PolicyRunner


class OnnxRunner(PolicyRunner):
    def __init__(self, manifest):
        super().__init__(manifest)
        try:
            import numpy as np
            import onnxruntime as ort
        except Exception as exc:
            raise RuntimeError(
                "ONNX policy requested but onnxruntime/numpy is unavailable. "
                "Install onnxruntime for desktop tests or onnxruntime-gpu on Jetson."
            ) from exc

        self.np = np
        self.contract = dict(manifest.isaac_contract or {})
        providers = self.contract.get("providers") or ["CPUExecutionProvider"]
        self.session = ort.InferenceSession(manifest.model_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.observation_dim = int(self.contract.get("observation_dim", 20))

    def run(self, observation):
        """Run ONNX inference and return a flat action vector."""

        obs = self._build_observation_vector(observation)
        output = self.session.run([self.output_name], {self.input_name: obs})[0]
        return self.np.asarray(output, dtype=self.np.float32).reshape(-1).tolist()

    def _build_observation_vector(self, observation):
        """Build IsaacLab WarpAUVTraj 20-D observation from ROS body state.

        IsaacLab order:
          target_quat_w(4), target_pos_error_b(3), target_lin_vel_b(3),
          root_quat_w(4), root_lin_vel_b(3), root_ang_vel_b(3)

        If /runtime/trajectory_target is present, target pose/velocity is
        converted from world/map coordinates into the policy's body-frame error
        terms. Manifest defaults remain as an explicit fallback for standalone
        ONNX smoke tests.
        """

        body = observation.get("/robot/body_state", {})
        target = observation.get("/runtime/trajectory_target", {})
        pose = body.get("pose", {})
        twist = body.get("twist", {})
        target_quat_w = self.contract.get("default_target_quat_w", [1.0, 0.0, 0.0, 0.0])
        target_pos_error_b = self.contract.get("default_target_pos_error_b", [0.0, 0.0, 0.0])
        target_lin_vel_b = self.contract.get("default_target_lin_vel_b", [0.0, 0.0, 0.0])
        root_pos_w = pose.get("position", [0.0, 0.0, 0.0])
        root_quat_w = pose.get("orientation", [1.0, 0.0, 0.0, 0.0])
        root_lin_vel_b = twist.get("linear", [0.0, 0.0, 0.0])
        root_ang_vel_b = twist.get("angular", [0.0, 0.0, 0.0])

        if target.get("valid"):
            target_pose = target.get("pose", {})
            target_twist = target.get("twist", {})
            target_pos_w = target_pose.get("position", root_pos_w)
            target_vel_w = target_twist.get("linear", [0.0, 0.0, 0.0])
            target_quat_w = target_pose.get("orientation", target_quat_w)
            root_conj = self._quat_conjugate_wxyz(root_quat_w)
            target_pos_error_b = self._quat_apply_wxyz(
                root_conj,
                [float(target_pos_w[i]) - float(root_pos_w[i]) for i in range(3)],
            )
            target_lin_vel_b = self._quat_apply_wxyz(root_conj, target_vel_w)

        values = (
            list(target_quat_w)
            + list(target_pos_error_b)
            + list(target_lin_vel_b)
            + list(root_quat_w)
            + list(root_lin_vel_b)
            + list(root_ang_vel_b)
        )
        if len(values) != self.observation_dim:
            raise RuntimeError(
                f"IsaacLab ONNX observation length {len(values)} does not match "
                f"configured observation_dim={self.observation_dim}"
            )
        return self.np.asarray([values], dtype=self.np.float32)

    @staticmethod
    def _quat_conjugate_wxyz(quat):
        return [float(quat[0]), -float(quat[1]), -float(quat[2]), -float(quat[3])]

    def _quat_apply_wxyz(self, quat, vector):
        """Rotate a vector by a wxyz quaternion using the IsaacLab convention."""

        q = self.np.asarray(quat, dtype=self.np.float32)
        v = self.np.asarray(vector, dtype=self.np.float32)
        xyz = q[1:4]
        t = 2.0 * self.np.cross(xyz, v)
        rotated = v + q[0] * t + self.np.cross(xyz, t)
        return rotated.reshape(-1).astype(float).tolist()
