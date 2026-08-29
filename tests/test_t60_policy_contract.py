"""Exact deployment-contract tests for t60_precision_v7/model_499."""

import hashlib
import sys
from pathlib import Path

import numpy as np


CORE_ROOT = Path(__file__).resolve().parents[1]
POLICY_SOURCE = CORE_ROOT / "ros_ws" / "src" / "robotcore_policy"
sys.path.insert(0, str(POLICY_SOURCE))

from robotcore_policy.policy_manifest import load_policy_manifest  # noqa: E402
from robotcore_policy.action_decoder import decode_thruster_action  # noqa: E402
from robotcore_policy.t60_observation import (  # noqa: E402
    HISTORY_INDICES,
    OBS_SCALE,
    T60ObservationState,
)


def observation(
    *,
    body_position=(0.0, 0.0, 0.0),
    body_orientation=(1.0, 0.0, 0.0, 0.0),
    body_linear_velocity=(0.0, 0.0, 0.0),
    imu_orientation=(1.0, 0.0, 0.0, 0.0),
    imu_angular_velocity=(0.0, 0.0, 0.0),
    target_position=(0.0, 0.0, 0.0),
    target_orientation=(1.0, 0.0, 0.0, 0.0),
    target_linear_velocity=(0.0, 0.0, 0.0),
    target_angular_velocity=(0.0, 0.0, 0.0),
    target_linear_acceleration=(0.0, 0.0, 0.0),
    applied_thruster_command=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
):
    return {
        "/robot/body_state": {
            "state_valid": True,
            "linear_velocity_valid": True,
            "pose": {
                "position": list(body_position),
                "orientation": list(body_orientation),
            },
            "twist": {
                "linear": list(body_linear_velocity),
            },
        },
        "/sensors/external_imu": {
            "orientation": list(imu_orientation),
            "angular_velocity": list(imu_angular_velocity),
        },
        "/runtime/trajectory_target": {
            "valid": True,
            "pose": {
                "position": list(target_position),
                "orientation": list(target_orientation),
            },
            "twist": {
                "linear": list(target_linear_velocity),
                "angular": list(target_angular_velocity),
            },
            "accel": {
                "linear": list(target_linear_acceleration),
                "angular": [0.0, 0.0, 0.0],
            },
        },
        "/control/thruster_cmd": {
            "action": list(applied_thruster_command),
        },
    }


def test_policy_action_validation_is_identity_without_clamp_or_mapping():
    action = [0.588237, -0.725222, -0.748287, 0.647278, 0.935745, 0.810303, 0.438161, 0.501731]
    assert decode_thruster_action(action) == action

    for invalid in (action[:7], [*action[:7], 1.01], [*action[:7], float("nan")]):
        try:
            decode_thruster_action(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid policy action must be rejected, not transformed")


def test_current_frame_has_exact_field_order_and_scaling():
    state = T60ObservationState()
    sample = observation(
        body_position=(1.0, 2.0, 3.0),
        body_linear_velocity=(0.1, -0.2, 0.3),
        imu_angular_velocity=(0.4, -0.8, 0.2),
        target_position=(1.25, 1.5, 3.1),
        target_linear_velocity=(0.4, 0.2, -0.4),
        target_angular_velocity=(0.8, -0.4, 0.2),
        target_linear_acceleration=(0.45, -0.9, 0.225),
        applied_thruster_command=(0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8),
    )

    vector = state.build(sample).reshape(-1)
    expected_raw = np.asarray(
        [
            0.25,
            -0.5,
            0.1,
            0.4,
            0.2,
            -0.4,
            0.3,
            0.4,
            -0.7,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -1.0,
            0.4,
            -0.8,
            0.2,
            0.8,
            -0.4,
            0.2,
            0.45,
            -0.9,
            0.225,
            0.1,
            -0.2,
            0.3,
            -0.4,
            0.5,
            -0.6,
            0.7,
            -0.8,
        ],
        dtype=np.float32,
    )
    assert vector.shape == (201,)
    assert np.allclose(vector[:33], expected_raw / OBS_SCALE, atol=1.0e-6)
    assert np.array_equal(vector[33:], np.zeros(168, dtype=np.float32))


def test_world_vectors_rotate_to_flu_body_and_attitude_error_is_unique():
    state = T60ObservationState()
    half_sqrt = np.sqrt(0.5)
    # The body is yawed +90 degrees in world. World +X is therefore body -Y.
    sample = observation(
        body_orientation=(half_sqrt, 0.0, 0.0, half_sqrt),
        target_position=(1.0, 0.0, 0.0),
        target_orientation=(-half_sqrt, 0.0, 0.0, -half_sqrt),
    )

    current = state.build(sample).reshape(-1)[:33]
    assert np.allclose(current[0:3], [0.0, -4.0, 0.0], atol=1.0e-6)
    assert np.allclose(current[9:13], [1.0, 0.0, 0.0, 0.0], atol=1.0e-6)
    assert np.allclose(current[13:16], [0.0, 0.0, -1.0], atol=1.0e-6)


def test_policy_attitude_uses_imu_tilt_and_localization_yaw():
    state = T60ObservationState()
    half_sqrt = np.sqrt(0.5)
    roll_30 = np.radians(30.0)
    target_yaw_90 = (half_sqrt, 0.0, 0.0, half_sqrt)
    imu_roll_30 = (
        np.cos(roll_30 / 2.0),
        np.sin(roll_30 / 2.0),
        0.0,
        0.0,
    )
    # The localization quaternion carries an intentionally wrong +45-degree
    # roll but the correct +90-degree map yaw. Policy tilt must come from IMU.
    current = state.build(
        observation(
            body_orientation=(0.6532815, 0.2705981, 0.2705981, 0.6532815),
            imu_orientation=imu_roll_30,
            target_orientation=target_yaw_90,
        )
    ).reshape(-1)[:33]

    assert np.allclose(
        current[9:13],
        [np.cos(roll_30 / 2.0), -np.sin(roll_30 / 2.0), 0.0, 0.0],
        atol=1.0e-6,
    )
    assert np.allclose(
        current[13:16],
        [0.0, -0.5, -np.sqrt(0.75)],
        atol=1.0e-6,
    )


def test_policy_attitude_and_body_rate_axes_are_robotcore_flu_identity():
    state = T60ObservationState()
    half_sqrt = np.sqrt(0.5)
    positive_axis_quaternions = (
        (half_sqrt, half_sqrt, 0.0, 0.0),  # +roll: left rises
        (half_sqrt, 0.0, half_sqrt, 0.0),  # +pitch: bow sinks
        (half_sqrt, 0.0, 0.0, half_sqrt),  # +yaw: bow turns left
    )

    for axis, target_quaternion in enumerate(positive_axis_quaternions):
        current = state.build(
            observation(
                imu_angular_velocity=(0.1, -0.2, 0.3),
                target_orientation=target_quaternion,
            )
        ).reshape(-1)[:33]
        expected_attitude_error = np.zeros(4, dtype=np.float32)
        expected_attitude_error[0] = half_sqrt
        expected_attitude_error[axis + 1] = half_sqrt
        assert np.allclose(
            current[9:13], expected_attitude_error, atol=1.0e-6
        )
        assert np.allclose(
            current[16:19],
            np.asarray([0.1, -0.2, 0.3]) / 0.8,
            atol=1.0e-6,
        )


def test_history_is_newest_first_and_committed_only_after_inference():
    state = T60ObservationState()
    first_action = np.asarray([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8])
    first = state.build(
        observation(
            target_position=(0.25, 0.0, 0.0),
            applied_thruster_command=first_action,
        )
    ).reshape(-1)
    assert np.count_nonzero(first[33:]) == 0

    state.commit()
    second_action = np.asarray([-0.8, 0.7, -0.6, 0.5, -0.4, 0.3, -0.2, 0.1])
    second = state.build(
        observation(
            target_position=(0.5, 0.0, 0.0),
            applied_thruster_command=second_action,
        )
    ).reshape(-1)

    assert np.allclose(second[25:33], second_action)
    assert np.allclose(second[33:54], first[:33][HISTORY_INDICES])
    assert np.count_nonzero(second[54:]) == 0

    state.commit()
    third = state.build(observation(target_position=(0.75, 0.0, 0.0))).reshape(-1)
    assert np.allclose(third[33:54], second[:33][HISTORY_INDICES])
    assert np.allclose(third[54:75], first[:33][HISTORY_INDICES])


def test_reset_zeros_history_without_replacing_applied_command_input():
    state = T60ObservationState()
    state.build(observation(target_position=(0.25, 0.0, 0.0)))
    state.commit()
    state.reset()

    applied = np.linspace(-0.8, 0.8, 8, dtype=np.float32)
    reset_input = state.build(
        observation(applied_thruster_command=applied)
    ).reshape(-1)
    assert np.allclose(reset_input[25:33], applied)
    assert np.array_equal(reset_input[33:], np.zeros(168, dtype=np.float32))


def test_deployable_manifest_resolves_colocated_model_and_exact_contract():
    manifest_path = (
        CORE_ROOT / "models" / "policies" / "t60_precision_v7_model_499" / "policy.yaml"
    )
    manifest = load_policy_manifest(str(manifest_path), "unused", "body")

    assert manifest.runner == "tensorrt"
    assert Path(manifest.model_path).is_file()
    assert manifest.isaac_contract["observation_layout"] == (
        "t60_precision_v7_history8"
    )
    assert manifest.isaac_contract["observation_dim"] == 201
    assert manifest.isaac_contract["history_length"] == 8
    assert manifest.isaac_contract["control_rate_hz"] == 25
    assert manifest.isaac_contract["physics_steps_per_action"] == 4
    assert manifest.isaac_contract["state_delay_s"] == 0.05
    assert manifest.input_schema == [
        "/robot/body_state",
        "/sensors/external_imu",
        "/runtime/trajectory_target",
        "/control/thruster_cmd",
    ]
    assert manifest.isaac_contract["angular_velocity_source"] == (
        "/sensors/external_imu"
    )
    assert manifest.isaac_contract["previous_action_source"] == (
        "/control/thruster_cmd"
    )
    assert manifest.output_schema["size"] == 8
    assert manifest.output_schema["output_layer"] == "tanh"
    assert manifest.output_schema["direct_hardware_command"] is True
    assert manifest.output_schema["channel_order"] == [
        "T1_front_right_vertical",
        "T2_rear_right_vertical",
        "T3_front_left_vertical",
        "T4_rear_left_vertical",
        "T5_rear_left_horizontal",
        "T6_rear_right_horizontal",
        "T7_front_left_horizontal",
        "T8_front_right_horizontal",
    ]
    assert manifest.output_schema["type"] == "direct_thruster_action"
    assert manifest.output_schema["pwm_mapping"] == "PWM_us = 1500 + 250 * action"
    assert manifest.deployment_validation["mode"] == "direct_thruster_action"
    assert manifest.deployment_validation["authority"] == "current_physical_vehicle"
    assert manifest.deployment_validation["actuator_alignment"] == (
        "trained_physical_T1_T8"
    )
    assert manifest.deployment_validation["state_frame"] == "robotcore_FLU"
    assert manifest.deployment_validation["angular_velocity_frame"] == "body_xyz"
    assert manifest.isaac_contract["exported_at"].isoformat() == "2026-08-29"
    assert manifest.isaac_contract["source_artifact"] == (
        "auv_traj_policy_mlp_history_8_2026-08-29_model_499.onnx"
    )
    assert manifest.isaac_contract["body_axes"] == ["forward", "left", "up"]
    assert manifest.isaac_contract["quaternion_semantics"] == "world_from_body"
    assert manifest.isaac_contract["angular_velocity_frame"] == "body"
    assert manifest.isaac_contract["angular_velocity_order"] == [
        "roll_rate",
        "pitch_rate",
        "yaw_rate",
    ]
    assert manifest.isaac_contract["positive_roll"] == "left_side_up_right_side_down"
    assert manifest.isaac_contract["positive_pitch"] == "bow_down_stern_up"
    assert manifest.isaac_contract["positive_yaw"] == "bow_left_ccw_from_above"
    digest = hashlib.sha256(Path(manifest.model_path).read_bytes()).hexdigest()
    assert digest == manifest.isaac_contract["sha256"]


def test_shadow_launch_has_no_control_topic():
    launch_source = (
        POLICY_SOURCE / "launch" / "t60_policy_shadow.launch.py"
    ).read_text(encoding="utf-8")

    assert "body_policy_node" in launch_source
    assert '"publish_rate_hz": 25.0' in launch_source
    assert "/control/" not in launch_source


def test_cpp_runtime_uses_history8_at_25_hz_and_direct_rl_actions():
    policy_source = (
        CORE_ROOT
        / "ros_ws"
        / "src"
        / "robotcore_policy_cpp"
        / "src"
        / "t60_policy_node.cpp"
    ).read_text(encoding="utf-8")
    authority_source = (
        CORE_ROOT
        / "ros_ws"
        / "src"
        / "robotcore_control_cpp"
        / "src"
        / "command_authority_node.cpp"
    ).read_text(encoding="utf-8")
    safety_config = (
        CORE_ROOT
        / "ros_ws"
        / "src"
        / "robotcore_control"
        / "config"
        / "real_pool_safety.yaml"
    ).read_text(encoding="utf-8")

    assert "constexpr std::size_t kHistoryFrames = 8;" in policy_source
    assert "static_assert(kObservationSize == 201);" in policy_source
    assert 'declare_parameter<double>("control_rate_hz", 25.0)' in policy_source
    assert "kHardwareActionScale" not in policy_source
    assert "command.action[channel];" in policy_source
    assert "kRlHardwareActionScale" not in authority_source
    assert "const double polarity" not in authority_source
    assert "action.data[channel]" in authority_source
    assert "publish_rate_hz: 50.0" in safety_config
    assert "pwm_limit_us: 250.0" in safety_config
