"""Contracts for managed PID profiles, task files, and experiment recording."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PID_CONFIG = ROOT / "ros_ws/src/robotcore_control/config/pid/default.json"
TASK_DIR = ROOT.parent / "ControlInterface/control_interface/config/tasks"
PID_NODE = ROOT / "ros_ws/src/robotcore_control/robotcore_control/six_dof_pid_node.py"
TRACKING_EXPERIMENT_NODE = (
    ROOT
    / "ros_ws/src/robotcore_runtime/robotcore_runtime/tracking_experiment_node.py"
)
TRACKING_MONITOR_NODE = (
    ROOT / "ros_ws/src/robotcore_runtime/robotcore_runtime/tracking_monitor_node.py"
)
COMMAND_AUTHORITY_NODE = (
    ROOT / "ros_ws/src/robotcore_control_cpp/src/command_authority_node.cpp"
)
SAFETY_CONFIG = ROOT / "ros_ws/src/robotcore_control/config/real_pool_safety.yaml"
EDGE_LAUNCH = (
    ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
)


def test_default_pid_document_contains_every_reloadable_gain():
    document = json.loads(PID_CONFIG.read_text(encoding="utf-8"))

    assert document["schema_version"] == 1
    assert len(document["outer_position_kp"]) == 3
    assert len(document["outer_orientation_kp"]) == 3
    assert document["outer_orientation_kp"][2] == 0.60
    assert len(document["max_linear_velocity_mps"]) == 3
    assert len(document["max_angular_velocity_rps"]) == 3
    assert document["max_linear_velocity_mps"][0] == 0.40
    for field in ("inner_kp", "inner_ki", "inner_kd", "integral_limit", "wrench_limit"):
        assert len(document[field]) == 6


def test_tracking_tasks_are_individual_named_controller_documents():
    task_paths = sorted(TASK_DIR.glob("*.json"))
    tasks = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in task_paths
        if path.name != "record_topics.json"
    ]

    assert {task["name"] for task in tasks} == {
        "circle",
        "pose_hold",
        "racetrack",
        "rl_pose_hold",
        "station_hold",
        "station_hold_fast",
        "spatial_lissajous",
        "straight_line",
    }
    for task in tasks:
        assert task["kind"] == "tracking_task"
        assert task["controller"] in {"pid", "rl", "selected"}
        assert task["duration_s"] > 0.0 or task.get("run_until_stopped") is True
        if "mode" in task:
            assert task["mode"] in {
                "altitude_hold",
                "station_hold",
                "station_hold_fast",
                "rl_policy",
            }
        else:
            assert task["controller"] == "selected"
            assert set(task["compatible_modes"]) == {
                "station_hold",
                "station_hold_fast",
                "rl_policy",
            }
        if task["controller"] == "rl":
            assert task["control_law"] == "t60_precision_v7_model_499"
        elif task["controller"] == "selected":
            assert task["control_law"] == "selected"
        elif task["name"] != "pose_hold":
            assert task["control_law"] == "pid"
        assert task["trajectory_name"] in {
            "none",
            "spatial_lissajous",
            "circle",
            "racetrack",
            "straight_line",
        }
        assert task["trajectory"]["trajectory_type"]
        assert (TASK_DIR / f"{task['name']}.json").is_file()

def test_pose_hold_is_manual_planar_control_with_automatic_height_hold():
    task = json.loads((TASK_DIR / "pose_hold.json").read_text(encoding="utf-8"))
    trajectory = task["trajectory"]

    assert task["label"] == "定高模式"
    assert task["run_until_stopped"] is True
    assert trajectory["trajectory_type"] == "hold"
    assert trajectory["attitude_mode"] == "hold_initial"
    assert trajectory["relative_to_initial_pose"] is False
    assert trajectory["center_z"] == 0.9
    assert trajectory["manual_vertical_speed_mps"] == 0.2


def test_rl_policy_none_reuses_fast_station_target_from_start_height():
    task = json.loads((TASK_DIR / "rl_pose_hold.json").read_text(encoding="utf-8"))
    fast = json.loads(
        (TASK_DIR / "station_hold_fast.json").read_text(encoding="utf-8")
    )
    trajectory = task["trajectory"]
    fast_trajectory = fast["trajectory"]

    assert task["mode"] == "rl_policy"
    assert task["trajectory_name"] == "none"
    assert task["controller"] == "rl"
    assert task["control_law"] == "t60_precision_v7_model_499"
    assert task["run_until_stopped"] is True
    assert trajectory["trajectory_type"] == "hold"
    assert trajectory["attitude_mode"] == fast_trajectory["attitude_mode"]
    assert trajectory["relative_to_initial_pose"] is False
    assert "center_z" not in trajectory
    assert "center_z" not in fast_trajectory
    assert (
        trajectory["manual_vertical_speed_mps"]
        == fast_trajectory["manual_vertical_speed_mps"]
    )
    assert (
        trajectory["station_linear_input_gain_mps"]
        == fast_trajectory["station_linear_input_gain_mps"]
    )
    assert (
        trajectory["station_lateral_input_gain_mps"]
        == fast_trajectory["station_lateral_input_gain_mps"]
    )
    assert (
        trajectory["station_yaw_input_gain_rps"]
        == fast_trajectory["station_yaw_input_gain_rps"]
    )


def test_station_hold_combines_direct_height_position_and_heading_pid():
    task = json.loads((TASK_DIR / "station_hold.json").read_text(encoding="utf-8"))
    trajectory = task["trajectory"]

    assert task["label"] == "定点模式"
    assert task["control_law"] == "pid"
    assert task["run_until_stopped"] is True
    assert trajectory["trajectory_type"] == "hold"
    assert trajectory["attitude_mode"] == "level_heading"
    assert trajectory["relative_to_initial_pose"] is False
    assert trajectory["center_z"] == 0.9
    assert trajectory["station_linear_input_gain_mps"] == 0.3
    assert trajectory["station_lateral_input_gain_mps"] == 0.2
    assert trajectory["station_yaw_input_gain_rps"] == 0.6


def test_fast_station_hold_has_dedicated_high_authority_control_type():
    standard = json.loads((TASK_DIR / "station_hold.json").read_text(encoding="utf-8"))
    fast = json.loads(
        (TASK_DIR / "station_hold_fast.json").read_text(encoding="utf-8")
    )

    assert fast["label"] == "快速定点模式"
    assert fast["mode"] == "station_hold_fast"
    assert fast["trajectory_name"] == "none"
    assert fast["trajectory"]["trajectory_type"] == "hold"
    assert "center_z" not in fast["trajectory"]
    assert standard["trajectory"]["station_linear_input_gain_mps"] == 0.3
    assert fast["trajectory"]["station_linear_input_gain_mps"] == 0.5
    assert fast["trajectory"]["station_lateral_input_gain_mps"] == 0.2
    assert fast["trajectory"]["station_yaw_input_gain_rps"] == standard["trajectory"][
        "station_yaw_input_gain_rps"
    ]
    assert fast["trajectory"]["manual_vertical_speed_mps"] == standard[
        "trajectory"
    ]["manual_vertical_speed_mps"]


def test_spatial_lissajous_task_uses_one_name_everywhere():
    task = json.loads(
        (TASK_DIR / "spatial_lissajous.json").read_text(encoding="utf-8")
    )
    trajectory = task["trajectory"]

    assert task["label"] == "Spatial Lissajous"
    assert task["compatible_modes"] == [
        "station_hold",
        "station_hold_fast",
        "rl_policy",
    ]
    assert task["controller"] == "selected"
    assert task["control_law"] == "selected"
    assert task["name"] == "spatial_lissajous"
    assert task["trajectory_name"] == "spatial_lissajous"
    assert trajectory["trajectory_type"] == "spatial_lissajous"
    assert trajectory["trajectory_speed_mps"] == 0.25
    assert "period_s" not in trajectory
    assert [trajectory["center_x"], trajectory["center_y"], trajectory["center_z"]] == [
        2.71,
        1.865,
        0.5,
    ]
    assert [trajectory["amp_x"], trajectory["amp_y"], trajectory["amp_z"]] == [
        2.0,
        1.0,
        0.3,
    ]


def test_rl_policy_runtime_and_hardware_authority_are_enabled():
    launch = EDGE_LAUNCH.read_text(encoding="utf-8")
    safety = SAFETY_CONFIG.read_text(encoding="utf-8")
    authority = COMMAND_AUTHORITY_NODE.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("enable_rl_policy_runtime", default_value="true")' in launch
    assert 'DeclareLaunchArgument("allow_rl_hardware", default_value="true")' in launch
    assert 'package="robotcore_policy_cpp"' in launch
    assert 'executable="t60_policy"' in launch
    assert '"policy_name": "t60_precision_v7_model_499"' in launch
    assert '"policy_layout_hash"' not in launch
    assert 'allow_rl_hardware: true' in safety
    assert 'declare_parameter("allow_rl_hardware", true)' in authority
    assert '"/policy/body/action"' in authority


def test_tracking_experiment_waits_for_fresh_rl_policy_inference():
    source = TRACKING_EXPERIMENT_NODE.read_text(encoding="utf-8")

    assert 'PolicyStatus,' in source
    assert '"/policy/body/status"' in source
    assert 'controller not in {' in source
    assert '"rl",' in source
    assert "self.policy_status_sequence > controller_sequence_at_target" in source
    assert "and policy.loaded" in source
    assert "and policy.input_ready" in source
    assert "int(policy.inference_count) > inference_count_at_target" in source


def test_planar_automatic_tasks_share_fixed_height_and_start_prelude():
    expected = {
        "circle": "path_tangent",
        "racetrack": "path_tangent",
        "straight_line": "fixed_path_heading",
    }

    for task_name, attitude_mode in expected.items():
        task = json.loads(
            (TASK_DIR / f"{task_name}.json").read_text(encoding="utf-8")
        )
        trajectory = task["trajectory"]

        assert task["compatible_modes"] == [
            "station_hold",
            "station_hold_fast",
            "rl_policy",
        ]
        assert task["controller"] == "selected"
        assert task["trajectory_name"] == task_name
        assert trajectory["trajectory_type"] == task_name
        assert trajectory["attitude_mode"] == attitude_mode
        assert trajectory["center_z"] == 0.5
        assert trajectory["hold_before_motion_s"] == 25.0
        assert trajectory["move_duration_s"] == 15.0
        assert trajectory["trajectory_ramp_s"] == 10.0


def test_managed_task_defaults_keep_initial_altitude_at_point_nine_metres():
    source = TRACKING_EXPERIMENT_NODE.read_text(encoding="utf-8")

    assert '"center_z": 0.9' in source


def test_tracking_experiment_does_not_abort_an_already_canceled_goal():
    source = TRACKING_EXPERIMENT_NODE.read_text(encoding="utf-8")

    assert "canceled = False" in source
    assert "canceled = True" in source
    assert "elif not canceled:" in source
    assert "elif not goal_handle.is_cancel_requested:" not in source


def test_tracking_experiment_prepares_trajectory_while_disarmed_then_rearms():
    source = TRACKING_EXPERIMENT_NODE.read_text(encoding="utf-8")

    neutralize = source.index('disarm_result = await self.set_authority("", False)')
    select = source.index(
        "select_result = await self.set_authority(controller, False)"
    )
    recorder = source.index("start_result = await self.run_start_client.call_async")
    configure = source.index("parameter_response = await self.parameter_client.call_async")
    validate = source.index("validation_result = await self.validate_client.call_async")
    reset = source.index("reset_result = await self.reset_client.call_async")
    ready = source.index("if not await self.wait_for_tracking_ready(", reset)
    rearm = source.index("arm_result = await self.set_authority(controller, True)")

    assert neutralize < select < recorder < configure < validate < reset < ready < rearm
    assert "await self.wait_for_disarmed_status(controller, 0.5)" in source
    assert "re-arm after trajectory reset failed" in source
    assert "arm_generation_before = int(self.authority.arm_generation)" in source
    assert "controller, arm_generation_before, 0.5" in source
    assert "official_start_result = await self.reset_client.call_async" in source
    assert "self.trajectory_target_sequence > target_sequence_before_reset" in source
    assert "self.pid_status_sequence > controller_sequence_at_target" in source
    assert "and pid.ready" in source
    assert "and pid.producing_command" in source
    assert "must be selected, healthy, and explicitly armed" not in source


def test_trajectory_envelope_contains_pose_hold_and_condition_limits():
    import yaml

    document = yaml.safe_load(SAFETY_CONFIG.read_text(encoding="utf-8"))
    authority = document["command_authority"]["ros__parameters"]
    pid = document["pid_controller"]["ros__parameters"]
    trajectory = document["trajectory_command"]["ros__parameters"]

    assert "pool_bounds_configured" not in authority
    assert "pool_min_xyz" not in authority
    assert "pool_max_xyz" not in authority
    assert trajectory["pool_min_xyz"] == [0.0, 0.0, 0.0]
    assert trajectory["pool_max_xyz"] == [5.42, 3.73, 1.0]
    assert trajectory["trajectory_limits_configured"] is True
    assert trajectory["max_linear_speed_mps"] == 0.55
    assert trajectory["max_angular_speed_rps"] == 0.60
    assert trajectory["attitude_min_rpy_deg"] == [-15.0, -15.0, -30.0]
    assert trajectory["attitude_max_rpy_deg"] == [15.0, 15.0, 30.0]
    assert pid["altitude_pwm_kp"] == 1.4
    assert pid["altitude_pwm_ki"] == 0.45
    assert pid["altitude_pwm_kd"] == 0.22
    assert pid["altitude_pwm_integral_limit"] == 0.5
    assert pid["altitude_velocity_filter_time_constant_s"] == 0.10
    assert pid["altitude_position_kp"] == 0.25
    assert pid["station_sway_pwm_kp"] == 1.0
    assert pid["fast_station_sway_pwm_kp"] == 0.60
    assert pid["fast_station_sway_pwm_feedforward_positive_gain"] == 1.80
    assert pid["fast_station_sway_pwm_feedforward_negative_gain"] == 1.90
    assert pid["fast_station_sway_effort_slew_rate_per_s"] == 1.20
    assert pid["station_sway_pwm_ki"] == 0.0
    assert pid["station_sway_integral_limit"] == 0.5
    assert pid["station_yaw_pwm_kp"] == 0.70
    assert pid["station_yaw_pwm_ki"] == 0.12
    assert pid["station_yaw_integral_limit"] == 0.8
    assert pid["station_yaw_heading_kp"] == 0.90
    assert pid["station_horizontal_axis_limit"] == 0.12
    assert pid["station_yaw_axis_limit"] == 0.20
    assert pid["station_surge_rate_limit"] == 0.40
    assert pid["fast_station_surge_rate_limit"] == 0.50
    assert pid["fast_station_sway_rate_limit"] == 0.20
    assert pid["fast_station_lateral_yaw_axis_limit"] == 0.10
    assert pid["fast_station_sway_yaw_compensation_gain"] == 0.055
    assert pid["fast_station_lateral_yaw_effort_slew_rate_per_s"] == 0.30
    assert pid["station_yaw_rate_limit"] == 0.60
    assert not any(name.startswith("lissajous_") for name in pid)
    assert pid["fast_station_level_pwm_limit"] == 0.10
    assert pid["fast_station_roll_angle_to_rate_kp"] == 1.30
    assert pid["fast_station_pitch_angle_to_rate_kp"] == 0.80
    assert pid["fast_station_roll_rate_kp"] == 0.20
    assert pid["fast_station_pitch_rate_kp"] == 0.35
    assert pid["fast_station_pitch_rate_ki"] == 0.20
    assert pid["fast_station_pitch_rate_integral_effort_limit"] == 0.10
    assert pid["fast_station_surge_pitch_decoupling_enabled"] is True
    assert pid["fast_station_surge_pitch_forward_gain"] == 0.08
    assert pid["fast_station_surge_pitch_reverse_gain"] == 0.06
    assert pid["fast_station_surge_pitch_limit"] == 0.06
    assert pid["imu_rate_history_samples"] == 5
    assert pid["imu_rate_prediction_horizon_s"] == 0.04
    assert pid["imu_rate_prediction_accel_limit_rps2"] == 4.0
    assert pid["imu_rate_prediction_delta_limit_rps"] == 0.12
    assert pid["imu_rate_filter_time_constant_s"] == 0.01
    assert pid["imu_orientation_filter_time_constant_s"] == 0.03
    assert '"control_rate_hz": 50.0' in EDGE_LAUNCH.read_text(encoding="utf-8")

    task = json.loads((TASK_DIR / "pose_hold.json").read_text(encoding="utf-8"))
    assert task["trajectory"]["manual_vertical_speed_mps"] <= trajectory[
        "max_linear_speed_mps"
    ]


def test_command_authority_has_no_vehicle_pool_bounds_trip_path():
    source = COMMAND_AUTHORITY_NODE.read_text(encoding="utf-8")

    assert "inside_pool" not in source
    assert "vehicle is outside configured pool bounds" not in source
    assert "pool_bounds_configured" not in source


def test_command_sources_are_isolated_and_authority_owns_final_output():
    authority = COMMAND_AUTHORITY_NODE.read_text(encoding="utf-8")
    pid = PID_NODE.read_text(encoding="utf-8")

    assert '"/control/manual/thruster_cmd", "manual", "web_operator"' in authority
    assert '"/control/pid/thruster_cmd", "pid", "pid_controller"' in authority
    assert '"/policy/body/action"' in authority
    assert 'command->source = "t60_policy"' in authority
    assert '"/control/thruster_cmd"' in authority
    legacy_shared_topic = "/control/" + "thruster_" + "candidate"
    assert legacy_shared_topic not in authority
    assert "altitude_manual_surge_yaw" in authority
    assert '"/control/thruster_cmd"' not in pid


def test_rosbag_topic_list_captures_target_state_pid_output_and_safety():
    topics = set(
        json.loads((TASK_DIR / "record_topics.json").read_text(encoding="utf-8"))["topics"]
    )

    assert {
        "/robot/body_state",
        "/runtime/trajectory_target",
        "/runtime/tracking_status",
        "/control/pid/wrench",
        "/control/manual/thruster_cmd",
        "/control/pid/thruster_cmd",
        "/policy/body/action",
        "/policy/body/status",
        "/control/thruster_cmd",
        "/control/authority/status",
        "/safety/events",
        "/zedx/zed_node/odom",
    } <= topics


def test_pid_node_exposes_station_live_document_and_direct_control():
    source = PID_NODE.read_text(encoding="utf-8")

    assert 'self.create_subscription(BodyState, "/robot/body_state"' in source
    assert 'self.declare_parameter("control_rate_hz", 50.0)' in source
    assert 'self.create_service(GetPidConfig, "/control/pid/config", self.on_get_config)' in source
    assert "response.config_json = json.dumps(" in source
    assert "response.configuration_hash = self.configuration_hash" in source
    assert 'ThrusterCommand, "/control/pid/thruster_cmd", 10' in source
    assert source.count("self.create_subscription(") == 4
    assert 'ThrusterCommand, "/control/manual/thruster_cmd"' not in source
    assert '"/control/authority/status"' in source
    assert '"/control/pwm_limit_us"' not in source
    assert "action_limit = float(message.action_limit)" in source
    assert 'control_mode = str(target.control_mode).lower()' in source
    assert 'altitude_mode = control_mode == "altitude_hold"' in source
    assert 'station_mode = control_mode in {"station_hold", "station_hold_fast"}' in source
    assert 'fast_station_mode = control_mode == "station_hold_fast"' in source
    assert 'missing.append("unsupported_trajectory_type")' in source
    assert 'missing.append("unsupported_control_mode")' in source
    assert 'missing.append("incompatible_trajectory_control_mode")' in source
    assert "elif direct_altitude_mode:" in source
    assert 'self.declare_parameter("imu_topic", "/sensors/external_imu")' in source
    assert 'self.declare_parameter("imu_rate_history_samples", 5)' in source
    assert 'self.declare_parameter("imu_rate_filter_time_constant_s", 0.01)' in source
    assert 'self.declare_parameter("imu_orientation_filter_time_constant_s", 0.03)' in source
    assert "qos_profile_sensor_data" in source
    assert 'missing.append("/sensors/external_imu")' in source
    assert "external_imu_not_base_link" not in source
    assert "external_imu_angular_velocity_invalid" not in source
    assert "status.imu_age_s = float(imu_age)" in source
    assert "imu_angular_velocity=self" not in source
    assert "measured_twist" not in source
    assert "timestamped_rate_prediction(" in source
    assert "reject_vector_outlier(" in source
    assert "self.imu_angular_velocity_filtered += rate_alpha" in source
    assert "else (sample_ns - self.imu_filter_ns) * 1e-9" in source
    assert "self.imu_ns = now_ns" in source
    assert "self.imu_filter_ns = sample_ns" in source
    assert "quaternion_slerp(" in source
    assert "orientation = attitude_with_heading(" in source
    assert "current_q = self.imu_orientation_filtered" in source
    assert "localization_q = quaternion_from_message(body.pose.orientation)" in source
    assert "map_to_body = quaternion_conjugate(localization_q)" in source
    assert "position_error_body = quaternion_apply(\n            map_to_body" in source
    assert "actual_linear_world = quaternion_apply(\n                localization_q, actual_linear_body" in source
    assert "direct_altitude_mode = altitude_mode or station_mode" in source
    assert "wrench_for_commands(reference_commands)" not in source
    assert "ThrusterAllocator" not in source
    assert "SixAxisPid" not in source
    assert '"thruster_config_path"' not in source
    assert ".allocate(" not in source

    tracking_monitor = TRACKING_MONITOR_NODE.read_text(encoding="utf-8")
    assert "map_yaw = self.quaternion_to_rpy(localization_quat_w)[2]" in tracking_monitor
    assert "control_attitude_w = self.rpy_quaternion(" in tracking_monitor
    assert "status.actual_orientation.w" in tracking_monitor
    assert "mix_station_pwm" not in source
    assert "station_controller" not in source
    assert "altitude_station_pwm_commands" in source
    assert "desired_surge_velocity" in source
    assert "desired_sway_velocity" in source
    assert 'self.declare_parameter("station_surge_pwm_kp", 1.5)' in source
    assert 'self.declare_parameter("station_surge_pwm_ki", 0.25)' in source
    assert 'self.declare_parameter("station_sway_pwm_kp", 1.0)' in source
    assert 'self.declare_parameter("fast_station_sway_pwm_kp", 0.60)' in source
    assert '"fast_station_sway_pwm_feedforward_positive_gain", 1.80' in source
    assert '"fast_station_sway_pwm_feedforward_negative_gain", 1.90' in source
    assert 'self.declare_parameter("fast_station_sway_effort_slew_rate_per_s", 1.20)' in source
    assert 'self.declare_parameter("station_sway_pwm_ki", 0.0)' in source
    assert 'self.declare_parameter("station_sway_integral_limit", 0.5)' in source
    assert 'self.declare_parameter("station_yaw_pwm_kp", 0.70)' in source
    assert 'self.declare_parameter("station_yaw_pwm_ki", 0.12)' in source
    assert 'self.declare_parameter("station_yaw_integral_limit", 0.8)' in source
    assert 'self.declare_parameter("lissajous_' not in source
    assert 'self.declare_parameter("fast_station_level_pwm_limit", 0.10)' in source
    assert 'self.declare_parameter("fast_station_roll_angle_to_rate_kp", 1.30)' in source
    assert 'self.declare_parameter("fast_station_pitch_angle_to_rate_kp", 0.80)' in source
    assert 'self.declare_parameter("fast_station_roll_rate_kp", 0.20)' in source
    assert 'self.declare_parameter("fast_station_pitch_rate_kp", 0.35)' in source
    assert 'self.declare_parameter("fast_station_pitch_rate_ki", 0.20)' in source
    assert '"fast_station_pitch_rate_integral_effort_limit", 0.10' in source
    assert 'self.declare_parameter("fast_station_surge_pitch_decoupling_enabled", True)' in source
    assert 'self.declare_parameter("fast_station_surge_pitch_forward_gain", 0.08)' in source
    assert 'self.declare_parameter("fast_station_surge_pitch_reverse_gain", 0.06)' in source
    assert 'self.declare_parameter("fast_station_surge_pitch_limit", 0.06)' in source
    assert "if fast_station_mode or trajectory_start_approach" in source
    assert "altitude_level_pwm_commands" not in source
    assert "level_attitude_pd_efforts(" not in source
    assert "commands[:4] =" not in source
    assert "level_attitude_rate_efforts(" in source
    assert "conditional_axis_integral_effort(" in source
    assert "self.fast_station_pitch_rate_integral_effort = 0.0" in source
    assert "surge_pitch_decoupling_effort(" in source
    assert "self.fast_station_level_angle_to_rate_kp" in source
    assert "self.fast_station_level_rate_kp" in source
    assert "individual PWM <=" in source
    assert 'self.declare_parameter("station_yaw_heading_kp", 0.90)' in source
    assert 'self.declare_parameter("station_horizontal_axis_limit", 0.12)' in source
    assert 'self.declare_parameter("station_yaw_axis_limit", 0.20)' in source
    assert 'self.declare_parameter("station_surge_rate_limit", 0.40)' in source
    assert 'self.declare_parameter("fast_station_surge_rate_limit", 0.50)' in source
    assert 'self.declare_parameter("fast_station_sway_rate_limit", 0.20)' in source
    assert (
        'self.declare_parameter("fast_station_lateral_yaw_axis_limit", 0.10)'
        in source
    )
    assert '"fast_station_sway_yaw_compensation_gain", 0.055' in source
    assert '"fast_station_lateral_yaw_effort_slew_rate_per_s", 0.30' in source
    assert 'self.declare_parameter("station_yaw_rate_limit", 0.60)' in source
    assert 'self.declare_parameter("altitude_position_kp", 0.25)' in source
    assert "self.station_yaw_heading_kp * orientation_error_body[2]" in source
    assert "self.fast_station_surge_rate_limit" in source
    assert "self.fast_station_sway_rate_limit" in source
    assert "self.fast_station_lateral_yaw_axis_limit" in source
    assert "directional_velocity_feedforward(" in source
    assert "slew_rate_limit(" in source
    assert "yaw_feedforward" in source
    assert "straight_lateral_maneuver" in source
    assert "sway_rate_limit" in source
    assert "self.outer_position_kp[:2]" in source
    assert 'trajectory_phase == "start_approach"' in source
    assert "target.trajectory_phase" in source
    assert 'getattr(target, "trajectory_phase"' not in source
    assert "station_velocity_setpoints(" in source
    assert "center_approach=trajectory_start_approach" in source
    assert "self.station_surge_pid.step(" in source
    assert "self.fast_station_sway_pid" in source
    assert "requested_sway_effort = sway_pid.step(" in source
    assert "self.station_yaw_pid.step(" in source
    assert "self.station_yaw_pid.reset()" in source
    assert "yaw_maneuver_active or straight_lateral_maneuver" in source
    assert 'yaw_integral_state = "maneuver-disabled"' in source
    assert 'yaw_integral_state = "lateral-disabled"' in source
    assert 'yaw_integral_state = "hold-reset"' in source
    assert 'yaw_integral_state = "zero-cross-reset"' in source
    assert "integral_reset_after_error_crossing(" in source
    assert "self.station_surge_pid.integral = surge_integral_before" in source
    assert "self.station_yaw_pid.integral = yaw_integral_before" in source
    assert '"surge/sway PID; yaw PI "' in source
    assert '"yaw I "' in source
    assert "allocate_linearized_reference" not in source


def test_pid_command_stays_neutral_between_arm_and_explicit_start():
    source = PID_NODE.read_text(encoding="utf-8")

    assert 'idle_mode = control_mode == "idle"' in source
    assert 'status_message = "ready; neutral until Start"' in source


def test_tracking_action_waits_with_ros_future_instead_of_asyncio_event_loop():
    source = TRACKING_EXPERIMENT_NODE.read_text(encoding="utf-8")

    assert "asyncio.sleep" not in source
    assert "from rclpy.task import Future" in source
    assert "await self.wait_for_next_feedback(0.1)" in source
    assert "callback_group=self.wait_callback_group" in source
    assert 'get_package_share_directory("robotcore_runtime")' not in source
    assert "task = self.load_task(task_name)" in source
    assert 'task.get("kind") != "tracking_task"' in source
    assert 'str(task.get("name", "")) != task_name' in source
    assert "if task_name not in {" not in source
    assert '"tracking strategy is not approved"' not in source


def test_estimated_body_state_uses_message_validity_without_stale_source_names():
    pid = PID_NODE.read_text(encoding="utf-8")
    monitor = TRACKING_MONITOR_NODE.read_text(encoding="utf-8")
    authority = COMMAND_AUTHORITY_NODE.read_text(encoding="utf-8")

    assert "state_valid or self.body.position_estimated" in pid
    assert "state_valid or self.last_body.position_estimated" in monitor
    assert "!body.state_valid && !body.position_estimated" in authority
    assert "estimated_pose_source_not_allowed" not in pid
    assert 'startswith("ZED VIO")' not in monitor
    assert "estimated localization source is not allowed" not in authority


def test_tracking_action_stops_trajectory_before_every_active_run_cleanup():
    source = TRACKING_EXPERIMENT_NODE.read_text(encoding="utf-8")

    assert 'Trigger, "/runtime/trajectory/stop"' in source
    assert source.count("await self.stop_trajectory()") == 2
    assert source.index("await self.stop_trajectory()", source.index("finally:")) < source.index(
        "await self.disarm(controller)", source.index("finally:")
    )
