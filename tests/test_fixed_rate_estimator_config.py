"""Static acceptance tests for the edge 60 Hz localisation pipeline."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SENSORS = ROOT / "ros_ws/src/eup_sensors"


def load_yaml(relative: str):
    return yaml.safe_load((SENSORS / relative).read_text(encoding="utf-8"))


def test_zed_vio_and_sensor_rates_keep_apriltag_image_load_bounded():
    parameters = load_yaml("config/zedx_minimal_open.yaml")["/**"]["ros__parameters"]

    assert parameters["general"]["grab_frame_rate"] == 60
    assert parameters["general"]["pub_frame_rate"] == 30.0
    assert parameters["sensors"]["sensors_pub_rate"] == 200.0
    assert parameters["pos_tracking"]["imu_fusion"] is True
    assert parameters["pos_tracking"]["publish_odom_pose"] is True


def test_fixed_rate_ekf_fuses_visual_state_and_only_external_gyro():
    parameters = load_yaml("config/tag_vio_external_imu_ekf.yaml")[
        "localization_ekf"
    ]["ros__parameters"]

    assert parameters["frequency"] == 60.0
    assert parameters["world_frame"] == "map"
    assert parameters["odom0"] == "/localization/aligned_vio_odom"
    assert parameters["imu0"] == "/sensors/external_imu"
    assert parameters["publish_tf"] is False
    assert parameters["publish_acceleration"] is False

    # State order: x/y/z, roll/pitch/yaw, vx/vy/vz, vroll/vpitch/vyaw, ax/ay/az.
    assert parameters["odom0_config"] == [True] * 9 + [False] * 6
    assert parameters["imu0_config"] == [False] * 9 + [True] * 3 + [False] * 3
    assert parameters["imu0_remove_gravitational_acceleration"] is False


def test_external_imu_requires_stationary_bias_calibration():
    config = load_yaml("config/external_imu.yaml")
    parameters = config["imu_conditioning"]["ros__parameters"]
    frames = config["vehicle_frames"]["ros__parameters"]

    assert parameters["input_topic"] == "/hardware/aboard_imu_raw"
    assert parameters["output_topic"] == "/sensors/external_imu"
    assert parameters["status_topic"] == "/localization/external_imu_ready"
    assert parameters["auto_start"] is True
    assert parameters["calibration_sample_count"] >= 200
    assert frames["base_to_imu_translation_m"] == [0.018, 0.0, 0.076]
    assert len(frames["base_to_imu_rpy_rad"]) == 3


def test_edge_launch_wires_one_canonical_fixed_rate_output():
    launch = (
        ROOT / "ros_ws/src/eup_bringup/launch/eup_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert 'executable="imu_conditioning_node"' in launch
    assert '"publish_imu": True' in launch
    assert '"output_odometry_topic": "/localization/aligned_vio_odom"' in launch
    assert 'package="robot_localization"' in launch
    assert 'name="localization_ekf"' in launch
    assert '("odometry/filtered", "/localization/fused_odom")' in launch
    assert '"fused_odometry_topic": "/localization/fused_odom"' in launch


def test_body_state_marks_velocity_stale_when_zed_vio_stops():
    fusion = (
        SENSORS / "eup_sensors/sensor_fusion_node.py"
    ).read_text(encoding="utf-8")

    assert (
        "body.linear_velocity_valid and self.zed_odometry_is_fresh()"
        in fusion
    )
