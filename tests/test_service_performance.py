"""Static checks for the split, performance-oriented production services."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SYSTEMD = ROOT / "host_manager/systemd"


def text(name: str) -> str:
    return (SYSTEMD / name).read_text(encoding="utf-8")


def test_robot_service_owns_latency_cores_and_maximum_jetson_profile():
    robot = text("robotcore.service")
    performance = text("robotcore-performance.service")
    argus_lifecycle = text("nvargus-robotcore.conf")
    zed_lifecycle = text("zed-x-robotcore.conf")

    assert "PartOf=robotcore-stack.target" in robot
    assert "Requires=dev-robotcore-aboard.device nvargus-daemon.service zed_x_daemon.service" in robot
    assert "KillMode=mixed" in robot
    assert "CPUAffinity=2 3 4 5 6 7" in robot
    assert "CPUWeight=10000" in robot
    assert "Nice=-10" in robot
    assert "LimitMEMLOCK=infinity" in robot
    assert "TimerSlackNSec=1us" in robot
    assert "enable_web_ui" not in robot
    assert "enable_pool_tracking:=true" in robot
    assert "enable_rl_policy_runtime:=true" in robot
    assert "allow_rl_hardware:=true" in robot
    assert "Restart=always" in robot
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in robot
    assert "CYCLONEDDS_URI=file:///etc/robotcore/cyclonedds.xml" in robot
    assert "ReadWritePaths=/var/lib/robotcore /home/nvidia/robotcore_logs" in robot
    assert "ExecStartPre=/usr/bin/test -w ${ROS_LOG_DIR}" in robot
    assert "ExecStartPre=/usr/bin/test -w ${ROBOTCORE_RUN_ROOT}" in robot
    assert "vio_tag_fusion_node" in robot
    assert "set -eo pipefail; source /opt/ros/humble/setup.bash" in robot
    assert 'setup.bash"; set -u; exec ros2 launch' in robot
    assert "set -euo pipefail; source /opt/ros/humble/setup.bash" not in robot
    assert "After=nvfancontrol.service" in performance
    assert "ExecStart=/usr/sbin/nvpmodel -m 0" in performance
    assert "ExecStart=/usr/bin/jetson_clocks --fan" in performance
    assert "PartOf=robotcore.service" in argus_lifecycle
    assert "Before=robotcore.service" in argus_lifecycle
    assert "PartOf=robotcore.service" in zed_lifecycle
    assert "Before=robotcore.service" in zed_lifecycle


def test_web_service_is_supervised_on_non_localization_cores():
    web = text("control-interface.service")

    assert "PartOf=robotcore-stack.target" in web
    assert "After=network-online.target robotcore-host-manager.service robotcore.service" in web
    assert "CPUAffinity=0 1" in web
    assert "CPUWeight=10" in web
    assert "Nice=10" in web
    assert "Restart=always" in web
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in web
    assert "ReadWritePaths=/home/nvidia/robotcore_logs" in web
    assert "ExecStartPre=/usr/bin/test -w ${ROS_LOG_DIR}" in web
    assert "imu_topic:=" not in web
    assert "set -eo pipefail; source /opt/ros/humble/setup.bash" in web
    assert 'setup.bash"; set -u; exec ros2 launch' in web
    assert "set -euo pipefail; source /opt/ros/humble/setup.bash" not in web


def test_host_manager_can_read_only_the_operator_log_tree():
    host_manager = text("robotcore-host-manager.service")

    assert "ProtectHome=tmpfs" in host_manager
    assert "BindReadOnlyPaths=/home/nvidia/robotcore_logs" in host_manager
    assert "ReadWritePaths=/run/robotcore /etc/robotcore /var/lib/robotcore" in host_manager


def test_stack_target_starts_both_services_and_edge_env_is_local_dds():
    target = text("robotcore-stack.target")
    environment = text("edge.env.example")

    assert "robotcore.service" in target
    assert "control-interface.service" in target
    assert "robotcore-performance.service" in target
    assert "ROS_LOCALHOST_ONLY=1" in environment
    assert "ROS_DOMAIN_ID=42" in environment
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in environment
    assert "CYCLONEDDS_URI=file:///etc/robotcore/cyclonedds.xml" in environment
    assert "ROS_LOG_DIR=/home/nvidia/robotcore_logs/ros" in environment
    assert "ROBOTCORE_RUN_ROOT=/home/nvidia/robotcore_logs/runs" in environment
    cyclone = (ROOT / "host_manager/config/cyclonedds.xml").read_text()
    assert "ROS_LOCALHOST_ONLY=1" in cyclone
    assert "<NetworkInterface" not in cyclone
    assert "<AllowMulticast>false</AllowMulticast>" in cyclone
    assert '<SocketReceiveBufferSize min="2MB"' in cyclone


def test_component_executor_thread_counts_are_bounded():
    launch = (
        ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert launch.count('parameters=[{"thread_num": 2}]') == 2
    assert 'parameters=[{"thread_num": 3}]' not in launch
    assert 'os.environ.get(\n        "ROBOTCORE_RUN_ROOT"' in launch
    assert 'DeclareLaunchArgument("enable_pool_tracking", default_value="true")' in launch


def test_installer_pins_and_validates_the_cpp_workspaces():
    installer = (ROOT / "scripts/install_robotcore_services.sh").read_text(
        encoding="utf-8"
    )

    assert "--robot-workspace" in installer
    assert "vio_tag_fusion_node" in installer
    assert 'set_env_value ROBOTCORE_WORKSPACE "${robot_workspace}"' in installer
    assert 'set_env_value CONTROL_INTERFACE_WORKSPACE "${web_workspace}"' in installer
    assert 'set_env_value ZED_WORKSPACE "${zed_workspace}"' in installer
    assert "set_env_value RMW_IMPLEMENTATION rmw_cyclonedds_cpp" in installer
    assert 'set_env_value ROS_LOG_DIR "${ros_log_dir}"' in installer
    assert 'set_env_value ROBOTCORE_RUN_ROOT "${run_root}"' in installer
    assert 'install -d -o robotcore -g nvidia -m 2770 "${log_root}"' in installer
    assert "install -d -o robotcore -g robotcore" in installer
    assert "nvargus-daemon.service.d/robotcore.conf" in installer
    assert "zed_x_daemon.service.d/robotcore.conf" in installer
    assert 'rm -f "${unit_root}/robotcore-camera-ipc-ready.service"' in installer
    assert "setfacl -m u:robotcore:--x /home/nvidia" in installer
    assert 'runuser -u robotcore -- test -r "${robot_workspace}' in installer
    assert "sysctl -p /etc/sysctl.d/99-robotcore-dds.conf" in installer
    assert "sysctl --system" not in installer
