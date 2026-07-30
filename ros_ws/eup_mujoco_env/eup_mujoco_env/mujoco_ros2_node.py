"""MuJoCo ROS 2 bridge.

This node loads the placeholder MJCF when MuJoCo is available, otherwise it
falls back to deterministic mock dynamics so the ROS graph can still run.
"""

import math
import os
import struct
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu

from eup_interfaces.msg import (
    ArmCommand,
    ArmState,
    BodyState,
    MujocoStatus,
    ThrusterCommand,
    ThrusterState,
    TrajectoryTarget,
)
from eup_interfaces.srv import MujocoCommand

from .hydrodynamics import HydrodynamicForceModel, quat_apply_wxyz, quat_conjugate_wxyz

try:
    import mujoco
except Exception:
    # Import failure is allowed on non-simulation machines; the node will still
    # publish mock state streams for interface validation.
    mujoco = None

try:
    import mujoco.viewer as mujoco_viewer
except Exception:
    # Viewer support is optional. Headless machines can still run the ROS bridge.
    mujoco_viewer = None


class MujocoRos2Node(Node):
    """MuJoCo backend skeleton that exposes the same ROS 2 contract as hardware."""

    def __init__(self):
        super().__init__("mujoco_ros2_node")
        self.declare_parameter("model_path", "eup_mujoco_env/models/bluerov2_heavy_generic.xml")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("physics_steps_per_tick", 0)
        self.declare_parameter("real_time_factor", 1.0)
        self.declare_parameter("enable_viewer", False)
        self.declare_parameter("viewer_sync_every_n_steps", 1)
        self.declare_parameter("enable_hydrodynamics", False)
        self.declare_parameter("publish_front_camera", True)
        self.declare_parameter("front_camera_width", 320)
        self.declare_parameter("front_camera_height", 180)
        self.declare_parameter("front_camera_name", "front_camera")
        self.declare_parameter("enable_trajectory_markers", True)
        self.declare_parameter("trajectory_trail_length", 24)
        self.declare_parameter("trajectory_trail_sample_period_s", 0.25)
        self.declare_parameter("thruster_force_model", "isaac_warpauv")
        self.declare_parameter("thruster_deadband", 0.08)
        self.declare_parameter("thruster_rotor_constant", 0.001)
        self.declare_parameter("thruster_time_constant_s", 0.05)
        self.declare_parameter("linear_thruster_force_n", 120.0)

        self.model = None
        self.data = None
        self.viewer = None
        self.renderer = None
        self.step_count = 0
        self.paused = False
        self.last_command = ""
        self.last_error = ""
        self.model_path = str(self.get_parameter("model_path").value)
        self.hydrodynamics = HydrodynamicForceModel()
        self.base_body_id = None
        self.target_marker_mocap_id = None
        self.actual_marker_mocap_id = None
        self.target_trail_mocap_ids = []
        self.actual_trail_mocap_ids = []
        self.target_trail_cursor = 0
        self.actual_trail_cursor = 0
        self.last_trail_record_ns = 0
        self.latest_trajectory_target = None
        # Last received commands are echoed into state topics so UI/logging can
        # verify command propagation even before real physics is enabled.
        self.last_thrusters = [0.0] * 8
        self.last_thruster_forces = [0.0] * 8
        self.thruster_omega_state = [0.0] * 8
        self.last_arm_targets = [0.0] * 6
        self.started_ns = self.get_clock().now().nanoseconds

        self.try_load_mujoco_model()
        self.try_launch_viewer()
        self.try_init_camera_renderer()

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.imu_pub = self.create_publisher(Imu, "/sensors/imu", 10)
        self.depth_pub = self.create_publisher(Image, "/sensors/front_depth/image", 10)
        self.front_camera_pub = self.create_publisher(Image, "/sensors/front_camera/image_raw", 10)
        self.body_pub = self.create_publisher(BodyState, "/robot/body_state", 10)
        self.arm_pub = self.create_publisher(ArmState, "/robot/arm_state", 10)
        self.thruster_pub = self.create_publisher(
            ThrusterState, "/robot/thruster_state", 10
        )
        self.status_pub = self.create_publisher(MujocoStatus, "/mujoco/status", 10)

        self.create_subscription(
            ThrusterCommand, "/control/thruster_cmd", self.on_thruster_cmd, 10
        )
        self.create_subscription(ArmCommand, "/control/arm_cmd", self.on_arm_cmd, 10)
        self.create_subscription(
            TrajectoryTarget,
            "/runtime/trajectory_target",
            self.on_trajectory_target,
            10,
        )
        self.create_service(MujocoCommand, "/mujoco/command", self.on_mujoco_command)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.tick)

    def try_load_mujoco_model(self):
        if mujoco is None:
            self.get_logger().warn("Python package 'mujoco' is not available; using mock dynamics.")
            return

        model_path = Path(self.model_path)
        if not model_path.exists():
            self.last_error = f"MuJoCo model not found: {model_path}"
            self.get_logger().warn(f"{self.last_error}; using mock dynamics.")
            return

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if self.base_body_id < 0:
            self.base_body_id = 1 if self.model.nbody > 1 else 0
        self.discover_trajectory_markers()
        self.get_logger().info(f"Loaded MuJoCo model: {model_path}")

    def discover_trajectory_markers(self):
        """Find optional mocap marker bodies used to visualize ROS trajectories."""

        if self.model is None or not bool(self.get_parameter("enable_trajectory_markers").value):
            return
        trail_length = max(0, int(self.get_parameter("trajectory_trail_length").value))
        self.target_marker_mocap_id = self.find_mocap_id("trajectory_target_marker")
        self.actual_marker_mocap_id = self.find_mocap_id("trajectory_actual_marker")
        self.target_trail_mocap_ids = self.find_mocap_series("trajectory_target_trail", trail_length)
        self.actual_trail_mocap_ids = self.find_mocap_series("trajectory_actual_trail", trail_length)
        if self.target_marker_mocap_id is not None or self.actual_marker_mocap_id is not None:
            self.get_logger().info("MuJoCo trajectory markers enabled from ROS topics.")

    def find_mocap_series(self, prefix, trail_length):
        return [
            mocap_id
            for index in range(trail_length)
            for mocap_id in [self.find_mocap_id(f"{prefix}_{index:02d}")]
            if mocap_id is not None
        ]

    def find_mocap_id(self, body_name):
        if self.model is None or mujoco is None:
            return None
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            return None
        mocap_id = int(self.model.body_mocapid[body_id])
        return mocap_id if mocap_id >= 0 else None

    def try_init_camera_renderer(self):
        """Initialize optional offscreen rendering for the front camera."""

        if not bool(self.get_parameter("publish_front_camera").value):
            return
        if self.model is None:
            return
        try:
            width = int(self.get_parameter("front_camera_width").value)
            height = int(self.get_parameter("front_camera_height").value)
            self.renderer = mujoco.Renderer(self.model, height=height, width=width)
            self.get_logger().info("MuJoCo front camera renderer initialized.")
        except Exception as exc:
            self.renderer = None
            self.get_logger().warn(
                "MuJoCo offscreen camera renderer unavailable; publishing synthetic "
                f"front camera frames. Error: {exc}"
            )

    def try_launch_viewer(self):
        """Open MuJoCo's passive viewer when requested and available."""

        if not bool(self.get_parameter("enable_viewer").value):
            self.get_logger().info("MuJoCo viewer disabled; running backend headless.")
            return
        if self.viewer is not None:
            return
        if self.model is None or self.data is None:
            self.last_error = "MuJoCo viewer requested but no model/data is loaded."
            self.get_logger().warn(self.last_error)
            return
        if mujoco_viewer is None:
            self.last_error = "MuJoCo viewer requested but mujoco.viewer is unavailable."
            self.get_logger().warn(
                f"{self.last_error} "
                "Install viewer dependencies or run with enable_viewer:=false."
            )
            return
        if not self.viewer_preflight_ok():
            self.get_logger().warn(self.last_error)
            return

        try:
            self.viewer = mujoco_viewer.launch_passive(self.model, self.data)
        except BaseException as exc:
            self.viewer = None
            self.last_error = str(exc)
            self.get_logger().warn(
                "Failed to open MuJoCo viewer. If this is an SSH/headless session, "
                f"check DISPLAY/GLFW/OpenGL. Error: {exc}"
            )
            return
        self.last_error = ""
        self.get_logger().info("MuJoCo passive viewer opened.")

    def viewer_preflight_ok(self):
        """Check whether GLFW can open before calling MuJoCo's passive viewer."""

        if os.name == "posix" and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        ):
            self.last_error = "MuJoCo viewer requires DISPLAY or WAYLAND_DISPLAY."
            return False
        try:
            import glfw
        except Exception as exc:
            self.last_error = f"MuJoCo viewer requires glfw: {exc}"
            return False

        try:
            if not glfw.init():
                self.last_error = "GLFW could not initialize; run headless or check X11/OpenGL."
                return False
        except Exception as exc:
            self.last_error = f"GLFW preflight failed: {exc}"
            return False
        finally:
            try:
                glfw.terminate()
            except Exception:
                pass
        return True

    def on_mujoco_command(self, request, response):
        """Handle simulator management commands from Web UI or CLI."""

        command = request.command.strip().lower()
        self.last_command = command
        response.accepted = True
        response.message = "ok"

        if command == "pause":
            self.paused = True
            self.last_error = ""
            response.message = "MuJoCo paused"
        elif command == "resume":
            self.paused = False
            self.last_error = ""
            response.message = "MuJoCo resumed"
        elif command == "reset":
            self.reset_simulation()
            if self.model is None or self.data is None:
                response.accepted = False
                response.message = "MuJoCo model is not loaded"
            else:
                self.last_error = ""
                response.message = "MuJoCo reset"
        elif command == "step":
            self.step_simulation(force=True)
            if self.model is None or self.data is None:
                response.accepted = False
                response.message = "MuJoCo model is not loaded"
            else:
                self.last_error = ""
                response.message = "MuJoCo stepped once"
        elif command == "viewer_on":
            self.set_parameters([Parameter("enable_viewer", Parameter.Type.BOOL, True)])
            self.try_launch_viewer()
            if self.viewer is None:
                response.accepted = False
                response.message = self.last_error or "MuJoCo viewer failed to open"
            else:
                response.message = "MuJoCo viewer opened"
        elif command == "viewer_off":
            self.close_viewer()
            self.set_parameters([Parameter("enable_viewer", Parameter.Type.BOOL, False)])
            self.last_error = ""
            response.message = "MuJoCo viewer closed"
        else:
            response.accepted = False
            response.message = f"Unsupported MuJoCo command: {request.command}"
            self.last_error = response.message

        response.paused = self.paused
        response.viewer_running = self.viewer is not None
        response.step_count = self.step_count
        return response

    def reset_simulation(self):
        """Reset MuJoCo data while preserving the loaded model and ROS graph."""

        if self.model is not None and self.data is not None:
            mujoco.mj_resetData(self.model, self.data)
        self.step_count = 0
        self.last_thrusters = [0.0] * 8
        self.last_thruster_forces = [0.0] * 8
        self.thruster_omega_state = [0.0] * 8
        self.last_arm_targets = [0.0] * 6
        self.started_ns = self.get_clock().now().nanoseconds

    def step_simulation(self, force=False):
        """Advance MuJoCo unless paused, or exactly once when force=True."""

        if self.model is None or self.data is None:
            return
        if self.paused and not force:
            return
        steps = 1 if force else self.physics_steps_per_tick()
        physics_dt = float(self.model.opt.timestep)
        for _ in range(steps):
            self.update_trajectory_markers()
            self.apply_thruster_controls(physics_dt)
            self.apply_hydrodynamics()
            mujoco.mj_step(self.model, self.data)
            self.step_count += 1
        self.update_trajectory_markers()
        self.sync_viewer()

    def physics_steps_per_tick(self):
        """Return MuJoCo substeps needed for target time and sim time to agree."""

        configured = int(self.get_parameter("physics_steps_per_tick").value)
        if configured > 0:
            return configured
        rate = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        timestep = max(1.0e-6, float(self.model.opt.timestep))
        real_time_factor = max(0.01, float(self.get_parameter("real_time_factor").value))
        return max(1, int(round(real_time_factor / (rate * timestep))))

    def apply_hydrodynamics(self):
        """Apply migrated WarpAUV fluid wrench through MuJoCo xfrc_applied.

        MuJoCo expects externally applied Cartesian body wrench in world
        coordinates.  The migrated Fossen helper returns body-frame fluid force
        and torque, so this method rotates them back to world coordinates before
        assigning data.xfrc_applied for the base body.
        """

        if not bool(self.get_parameter("enable_hydrodynamics").value):
            return
        if self.model is None or self.data is None or self.base_body_id is None:
            return
        if self.model.nq < 7 or self.model.nv < 6:
            return

        self.data.xfrc_applied[:, :] = 0.0
        root_quat_w = [float(value) for value in self.data.qpos[3:7]]
        root_linvel_w = [float(value) for value in self.data.qvel[0:3]]
        root_angvel_w = [float(value) for value in self.data.qvel[3:6]]
        root_conj = quat_conjugate_wxyz(root_quat_w)
        root_linvel_b = quat_apply_wxyz(root_conj, root_linvel_w)
        root_angvel_b = quat_apply_wxyz(root_conj, root_angvel_w)
        force_b, torque_b = self.hydrodynamics.calculate_fluid_wrench(
            root_quat_w=root_quat_w,
            root_linvel_b=root_linvel_b,
            root_angvel_b=root_angvel_b,
        )
        force_w = quat_apply_wxyz(root_quat_w, force_b)
        torque_w = quat_apply_wxyz(root_quat_w, torque_b)
        for axis in range(3):
            self.data.xfrc_applied[self.base_body_id, axis] = force_w[axis]
            self.data.xfrc_applied[self.base_body_id, axis + 3] = torque_w[axis]

    def on_thruster_cmd(self, msg):
        self.last_thrusters = [float(value) for value in msg.normalized]

    def apply_thruster_controls(self, physics_dt):
        """Apply normalized ROS thruster commands using the selected force model."""

        if self.data is None or self.model is None:
            return
        force_model = str(self.get_parameter("thruster_force_model").value).lower()
        forces = []
        for index, normalized in enumerate(self.last_thrusters[: self.model.nu]):
            if force_model == "isaac_warpauv":
                force = self.isaac_warpauv_thruster_force(index, normalized, physics_dt)
            else:
                scale = float(self.get_parameter("linear_thruster_force_n").value)
                force = max(-1.0, min(1.0, float(normalized))) * scale
            self.data.ctrl[index] = force
            forces.append(float(force))
        self.last_thruster_forces = forces + [0.0] * max(0, 8 - len(forces))

    def isaac_warpauv_thruster_force(self, index, normalized, physics_dt):
        """Match IsaacLab WarpAUV PWM curve and first-order rotor dynamics."""

        command = max(-1.0, min(1.0, float(normalized)))
        deadband = float(self.get_parameter("thruster_deadband").value)
        if abs(command) < deadband:
            omega_cmd = 0.0
        elif command >= deadband:
            omega_cmd = -139.0 * command**2 + 500.0 * command + 8.28
        else:
            omega_cmd = 161.0 * command**2 + 517.86 * command - 5.72

        tau = float(self.get_parameter("thruster_time_constant_s").value)
        if tau <= 0.0:
            omega = omega_cmd
        else:
            alpha = math.exp(-max(0.0, float(physics_dt)) / tau)
            omega = self.thruster_omega_state[index] * alpha + (1.0 - alpha) * omega_cmd
        self.thruster_omega_state[index] = omega

        rotor_constant = float(self.get_parameter("thruster_rotor_constant").value)
        return rotor_constant * abs(omega) * omega

    def on_arm_cmd(self, msg):
        if msg.joint_targets:
            self.last_arm_targets = [float(value) for value in msg.joint_targets[:6]]

    def on_trajectory_target(self, msg):
        """Cache the latest ROS trajectory target for MuJoCo marker rendering."""

        self.latest_trajectory_target = (
            float(msg.target_pose.position.x),
            float(msg.target_pose.position.y),
            float(msg.target_pose.position.z),
        )

    def update_trajectory_markers(self):
        """Move mocap markers to show expected and actual paths in the viewer."""

        if self.model is None or self.data is None:
            return
        if not bool(self.get_parameter("enable_trajectory_markers").value):
            return

        target_pos = self.latest_trajectory_target
        actual_pos = self.current_root_position()
        if target_pos is not None and self.target_marker_mocap_id is not None:
            self.data.mocap_pos[self.target_marker_mocap_id] = target_pos
        if actual_pos is not None and self.actual_marker_mocap_id is not None:
            self.data.mocap_pos[self.actual_marker_mocap_id] = actual_pos

        now_ns = self.get_clock().now().nanoseconds
        sample_period_ns = int(
            max(0.02, float(self.get_parameter("trajectory_trail_sample_period_s").value)) * 1e9
        )
        if now_ns - self.last_trail_record_ns < sample_period_ns:
            return
        self.last_trail_record_ns = now_ns
        self.record_trail_sample(target_pos, self.target_trail_mocap_ids, "target")
        self.record_trail_sample(actual_pos, self.actual_trail_mocap_ids, "actual")

    def current_root_position(self):
        if self.data is None or self.model is None or self.model.nq < 3:
            return None
        return tuple(float(value) for value in self.data.qpos[0:3])

    def record_trail_sample(self, position, mocap_ids, trail_kind):
        if position is None or not mocap_ids:
            return
        if trail_kind == "target":
            index = self.target_trail_cursor % len(mocap_ids)
            self.target_trail_cursor += 1
        else:
            index = self.actual_trail_cursor % len(mocap_ids)
            self.actual_trail_cursor += 1
        self.data.mocap_pos[mocap_ids[index]] = position

    def tick(self):
        now = self.get_clock().now()
        elapsed = (now.nanoseconds - self.started_ns) * 1e-9
        self.step_simulation()

        depth = 1.5 + 0.05 * math.sin(elapsed)
        self.publish_clock(now)
        self.imu_pub.publish(self.make_imu(now))
        self.depth_pub.publish(self.make_depth_image(now, depth))
        if bool(self.get_parameter("publish_front_camera").value):
            self.front_camera_pub.publish(self.make_front_camera_image(now))
        self.body_pub.publish(self.make_body_state(now, depth))
        self.arm_pub.publish(self.make_arm_state(now))
        self.thruster_pub.publish(self.make_thruster_state(now))
        self.status_pub.publish(self.make_mujoco_status(now))

    def publish_clock(self, now):
        msg = Clock()
        msg.clock = now.to_msg()
        self.clock_pub.publish(msg)

    def make_imu(self, now):
        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.w = 1.0
        msg.linear_acceleration.z = 9.81
        return msg

    def make_depth_image(self, now, depth):
        width = 8
        height = 6
        values = [float(depth)] * (width * height)
        msg = Image()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "front_depth_optical_frame"
        msg.height = height
        msg.width = width
        msg.encoding = "32FC1"
        msg.is_bigendian = False
        msg.step = width * 4
        # Keep the mock depth image small to avoid bloating early rosbag2 runs.
        msg.data = struct.pack("<" + "f" * len(values), *values)
        return msg

    def make_front_camera_image(self, now):
        """Publish a renderable RGB camera frame for web/ROS consumers."""

        width = int(self.get_parameter("front_camera_width").value)
        height = int(self.get_parameter("front_camera_height").value)
        frame = self.render_front_camera(width, height)

        msg = Image()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "front_camera_optical_frame"
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = frame
        return msg

    def render_front_camera(self, width, height):
        """Return RGB bytes from MuJoCo renderer or a deterministic fallback."""

        if self.renderer is not None and self.data is not None:
            try:
                camera_name = str(self.get_parameter("front_camera_name").value)
                self.renderer.update_scene(self.data, camera=camera_name)
                return self.renderer.render().tobytes()
            except Exception as exc:
                self.renderer = None
                self.get_logger().warn(
                    "MuJoCo front camera render failed; switching to synthetic frames. "
                    f"Error: {exc}"
                )

        return self.make_synthetic_front_camera(width, height)

    def make_synthetic_front_camera(self, width, height):
        """Generate a small RGB frame when offscreen rendering is unavailable."""

        horizon = int(height * (0.45 + 0.08 * math.sin(self.step_count * 0.05)))
        data = bytearray(width * height * 3)
        for y in range(height):
            for x in range(width):
                offset = (y * width + x) * 3
                if y < horizon:
                    data[offset : offset + 3] = bytes((70, 145, 170))
                else:
                    shade = int(45 + 25 * math.sin((x + self.step_count) * 0.04))
                    data[offset : offset + 3] = bytes((8, max(35, shade), 82))
        return bytes(data)

    def make_body_state(self, now, depth):
        msg = BodyState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        if self.data is not None and self.model is not None and self.model.nq >= 7:
            pos = [float(value) for value in self.data.qpos[0:3]]
            quat = [float(value) for value in self.data.qpos[3:7]]
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.position.z = pos[2]
            msg.pose.orientation.w = quat[0]
            msg.pose.orientation.x = quat[1]
            msg.pose.orientation.y = quat[2]
            msg.pose.orientation.z = quat[3]
            if self.model.nv >= 6:
                root_conj = quat_conjugate_wxyz(quat)
                lin_b = quat_apply_wxyz(root_conj, [float(value) for value in self.data.qvel[0:3]])
                ang_b = quat_apply_wxyz(root_conj, [float(value) for value in self.data.qvel[3:6]])
                msg.twist.linear.x = lin_b[0]
                msg.twist.linear.y = lin_b[1]
                msg.twist.linear.z = lin_b[2]
                msg.twist.angular.x = ang_b[0]
                msg.twist.angular.y = ang_b[1]
                msg.twist.angular.z = ang_b[2]
                msg.linear_velocity_valid = all(math.isfinite(value) for value in lin_b)
            msg.depth_m = max(0.0, -pos[2])
        else:
            msg.pose.orientation.w = 1.0
            msg.twist.linear.x = sum(self.last_thrusters[4:8]) * 0.01
            msg.depth_m = float(depth)
        msg.altitude_m = 5.0
        msg.state_valid = True
        return msg

    def make_arm_state(self, now):
        msg = ArmState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "arm_base"
        msg.joint_names = [f"joint_{index + 1}" for index in range(6)]
        msg.position = self.last_arm_targets
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        msg.state_valid = True
        return msg

    def make_thruster_state(self, now):
        msg = ThrusterState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_link"
        msg.normalized_feedback = self.last_thrusters
        msg.pwm_us = [int(1500 + value * 400) for value in self.last_thrusters]
        msg.healthy = [True] * 8
        return msg

    def make_mujoco_status(self, now):
        """Build the status topic consumed by the web operator UI."""

        msg = MujocoStatus()
        msg.header.stamp = now.to_msg()
        msg.model_path = self.model_path
        msg.model_loaded = self.model is not None and self.data is not None
        msg.paused = self.paused
        msg.viewer_enabled = bool(self.get_parameter("enable_viewer").value)
        msg.viewer_running = self.viewer is not None
        msg.step_count = self.step_count
        msg.sim_time_s = float(self.data.time) if self.data is not None else 0.0
        msg.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        msg.last_command = self.last_command
        msg.last_error = self.last_error
        return msg

    def close_viewer(self):
        """Close the optional MuJoCo viewer if it is open."""

        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None

    def sync_viewer(self):
        """Synchronize the passive viewer with simulation data."""

        if self.viewer is None:
            return
        sync_every = max(1, int(self.get_parameter("viewer_sync_every_n_steps").value))
        if self.step_count % sync_every != 0:
            return
        try:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.viewer.close()
                self.viewer = None
                self.get_logger().info("MuJoCo viewer closed.")
        except Exception as exc:
            self.get_logger().warn(f"MuJoCo viewer sync failed; closing viewer: {exc}")
            self.viewer = None

    def destroy_node(self):
        """Close the optional viewer before ROS tears the node down."""

        if self.viewer is not None:
            self.close_viewer()
        if self.renderer is not None and hasattr(self.renderer, "close"):
            self.renderer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoRos2Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
