"""Isaac-compatible trajectory target publisher.

The exported WarpAUV trajectory policy expects target pose and target linear
velocity as part of its 20-D observation.  IsaacLab generated those commands
inside the environment; in EUP they live in ROS so MuJoCo, logging, UI, and
future hardware playback can all see the same target stream.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from eup_interfaces.msg import TrajectoryTarget


@dataclass
class TrajectorySample:
    """One target sample in world/map coordinates."""

    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]


class TrajectoryCommandNode(Node):
    """Publishes deterministic trajectory targets matching Isaac AUV eval code."""

    def __init__(self):
        super().__init__("trajectory_command_node")
        self.declare_parameter("trajectory_type", "lissajous")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", -1.5)
        self.declare_parameter("amp_x", 1.5)
        self.declare_parameter("amp_y", 0.75)
        self.declare_parameter("amp_z", 0.4)
        self.declare_parameter("period_s", 16.0)
        self.declare_parameter("radius_min", 0.3)
        self.declare_parameter("radius_max", 1.5)
        self.declare_parameter("chirp_rate", 2.2)

        self.started_ns = self.get_clock().now().nanoseconds
        self.pub = self.create_publisher(TrajectoryTarget, "/runtime/trajectory_target", 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.tick)

    def tick(self):
        """Publish the next target using ROS time since node startup."""

        now = self.get_clock().now()
        time_s = (now.nanoseconds - self.started_ns) * 1e-9
        trajectory_type = str(self.get_parameter("trajectory_type").value).lower()
        sample = self.sample(trajectory_type, time_s)

        msg = TrajectoryTarget()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.target_pose.position.x = sample.position[0]
        msg.target_pose.position.y = sample.position[1]
        msg.target_pose.position.z = sample.position[2]
        msg.target_pose.orientation.w = 1.0
        msg.target_twist.linear.x = sample.velocity[0]
        msg.target_twist.linear.y = sample.velocity[1]
        msg.target_twist.linear.z = sample.velocity[2]
        msg.target_accel.linear.x = sample.acceleration[0]
        msg.target_accel.linear.y = sample.acceleration[1]
        msg.target_accel.linear.z = sample.acceleration[2]
        msg.trajectory_type = trajectory_type
        msg.time_s = float(time_s)
        msg.valid = True
        self.pub.publish(msg)

    def sample(self, trajectory_type: str, time_s: float) -> TrajectorySample:
        """Evaluate one of the Isaac AUV trajectory families."""

        center = (
            float(self.get_parameter("center_x").value),
            float(self.get_parameter("center_y").value),
            float(self.get_parameter("center_z").value),
        )
        amp_x = float(self.get_parameter("amp_x").value)
        amp_y = float(self.get_parameter("amp_y").value)
        amp_z = float(self.get_parameter("amp_z").value)
        period = max(0.1, float(self.get_parameter("period_s").value))
        omega = 2.0 * math.pi / period
        phase_x = omega * time_s
        phase_y = 2.0 * omega * time_s

        if trajectory_type == "circle":
            offset = (
                amp_x * math.cos(phase_x),
                amp_y * math.sin(phase_x),
                0.0,
            )
            velocity = (
                -amp_x * omega * math.sin(phase_x),
                amp_y * omega * math.cos(phase_x),
                0.0,
            )
            acceleration = (
                -amp_x * omega**2 * math.cos(phase_x),
                -amp_y * omega**2 * math.sin(phase_x),
                0.0,
            )
        elif trajectory_type == "helix":
            offset = (
                amp_x * math.cos(phase_x),
                amp_y * math.sin(phase_x),
                amp_z * math.sin(phase_y),
            )
            velocity = (
                -amp_x * omega * math.sin(phase_x),
                amp_y * omega * math.cos(phase_x),
                2.0 * amp_z * omega * math.cos(phase_y),
            )
            acceleration = (
                -amp_x * omega**2 * math.cos(phase_x),
                -amp_y * omega**2 * math.sin(phase_x),
                -4.0 * amp_z * omega**2 * math.sin(phase_y),
            )
        elif trajectory_type == "spiral":
            offset, velocity, acceleration = self._sample_spiral(time_s, omega)
        elif trajectory_type == "chirp":
            offset, velocity, acceleration = self._sample_chirp(time_s, omega)
        elif trajectory_type == "racetrack":
            offset, velocity, acceleration = self._sample_racetrack(time_s)
        elif trajectory_type in {"sine_x", "sine_y", "sine_z"}:
            axis = {"sine_x": 0, "sine_y": 1, "sine_z": 2}[trajectory_type]
            offset = [0.0, 0.0, 0.0]
            velocity = [0.0, 0.0, 0.0]
            acceleration = [0.0, 0.0, 0.0]
            offset[axis] = amp_x * math.sin(phase_x)
            velocity[axis] = amp_x * omega * math.cos(phase_x)
            acceleration[axis] = -amp_x * omega**2 * math.sin(phase_x)
            offset = tuple(offset)
            velocity = tuple(velocity)
            acceleration = tuple(acceleration)
        else:
            # Isaac's default trajectory policy was trained/evaluated on a
            # Lissajous-style figure-eight, so unknown names fall back there.
            offset = (
                amp_x * math.sin(phase_x),
                amp_y * math.sin(phase_y),
                0.0,
            )
            velocity = (
                amp_x * omega * math.cos(phase_x),
                2.0 * amp_y * omega * math.cos(phase_y),
                0.0,
            )
            acceleration = (
                -amp_x * omega**2 * math.sin(phase_x),
                -4.0 * amp_y * omega**2 * math.sin(phase_y),
                0.0,
            )

        position = (
            center[0] + offset[0],
            center[1] + offset[1],
            center[2] + offset[2],
        )
        return TrajectorySample(position=position, velocity=velocity, acceleration=acceleration)

    def _sample_spiral(self, time_s: float, omega: float):
        radius_min = float(self.get_parameter("radius_min").value)
        radius_max = float(self.get_parameter("radius_max").value)
        radius_range = radius_max - radius_min
        radial_phase = 0.5 * omega * time_s
        radius = radius_min + 0.5 * radius_range * (1.0 - math.cos(radial_phase))
        radius_dot = 0.25 * radius_range * omega * math.sin(radial_phase)
        radius_ddot = 0.125 * radius_range * omega**2 * math.cos(radial_phase)
        phase = omega * time_s
        offset = (
            radius * math.cos(phase),
            radius * math.sin(phase),
            0.0,
        )
        velocity = (
            radius_dot * math.cos(phase) - radius * omega * math.sin(phase),
            radius_dot * math.sin(phase) + radius * omega * math.cos(phase),
            0.0,
        )
        acceleration = (
            radius_ddot * math.cos(phase)
            - 2.0 * radius_dot * omega * math.sin(phase)
            - radius * omega**2 * math.cos(phase),
            radius_ddot * math.sin(phase)
            + 2.0 * radius_dot * omega * math.cos(phase)
            - radius * omega**2 * math.sin(phase),
            0.0,
        )
        return offset, velocity, acceleration

    def _sample_chirp(self, time_s: float, omega: float):
        amp_x = float(self.get_parameter("amp_x").value)
        amp_y = float(self.get_parameter("amp_y").value)
        chirp_rate = float(self.get_parameter("chirp_rate").value)
        period = max(0.1, float(self.get_parameter("period_s").value))
        w1 = chirp_rate * omega
        chirp_k = (w1 - omega) / period
        phase = omega * time_s + 0.5 * chirp_k * time_s**2
        chirp_omega = omega + chirp_k * time_s
        offset = (
            amp_x * math.sin(phase),
            amp_y * math.sin(2.0 * phase),
            0.0,
        )
        velocity = (
            amp_x * chirp_omega * math.cos(phase),
            2.0 * amp_y * chirp_omega * math.cos(2.0 * phase),
            0.0,
        )
        acceleration = (
            amp_x * (chirp_k * math.cos(phase) - chirp_omega**2 * math.sin(phase)),
            2.0
            * amp_y
            * (chirp_k * math.cos(2.0 * phase) - 2.0 * chirp_omega**2 * math.sin(2.0 * phase)),
            0.0,
        )
        return offset, velocity, acceleration

    def _sample_racetrack(self, time_s: float):
        amp_x = float(self.get_parameter("amp_x").value)
        radius = max(0.05, float(self.get_parameter("amp_y").value))
        period = max(0.1, float(self.get_parameter("period_s").value))
        half_straight = max(0.1, amp_x - radius)
        path_length = 4.0 * half_straight + 2.0 * math.pi * radius
        speed = path_length / period
        s = (speed * time_s) % path_length

        if s < 2.0 * half_straight:
            return (
                (-half_straight + s, radius, 0.0),
                (speed, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
        if s < 2.0 * half_straight + math.pi * radius:
            local_s = s - 2.0 * half_straight
            theta = math.pi / 2.0 - local_s / radius
            return (
                half_straight + radius * math.cos(theta),
                radius * math.sin(theta),
                0.0,
            ), (
                speed * math.sin(theta),
                -speed * math.cos(theta),
                0.0,
            ), (
                -(speed**2 / radius) * math.cos(theta),
                -(speed**2 / radius) * math.sin(theta),
                0.0,
            )
        if s < 4.0 * half_straight + math.pi * radius:
            local_s = s - (2.0 * half_straight + math.pi * radius)
            return (
                (half_straight - local_s, -radius, 0.0),
                (-speed, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )

        local_s = s - (4.0 * half_straight + math.pi * radius)
        theta = -math.pi / 2.0 - local_s / radius
        return (
            -half_straight + radius * math.cos(theta),
            radius * math.sin(theta),
            0.0,
        ), (
            speed * math.sin(theta),
            -speed * math.cos(theta),
            0.0,
        ), (
            -(speed**2 / radius) * math.cos(theta),
            -(speed**2 / radius) * math.sin(theta),
            0.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
