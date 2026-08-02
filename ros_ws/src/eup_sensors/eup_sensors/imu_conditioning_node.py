"""Bias-calibrate UART8 gyro data before it enters the fixed-rate estimator."""

from __future__ import annotations

from copy import deepcopy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .imu_conditioning_math import gyro_calibration, is_stationary, message_period_is_usable, vector3


def stamp_ns(message: Imu) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def finite_vector(message_vector) -> np.ndarray | None:
    try:
        return vector3([message_vector.x, message_vector.y, message_vector.z])
    except ValueError:
        return None


class ImuConditioningNode(Node):
    """Publish calibrated angular velocity while leaving unsafe fields unavailable."""

    def __init__(self):
        super().__init__("imu_conditioning")
        self.declare_parameter("input_topic", "/hardware/aboard_imu_raw")
        self.declare_parameter("output_topic", "/sensors/external_imu")
        self.declare_parameter("status_topic", "/localization/external_imu_ready")
        self.declare_parameter("calibration_sample_count", 200)
        self.declare_parameter("minimum_sample_interval_s", 0.01)
        self.declare_parameter("maximum_sample_gap_s", 0.20)
        self.declare_parameter("stationary_gyro_limit_rps", 0.04)
        self.declare_parameter("stationary_acceleration_tolerance_mps2", 1.0)
        self.declare_parameter("gyro_noise_floor_rps", 0.01)

        self.publisher = self.create_publisher(
            Imu, str(self.get_parameter("output_topic").value), 50
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            Bool, str(self.get_parameter("status_topic").value), status_qos
        )
        self.create_subscription(
            Imu, str(self.get_parameter("input_topic").value), self.on_imu, 100
        )
        self.create_service(Trigger, "~/calibrate", self.on_calibrate)

        self.samples: list[np.ndarray] = []
        self.gyro_bias = np.zeros(3, dtype=np.float64)
        self.gyro_stddev = np.full(3, 0.01, dtype=np.float64)
        self.last_input_stamp_ns: int | None = None
        self.last_calibration_stamp_ns: int | None = None
        self.collecting = False
        self.ready = False
        self.last_progress_bucket = -1
        # The ControlInterface Status action is the only operator calibration
        # entry point. Do not begin collecting stationary samples at startup.
        self.publish_status()

    def start_calibration(self):
        self.samples = []
        self.last_input_stamp_ns = None
        self.last_calibration_stamp_ns = None
        self.collecting = True
        self.ready = False
        self.last_progress_bucket = -1
        self.publish_status()
        self.get_logger().info(
            "External IMU gyro calibration started; keep the vehicle stationary"
        )

    def on_calibrate(self, _request: Trigger.Request, response: Trigger.Response):
        self.start_calibration()
        response.success = True
        response.message = "external IMU calibration restarted; keep the vehicle stationary"
        return response

    def on_imu(self, message: Imu):
        sample_stamp_ns = stamp_ns(message)
        if sample_stamp_ns <= 0:
            sample_stamp_ns = self.get_clock().now().nanoseconds
        if not message_period_is_usable(
            self.last_input_stamp_ns,
            sample_stamp_ns,
            float(self.get_parameter("maximum_sample_gap_s").value),
        ):
            self.last_input_stamp_ns = sample_stamp_ns
            return
        self.last_input_stamp_ns = sample_stamp_ns

        acceleration = finite_vector(message.linear_acceleration)
        angular_velocity = finite_vector(message.angular_velocity)
        if acceleration is None or angular_velocity is None:
            return
        if self.collecting:
            self.collect_sample(acceleration, angular_velocity, sample_stamp_ns)
        if not self.ready:
            return

        corrected = angular_velocity - self.gyro_bias
        output = deepcopy(message)
        output.angular_velocity.x = float(corrected[0])
        output.angular_velocity.y = float(corrected[1])
        output.angular_velocity.z = float(corrected[2])
        self.set_covariance_diagonal(
            output.angular_velocity_covariance, self.gyro_stddev**2
        )
        # UART8 does not provide orientation. Acceleration is retained for
        # diagnostics, but it is not gravity-compensated and must not enter the
        # position estimator until mounting and gravity handling are validated.
        output.orientation_covariance[0] = -1.0
        output.linear_acceleration_covariance[0] = -1.0
        self.publisher.publish(output)

    def collect_sample(
        self, acceleration: np.ndarray, angular_velocity: np.ndarray, sample_stamp_ns: int
    ):
        minimum_interval_ns = int(
            max(0.0, float(self.get_parameter("minimum_sample_interval_s").value)) * 1e9
        )
        if (
            self.last_calibration_stamp_ns is not None
            and sample_stamp_ns - self.last_calibration_stamp_ns < minimum_interval_ns
        ):
            return
        if not is_stationary(
            acceleration,
            angular_velocity,
            acceleration_tolerance_mps2=float(
                self.get_parameter("stationary_acceleration_tolerance_mps2").value
            ),
            angular_velocity_limit_rps=float(
                self.get_parameter("stationary_gyro_limit_rps").value
            ),
        ):
            return
        self.samples.append(angular_velocity)
        self.last_calibration_stamp_ns = sample_stamp_ns
        count = len(self.samples)
        bucket = count // 50
        if bucket != self.last_progress_bucket:
            self.last_progress_bucket = bucket
            self.get_logger().info(
                f"External IMU calibration accepted {count}/{self.required_samples()} samples"
            )
        if count >= self.required_samples():
            self.finish_calibration()

    def finish_calibration(self):
        self.gyro_bias, self.gyro_stddev = gyro_calibration(
            self.samples,
            noise_floor_rps=float(self.get_parameter("gyro_noise_floor_rps").value),
        )
        self.collecting = False
        self.ready = True
        self.publish_status()
        self.get_logger().info(
            "External IMU gyro calibration complete; bias="
            f"{np.array2string(self.gyro_bias, precision=6)}, stddev="
            f"{np.array2string(self.gyro_stddev, precision=6)}"
        )

    def required_samples(self) -> int:
        return max(1, int(self.get_parameter("calibration_sample_count").value))

    @staticmethod
    def set_covariance_diagonal(covariance, variances: np.ndarray):
        for index in range(9):
            covariance[index] = 0.0
        covariance[0] = float(variances[0])
        covariance[4] = float(variances[1])
        covariance[8] = float(variances[2])

    def publish_status(self):
        message = Bool()
        message.data = bool(self.ready)
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ImuConditioningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
