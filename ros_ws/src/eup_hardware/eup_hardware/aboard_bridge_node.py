"""ROS-side bridge for the STM32 A-board UART6 motor interface."""

import os
import fcntl
import math
import select
import termios
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from eup_interfaces.msg import BoardStatus, ThrusterCommand, ThrusterState

from .packet import (
    A_BOARD_PWM_FEEDBACK_HEADER,
    A_BOARD_PWM_FEEDBACK_LEN,
    A_BOARD_TELEMETRY_HEADER,
    A_BOARD_TELEMETRY_LEN,
    build_uart_direct_pwm_frame,
    normalized_to_direct_pwm_offsets,
    parse_uart_telemetry_frame,
    parse_uart_pwm_feedback_frame,
)


def baud_constant(baud):
    mapping = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    if baud not in mapping:
        raise ValueError(f"unsupported baud rate without pyserial: {baud}")
    return mapping[baud]


def configure_port(fd, baud):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = baud_constant(baud)
    attrs[5] = baud_constant(baud)
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


class AboardBridgeNode(Node):
    """Bridge ROS thruster commands to A-board UART6 and publish real readings."""

    def __init__(self):
        super().__init__("aboard_bridge_node")
        self.declare_parameter(
            "serial_port",
            "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A033320-if00",
        )
        self.declare_parameter("baud", 115200)
        self.declare_parameter("neutral_us", 1500)
        # 1500 us neutral with a hard 1400–1600 us command range.
        self.declare_parameter("span_us", 100)
        self.declare_parameter("thruster_channel_offset", 0)
        self.declare_parameter("heartbeat_timeout_ms", 250)
        # Maintain an explicit neutral PWM heartbeat while Disarmed.  100 Hz
        # leaves margin for ESC watchdogs that are stricter than 50 Hz.
        self.declare_parameter("command_write_hz", 100.0)
        self.declare_parameter("command_timeout_ms", 150)
        # Read the serial receive queue often enough that the estimator can
        # consume a fresh IMU sample for every 60 Hz policy action.  This does
        # not fabricate samples: the actual IMU publication rate remains
        # bounded by the A-board telemetry rate.
        self.declare_parameter("poll_rate_hz", 60.0)
        self.declare_parameter("imu_topic", "/hardware/aboard_imu")
        self.declare_parameter("imu_frame_id", "aboard_imu_link")
        self.declare_parameter("publish_imu", False)
        self.declare_parameter("prefer_uart8_imu", True)
        self.declare_parameter("uart8_imu_timeout_ms", 1000)
        # New A-board firmware must forward the Bewei counter in frame-3 word
        # 7. Fail closed when old firmware repeats its latest sample with a new
        # Jetson timestamp, since that corrupts rate and integration metrics.
        self.declare_parameter("require_uart8_sample_id", True)
        self.declare_parameter("attitude_timeout_ms", 250)
        # Conservative startup noise values for state estimators.  They must
        # be replaced with values measured from a stationary vehicle before
        # enabling linear-acceleration position propagation.
        self.declare_parameter("imu_orientation_stddev_rad", math.radians(5.0))
        self.declare_parameter("imu_angular_velocity_stddev_rps", 0.05)
        self.declare_parameter("imu_linear_acceleration_stddev_mps2", 0.5)
        self.declare_parameter("depth_topic", "/hardware/aboard_depth_m")
        self.declare_parameter("depth_velocity_topic", "/hardware/aboard_depth_velocity_mps")

        self.serial_fd = None
        self.serial_lock_conflict = False
        self.rx_buffer = bytearray()
        self.last_enable = False
        self.last_command_offsets = [0] * 16
        self.last_command_time = 0.0
        self.last_feedback_pwm = []
        self.last_feedback_time = 0.0
        self.last_attitude_telemetry = {}
        self.last_attitude_time = 0.0
        self.last_legacy_acceleration = {}
        self.last_uart8_imu_time = 0.0
        self.last_uart8_sample_id = None
        self.feedback_frames = 0
        self.bad_feedback_frames = 0

        self.status_pub = self.create_publisher(BoardStatus, "/hardware/board_status", 10)
        self.thruster_state_pub = self.create_publisher(ThrusterState, "/robot/thruster_state", 10)
        self.imu_pub = None
        if bool(self.get_parameter("publish_imu").value):
            self.imu_pub = self.create_publisher(Imu, str(self.get_parameter("imu_topic").value), 20)
        self.depth_pub = self.create_publisher(Float32, str(self.get_parameter("depth_topic").value), 20)
        self.depth_velocity_pub = self.create_publisher(
            Float32, str(self.get_parameter("depth_velocity_topic").value), 20
        )
        self.create_subscription(ThrusterCommand, "/control/thruster_cmd", self.on_cmd, 10)

        self.open_serial()
        if self.serial_lock_conflict:
            raise RuntimeError("another aboard_bridge_node already owns the A-board serial port")
        poll_rate = max(float(self.get_parameter("poll_rate_hz").value), 1.0)
        self.timer = self.create_timer(1.0 / poll_rate, self.poll_and_publish)
        self.command_timer = self.create_timer(
            1.0 / max(float(self.get_parameter("command_write_hz").value), 1.0),
            self.write_latest_command,
        )

    def open_serial(self):
        if self.serial_fd is not None:
            return
        port = str(self.get_parameter("serial_port").value)
        baud = int(self.get_parameter("baud").value)
        try:
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            configure_port(fd, baud)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(fd)
            self.serial_lock_conflict = True
            self.get_logger().fatal(
                f"A-board serial port {port} is already owned by another aboard_bridge_node"
            )
            return
        except OSError as exc:
            self.get_logger().error(f"Failed to open A-board serial port {port}: {exc}")
            return
        self.serial_fd = fd
        self.get_logger().info(f"Opened A-board UART bridge on {port} at {baud} baud")

    def close_serial(self):
        if self.serial_fd is not None:
            os.close(self.serial_fd)
            self.serial_fd = None

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()

    def on_cmd(self, msg):
        span = int(self.get_parameter("span_us").value)
        channel_offset = int(self.get_parameter("thruster_channel_offset").value)
        self.last_enable = bool(msg.enable)

        if self.serial_fd is None:
            self.open_serial()
        if self.serial_fd is None:
            return

        offsets = (
            normalized_to_direct_pwm_offsets(msg.normalized, span, channel_offset)
            if msg.enable
            else [0] * 16
        )
        self.last_command_offsets = offsets
        self.last_command_time = time.monotonic()
        self.write_offsets(offsets)

    def write_latest_command(self):
        timeout_s = float(self.get_parameter("command_timeout_ms").value) / 1000.0
        if self.last_enable and (time.monotonic() - self.last_command_time) <= timeout_s:
            offsets = self.last_command_offsets
        else:
            offsets = [0] * 16
            self.last_enable = False
        self.write_offsets(offsets)

    def write_offsets(self, offsets):
        if self.serial_fd is None:
            self.open_serial()
        if self.serial_fd is None:
            return
        frame = build_uart_direct_pwm_frame(offsets)
        try:
            remaining = memoryview(frame)
            while remaining:
                try:
                    written = os.write(self.serial_fd, remaining)
                except BlockingIOError:
                    _, writable, _ = select.select([], [self.serial_fd], [], 0.05)
                    if not writable:
                        raise
                    continue
                if written <= 0:
                    raise OSError("A-board serial write returned zero bytes")
                remaining = remaining[written:]
            termios.tcdrain(self.serial_fd)
        except OSError as exc:
            self.get_logger().error(f"Failed to write A-board command frame: {exc}")
            self.close_serial()

    def poll_and_publish(self):
        if self.serial_fd is None:
            self.open_serial()
        self.read_serial_feedback()
        self.publish_status()
        self.publish_thruster_state()

    def read_serial_feedback(self):
        if self.serial_fd is None:
            return
        while True:
            try:
                chunk = os.read(self.serial_fd, 4096)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().error(f"Failed to read A-board serial feedback: {exc}")
                self.close_serial()
                break
            if not chunk:
                break
            self.rx_buffer.extend(chunk)

        while True:
            start, header = self.next_frame_start()
            if start < 0:
                if len(self.rx_buffer) > max(A_BOARD_PWM_FEEDBACK_LEN, A_BOARD_TELEMETRY_LEN):
                    del self.rx_buffer[:-1]
                return
            if start:
                del self.rx_buffer[:start]
            frame_len = A_BOARD_PWM_FEEDBACK_LEN if header == A_BOARD_PWM_FEEDBACK_HEADER else A_BOARD_TELEMETRY_LEN
            if len(self.rx_buffer) < frame_len:
                return

            frame = bytes(self.rx_buffer[:frame_len])
            if header == A_BOARD_PWM_FEEDBACK_HEADER:
                parsed = self.parse_pwm_feedback(frame)
            else:
                parsed = self.parse_telemetry(frame)
            if parsed:
                del self.rx_buffer[:frame_len]

    def next_frame_start(self):
        starts = []
        for header in (A_BOARD_PWM_FEEDBACK_HEADER, A_BOARD_TELEMETRY_HEADER):
            index = self.rx_buffer.find(header)
            if index >= 0:
                starts.append((index, header))
        if not starts:
            return -1, b""
        return min(starts, key=lambda item: item[0])

    def parse_pwm_feedback(self, frame: bytes):
        try:
            self.last_feedback_pwm = parse_uart_pwm_feedback_frame(frame)
        except ValueError:
            self.bad_feedback_frames += 1
            del self.rx_buffer[:1]
            return False

        self.feedback_frames += 1
        self.last_feedback_time = time.monotonic()
        return True

    def parse_telemetry(self, frame: bytes):
        try:
            telemetry = parse_uart_telemetry_frame(frame)
        except ValueError:
            del self.rx_buffer[:1]
            return False
        if telemetry is not None:
            # UART8 and the legacy A-board IMU use different telemetry frame
            # layouts.  An invalid UART8 frame is a *missing* IMU sample, not
            # a legacy frame with zero-valued acceleration.
            if "uart8_imu_valid" in telemetry:
                if telemetry["uart8_imu_valid"]:
                    sample_id = int(telemetry.get("uart8_imu_sample_id", 0))
                    if bool(self.get_parameter("require_uart8_sample_id").value):
                        if self.last_uart8_sample_id is None and sample_id == 0:
                            return True
                        if sample_id == self.last_uart8_sample_id:
                            return True
                    self.last_uart8_sample_id = sample_id
                    self.last_uart8_imu_time = time.monotonic()
                    # UART8 has raw gyro and specific force but no attitude.
                    # Reuse the recent legacy attitude only when it is fresh;
                    # otherwise publish the ROS orientation-unavailable
                    # sentinel instead of pretending it is valid.
                    self.publish_imu(self.with_recent_attitude(telemetry))
                return True
            if "depth_m" in telemetry:
                self.publish_depth(telemetry["depth_m"])
            if "depth_velocity" in telemetry:
                self.publish_depth_velocity(telemetry["depth_velocity"])
            if all(key in telemetry for key in ("roll_deg", "pitch_deg", "yaw_deg")):
                self.last_attitude_telemetry = dict(telemetry)
                self.last_attitude_time = time.monotonic()
            if all(key in telemetry for key in ("accel_x_mps2", "accel_y_mps2", "accel_z_mps2")):
                self.last_legacy_acceleration = dict(telemetry)
                # The legacy protocol sends attitude/gyro and acceleration in
                # separate frames. Publish exactly once, when the matching
                # acceleration frame arrives. The old code published once for
                # each frame, causing 1 ms duplicate IMU samples.
                if not self.uart8_imu_is_fresh():
                    assembled = self.with_recent_attitude(self.last_legacy_acceleration)
                    if self.has_imu_motion_fields(assembled):
                        self.publish_imu(assembled)
        return True

    def uart8_imu_is_fresh(self):
        if not bool(self.get_parameter("prefer_uart8_imu").value):
            return False
        timeout_s = max(
            0.0,
            float(self.get_parameter("uart8_imu_timeout_ms").value) / 1000.0,
        )
        return (
            self.last_uart8_imu_time > 0.0
            and time.monotonic() - self.last_uart8_imu_time <= timeout_s
        )

    def attitude_is_fresh(self):
        timeout_s = max(0.0, float(self.get_parameter("attitude_timeout_ms").value) / 1000.0)
        return (
            self.last_attitude_time > 0.0
            and time.monotonic() - self.last_attitude_time <= timeout_s
        )

    def with_recent_attitude(self, telemetry):
        """Add a recent legacy attitude without overwriting raw IMU fields."""

        combined = {}
        if self.attitude_is_fresh():
            combined.update(self.last_attitude_telemetry)
        combined.update(telemetry)
        return combined

    @staticmethod
    def has_imu_motion_fields(telemetry):
        required = (
            "gyro_x_dps",
            "gyro_y_dps",
            "gyro_z_dps",
            "accel_x_mps2",
            "accel_y_mps2",
            "accel_z_mps2",
        )
        return all(key in telemetry for key in required)

    def publish_depth(self, depth_m):
        msg = Float32()
        msg.data = float(depth_m)
        self.depth_pub.publish(msg)

    def publish_depth_velocity(self, depth_velocity_mps):
        """Publish the A-board's measured positive-down depth velocity."""

        msg = Float32()
        msg.data = float(depth_velocity_mps)
        self.depth_velocity_pub.publish(msg)

    def publish_imu(self, telemetry):
        if self.imu_pub is None:
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("imu_frame_id").value)

        if all(key in telemetry for key in ("roll_deg", "pitch_deg", "yaw_deg")):
            roll = math.radians(float(telemetry["roll_deg"]))
            pitch = math.radians(float(telemetry["pitch_deg"]))
            yaw = math.radians(float(telemetry["yaw_deg"]))
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            msg.orientation.w = cr * cp * cy + sr * sp * sy
            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy
            orientation_variance = self.imu_variance("imu_orientation_stddev_rad")
            for index in (0, 4, 8):
                msg.orientation_covariance[index] = orientation_variance
        else:
            # UART8 carries gyro and acceleration but not an attitude estimate.
            msg.orientation_covariance[0] = -1.0
        if all(key in telemetry for key in ("gyro_x_dps", "gyro_y_dps", "gyro_z_dps")):
            msg.angular_velocity.x = math.radians(float(telemetry["gyro_x_dps"]))
            msg.angular_velocity.y = math.radians(float(telemetry["gyro_y_dps"]))
            msg.angular_velocity.z = math.radians(float(telemetry["gyro_z_dps"]))
            angular_velocity_variance = self.imu_variance("imu_angular_velocity_stddev_rps")
            for index in (0, 4, 8):
                msg.angular_velocity_covariance[index] = angular_velocity_variance
        else:
            msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration.x = float(telemetry.get("accel_x_mps2", math.nan))
        msg.linear_acceleration.y = float(telemetry.get("accel_y_mps2", math.nan))
        msg.linear_acceleration.z = float(telemetry.get("accel_z_mps2", math.nan))
        if not all(
            math.isfinite(value)
            for value in (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
        ):
            msg.linear_acceleration_covariance[0] = -1.0
        else:
            linear_acceleration_variance = self.imu_variance("imu_linear_acceleration_stddev_mps2")
            for index in (0, 4, 8):
                msg.linear_acceleration_covariance[index] = linear_acceleration_variance
        self.imu_pub.publish(msg)

    def imu_variance(self, parameter_name):
        """Return a finite positive variance for a published IMU component."""

        stddev = float(self.get_parameter(parameter_name).value)
        if not math.isfinite(stddev) or stddev <= 0.0:
            self.get_logger().warn(f"{parameter_name} must be positive; using 1.0")
            stddev = 1.0
        return stddev * stddev

    def feedback_fresh(self):
        if not self.last_feedback_pwm:
            return False
        timeout_s = float(self.get_parameter("heartbeat_timeout_ms").value) / 1000.0
        return (time.monotonic() - self.last_feedback_time) <= timeout_s

    def publish_status(self):
        fresh = self.feedback_fresh()
        status = BoardStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.connected = bool(self.serial_fd is not None and fresh)
        status.heartbeat_ok = bool(fresh)
        status.estop_active = not self.last_enable
        status.failsafe_active = not fresh
        status.pwm_us = [int(value) for value in self.last_feedback_pwm]
        status.bus_voltage_v = math.nan
        status.board_temp_c = math.nan
        # The current A-board feedback protocol has no firmware-version field.
        # Leave this empty rather than presenting a bridge/protocol label as a
        # hardware-reported firmware revision.
        status.firmware_version = ""
        self.status_pub.publish(status)

    def publish_thruster_state(self):
        fresh = self.feedback_fresh()
        neutral = int(self.get_parameter("neutral_us").value)
        span = max(1, int(self.get_parameter("span_us").value))
        channel_offset = max(0, min(8, int(self.get_parameter("thruster_channel_offset").value)))
        feedback = [0.0] * 8
        for index, pwm_us in enumerate(self.last_feedback_pwm[channel_offset : channel_offset + 8]):
            feedback[index] = max(-1.0, min(1.0, (float(pwm_us) - neutral) / span))

        state = ThrusterState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.normalized_feedback = feedback
        state.pwm_us = [int(value) for value in self.last_feedback_pwm]
        state.healthy = [bool(fresh)] * len(state.pwm_us)
        self.thruster_state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = AboardBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
