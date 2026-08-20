#include "robotcore_sensors/imu_conditioning.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace robotcore_sensors
{
class ImuConditionerComponent final : public rclcpp::Node
{
public:
  explicit ImuConditionerComponent(const rclcpp::NodeOptions & options)
  : Node("imu_conditioning", options), updater_(this)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/hardware/aboard_imu_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/sensors/external_imu");
    gyro_cutoff_hz_ = declare_parameter<double>("gyro_low_pass_cutoff_hz", 20.0);
    accel_cutoff_hz_ = declare_parameter<double>("accel_low_pass_cutoff_hz", 15.0);
    filter_reset_gap_s_ = declare_parameter<double>("filter_reset_gap_s", 0.20);
    gyro_floor_ = declare_parameter<double>("gyro_noise_floor_rps", 0.01);
    accel_floor_ = declare_parameter<double>("accel_noise_floor_mps2", 0.15);
    const auto mounting = declare_parameter<std::vector<double>>(
      "base_to_imu_rpy_rad", {0.0, 0.0, 0.0});
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    if (mounting.size() != 3U) {
      throw std::runtime_error("base_to_imu_rpy_rad requires three values");
    }
    if (gyro_cutoff_hz_ <= 0.0 || accel_cutoff_hz_ <= 0.0 || filter_reset_gap_s_ <= 0.0) {
      throw std::runtime_error("IMU low-pass cutoffs and reset gap must be positive");
    }
    base_from_imu_ = Eigen::AngleAxisd(mounting[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(mounting[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(mounting[0], Eigen::Vector3d::UnitX());
    gyro_filter_ = std::make_unique<TimestampedVectorLowPass>(
      gyro_cutoff_hz_, filter_reset_gap_s_);
    accel_filter_ = std::make_unique<TimestampedVectorLowPass>(
      accel_cutoff_hz_, filter_reset_gap_s_);

    output_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic_, rclcpp::SensorDataQoS().keep_last(8));
    input_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(32),
      std::bind(&ImuConditionerComponent::on_imu, this, std::placeholders::_1));
    updater_.setHardwareID("external-uart8-imu");
    updater_.add("External IMU conditioning", this, &ImuConditionerComponent::diagnose);
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {updater_.force_update();});
    RCLCPP_INFO(
      get_logger(),
      "Using factory-calibrated IMU output with %.1f Hz gyro and %.1f Hz acceleration low-pass",
      gyro_cutoff_hz_, accel_cutoff_hz_);
  }

private:
  static Eigen::Vector3d vector(const geometry_msgs::msg::Vector3 & value)
  {return {value.x, value.y, value.z};}

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const Eigen::Vector3d raw_gyro = vector(message->angular_velocity);
    const Eigen::Vector3d raw_accel = vector(message->linear_acceleration);
    const std::int64_t stamp = rclcpp::Time(message->header.stamp).nanoseconds();
    if (stamp <= 0 || !raw_gyro.allFinite() || !raw_accel.allFinite()) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++invalid_frames_;
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    ++input_count_;
    if (last_stamp_ns_ > 0 && stamp <= last_stamp_ns_) {
      ++non_monotonic_frames_;
      return;
    }
    const Eigen::Vector3d base_gyro = rotate_imu_vector_to_base(raw_gyro, base_from_imu_);
    const Eigen::Vector3d base_accel = rotate_imu_vector_to_base(raw_accel, base_from_imu_);
    const Eigen::Vector3d filtered_gyro = gyro_filter_->update(base_gyro, stamp);
    const Eigen::Vector3d filtered_accel = accel_filter_->update(base_accel, stamp);

    sensor_msgs::msg::Imu output = *message;
    output.header.frame_id = base_frame_id_;
    output.orientation.x = 0.0;
    output.orientation.y = 0.0;
    output.orientation.z = 0.0;
    output.orientation.w = 1.0;
    output.angular_velocity.x = filtered_gyro.x();
    output.angular_velocity.y = filtered_gyro.y();
    output.angular_velocity.z = filtered_gyro.z();
    output.linear_acceleration.x = filtered_accel.x();
    output.linear_acceleration.y = filtered_accel.y();
    output.linear_acceleration.z = filtered_accel.z();
    output.orientation_covariance.fill(0.0);
    output.orientation_covariance[0] = -1.0;
    const Eigen::Matrix3d gyro_covariance = condition_imu_covariance(
      message->angular_velocity_covariance, base_from_imu_, gyro_floor_);
    const Eigen::Matrix3d accel_covariance = condition_imu_covariance(
      message->linear_acceleration_covariance, base_from_imu_, accel_floor_);
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        output.angular_velocity_covariance[3 * row + column] = gyro_covariance(row, column);
        output.linear_acceleration_covariance[3 * row + column] = accel_covariance(row, column);
      }
    }
    output_pub_->publish(output);
    last_stamp_ns_ = stamp;
    ++published_count_;
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool flowing = published_count_ > 0U;
    status.summary(
      flowing ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      flowing ? "factory-calibrated IMU data flowing" : "waiting for external IMU data");
    status.add("input_frames", input_count_);
    status.add("published_frames", published_count_);
    status.add("invalid_frames", invalid_frames_);
    status.add("non_monotonic_frames", non_monotonic_frames_);
    status.add("gyro_low_pass_cutoff_hz", gyro_cutoff_hz_);
    status.add("accel_low_pass_cutoff_hz", accel_cutoff_hz_);
  }

  std::string input_topic_, output_topic_, base_frame_id_;
  double gyro_cutoff_hz_{}, accel_cutoff_hz_{}, filter_reset_gap_s_{};
  double gyro_floor_{}, accel_floor_{};
  Eigen::Matrix3d base_from_imu_{Eigen::Matrix3d::Identity()};
  std::unique_ptr<TimestampedVectorLowPass> gyro_filter_;
  std::unique_ptr<TimestampedVectorLowPass> accel_filter_;
  std::mutex state_mutex_;
  std::int64_t last_stamp_ns_{};
  std::uint64_t input_count_{}, published_count_{}, invalid_frames_{}, non_monotonic_frames_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr output_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr input_sub_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace robotcore_sensors
RCLCPP_COMPONENTS_REGISTER_NODE(robotcore_sensors::ImuConditionerComponent)
