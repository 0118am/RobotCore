#include "eup_sensors/geometry.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

namespace eup_sensors
{
class ImuConditionerComponent final : public rclcpp::Node
{
public:
  explicit ImuConditionerComponent(const rclcpp::NodeOptions & options)
  : Node("imu_conditioning", options), updater_(this)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/hardware/aboard_imu_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/sensors/external_imu");
    fusion_output_topic_ = declare_parameter<std::string>(
      "fusion_output_topic", "/sensors/external_imu_specific_force");
    status_topic_ = declare_parameter<std::string>("status_topic", "/localization/external_imu_ready");
    calibration_file_ = declare_parameter<std::string>(
      "calibration_file", "/etc/robotcore/external_imu_calibration.yaml");
    expected_sensor_serial_ = declare_parameter<std::string>("expected_sensor_serial", "");
    expected_config_hash_ = declare_parameter<std::string>("expected_config_hash", "");
    required_samples_ = declare_parameter<int>("calibration_sample_count", 250);
    timeout_s_ = declare_parameter<double>("calibration_timeout_s", 10.0);
    gyro_limit_ = declare_parameter<double>("stationary_gyro_limit_rps", 0.04);
    accel_tolerance_ = declare_parameter<double>("stationary_acceleration_tolerance_mps2", 1.0);
    gyro_floor_ = declare_parameter<double>("gyro_noise_floor_rps", 0.01);
    accel_floor_ = declare_parameter<double>("accel_noise_floor_mps2", 0.15);
    const auto mounting = declare_parameter<std::vector<double>>(
      "base_to_imu_rpy_rad", {0.0, 0.0, 0.0});
    if (mounting.size() != 3U) {throw std::runtime_error("base_to_imu_rpy_rad requires three values");}
    base_from_imu_ = Eigen::AngleAxisd(mounting[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(mounting[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(mounting[0], Eigen::Vector3d::UnitX());
    load_persistent_calibration();

    output_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS().keep_last(8));
    fusion_output_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      fusion_output_topic_, rclcpp::SensorDataQoS().keep_last(8));
    // Humble intra-process communication rejects transient-local durability.
    // Disable IPC only for this low-rate latched readiness publisher; the
    // 100 Hz IMU publisher remains zero-copy inside the estimator container.
    rclcpp::PublisherOptions status_options;
    status_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    status_pub_ = create_publisher<std_msgs::msg::Bool>(status_topic_,
      rclcpp::QoS(1).reliable().transient_local(), status_options);
    input_sub_ = create_subscription<sensor_msgs::msg::Imu>(input_topic_, rclcpp::SensorDataQoS().keep_last(32),
      std::bind(&ImuConditionerComponent::on_imu, this, std::placeholders::_1));
    calibrate_service_ = create_service<std_srvs::srv::Trigger>(
      "~/calibrate", std::bind(&ImuConditionerComponent::on_calibrate, this,
      std::placeholders::_1, std::placeholders::_2));
    updater_.setHardwareID("external-uart8-imu");
    updater_.add("External IMU calibration", this, &ImuConditionerComponent::diagnose);
    diagnostic_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      check_calibration_timeout();
      updater_.force_update();
    });
    // Calibration is intentionally operator-triggered from the ControlInterface
    // Status panel. Publish the initial not-ready state without collecting.
    publish_ready();
  }

private:
  static Eigen::Vector3d vector(const geometry_msgs::msg::Vector3 & v)
  {return {v.x, v.y, v.z};}

  void load_persistent_calibration()
  {
    accel_matrix_.setIdentity();
    gyro_matrix_.setIdentity();
    persistent_accel_bias_.setZero();
    persistent_gyro_bias_.setZero();
    try {
      const YAML::Node root = YAML::LoadFile(calibration_file_);
      const auto matrix = root["accel_matrix"].as<std::vector<double>>();
      const auto accel_bias = root["accel_bias_mps2"].as<std::vector<double>>();
      const auto gyro_bias = root["gyro_bias_rps"].as<std::vector<double>>();
      if (matrix.size() != 9U || accel_bias.size() != 3U || gyro_bias.size() != 3U) {
        throw std::runtime_error("calibration arrays have wrong dimensions");
      }
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {accel_matrix_(row, col) = matrix[3 * row + col];}
        persistent_accel_bias_[row] = accel_bias[row];
        persistent_gyro_bias_[row] = gyro_bias[row];
      }
      if (root["gyro_matrix"]) {
        const auto values = root["gyro_matrix"].as<std::vector<double>>();
        if (values.size() != 9U) {throw std::runtime_error("gyro_matrix has wrong dimensions");}
        for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col) {
          gyro_matrix_(row, col) = values[3 * row + col];
        }
      }
      if (!root["sensor_serial"] || !root["config_hash"]) {
        throw std::runtime_error("calibration requires sensor_serial and config_hash");
      }
      calibration_serial_ = root["sensor_serial"].as<std::string>();
      calibration_hash_ = root["config_hash"].as<std::string>();
      const bool hash_valid = calibration_hash_.size() == 71U &&
        calibration_hash_.rfind("sha256:", 0U) == 0U &&
        std::all_of(calibration_hash_.begin() + 7, calibration_hash_.end(),
          [](unsigned char value) {return std::isxdigit(value) != 0;});
      if (calibration_serial_.empty() || !hash_valid) {
        throw std::runtime_error("calibration identity/hash is empty or still a template");
      }
      if (!expected_sensor_serial_.empty() && calibration_serial_ != expected_sensor_serial_) {
        throw std::runtime_error("calibration sensor_serial does not match configured sensor");
      }
      if (!expected_config_hash_.empty() && calibration_hash_ != expected_config_hash_) {
        throw std::runtime_error("calibration config_hash does not match configured hash");
      }
      persistent_calibration_valid_ = accel_matrix_.allFinite() && accel_matrix_.determinant() > 1e-6;
    } catch (const std::exception & error) {
      persistent_calibration_valid_ = false;
      calibration_error_ = error.what();
      RCLCPP_WARN(get_logger(), "Acceleration calibration unavailable; gyro-only fallback: %s", error.what());
    }
  }

  void restart_calibration()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    gyro_samples_.clear(); accel_samples_.clear();
    ready_ = false; accel_enabled_ = false; timed_out_ = false; collecting_ = true;
    input_count_ = 0;
    calibration_start_ = now();
    publish_ready();
  }

  void check_calibration_timeout()
  {
    bool newly_timed_out = false;
    std::uint64_t input_count = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (collecting_ && !ready_ && !timed_out_ &&
        (now() - calibration_start_).seconds() > timeout_s_)
      {
        timed_out_ = true;
        collecting_ = false;
        newly_timed_out = true;
        input_count = input_count_;
      }
    }
    if (newly_timed_out) {
      publish_ready();
      if (input_count == 0U) {
        RCLCPP_ERROR(get_logger(), "IMU calibration timed out: no input frames received");
      } else {
        RCLCPP_ERROR(get_logger(),
          "IMU calibration timed out after %llu input frames; keep vehicle stationary",
          static_cast<unsigned long long>(input_count));
      }
    }
  }

  void on_calibrate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    restart_calibration();
    response->success = true;
    response->message = "IMU calibration started; keep vehicle stationary";
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const Eigen::Vector3d raw_gyro = vector(message->angular_velocity);
    const Eigen::Vector3d raw_accel = vector(message->linear_acceleration);
    if (!raw_gyro.allFinite() || !raw_accel.allFinite()) {return;}
    const Eigen::Vector3d gyro = gyro_matrix_ * (raw_gyro - persistent_gyro_bias_);
    const Eigen::Vector3d accel = accel_matrix_ * (raw_accel - persistent_accel_bias_);
    Eigen::Vector3d startup_gyro_bias;
    Eigen::Vector3d startup_accel_baseline;
    Eigen::Vector3d startup_accel_residual;
    bool accel_enabled = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++input_count_;
      if (!ready_) {
        if (!collecting_) {return;}
        if (gyro.norm() <= gyro_limit_ && std::abs(accel.norm() - 9.80665) <= accel_tolerance_) {
          gyro_samples_.push_back(gyro);
          accel_samples_.push_back(accel);
        }
        if (static_cast<int>(gyro_samples_.size()) >= std::max(1, required_samples_)) {
          finish_calibration();
        } else if ((now() - calibration_start_).seconds() > timeout_s_) {
          timed_out_ = true;
          collecting_ = false;
        }
        if (!ready_) {return;}
      }
      startup_gyro_bias = startup_gyro_bias_;
      startup_accel_baseline = startup_accel_baseline_;
      startup_accel_residual = startup_accel_residual_;
      accel_enabled = accel_enabled_;
    }
    sensor_msgs::msg::Imu output = *message;
    const Eigen::Vector3d corrected_gyro = gyro - startup_gyro_bias;
    const Eigen::Vector3d zeroed_accel = accel - startup_accel_baseline;
    const Eigen::Vector3d fusion_accel = accel - startup_accel_residual;
    output.angular_velocity.x = corrected_gyro.x();
    output.angular_velocity.y = corrected_gyro.y();
    output.angular_velocity.z = corrected_gyro.z();
    output.linear_acceleration.x = zeroed_accel.x();
    output.linear_acceleration.y = zeroed_accel.y();
    output.linear_acceleration.z = zeroed_accel.z();
    output.orientation_covariance[0] = -1.0;
    output.angular_velocity_covariance.fill(0.0);
    output.angular_velocity_covariance[0] = gyro_floor_ * gyro_floor_;
    output.angular_velocity_covariance[4] = gyro_floor_ * gyro_floor_;
    output.angular_velocity_covariance[8] = gyro_floor_ * gyro_floor_;
    output.linear_acceleration_covariance.fill(0.0);
    output.linear_acceleration_covariance[0] = accel_floor_ * accel_floor_;
    output.linear_acceleration_covariance[4] = accel_floor_ * accel_floor_;
    output.linear_acceleration_covariance[8] = accel_floor_ * accel_floor_;
    output_pub_->publish(output);

    // The operator/logging topic is intentionally referenced to the flat,
    // stationary startup pose.  The ESKF must instead receive ROS-standard
    // specific force with gravity preserved, so publish that on a dedicated
    // topic rather than silently changing the estimator's input semantics.
    sensor_msgs::msg::Imu fusion_output = output;
    fusion_output.linear_acceleration.x = fusion_accel.x();
    fusion_output.linear_acceleration.y = fusion_accel.y();
    fusion_output.linear_acceleration.z = fusion_accel.z();
    if (!accel_enabled) {
      fusion_output.linear_acceleration_covariance.fill(0.0);
      fusion_output.linear_acceleration_covariance[0] = -1.0;
    }
    fusion_output_pub_->publish(fusion_output);
    ++published_count_;
    last_stamp_ = rclcpp::Time(message->header.stamp);
  }

  void finish_calibration()
  {
    startup_gyro_bias_.setZero();
    startup_accel_baseline_.setZero();
    startup_accel_residual_.setZero();
    for (const auto & sample : gyro_samples_) {startup_gyro_bias_ += sample;}
    startup_gyro_bias_ /= static_cast<double>(gyro_samples_.size());
    for (const auto & sample : accel_samples_) {startup_accel_baseline_ += sample;}
    startup_accel_baseline_ /= static_cast<double>(accel_samples_.size());

    // The operator guarantees that base_link is level and stationary during
    // startup calibration.  That makes the expected gravity vector known from
    // the configured rigid mounting alone; no VIO, camera, or AprilTag input is
    // required for this calibration.
    const Eigen::Vector3d expected =
      base_from_imu_.transpose() * Eigen::Vector3d(0.0, 0.0, 9.80665);
    startup_accel_residual_ = startup_accel_baseline_ - expected;
    accel_enabled_ = startup_accel_residual_.norm() < 1.5;
    ready_ = true;
    collecting_ = false;
    publish_ready();
    RCLCPP_INFO(get_logger(), "IMU calibrated from %zu samples; acceleration %s",
      gyro_samples_.size(), accel_enabled_ ? "enabled" : "disabled");
  }

  void publish_ready()
  {
    std_msgs::msg::Bool status; status.data = ready_; status_pub_->publish(status);
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const int level = ready_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      (timed_out_ ? diagnostic_msgs::msg::DiagnosticStatus::ERROR : diagnostic_msgs::msg::DiagnosticStatus::WARN);
    const std::string message = ready_ ? "calibrated" :
      (timed_out_ ? (input_count_ == 0U ? "no IMU telemetry received before calibration timeout" :
      "calibration timed out") :
      (collecting_ ? "collecting stationary samples" : "waiting for operator calibration"));
    status.summary(level, message);
    status.add("imu_calibrated", ready_);
    status.add("accel_fusion_enabled", accel_enabled_);
    status.add("persistent_calibration_valid", persistent_calibration_valid_);
    status.add("calibration_samples", gyro_samples_.size());
    status.add("input_frames", input_count_);
    status.add("calibration_hash", calibration_hash_);
    status.add("calibration_sensor_serial", calibration_serial_);
    status.add("calibration_error", calibration_error_);
    status.add("published_count", published_count_);
  }

  std::string input_topic_, output_topic_, fusion_output_topic_, status_topic_;
  std::string calibration_file_, calibration_hash_, calibration_serial_;
  std::string calibration_error_, expected_sensor_serial_, expected_config_hash_;
  int required_samples_{}; double timeout_s_{}, gyro_limit_{}, accel_tolerance_{}, gyro_floor_{}, accel_floor_{};
  Eigen::Matrix3d accel_matrix_{Eigen::Matrix3d::Identity()}, gyro_matrix_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d base_from_imu_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d persistent_accel_bias_{Eigen::Vector3d::Zero()}, persistent_gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d startup_gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d startup_accel_baseline_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d startup_accel_residual_{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> gyro_samples_, accel_samples_;
  std::mutex state_mutex_;
  bool persistent_calibration_valid_{false}, ready_{false};
  bool accel_enabled_{false}, timed_out_{false}, collecting_{false};
  std::uint64_t input_count_{}, published_count_{};
  rclcpp::Time calibration_start_{0, 0, RCL_ROS_TIME}, last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr output_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr fusion_output_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr input_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace eup_sensors
RCLCPP_COMPONENTS_REGISTER_NODE(eup_sensors::ImuConditionerComponent)
