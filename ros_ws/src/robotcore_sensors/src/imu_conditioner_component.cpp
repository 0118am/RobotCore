#include "robotcore_sensors/geometry.hpp"
#include "robotcore_sensors/imu_conditioning.hpp"

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
#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <numeric>
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
    status_topic_ = declare_parameter<std::string>("status_topic", "/localization/external_imu_ready");
    calibration_file_ = declare_parameter<std::string>(
      "calibration_file", "/etc/robotcore/external_imu_calibration.yaml");
    required_samples_ = declare_parameter<int>("calibration_sample_count", 250);
    timeout_s_ = declare_parameter<double>("calibration_timeout_s", 10.0);
    input_freshness_timeout_s_ = declare_parameter<double>("input_freshness_timeout_s", 0.5);
    gyro_limit_ = declare_parameter<double>("stationary_gyro_limit_rps", 0.04);
    accel_tolerance_ = declare_parameter<double>("stationary_acceleration_tolerance_mps2", 1.0);
    gyro_floor_ = declare_parameter<double>("gyro_noise_floor_rps", 0.01);
    accel_floor_ = declare_parameter<double>("accel_noise_floor_mps2", 0.15);
    const auto mounting = declare_parameter<std::vector<double>>(
      "base_to_imu_rpy_rad", {0.0, 0.0, 0.0});
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    if (mounting.size() != 3U) {throw std::runtime_error("base_to_imu_rpy_rad requires three values");}
    base_from_imu_ = Eigen::AngleAxisd(mounting[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(mounting[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(mounting[0], Eigen::Vector3d::UnitX());
    load_persistent_calibration();

    output_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS().keep_last(8));
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
    // Service startup only reloads the one persistent device file. Do not
    // silently redefine zero at boot; a stationary bias reset is an explicit,
    // repeatable operator action and does not interrupt the IMU stream.
    ready_ = false;
    accel_enabled_ = persistent_calibration_valid_;
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
      persistent_calibration_valid_ = persistent_imu_calibration_valid(
        accel_matrix_, gyro_matrix_, persistent_accel_bias_, persistent_gyro_bias_);
      if (!persistent_calibration_valid_) {
        throw std::runtime_error("calibration contains non-finite or singular coefficients");
      }
      calibration_error_.clear();
      RCLCPP_INFO(
        get_logger(), "Loaded external IMU calibration for this service start: %s",
        calibration_file_.c_str());
    } catch (const std::exception & error) {
      persistent_calibration_valid_ = false;
      // Never apply coefficients from a file that failed parsing or numeric
      // validation. Gyro telemetry remains available for an explicit runtime
      // bias reset, while
      // acceleration is explicitly marked unavailable to the ESKF.
      accel_matrix_.setIdentity();
      gyro_matrix_.setIdentity();
      persistent_accel_bias_.setZero();
      persistent_gyro_bias_.setZero();
      calibration_error_ = error.what();
      RCLCPP_WARN(get_logger(), "Persistent IMU calibration unavailable; gyro-only fallback: %s", error.what());
    }
  }

  void restart_calibration()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    gyro_samples_.clear(); accel_samples_.clear();
    ready_ = false; timed_out_ = false; collecting_ = true;
    calibration_input_count_ = 0;
    calibration_start_ = now();
    publish_ready();
  }

  void check_calibration_timeout()
  {
    bool newly_timed_out = false;
    std::uint64_t input_count = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (collecting_ && !timed_out_ &&
        (now() - calibration_start_).seconds() > timeout_s_)
      {
        timed_out_ = true;
        collecting_ = false;
        newly_timed_out = true;
        input_count = calibration_input_count_;
      }
    }
    if (newly_timed_out) {
      if (input_count == 0U) {
        RCLCPP_ERROR(get_logger(), "Manual IMU bias reset timed out: no input frames received");
      } else {
        RCLCPP_ERROR(get_logger(),
          "Manual IMU bias reset timed out after %llu input frames; keep vehicle stationary",
          static_cast<unsigned long long>(input_count));
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ready_ = manual_bias_valid_;
      }
      publish_ready();
    }
  }

  void on_calibrate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto request_time = now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (collecting_) {
        response->success = true;
        response->message = "IMU calibration already in progress";
        return;
      }
      const double input_age_s = (request_time - last_input_arrival_).seconds();
      const bool input_fresh = have_input_ && input_age_s >= 0.0 &&
        input_age_s <= std::max(0.01, input_freshness_timeout_s_);
      if (!input_fresh) {
        response->success = false;
        response->message = have_input_ ?
          "IMU calibration rejected: raw IMU telemetry is stale" :
          "IMU calibration rejected: no raw IMU telemetry received";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }
    }
    restart_calibration();
    response->success = true;
    response->message = "Manual IMU bias reset started; keep vehicle level and stationary";
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const Eigen::Vector3d raw_gyro = vector(message->angular_velocity);
    const Eigen::Vector3d raw_accel = vector(message->linear_acceleration);
    if (!raw_gyro.allFinite() || !raw_accel.allFinite()) {return;}
    const Eigen::Vector3d gyro = calibrate_sensor_vector(
      raw_gyro, persistent_gyro_bias_, gyro_matrix_);
    const Eigen::Vector3d accel = calibrate_sensor_vector(
      raw_accel, persistent_accel_bias_, accel_matrix_);
    Eigen::Vector3d runtime_gyro_bias;
    Eigen::Vector3d runtime_accel_residual;
    bool accel_enabled = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_input_arrival_ = now();
      have_input_ = true;
      ++input_count_;
      if (collecting_) {
        ++calibration_input_count_;
        const bool gyro_stationary = gyro.norm() <= gyro_limit_;
        const bool accel_stationary = std::abs(accel.norm() - 9.80665) <= accel_tolerance_;
        if (gyro_stationary && accel_stationary) {
          gyro_samples_.push_back(gyro);
          accel_samples_.push_back(accel);
        }
        if (static_cast<int>(gyro_samples_.size()) >= std::max(1, required_samples_)) {
          finish_calibration();
        }
      }
      runtime_gyro_bias = runtime_gyro_bias_;
      runtime_accel_residual = runtime_accel_residual_;
      accel_enabled = accel_enabled_;
    }
    sensor_msgs::msg::Imu output = *message;
    const Eigen::Vector3d corrected_gyro = gyro - runtime_gyro_bias;
    const Eigen::Vector3d corrected_accel = accel - runtime_accel_residual;
    const Eigen::Vector3d base_gyro = rotate_imu_vector_to_base(corrected_gyro, base_from_imu_);
    const Eigen::Vector3d base_accel = rotate_imu_vector_to_base(corrected_accel, base_from_imu_);
    output.header.frame_id = base_frame_id_;
    output.orientation.x = 0.0;
    output.orientation.y = 0.0;
    output.orientation.z = 0.0;
    output.orientation.w = 1.0;
    output.angular_velocity.x = base_gyro.x();
    output.angular_velocity.y = base_gyro.y();
    output.angular_velocity.z = base_gyro.z();
    output.linear_acceleration.x = base_accel.x();
    output.linear_acceleration.y = base_accel.y();
    output.linear_acceleration.z = base_accel.z();
    output.orientation_covariance.fill(0.0);
    output.orientation_covariance[0] = -1.0;
    const Eigen::Matrix3d gyro_covariance = condition_imu_covariance(
      message->angular_velocity_covariance, gyro_matrix_, base_from_imu_, gyro_floor_);
    const Eigen::Matrix3d accel_covariance = condition_imu_covariance(
      message->linear_acceleration_covariance, accel_matrix_, base_from_imu_, accel_floor_);
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        output.angular_velocity_covariance[3 * row + column] = gyro_covariance(row, column);
        output.linear_acceleration_covariance[3 * row + column] = accel_covariance(row, column);
      }
    }
    if (!accel_enabled) {
      output.linear_acceleration_covariance.fill(0.0);
      output.linear_acceleration_covariance[0] = -1.0;
    }
    // Publish one ROS-standard corrected stream for every consumer. Linear
    // acceleration is specific force, so a stationary level sensor reports
    // approximately +9.80665 m/s^2 on Z. The ESKF removes gravity using its
    // attitude; the UI displays the exact same calibrated sample.
    output_pub_->publish(output);
    ++published_count_;
    last_stamp_ = rclcpp::Time(message->header.stamp);
  }

  void finish_calibration()
  {
    runtime_gyro_bias_.setZero();
    Eigen::Vector3d measured_accel = Eigen::Vector3d::Zero();
    for (const auto & sample : gyro_samples_) {runtime_gyro_bias_ += sample;}
    runtime_gyro_bias_ /= static_cast<double>(gyro_samples_.size());
    for (const auto & sample : accel_samples_) {measured_accel += sample;}
    measured_accel /= static_cast<double>(accel_samples_.size());

    // The operator guarantees that base_link is level and stationary during
    // manual reset. That makes the expected gravity vector known from
    // the configured rigid mounting alone; no VIO, camera, or AprilTag input is
    // required for this calibration.
    const Eigen::Vector3d expected =
      base_from_imu_.transpose() * Eigen::Vector3d(0.0, 0.0, 9.80665);
    runtime_accel_residual_ = measured_accel - expected;
    accel_enabled_ = acceleration_fusion_enabled(
      persistent_calibration_valid_, runtime_accel_residual_.norm(), 1.5);
    manual_bias_valid_ = true;
    ready_ = true;
    timed_out_ = false;
    collecting_ = false;
    publish_ready();
    RCLCPP_INFO(get_logger(), "Manual IMU bias reset from %zu samples; acceleration %s",
      gyro_samples_.size(), accel_enabled_ ? "enabled" : "disabled");
  }

  void publish_ready()
  {
    std_msgs::msg::Bool status; status.data = ready_; status_pub_->publish(status);
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const int level = (!collecting_ && !timed_out_ && accel_enabled_) ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    const std::string message = collecting_ ? "collecting manual stationary bias reset" :
      (timed_out_ ? "manual bias reset timed out; previous correction retained" :
      (accel_enabled_ ? (manual_bias_valid_ ? "calibrated; manual bias reset applied" :
      "persistent calibration loaded") :
      (manual_bias_valid_ ? "manual gyro bias reset; acceleration fusion disabled" :
      "gyro-only fallback; acceleration fusion disabled")));
    status.summary(level, message);
    status.add("imu_calibrated", ready_);
    status.add("accel_fusion_enabled", accel_enabled_);
    status.add("persistent_calibration_valid", persistent_calibration_valid_);
    status.add("manual_bias_reset_valid", manual_bias_valid_);
    status.add("calibration_samples", gyro_samples_.size());
    status.add("input_frames", input_count_);
    status.add("calibration_file", calibration_file_);
    status.add("calibration_error", calibration_error_);
    status.add("published_count", published_count_);
  }

  std::string input_topic_, output_topic_;
  std::string status_topic_, base_frame_id_;
  std::string calibration_file_, calibration_error_;
  int required_samples_{};
  double timeout_s_{}, input_freshness_timeout_s_{}, gyro_limit_{}, accel_tolerance_{};
  double gyro_floor_{}, accel_floor_{};
  Eigen::Matrix3d accel_matrix_{Eigen::Matrix3d::Identity()}, gyro_matrix_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d base_from_imu_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d persistent_accel_bias_{Eigen::Vector3d::Zero()}, persistent_gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d runtime_gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d runtime_accel_residual_{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> gyro_samples_, accel_samples_;
  std::mutex state_mutex_;
  bool persistent_calibration_valid_{false}, ready_{false};
  bool accel_enabled_{false}, timed_out_{false}, collecting_{false}, have_input_{false};
  bool manual_bias_valid_{false};
  std::uint64_t input_count_{}, calibration_input_count_{}, published_count_{};
  rclcpp::Time calibration_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_input_arrival_{0, 0, RCL_ROS_TIME}, last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr input_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace robotcore_sensors
RCLCPP_COMPONENTS_REGISTER_NODE(robotcore_sensors::ImuConditionerComponent)
