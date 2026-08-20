#include "robotcore_sensors/geometry.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <robotcore_interfaces/msg/april_tag_pose_estimate.hpp>
#include <robotcore_interfaces/msg/body_state.hpp>
#include <robotcore_interfaces/msg/localization_status.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace robotcore_sensors
{
namespace
{
constexpr std::int64_t kArrivalWindowNs = 5000000000LL;

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

Eigen::Isometry3d pose(const geometry_msgs::msg::Pose & message)
{
  Eigen::Quaterniond orientation(
    message.orientation.w, message.orientation.x,
    message.orientation.y, message.orientation.z);
  if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-9) {
    throw std::runtime_error("invalid pose quaternion");
  }
  return pose_transform(
    {message.position.x, message.position.y, message.position.z},
    orientation.normalized());
}

void set_pose(geometry_msgs::msg::Pose & message, const Eigen::Isometry3d & transform)
{
  const Eigen::Quaterniond orientation(transform.linear());
  message.position.x = transform.translation().x();
  message.position.y = transform.translation().y();
  message.position.z = transform.translation().z();
  message.orientation.x = orientation.x();
  message.orientation.y = orientation.y();
  message.orientation.z = orientation.z();
  message.orientation.w = orientation.w();
}

template<typename Array>
Eigen::Matrix<double, 6, 6> covariance6(const Array & values)
{
  Eigen::Matrix<double, 6, 6> covariance;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      const double value = values[6 * row + column];
      covariance(row, column) = std::isfinite(value) ? value : 0.0;
    }
  }
  return 0.5 * (covariance + covariance.transpose());
}

Eigen::Matrix<double, 6, 6> adjoint(const Eigen::Isometry3d & target_from_source)
{
  Eigen::Matrix<double, 6, 6> result = Eigen::Matrix<double, 6, 6>::Zero();
  result.block<3, 3>(0, 0) = target_from_source.linear();
  result.block<3, 3>(0, 3) =
    skew(target_from_source.translation()) * target_from_source.linear();
  result.block<3, 3>(3, 3) = target_from_source.linear();
  return result;
}

void enforce_covariance_floors(
  Eigen::Matrix<double, 6, 6> & covariance,
  const std::array<double, 6> & floors)
{
  covariance = 0.5 * (covariance + covariance.transpose());
  for (int index = 0; index < 6; ++index) {
    covariance(index, index) = std::max(covariance(index, index), floors[index]);
  }
}

template<typename Array>
void store_covariance(Array & output, const Eigen::Matrix<double, 6, 6> & covariance)
{
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      output[6 * row + column] = covariance(row, column);
    }
  }
}

double arrival_rate_hz(
  const std::deque<std::int64_t> & arrivals, std::int64_t now_ns)
{
  if (arrivals.size() < 2U || now_ns < arrivals.back() ||
    now_ns - arrivals.back() > kArrivalWindowNs)
  {
    return 0.0;
  }
  const auto duration_ns = arrivals.back() - arrivals.front();
  return duration_ns > 0 ?
    static_cast<double>(arrivals.size() - 1U) * 1e9 /
    static_cast<double>(duration_ns) : 0.0;
}

void prune_arrivals(std::deque<std::int64_t> & arrivals, std::int64_t now_ns)
{
  while (!arrivals.empty() && now_ns - arrivals.front() > kArrivalWindowNs) {
    arrivals.pop_front();
  }
}
}  // namespace

class VioTagFusionComponent final : public rclcpp::Node
{
public:
  explicit VioTagFusionComponent(const rclcpp::NodeOptions & options)
  : Node("vio_tag_fusion", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    updater_(this)
  {
    history_duration_s_ = declare_parameter<double>("history_duration_s", 3.0);
    vio_arrival_timeout_s_ = declare_parameter<double>("vio_arrival_timeout_s", 0.30);
    vio_prediction_horizon_s_ = declare_parameter<double>("vio_prediction_horizon_s", 0.50);
    tag_fresh_s_ = declare_parameter<double>("tag_fresh_s", 0.35);
    tag_innovation_gate_m_ = declare_parameter<double>("tag_innovation_gate_m", 0.50);
    alignment_translation_walk_ = declare_parameter<double>(
      "alignment_translation_walk_m_sqrt_s", 0.01);
    alignment_rotation_walk_ = declare_parameter<double>(
      "alignment_rotation_walk_rad_sqrt_s", 0.005);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    if (history_duration_s_ <= 0.0 || vio_arrival_timeout_s_ <= 0.0 ||
      vio_prediction_horizon_s_ <= 0.0 || tag_fresh_s_ <= 0.0 ||
      tag_innovation_gate_m_ <= 0.0 || alignment_translation_walk_ < 0.0 ||
      alignment_rotation_walk_ < 0.0)
    {
      throw std::runtime_error("VIO/Tag fusion timing, gate and noise parameters are invalid");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("vio_topic", "/zedx/zed_node/odom"), sensor_qos,
      std::bind(&VioTagFusionComponent::on_vio, this, std::placeholders::_1));
    tag_sub_ = create_subscription<robotcore_interfaces::msg::AprilTagPoseEstimate>(
      declare_parameter<std::string>("tag_topic", "/localization/apriltag_pose"), sensor_qos,
      std::bind(&VioTagFusionComponent::on_tag, this, std::placeholders::_1));

    fused_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/localization/fused_odom", rclcpp::QoS(1).reliable());
    body_pub_ = create_publisher<robotcore_interfaces::msg::BodyState>(
      "/robot/body_state", rclcpp::QoS(1).reliable());
    status_pub_ = create_publisher<robotcore_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(1).reliable());

    const double output_hz = declare_parameter<double>("output_rate_hz", 60.0);
    const double status_hz = declare_parameter<double>("status_rate_hz", 10.0);
    status_period_ns_ = static_cast<std::int64_t>(1e9 / std::max(1.0, status_hz));
    output_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, output_hz)),
      std::bind(&VioTagFusionComponent::publish, this));

    updater_.setHardwareID("vio-tag-fusion");
    updater_.add("Localization fusion", this, &VioTagFusionComponent::diagnose);
  }

private:
  struct VioSample
  {
    std::int64_t stamp_ns{};
    std::int64_t arrival_ns{};
    Eigen::Isometry3d odom_from_base{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d body_linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d body_angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Matrix<double, 6, 6> pose_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix<double, 6, 6> twist_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
  };

  bool current_epoch(std::int64_t measurement_ns, std::int64_t arrival_ns) const
  {
    constexpr std::int64_t maximum_age_ns = 2000000000LL;
    return measurement_ns > 0 &&
           measurement_ns >= arrival_ns - maximum_age_ns &&
           measurement_ns <= arrival_ns + maximum_age_ns;
  }

  bool update_cached_extrinsic(const std::string & source_frame)
  {
    if (source_frame == base_frame_) {
      cached_source_frame_ = source_frame;
      base_from_vio_source_ = Eigen::Isometry3d::Identity();
      return true;
    }
    if (base_from_vio_source_ && cached_source_frame_ == source_frame) {return true;}
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, source_frame, tf2::TimePointZero);
      base_from_vio_source_ = tf2::transformToEigen(transform);
      cached_source_frame_ = source_frame;
      return true;
    } catch (const std::exception & error) {
      ++transform_failures_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for VIO transform %s <- %s: %s",
        base_frame_.c_str(), source_frame.c_str(), error.what());
      return false;
    }
  }

  void on_vio(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto arrival_ns = now().nanoseconds();
    const auto measurement_ns = stamp_ns(message->header.stamp);
    if (!current_epoch(measurement_ns, arrival_ns)) {
      ++timestamp_epoch_rejections_;
      return;
    }
    if (measurement_ns <= last_vio_stamp_ns_) {
      ++duplicate_or_old_vio_drops_;
      return;
    }
    if (message->child_frame_id.empty() || !update_cached_extrinsic(message->child_frame_id)) {
      return;
    }

    VioSample sample;
    try {
      const Eigen::Isometry3d odom_from_source = pose(message->pose.pose);
      sample.odom_from_base = odom_from_source * base_from_vio_source_->inverse();
    } catch (...) {
      ++invalid_vio_drops_;
      return;
    }

    Eigen::Matrix<double, 6, 1> source_twist;
    source_twist <<
      message->twist.twist.linear.x, message->twist.twist.linear.y,
      message->twist.twist.linear.z, message->twist.twist.angular.x,
      message->twist.twist.angular.y, message->twist.twist.angular.z;
    const Eigen::Matrix<double, 6, 1> body_twist =
      adjoint(*base_from_vio_source_) * source_twist;
    if (!body_twist.allFinite()) {
      ++invalid_vio_drops_;
      return;
    }

    sample.stamp_ns = measurement_ns;
    sample.arrival_ns = arrival_ns;
    sample.body_linear_velocity = body_twist.head<3>();
    sample.body_angular_velocity = body_twist.tail<3>();
    const auto covariance_transform = adjoint(*base_from_vio_source_);
    sample.pose_covariance = covariance_transform *
      covariance6(message->pose.covariance) * covariance_transform.transpose();
    sample.twist_covariance = covariance_transform *
      covariance6(message->twist.covariance) * covariance_transform.transpose();
    enforce_covariance_floors(
      sample.pose_covariance,
      {1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5});
    enforce_covariance_floors(
      sample.twist_covariance,
      {9e-4, 9e-4, 9e-4, 1e-3, 1e-3, 1e-3});

    last_vio_stamp_ns_ = measurement_ns;
    last_vio_arrival_ns_ = arrival_ns;
    vio_transport_s_ = (arrival_ns - measurement_ns) * 1e-9;
    history_.push_back(sample);
    vio_arrivals_.push_back(arrival_ns);
    prune_history();
    prune_arrivals(vio_arrivals_, arrival_ns);
  }

  void reset_alignment()
  {
    map_from_odom_.reset();
    alignment_covariance_.setIdentity();
    last_alignment_stamp_ns_ = 0;
    last_tag_stamp_ns_ = 0;
    last_tag_measurement_stamp_ns_ = 0;
    last_tag_arrival_ns_ = 0;
    last_absolute_stamp_ns_ = 0;
    last_tag_translation_residual_ = NAN;
    last_tag_angle_residual_deg_ = NAN;
  }

  std::optional<VioSample> sample_at(std::int64_t target_ns) const
  {
    if (history_.empty()) {return std::nullopt;}
    constexpr std::int64_t tolerance_ns = 150000000LL;
    const auto upper = std::lower_bound(
      history_.begin(), history_.end(), target_ns,
      [](const VioSample & sample, std::int64_t stamp) {
        return sample.stamp_ns < stamp;
      });

    if (upper == history_.begin()) {
      return std::llabs(upper->stamp_ns - target_ns) <= tolerance_ns ?
        std::optional<VioSample>(*upper) : std::nullopt;
    }
    if (upper == history_.end()) {
      const auto & sample = history_.back();
      return std::llabs(sample.stamp_ns - target_ns) <= tolerance_ns ?
        std::optional<VioSample>(sample) : std::nullopt;
    }

    const auto lower = std::prev(upper);
    if (target_ns - lower->stamp_ns > tolerance_ns ||
      upper->stamp_ns - target_ns > tolerance_ns)
    {
      const auto lower_error = std::llabs(target_ns - lower->stamp_ns);
      const auto upper_error = std::llabs(upper->stamp_ns - target_ns);
      const auto & nearest = lower_error <= upper_error ? *lower : *upper;
      return std::min(lower_error, upper_error) <= tolerance_ns ?
        std::optional<VioSample>(nearest) : std::nullopt;
    }

    const double denominator = static_cast<double>(upper->stamp_ns - lower->stamp_ns);
    const double alpha = denominator > 0.0 ?
      static_cast<double>(target_ns - lower->stamp_ns) / denominator : 0.0;
    VioSample interpolated;
    interpolated.stamp_ns = target_ns;
    interpolated.arrival_ns = std::max(lower->arrival_ns, upper->arrival_ns);
    interpolated.odom_from_base = blend_transform(
      lower->odom_from_base, upper->odom_from_base, alpha);
    interpolated.body_linear_velocity =
      (1.0 - alpha) * lower->body_linear_velocity + alpha * upper->body_linear_velocity;
    interpolated.body_angular_velocity =
      (1.0 - alpha) * lower->body_angular_velocity + alpha * upper->body_angular_velocity;
    interpolated.pose_covariance =
      (1.0 - alpha) * lower->pose_covariance + alpha * upper->pose_covariance;
    interpolated.twist_covariance =
      (1.0 - alpha) * lower->twist_covariance + alpha * upper->twist_covariance;
    return interpolated;
  }

  Eigen::Matrix<double, 6, 6> alignment_measurement_covariance(
    const Eigen::Matrix<double, 6, 6> & tag_covariance,
    const VioSample & vio,
    const Eigen::Matrix3d & map_from_odom_rotation) const
  {
    Eigen::Matrix<double, 6, 6> rotation = Eigen::Matrix<double, 6, 6>::Zero();
    rotation.block<3, 3>(0, 0) = map_from_odom_rotation;
    rotation.block<3, 3>(3, 3) = map_from_odom_rotation;
    Eigen::Matrix<double, 6, 6> result =
      tag_covariance + rotation * vio.pose_covariance * rotation.transpose();
    enforce_covariance_floors(result, {1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5});
    return result;
  }

  void update_alignment(
    const Eigen::Isometry3d & candidate,
    const Eigen::Matrix<double, 6, 6> & measurement_covariance,
    std::int64_t stamp_ns)
  {
    if (!map_from_odom_) {
      map_from_odom_ = candidate;
      alignment_covariance_ = measurement_covariance;
      last_alignment_stamp_ns_ = stamp_ns;
      return;
    }

    if (last_alignment_stamp_ns_ > 0 && stamp_ns > last_alignment_stamp_ns_) {
      const double dt = static_cast<double>(stamp_ns - last_alignment_stamp_ns_) * 1e-9;
      alignment_covariance_.block<3, 3>(0, 0).diagonal().array() +=
        alignment_translation_walk_ * alignment_translation_walk_ * dt;
      alignment_covariance_.block<3, 3>(3, 3).diagonal().array() +=
        alignment_rotation_walk_ * alignment_rotation_walk_ * dt;
    }

    Eigen::Matrix<double, 6, 1> innovation;
    innovation.head<3>() = candidate.translation() - map_from_odom_->translation();
    innovation.tail<3>() = log_quaternion(
      Eigen::Quaterniond(map_from_odom_->linear()).conjugate() *
      Eigen::Quaterniond(candidate.linear()));
    const Eigen::Matrix<double, 6, 6> innovation_covariance =
      alignment_covariance_ + measurement_covariance;
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> decomposition(innovation_covariance);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
      ++alignment_update_failures_;
      return;
    }
    const Eigen::Matrix<double, 6, 6> gain =
      alignment_covariance_ * decomposition.solve(
      Eigen::Matrix<double, 6, 6>::Identity());
    const Eigen::Matrix<double, 6, 1> correction = gain * innovation;
    map_from_odom_->translation() += correction.head<3>();
    map_from_odom_->linear() =
      (Eigen::Quaterniond(map_from_odom_->linear()) *
      exp_quaternion(correction.tail<3>())).normalized().toRotationMatrix();

    const Eigen::Matrix<double, 6, 6> identity =
      Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 6> residual = identity - gain;
    alignment_covariance_ =
      residual * alignment_covariance_ * residual.transpose() +
      gain * measurement_covariance * gain.transpose();
    alignment_covariance_ = 0.5 *
      (alignment_covariance_ + alignment_covariance_.transpose());
    last_alignment_stamp_ns_ = stamp_ns;
  }

  void on_tag(const robotcore_interfaces::msg::AprilTagPoseEstimate::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto arrival_ns = now().nanoseconds();
    if (message->map_generation < last_tag_map_generation_) {return;}
    const bool map_changed = last_tag_map_generation_ != 0U &&
      message->map_generation > last_tag_map_generation_;
    if (message->relocalization_requested || map_changed) {reset_alignment();}
    last_tag_map_generation_ = message->map_generation;
    last_tag_estimate_ = *message;
    last_tag_frame_arrival_ns_ = arrival_ns;
    tag_frame_arrivals_.push_back(arrival_ns);
    prune_arrivals(tag_frame_arrivals_, arrival_ns);
    if (!message->pose_valid || message->header.frame_id != map_frame_) {return;}

    const auto tag_stamp_ns = stamp_ns(message->header.stamp);
    if (!current_epoch(tag_stamp_ns, arrival_ns)) {
      ++timestamp_epoch_rejections_;
      return;
    }
    if (tag_stamp_ns <= last_tag_measurement_stamp_ns_) {
      ++duplicate_or_old_tag_drops_;
      return;
    }
    last_tag_measurement_stamp_ns_ = tag_stamp_ns;
    const auto vio = sample_at(tag_stamp_ns);
    if (!vio) {
      ++tag_without_vio_drops_;
      return;
    }

    Eigen::Isometry3d map_from_base;
    try {
      map_from_base = pose(message->pose.pose);
    } catch (...) {
      return;
    }
    const Eigen::Isometry3d candidate =
      map_from_base * vio->odom_from_base.inverse();
    if (map_from_odom_) {
      const Eigen::Isometry3d predicted_map_from_base =
        *map_from_odom_ * vio->odom_from_base;
      last_tag_translation_residual_ =
        (map_from_base.translation() - predicted_map_from_base.translation()).norm();
      constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
      last_tag_angle_residual_deg_ = rotation_distance(
        map_from_base.linear(), predicted_map_from_base.linear()) * radians_to_degrees;
      if (last_tag_translation_residual_ > tag_innovation_gate_m_) {
        ++tag_gate_rejections_;
        return;
      }
    } else {
      last_tag_translation_residual_ = 0.0;
      last_tag_angle_residual_deg_ = 0.0;
    }

    const auto tag_covariance = covariance6(message->pose.covariance);
    const auto measurement_covariance = alignment_measurement_covariance(
      tag_covariance, *vio, candidate.linear());
    update_alignment(candidate, measurement_covariance, tag_stamp_ns);
    if (!map_from_odom_) {return;}
    last_tag_stamp_ns_ = tag_stamp_ns;
    last_absolute_stamp_ns_ = tag_stamp_ns;
    last_tag_arrival_ns_ = arrival_ns;
    tag_transport_s_ = (arrival_ns - tag_stamp_ns) * 1e-9;
    tag_arrivals_.push_back(arrival_ns);
    prune_arrivals(tag_arrivals_, arrival_ns);
  }

  void prune_history()
  {
    if (history_.empty()) {return;}
    const auto newest_ns = history_.back().stamp_ns;
    while (history_.size() > 2U &&
      newest_ns - history_.front().stamp_ns > history_duration_s_ * 1e9)
    {
      history_.pop_front();
    }
  }

  Eigen::Isometry3d predicted_odom_from_base(
    const VioSample & sample, double prediction_age_s) const
  {
    Eigen::Isometry3d predicted = sample.odom_from_base;
    if (prediction_age_s <= 0.0) {return predicted;}
    predicted.translation() +=
      predicted.linear() * sample.body_linear_velocity * prediction_age_s;
    predicted.linear() =
      (Eigen::Quaterniond(predicted.linear()) *
      exp_quaternion(sample.body_angular_velocity * prediction_age_s))
      .normalized().toRotationMatrix();
    return predicted;
  }

  Eigen::Matrix<double, 6, 6> output_pose_covariance(
    const VioSample & sample, double prediction_age_s, bool map_output) const
  {
    Eigen::Matrix<double, 6, 6> covariance = sample.pose_covariance;
    const double age = std::max(0.0, prediction_age_s);
    covariance.block<3, 3>(0, 0).diagonal().array() += 0.04 * age * age;
    covariance.block<3, 3>(3, 3).diagonal().array() += 0.01 * age * age;
    if (!map_output) {return covariance;}
    Eigen::Matrix<double, 6, 6> rotation = Eigen::Matrix<double, 6, 6>::Zero();
    rotation.block<3, 3>(0, 0) = map_from_odom_->linear();
    rotation.block<3, 3>(3, 3) = map_from_odom_->linear();
    return rotation * covariance * rotation.transpose() + alignment_covariance_;
  }

  static std::string localization_source(bool absolute)
  {
    return absolute ? "AprilTag+ZED VIO" : "ZED VIO after Tag loss";
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (history_.empty()) {return;}
    const auto stamp = now();
    const auto now_ns = stamp.nanoseconds();
    prune_arrivals(vio_arrivals_, now_ns);
    prune_arrivals(tag_arrivals_, now_ns);
    prune_arrivals(tag_frame_arrivals_, now_ns);
    prune_arrivals(fused_arrivals_, now_ns);

    const auto & sample = history_.back();
    const double arrival_age_s = (now_ns - sample.arrival_ns) * 1e-9;
    const double measurement_age_s = (now_ns - sample.stamp_ns) * 1e-9;
    const double tag_arrival_age_s = last_tag_arrival_ns_ > 0 ?
      (now_ns - last_tag_arrival_ns_) * 1e-9 : INFINITY;
    const double tag_measurement_age_s = last_tag_stamp_ns_ > 0 ?
      (now_ns - last_tag_stamp_ns_) * 1e-9 : INFINITY;
    const bool vio_usable = arrival_age_s >= 0.0 &&
      arrival_age_s <= vio_arrival_timeout_s_ && measurement_age_s >= 0.0 &&
      measurement_age_s <= vio_prediction_horizon_s_;
    const bool absolute_valid = vio_usable && map_from_odom_ &&
      tag_arrival_age_s <= tag_fresh_s_;
    const bool estimated = vio_usable && !absolute_valid;

    const Eigen::Isometry3d odom_from_base = predicted_odom_from_base(
      sample, std::clamp(measurement_age_s, 0.0, vio_prediction_horizon_s_));
    const bool map_output = map_from_odom_.has_value();
    const Eigen::Isometry3d output_from_base = map_output ?
      *map_from_odom_ * odom_from_base : odom_from_base;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = map_output ? map_frame_ : odom_frame_;
    odometry.child_frame_id = base_frame_;
    set_pose(odometry.pose.pose, output_from_base);
    odometry.twist.twist.linear.x = sample.body_linear_velocity.x();
    odometry.twist.twist.linear.y = sample.body_linear_velocity.y();
    odometry.twist.twist.linear.z = sample.body_linear_velocity.z();
    odometry.twist.twist.angular.x = sample.body_angular_velocity.x();
    odometry.twist.twist.angular.y = sample.body_angular_velocity.y();
    odometry.twist.twist.angular.z = sample.body_angular_velocity.z();
    store_covariance(
      odometry.pose.covariance,
      output_pose_covariance(sample, measurement_age_s, map_output));
    store_covariance(odometry.twist.covariance, sample.twist_covariance);
    fused_pub_->publish(odometry);
    fused_arrivals_.push_back(now_ns);

    robotcore_interfaces::msg::BodyState body;
    body.header = odometry.header;
    body.pose = odometry.pose.pose;
    body.twist = odometry.twist.twist;
    body.linear_velocity_valid = vio_usable;
    body.state_valid = absolute_valid;
    body.position_estimated = estimated;
    body.localization_source = vio_usable ? localization_source(absolute_valid) : "";
    body_pub_->publish(body);

    if (last_status_publish_ns_ == 0 ||
      now_ns - last_status_publish_ns_ >= status_period_ns_)
    {
      publish_status(
        stamp, odometry, measurement_age_s, arrival_age_s,
        tag_measurement_age_s, tag_arrival_age_s,
        vio_usable, absolute_valid, estimated);
      last_status_publish_ns_ = now_ns;
    }
  }

  void publish_status(
    const rclcpp::Time & stamp, const nav_msgs::msg::Odometry & odometry,
    double vio_measurement_age_s, double vio_arrival_age_s,
    double tag_measurement_age_s, double tag_arrival_age_s,
    bool vio_usable, bool absolute_valid, bool estimated)
  {
    robotcore_interfaces::msg::LocalizationStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = odometry.header.frame_id;
    if (last_vio_stamp_ns_ > 0) {
      status.last_vio_stamp = rclcpp::Time(last_vio_stamp_ns_, RCL_ROS_TIME);
    }
    if (last_tag_stamp_ns_ > 0) {
      status.last_tag_stamp = rclcpp::Time(last_tag_stamp_ns_, RCL_ROS_TIME);
      status.last_absolute_fix_stamp = rclcpp::Time(last_absolute_stamp_ns_, RCL_ROS_TIME);
    }
    status.vio_age_s = vio_measurement_age_s;
    status.tag_age_s = tag_measurement_age_s;
    status.absolute_fix_age_s = tag_measurement_age_s;
    status.apriltag_frame_age_s = last_tag_frame_arrival_ns_ > 0 ?
      (stamp.nanoseconds() - last_tag_frame_arrival_ns_) * 1e-9 : INFINITY;
    status.vio_rate_hz = arrival_rate_hz(vio_arrivals_, stamp.nanoseconds());
    status.tag_rate_hz = arrival_rate_hz(tag_arrivals_, stamp.nanoseconds());
    status.fused_rate_hz = arrival_rate_hz(fused_arrivals_, stamp.nanoseconds());
    status.apriltag_frame_rate_hz = arrival_rate_hz(
      tag_frame_arrivals_, stamp.nanoseconds());
    status.vio_transport_delay_s = vio_transport_s_;
    status.tag_transport_delay_s = tag_transport_s_;
    status.tag_vio_translation_residual_m = last_tag_translation_residual_;
    status.tag_vio_angle_residual_deg = last_tag_angle_residual_deg_;
    status.pose_covariance = odometry.pose.covariance;
    status.twist_covariance = odometry.twist.covariance;
    status.detected_tag_count = last_tag_estimate_.detected_tag_count;
    status.mapped_tag_count = last_tag_estimate_.mapped_tag_count;
    status.inlier_tag_count = last_tag_estimate_.inlier_tag_count;
    status.tag_reprojection_rms_px = last_tag_estimate_.reprojection_rms_px;
    status.minimum_tag_edge_px = last_tag_estimate_.minimum_tag_edge_px;
    status.tag_pose_published = last_tag_estimate_.pose_valid;
    status.vio_fresh = vio_usable;
    status.tag_fresh = tag_arrival_age_s <= tag_fresh_s_;
    status.tag_consistent = std::isfinite(last_tag_translation_residual_) &&
      last_tag_translation_residual_ <= tag_innovation_gate_m_;
    status.absolute_fix_valid = absolute_valid;
    status.position_estimated = estimated;
    status.localization_source = vio_usable ? localization_source(absolute_valid) : "";
    status.tag_observation_class = last_tag_estimate_.pose_valid ? "primary" : "";
    status.apriltag_rejection_reason = last_tag_estimate_.rejection_reason;
    if (vio_arrival_age_s > vio_arrival_timeout_s_) {
      status.rejection_reason = "ZED VIO stream is stale";
    } else if (vio_measurement_age_s > vio_prediction_horizon_s_) {
      status.rejection_reason = "ZED VIO measurement latency exceeds prediction horizon";
    } else if (!absolute_valid) {
      status.rejection_reason = "absolute Tag fix is stale; using local VIO";
    }
    status_pub_->publish(status);
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now_ns = now().nanoseconds();
    prune_arrivals(vio_arrivals_, now_ns);
    prune_arrivals(tag_arrivals_, now_ns);
    prune_arrivals(tag_frame_arrivals_, now_ns);
    prune_arrivals(fused_arrivals_, now_ns);
    const double arrival_age_s = last_vio_arrival_ns_ > 0 ?
      (now_ns - last_vio_arrival_ns_) * 1e-9 : INFINITY;
    const double measurement_age_s = last_vio_stamp_ns_ > 0 ?
      (now_ns - last_vio_stamp_ns_) * 1e-9 : INFINITY;
    const double tag_arrival_age_s = last_tag_arrival_ns_ > 0 ?
      (now_ns - last_tag_arrival_ns_) * 1e-9 : INFINITY;
    const bool vio_usable = arrival_age_s <= vio_arrival_timeout_s_ &&
      measurement_age_s <= vio_prediction_horizon_s_;
    const bool absolute_valid = vio_usable && map_from_odom_ &&
      tag_arrival_age_s <= tag_fresh_s_;
    int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "AprilTag-aligned ZED VIO";
    if (history_.empty()) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "waiting for ZED VIO";
    } else if (!vio_usable) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "ZED VIO is stale or too delayed";
    } else if (!absolute_valid) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "local VIO estimate after Tag loss";
    }
    status.summary(level, summary);
    status.add("vio_rate_hz", arrival_rate_hz(vio_arrivals_, now_ns));
    status.add("vio_arrival_age_s", arrival_age_s);
    status.add("vio_measurement_age_s", measurement_age_s);
    status.add("vio_transport_delay_s", vio_transport_s_);
    status.add("tag_rate_hz", arrival_rate_hz(tag_arrivals_, now_ns));
    status.add("apriltag_frame_rate_hz", arrival_rate_hz(tag_frame_arrivals_, now_ns));
    status.add("fused_publish_rate_hz", arrival_rate_hz(fused_arrivals_, now_ns));
    status.add("absolute_fix_valid", absolute_valid);
    status.add("history_size", history_.size());
    status.add("duplicate_or_old_vio_drops", duplicate_or_old_vio_drops_);
    status.add("invalid_vio_drops", invalid_vio_drops_);
    status.add("transform_failures", transform_failures_);
    status.add("timestamp_epoch_rejections", timestamp_epoch_rejections_);
    status.add("tag_without_vio_drops", tag_without_vio_drops_);
    status.add("duplicate_or_old_tag_drops", duplicate_or_old_tag_drops_);
    status.add("tag_gate_rejections", tag_gate_rejections_);
    status.add("alignment_update_failures", alignment_update_failures_);
  }

  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  diagnostic_updater::Updater updater_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
  rclcpp::Subscription<robotcore_interfaces::msg::AprilTagPoseEstimate>::SharedPtr tag_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
  rclcpp::Publisher<robotcore_interfaces::msg::BodyState>::SharedPtr body_pub_;
  rclcpp::Publisher<robotcore_interfaces::msg::LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr output_timer_;

  std::deque<VioSample> history_;
  std::deque<std::int64_t> vio_arrivals_, tag_arrivals_, tag_frame_arrivals_, fused_arrivals_;
  std::optional<Eigen::Isometry3d> base_from_vio_source_;
  std::optional<Eigen::Isometry3d> map_from_odom_;
  Eigen::Matrix<double, 6, 6> alignment_covariance_{
    Eigen::Matrix<double, 6, 6>::Identity()};
  robotcore_interfaces::msg::AprilTagPoseEstimate last_tag_estimate_;
  std::string cached_source_frame_, map_frame_, odom_frame_, base_frame_;

  double history_duration_s_{};
  double vio_arrival_timeout_s_{};
  double vio_prediction_horizon_s_{};
  double tag_fresh_s_{};
  double tag_innovation_gate_m_{};
  double alignment_translation_walk_{};
  double alignment_rotation_walk_{};
  double vio_transport_s_{NAN};
  double tag_transport_s_{NAN};
  double last_tag_translation_residual_{NAN};
  double last_tag_angle_residual_deg_{NAN};

  std::int64_t last_vio_stamp_ns_{};
  std::int64_t last_vio_arrival_ns_{};
  std::int64_t last_tag_measurement_stamp_ns_{};
  std::int64_t last_tag_stamp_ns_{};
  std::int64_t last_tag_arrival_ns_{};
  std::int64_t last_absolute_stamp_ns_{};
  std::int64_t last_tag_frame_arrival_ns_{};
  std::int64_t last_alignment_stamp_ns_{};
  std::int64_t status_period_ns_{100000000LL};
  std::int64_t last_status_publish_ns_{};
  std::uint64_t last_tag_map_generation_{};
  std::uint64_t duplicate_or_old_vio_drops_{};
  std::uint64_t invalid_vio_drops_{};
  std::uint64_t transform_failures_{};
  std::uint64_t timestamp_epoch_rejections_{};
  std::uint64_t tag_without_vio_drops_{};
  std::uint64_t duplicate_or_old_tag_drops_{};
  std::uint64_t tag_gate_rejections_{};
  std::uint64_t alignment_update_failures_{};
};
}  // namespace robotcore_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(robotcore_sensors::VioTagFusionComponent)
