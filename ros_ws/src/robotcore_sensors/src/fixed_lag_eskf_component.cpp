#include "robotcore_sensors/fixed_lag_eskf.hpp"
#include "robotcore_sensors/geometry.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robotcore_interfaces/msg/april_tag_pose_status.hpp>
#include <robotcore_interfaces/msg/body_state.hpp>
#include <robotcore_interfaces/msg/localization_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

namespace robotcore_sensors
{
namespace
{
std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;}

Eigen::Quaterniond quaternion(const geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond result(q.w, q.x, q.y, q.z);
  if (result.norm() < 1e-9 || !result.coeffs().allFinite()) {throw std::runtime_error("invalid quaternion");}
  return result.normalized();
}

Eigen::Isometry3d pose(const geometry_msgs::msg::Pose & p)
{return pose_transform({p.position.x, p.position.y, p.position.z}, quaternion(p.orientation));}

void set_pose(geometry_msgs::msg::Pose & p, const Eigen::Isometry3d & t)
{
  const Eigen::Quaterniond q(t.linear());
  p.position.x = t.translation().x(); p.position.y = t.translation().y(); p.position.z = t.translation().z();
  p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
}

template<typename Array>
Eigen::Matrix<double, 6, 6> covariance6(const Array & array)
{
  Eigen::Matrix<double, 6, 6> result;
  for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) {
    const double value = array[6 * r + c]; result(r, c) = std::isfinite(value) ? value : 0.0;
  }
  return 0.5 * (result + result.transpose());
}
}  // namespace

class FixedLagEskfComponent final : public rclcpp::Node
{
public:
  explicit FixedLagEskfComponent(const rclcpp::NodeOptions & options)
  : Node("fixed_lag_eskf", options), updater_(this)
  {
    history_duration_s_ = declare_parameter<double>("history_duration_s", 3.0);
    imu_stale_s_ = declare_parameter<double>("imu_stale_s", 0.05);
    vio_stale_s_ = declare_parameter<double>("vio_stale_s", 0.35);
    inertial_horizon_s_ = declare_parameter<double>("inertial_only_horizon_s", 0.5);
    tag_fresh_s_ = declare_parameter<double>("tag_fresh_s", 0.5);
    alignment_alpha_ = declare_parameter<double>("alignment_correction_alpha", 0.25);
    alignment_max_m_ = declare_parameter<double>("alignment_max_correction_m", 0.75);
    alignment_max_rad_ = declare_parameter<double>("alignment_max_correction_angle_deg", 20.0) * M_PI / 180.0;
    alignment_threshold_m_ = declare_parameter<double>("alignment_recalibration_threshold_m", 0.05);
    alignment_threshold_rad_ = declare_parameter<double>("alignment_recalibration_threshold_deg", 2.0) * M_PI / 180.0;
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    EskfNoise noise;
    noise.gyro_noise = declare_parameter<double>("gyro_noise_rps_sqrt_hz", 0.015);
    noise.accel_noise = declare_parameter<double>("accel_noise_mps2_sqrt_hz", 0.20);
    noise.gyro_bias_walk = declare_parameter<double>("gyro_bias_walk_rps2_sqrt_hz", 0.0005);
    noise.accel_bias_walk = declare_parameter<double>("accel_bias_walk_mps3_sqrt_hz", 0.01);
    filter_ = FixedLagEskf(noise);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(8);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/sensors/external_imu"), sensor_qos,
      std::bind(&FixedLagEskfComponent::on_imu, this, std::placeholders::_1));
    vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("vio_topic", "/localization/zed_odom"), sensor_qos,
      std::bind(&FixedLagEskfComponent::on_vio, this, std::placeholders::_1));
    tag_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      declare_parameter<std::string>("tag_topic", "/localization/apriltag_pose"), sensor_qos,
      std::bind(&FixedLagEskfComponent::on_tag, this, std::placeholders::_1));
    tag_status_sub_ = create_subscription<robotcore_interfaces::msg::AprilTagPoseStatus>(
      "/localization/apriltag/pose_status", sensor_qos,
      std::bind(&FixedLagEskfComponent::on_tag_status, this, std::placeholders::_1));
    fused_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/fused_odom", rclcpp::QoS(5).reliable());
    aligned_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/aligned_vio_odom", sensor_qos);
    body_pub_ = create_publisher<robotcore_interfaces::msg::BodyState>("/robot/body_state", rclcpp::QoS(10).reliable());
    status_pub_ = create_publisher<robotcore_interfaces::msg::LocalizationStatus>("/localization/status", rclcpp::QoS(10).reliable());
    relocalize_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/tag_vio/relocalize", std::bind(&FixedLagEskfComponent::on_relocalize, this,
      std::placeholders::_1, std::placeholders::_2));
    relocalize_event_pub_ = create_publisher<std_msgs::msg::Empty>(
      "/localization/relocalize_event", rclcpp::QoS(1).reliable());
    relocalize_event_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/localization/relocalize_event", rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Empty::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_); reset_alignment();
      });
    const double output_hz = declare_parameter<double>("output_rate_hz", 60.0);
    output_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, output_hz)),
      std::bind(&FixedLagEskfComponent::publish, this));
    updater_.setHardwareID("fixed-lag-eskf");
    updater_.add("Localization estimator", this, &FixedLagEskfComponent::diagnose);
  }

private:
  struct HistoryEntry {ImuSample imu; EskfState predicted_state; EskfState state;};
  struct VioMeasurement
  {
    std::int64_t stamp{};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Matrix<double, 6, 6> pose_covariance{Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix3d velocity_covariance{Eigen::Matrix3d::Identity()};
  };
  struct AlignmentCandidate {std::int64_t stamp{}; Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};};

  ImuSample imu_sample(const sensor_msgs::msg::Imu & message) const
  {
    ImuSample sample; sample.stamp_ns = stamp_ns(message.header.stamp);
    sample.gyro = {message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z};
    sample.accel = {message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z};
    sample.accel_valid = message.linear_acceleration_covariance[0] >= 0.0 && sample.accel.allFinite();
    return sample;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto sample = imu_sample(*message);
    if (sample.stamp_ns <= 0 || !sample.gyro.allFinite()) {return;}
    last_received_imu_ = sample; have_imu_ = true; accel_enabled_ = sample.accel_valid;
    last_imu_arrival_ns_ = now().nanoseconds(); imu_arrivals_.push_back(last_imu_arrival_ns_);
    prune_arrivals(imu_arrivals_, last_imu_arrival_ns_);
    if (!filter_.initialized()) {return;}
    if (!filter_.propagate(sample)) {++imu_rejected_; return;}
    history_.push_back({sample, filter_.state(), filter_.state()});
    prune_history();
  }

  void on_vio(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto measurement_stamp = stamp_ns(message->header.stamp);
    if (measurement_stamp <= 0 || message->child_frame_id != base_frame_) {return;}
    // ZED is a single ordered source. Replaying duplicate or older VIO after a
    // newer accepted sample can invalidate the meaning of source freshness and
    // adds needless fixed-lag work under callback jitter.
    if (measurement_stamp <= vio_freshness_.stamp_ns) {
      ++late_measurement_drops_;
      return;
    }
    Eigen::Isometry3d measured;
    try {measured = pose(message->pose.pose);} catch (...) {return;}
    const Eigen::Vector3d body_velocity(
      message->twist.twist.linear.x, message->twist.twist.linear.y, message->twist.twist.linear.z);
    const Eigen::Vector3d world_velocity = measured.linear() * body_velocity;
    if (!world_velocity.allFinite()) {return;}
    const auto arrival_ns = now().nanoseconds();
    if (!filter_.initialized()) {
      ImuSample initial_imu = have_imu_ ? last_received_imu_ : ImuSample{};
      initial_imu.stamp_ns = measurement_stamp;
      EskfState state; state.stamp_ns = measurement_stamp; state.position = measured.translation();
      state.orientation = Eigen::Quaterniond(measured.linear()).normalized(); state.velocity = world_velocity;
      state.covariance.setZero();
      const auto pcov = covariance6(message->pose.covariance); const auto tcov = covariance6(message->twist.covariance);
      state.covariance.block<3, 3>(0, 0) = pcov.block<3, 3>(0, 0).diagonal().cwiseMax(1e-4).asDiagonal();
      state.covariance.block<3, 3>(3, 3) = tcov.block<3, 3>(0, 0).diagonal().cwiseMax(9e-4).asDiagonal();
      state.covariance.block<3, 3>(6, 6) = pcov.block<3, 3>(3, 3).diagonal().cwiseMax(1e-5).asDiagonal();
      state.covariance.block<3, 3>(9, 9).diagonal().setConstant(0.01);
      state.covariance.block<3, 3>(12, 12).diagonal().setConstant(0.25);
      filter_.initialize(state, initial_imu);
      history_.push_back({initial_imu, filter_.state(), filter_.state()});
      history_high_water_ = std::max(history_high_water_, history_.size());
      record_accepted_vio(true, measurement_stamp, arrival_ns);
      return;
    }
    // Loss of the external IMU must not prevent fresh VIO from advancing the
    // estimator. Bridge the missing interval with constant velocity and zero
    // body rate, then let the VIO pose/velocity update provide the constraint.
    // This branch is used only after the 50 ms IMU freshness deadline.
    const bool imu_stale = !have_imu_ ||
      (last_imu_arrival_ns_ > 0 && (now().nanoseconds() - last_imu_arrival_ns_) * 1e-9 > imu_stale_s_);
    const bool bridge_for_vio = imu_stale && measurement_stamp > filter_.state().stamp_ns;
    const EskfState state_before_bridge = filter_.state();
    const ImuSample imu_before_bridge = filter_.last_imu();
    const std::size_t history_size_before_bridge = history_.size();
    if (bridge_for_vio) {
      EskfState bridge_state = state_before_bridge;
      ImuSample bridge_imu = imu_before_bridge;
      bridge_imu.gyro = bridge_state.gyro_bias;
      bridge_imu.accel.setZero(); bridge_imu.accel_valid = false;
      filter_.set_state(bridge_state, bridge_imu);
      while (filter_.state().stamp_ns < measurement_stamp) {
        ImuSample next = bridge_imu;
        next.stamp_ns = std::min(
          measurement_stamp, filter_.state().stamp_ns + static_cast<std::int64_t>(100000000LL));
        if (!filter_.propagate(next)) {break;}
        bridge_imu = next;
      }
      history_.push_back({filter_.last_imu(), filter_.state(), filter_.state()});
    }
    const auto index = history_index(measurement_stamp, 0.12);
    if (!index) {
      if (bridge_for_vio) {
        history_.resize(history_size_before_bridge);
        filter_.set_state(state_before_bridge, imu_before_bridge);
      }
      ++late_measurement_drops_;
      return;
    }
    auto pose_covariance = covariance6(message->pose.covariance);
    for (int i = 0; i < 3; ++i) {pose_covariance(i, i) = std::max(pose_covariance(i, i), 1e-4); pose_covariance(i + 3, i + 3) = std::max(pose_covariance(i + 3, i + 3), 1e-5);}
    Eigen::Matrix3d velocity_covariance = covariance6(message->twist.covariance).block<3, 3>(0, 0);
    velocity_covariance.diagonal() = velocity_covariance.diagonal().cwiseMax(9e-4);
    VioMeasurement event;
    event.stamp = measurement_stamp; event.position = measured.translation();
    event.orientation = Eigen::Quaterniond(measured.linear()).normalized();
    event.velocity = world_velocity; event.pose_covariance = pose_covariance;
    event.velocity_covariance = velocity_covariance;
    const auto insertion = std::lower_bound(vio_measurements_.begin(), vio_measurements_.end(), event.stamp,
      [](const VioMeasurement & candidate, std::int64_t stamp) {return candidate.stamp < stamp;});
    if (insertion != vio_measurements_.end() && insertion->stamp == event.stamp) {
      if (bridge_for_vio) {
        history_.resize(history_size_before_bridge);
        filter_.set_state(state_before_bridge, imu_before_bridge);
      }
      return;
    }
    vio_measurements_.insert(insertion, event);
    const bool accepted = replay_from(*index, measurement_stamp);
    ++fixed_lag_replays_;
    record_accepted_vio(accepted, measurement_stamp, arrival_ns);
    if (!accepted) {
      const auto rejected = std::lower_bound(
        vio_measurements_.begin(), vio_measurements_.end(), measurement_stamp,
        [](const VioMeasurement & candidate, std::int64_t stamp) {return candidate.stamp < stamp;});
      if (rejected != vio_measurements_.end() && rejected->stamp == measurement_stamp) {
        vio_measurements_.erase(rejected);
      }
      if (bridge_for_vio) {
        history_.resize(history_size_before_bridge);
        filter_.set_state(state_before_bridge, imu_before_bridge);
      }
      return;
    }
    prune_history();
  }

  void record_accepted_vio(
    bool accepted, std::int64_t measurement_stamp, std::int64_t arrival_ns)
  {
    if (!vio_freshness_.record(accepted, measurement_stamp, arrival_ns)) {return;}
    vio_transport_s_ = (arrival_ns - measurement_stamp) * 1e-9;
    vio_arrivals_.push_back(arrival_ns);
    prune_arrivals(vio_arrivals_, arrival_ns);
  }

  bool apply_vio_measurement(const VioMeasurement & measurement)
  {
    const EskfState state_before = filter_.state();
    const ImuSample imu_before = filter_.last_imu();
    const bool pose_ok = filter_.update_pose(
      measurement.position, measurement.orientation, measurement.pose_covariance);
    const bool velocity_ok = pose_ok && filter_.update_velocity(
      measurement.velocity, measurement.velocity_covariance);
    if (pose_ok && velocity_ok) {return true;}
    filter_.set_state(state_before, imu_before);
    ++vio_gate_rejections_;
    return false;
  }

  bool replay_from(std::size_t first, std::int64_t tracked_stamp)
  {
    bool tracked_accepted = false;
    filter_.set_state(history_[first].predicted_state, history_[first].imu);
    for (std::size_t i = first; i < history_.size(); ++i) {
      if (i > first) {
        if (!filter_.propagate(history_[i].imu)) {++imu_rejected_; continue;}
        history_[i].predicted_state = filter_.state();
      }
      for (const auto & event : vio_measurements_) {
        const auto event_index = history_index(event.stamp, 0.12);
        if (event_index && *event_index == i) {
          const bool accepted = apply_vio_measurement(event);
          if (event.stamp == tracked_stamp) {tracked_accepted = accepted;}
        }
      }
      history_[i].state = filter_.state();
    }
    return tracked_accepted;
  }

  void on_tag(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!relocalize_requested_ || message->header.frame_id != map_frame_) {return;}
    const auto tag_stamp = stamp_ns(message->header.stamp); const auto index = history_index(tag_stamp, 0.12);
    if (!index) {++late_measurement_drops_; return;}
    Eigen::Isometry3d map_from_base;
    try {map_from_base = pose(message->pose.pose);} catch (...) {return;}
    const auto & local = history_[*index].state;
    const Eigen::Isometry3d odom_from_base = pose_transform(local.position, local.orientation);
    const Eigen::Isometry3d observed = map_from_base * odom_from_base.inverse();
    last_tag_translation_residual_ = map_from_odom_
      ? (map_from_base.translation() - ((*map_from_odom_) * odom_from_base).translation()).norm()
      : std::numeric_limits<double>::quiet_NaN();
    last_tag_angle_residual_deg_ = map_from_odom_
      ? rotation_distance(map_from_base.linear(), ((*map_from_odom_) * odom_from_base).linear()) * 180.0 / M_PI
      : std::numeric_limits<double>::quiet_NaN();
    add_alignment_candidate(observed, tag_stamp);
    if (alignment_candidates_.size() < 4U) {return;}
    const auto confirmed = representative_alignment(); alignment_candidates_.clear();
    if (relocalization_pending_ || !map_from_odom_) {
      map_from_odom_ = confirmed; relocalization_pending_ = false;
    } else {
      const double correction = (confirmed.translation() - map_from_odom_->translation()).norm();
      const double angle = rotation_distance(confirmed.linear(), map_from_odom_->linear());
      if (correction > alignment_max_m_ || angle > alignment_max_rad_) {++tag_gate_rejections_; return;}
      if (correction >= alignment_threshold_m_ || angle >= alignment_threshold_rad_) {
        map_from_odom_ = blend_transform(*map_from_odom_, confirmed, alignment_alpha_);
      }
    }
    last_tag_stamp_ns_ = tag_stamp; last_absolute_stamp_ns_ = tag_stamp;
    last_tag_arrival_ns_ = now().nanoseconds(); tag_transport_s_ = (last_tag_arrival_ns_ - tag_stamp) * 1e-9;
    tag_fusion_latency_ms_.push_back(tag_transport_s_ * 1000.0);
    while (tag_fusion_latency_ms_.size() > 300U) {tag_fusion_latency_ms_.pop_front();}
    tag_arrivals_.push_back(last_tag_arrival_ns_); prune_arrivals(tag_arrivals_, last_tag_arrival_ns_);
  }

  void on_tag_status(const robotcore_interfaces::msg::AprilTagPoseStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_); last_tag_status_ = *message;
    last_tag_frame_arrival_ns_ = now().nanoseconds(); tag_frame_arrivals_.push_back(last_tag_frame_arrival_ns_);
    prune_arrivals(tag_frame_arrivals_, last_tag_frame_arrival_ns_);
  }

  void on_relocalize(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_); reset_alignment();
    relocalize_event_pub_->publish(std_msgs::msg::Empty{}); response->success = true;
    response->message = "map-to-odom reset; waiting for four consistent quality-gated Tag poses";
  }

  void reset_alignment()
  {
    map_from_odom_.reset(); alignment_candidates_.clear();
    relocalize_requested_ = true; relocalization_pending_ = true;
  }

  std::optional<std::size_t> history_index(std::int64_t target, double tolerance_s) const
  {
    if (history_.empty()) {return std::nullopt;}
    std::size_t best = 0U; auto error = std::llabs(history_[0].state.stamp_ns - target);
    for (std::size_t i = 1U; i < history_.size(); ++i) {
      const auto candidate = std::llabs(history_[i].state.stamp_ns - target);
      if (candidate < error) {best = i; error = candidate;}
    }
    return error <= static_cast<std::int64_t>(tolerance_s * 1e9) ? std::optional<std::size_t>(best) : std::nullopt;
  }

  void add_alignment_candidate(const Eigen::Isometry3d & candidate, std::int64_t stamp)
  {
    if (!alignment_candidates_.empty() && (stamp - alignment_candidates_.back().stamp) > 500000000LL) {
      alignment_candidates_.clear();
    }
    if (!alignment_candidates_.empty()) {
      const auto & first = alignment_candidates_.front().transform;
      if ((candidate.translation() - first.translation()).norm() > 0.20 ||
          rotation_distance(candidate.linear(), first.linear()) > 12.0 * M_PI / 180.0) {
        alignment_candidates_.clear();
      }
    }
    alignment_candidates_.push_back({stamp, candidate});
    while (alignment_candidates_.size() > 4U) {alignment_candidates_.pop_front();}
  }

  Eigen::Isometry3d representative_alignment() const
  {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q(alignment_candidates_.front().transform.linear());
    for (std::size_t i = 0; i < alignment_candidates_.size(); ++i) {
      translation += alignment_candidates_[i].transform.translation();
      if (i > 0U) {q = q.slerp(1.0 / static_cast<double>(i + 1U), Eigen::Quaterniond(alignment_candidates_[i].transform.linear())).normalized();}
    }
    return pose_transform(translation / static_cast<double>(alignment_candidates_.size()), q);
  }

  void prune_history()
  {
    history_high_water_ = std::max(history_high_water_, history_.size());
    if (history_.empty()) {return;} const auto newest = history_.back().state.stamp_ns;
    while (history_.size() > 2U && newest - history_.front().state.stamp_ns > history_duration_s_ * 1e9) {history_.pop_front();}
    if (!history_.empty()) {
      const auto oldest_allowed = history_.front().state.stamp_ns - 120000000LL;
      while (!vio_measurements_.empty() && vio_measurements_.front().stamp < oldest_allowed) {
        vio_measurements_.pop_front();
      }
    }
  }

  static void prune_arrivals(std::deque<std::int64_t> & values, std::int64_t newest)
  {while (!values.empty() && newest - values.front() > 5000000000LL) {values.pop_front();}}

  static double rate(const std::deque<std::int64_t> & values)
  {
    if (values.size() < 2U) {return 0.0;}
    return static_cast<double>(values.size() - 1U) * 1e9 / static_cast<double>(values.back() - values.front());
  }

  static double percentile95(const std::deque<double> & values)
  {
    if (values.empty()) {return INFINITY;}
    auto sorted = std::vector<double>(values.begin(), values.end());
    const auto index = static_cast<std::size_t>(std::ceil(0.95 * sorted.size())) - 1U;
    std::nth_element(sorted.begin(), sorted.begin() + index, sorted.end());
    return sorted[index];
  }

  std::string localization_source(bool absolute, std::int64_t now_ns) const
  {
    const bool imu_fresh = have_imu_ && last_imu_arrival_ns_ > 0 &&
      (now_ns - last_imu_arrival_ns_) * 1e-9 <= imu_stale_s_;
    if (imu_fresh) {
      const std::string source = absolute ? "Tag+ZED VIO+External IMU" : "ZED VIO+External IMU";
      return accel_enabled_ ? source : source + " (gyro-only)";
    }
    const std::string source = absolute ? "Tag+ZED VIO" : "ZED VIO";
    return source + " (External IMU unavailable)";
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex_); if (!filter_.initialized()) {return;}
    const auto now_time = now(); const auto now_ns = now_time.nanoseconds();
    FixedLagEskf output_filter = filter_; const double prediction_age = (now_ns - filter_.state().stamp_ns) * 1e-9;
    if (have_imu_ && prediction_age > 0.0 && prediction_age <= imu_stale_s_) {
      auto predicted = last_received_imu_; predicted.stamp_ns = now_ns; output_filter.propagate(predicted);
    }
    const auto & state = output_filter.state();
    state_age_ms_.push_back((now_ns - state.stamp_ns) * 1e-6);
    while (state_age_ms_.size() > 300U) {state_age_ms_.pop_front();}
    const double vio_age = vio_freshness_.stamp_ns > 0 ?
      (now_ns - vio_freshness_.stamp_ns) * 1e-9 : INFINITY;
    const double tag_age = last_tag_stamp_ns_ > 0 ? (now_ns - last_tag_stamp_ns_) * 1e-9 : INFINITY;
    const bool pose_usable = map_from_odom_ && (vio_age <= vio_stale_s_ || vio_age <= inertial_horizon_s_);
    const bool absolute_valid = pose_usable && tag_age <= tag_fresh_s_;
    const bool estimated = pose_usable && !absolute_valid;
    if (!map_from_odom_) {return;}
    const Eigen::Isometry3d odom_from_base = pose_transform(state.position, state.orientation);
    const Eigen::Isometry3d map_from_base = *map_from_odom_ * odom_from_base;
    const Eigen::Vector3d body_velocity = state.orientation.conjugate() * state.velocity;
    const Eigen::Vector3d angular = last_received_imu_.gyro - state.gyro_bias;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(state.stamp_ns, RCL_ROS_TIME);
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    set_pose(odom.pose.pose, map_from_base); odom.twist.twist.linear.x = body_velocity.x();
    odom.twist.twist.linear.y = body_velocity.y(); odom.twist.twist.linear.z = body_velocity.z();
    odom.twist.twist.angular.x = angular.x(); odom.twist.twist.angular.y = angular.y(); odom.twist.twist.angular.z = angular.z();
    fill_covariances(odom, state);
    fused_pub_->publish(odom); aligned_pub_->publish(odom);

    robotcore_interfaces::msg::BodyState body; body.header = odom.header; body.pose = odom.pose.pose; body.twist = odom.twist.twist;
    body.linear_velocity_valid = vio_age <= inertial_horizon_s_;
    body.state_valid = absolute_valid;
    body.position_estimated = estimated;
    body.localization_source = localization_source(absolute_valid, now_ns);
    body_pub_->publish(body);
    publish_localization_status(now_time, odom, vio_age, tag_age, pose_usable, absolute_valid, estimated);
    fused_arrivals_.push_back(now_ns); prune_arrivals(fused_arrivals_, now_ns); updater_.force_update();
  }

  void fill_covariances(nav_msgs::msg::Odometry & odom, const EskfState & state) const
  {
    odom.pose.covariance.fill(0.0); odom.twist.covariance.fill(0.0);
    const Eigen::Matrix3d map_rotation = map_from_odom_->linear();
    const Eigen::Matrix3d position_cov = map_rotation * state.covariance.block<3, 3>(0, 0) * map_rotation.transpose();
    const Eigen::Matrix3d attitude_cov = state.covariance.block<3, 3>(6, 6);
    const Eigen::Matrix3d velocity_cov = state.orientation.conjugate().toRotationMatrix() *
      state.covariance.block<3, 3>(3, 3) * state.orientation.toRotationMatrix();
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) {
      odom.pose.covariance[6 * r + c] = position_cov(r, c);
      odom.pose.covariance[6 * (r + 3) + c + 3] = attitude_cov(r, c);
      odom.twist.covariance[6 * r + c] = velocity_cov(r, c);
    }
    odom.twist.covariance[21] = odom.twist.covariance[28] = odom.twist.covariance[35] = 0.01;
  }

  void publish_localization_status(const rclcpp::Time & stamp, const nav_msgs::msg::Odometry & odom,
    double vio_age, double tag_age, bool pose_usable, bool absolute, bool estimated)
  {
    robotcore_interfaces::msg::LocalizationStatus status; status.header.stamp = stamp; status.header.frame_id = map_frame_;
    if (vio_freshness_.stamp_ns > 0) {
      status.last_vio_stamp = rclcpp::Time(vio_freshness_.stamp_ns, RCL_ROS_TIME);
    }
    if (last_tag_stamp_ns_ > 0) {status.last_tag_stamp = rclcpp::Time(last_tag_stamp_ns_, RCL_ROS_TIME);}
    if (last_absolute_stamp_ns_ > 0) {status.last_absolute_fix_stamp = rclcpp::Time(last_absolute_stamp_ns_, RCL_ROS_TIME);}
    status.vio_age_s = vio_age; status.tag_age_s = tag_age;
    status.absolute_fix_age_s = last_absolute_stamp_ns_ > 0 ? (stamp.nanoseconds() - last_absolute_stamp_ns_) * 1e-9 : INFINITY;
    status.apriltag_frame_age_s = last_tag_frame_arrival_ns_ > 0 ? (stamp.nanoseconds() - last_tag_frame_arrival_ns_) * 1e-9 : INFINITY;
    status.vio_rate_hz = rate(vio_arrivals_); status.tag_rate_hz = rate(tag_arrivals_);
    status.fused_rate_hz = rate(fused_arrivals_); status.apriltag_frame_rate_hz = rate(tag_frame_arrivals_);
    status.vio_transport_delay_s = vio_transport_s_; status.tag_transport_delay_s = tag_transport_s_;
    status.tag_vio_translation_residual_m = last_tag_translation_residual_;
    status.tag_vio_angle_residual_deg = last_tag_angle_residual_deg_;
    status.pose_covariance = odom.pose.covariance; status.twist_covariance = odom.twist.covariance;
    status.detected_tag_count = last_tag_status_.detected_tag_count; status.mapped_tag_count = last_tag_status_.mapped_tag_count;
    status.inlier_tag_count = last_tag_status_.inlier_tag_count; status.tag_reprojection_rms_px = last_tag_status_.reprojection_rms_px;
    status.minimum_tag_edge_px = last_tag_status_.minimum_tag_edge_px; status.tag_pose_published = last_tag_status_.pose_published;
    status.vio_fresh = vio_age <= vio_stale_s_; status.tag_fresh = tag_age <= tag_fresh_s_;
    status.tag_consistent = std::isfinite(last_tag_translation_residual_) && last_tag_translation_residual_ <= 0.50 &&
      last_tag_angle_residual_deg_ <= 10.0; status.absolute_fix_valid = absolute; status.position_estimated = estimated;
    status.localization_source = pose_usable ? localization_source(absolute, stamp.nanoseconds()) : "";
    status.tag_observation_class = last_tag_status_.degraded ? "degraded" :
      (last_tag_status_.pose_published ? "primary" : "");
    status.apriltag_rejection_reason = last_tag_status_.rejection_reason;
    status.rejection_reason = pose_usable ? "" : "no map alignment or VIO/inertial state is stale";
    status_pub_->publish(status);
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    const auto now_ns = now().nanoseconds(); const double imu_age_ms = have_imu_ ? (now_ns - last_received_imu_.stamp_ns) * 1e-6 : INFINITY;
    const int level = filter_.initialized() && map_from_odom_ ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.summary(level, map_from_odom_ ? "estimating" : "waiting for relocalized map alignment");
    status.add("imu_rate_hz", rate(imu_arrivals_)); status.add("imu_age_ms", imu_age_ms);
    status.add("imu_calibrated", have_imu_); status.add("accel_fusion_enabled", accel_enabled_);
    status.add("state_age_ms", filter_.initialized() ? (now_ns - filter_.state().stamp_ns) * 1e-6 : INFINITY);
    status.add("state_age_p95_ms", percentile95(state_age_ms_));
    status.add("tag_fusion_ms", tag_transport_s_ * 1000.0); status.add("fixed_lag_replays", fixed_lag_replays_);
    status.add("tag_fusion_p95_ms", percentile95(tag_fusion_latency_ms_));
    status.add("late_measurement_drops", late_measurement_drops_); status.add("imu_rejected", imu_rejected_);
    status.add("vio_gate_rejections", vio_gate_rejections_); status.add("tag_gate_rejections", tag_gate_rejections_);
    status.add("history_size", history_.size()); status.add("history_high_water", history_high_water_);
    status.add("history_capacity_target", static_cast<int>(history_duration_s_ * 100.0));
  }

  std::mutex mutex_; FixedLagEskf filter_; std::deque<HistoryEntry> history_;
  std::deque<VioMeasurement> vio_measurements_; ImuSample last_received_imu_;
  bool have_imu_{false}, accel_enabled_{false}, relocalize_requested_{true}, relocalization_pending_{true};
  std::optional<Eigen::Isometry3d> map_from_odom_; std::deque<AlignmentCandidate> alignment_candidates_;
  double history_duration_s_{}, imu_stale_s_{}, vio_stale_s_{}, inertial_horizon_s_{}, tag_fresh_s_{};
  double alignment_alpha_{}, alignment_max_m_{}, alignment_max_rad_{}, alignment_threshold_m_{}, alignment_threshold_rad_{};
  std::string map_frame_, odom_frame_, base_frame_;
  AcceptedMeasurementFreshness vio_freshness_;
  std::int64_t last_imu_arrival_ns_{}, last_tag_stamp_ns_{};
  std::int64_t last_tag_arrival_ns_{}, last_absolute_stamp_ns_{}, last_tag_frame_arrival_ns_{};
  double vio_transport_s_{NAN}, tag_transport_s_{NAN}, last_tag_translation_residual_{NAN}, last_tag_angle_residual_deg_{NAN};
  std::uint64_t fixed_lag_replays_{}, late_measurement_drops_{}, imu_rejected_{}, vio_gate_rejections_{}, tag_gate_rejections_{};
  std::size_t history_high_water_{};
  std::deque<std::int64_t> imu_arrivals_, vio_arrivals_, tag_arrivals_, tag_frame_arrivals_, fused_arrivals_;
  std::deque<double> state_age_ms_, tag_fusion_latency_ms_;
  robotcore_interfaces::msg::AprilTagPoseStatus last_tag_status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tag_sub_;
  rclcpp::Subscription<robotcore_interfaces::msg::AprilTagPoseStatus>::SharedPtr tag_status_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_, aligned_pub_;
  rclcpp::Publisher<robotcore_interfaces::msg::BodyState>::SharedPtr body_pub_;
  rclcpp::Publisher<robotcore_interfaces::msg::LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_service_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr relocalize_event_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr relocalize_event_sub_;
  rclcpp::TimerBase::SharedPtr output_timer_; diagnostic_updater::Updater updater_;
};
}  // namespace robotcore_sensors
RCLCPP_COMPONENTS_REGISTER_NODE(robotcore_sensors::FixedLagEskfComponent)
