#include "robotcore_sensors/covariance.hpp"
#include "robotcore_sensors/geometry.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <robotcore_interfaces/msg/april_tag_pose_estimate.hpp>
#include <robotcore_interfaces/msg/body_state.hpp>
#include <robotcore_interfaces/msg/localization_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <zed_msgs/msg/pos_track_status.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <pthread.h>
#include <sched.h>

namespace robotcore_sensors
{
namespace
{
using Matrix12d = Eigen::Matrix<double, 12, 12>;
constexpr std::int64_t kRateWindowNs = 5000000000LL;

void configure_fifo_thread(
  const rclcpp::Logger & logger, const char * thread_name, int priority)
{
  if (priority <= 0) {return;}
  sched_param parameters{};
  parameters.sched_priority = priority;
  const int error = pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters);
  if (error != 0) {
    RCLCPP_ERROR(
      logger, "Failed to set %s to SCHED_FIFO/%d: %s",
      thread_name, priority, std::strerror(error));
    return;
  }
  RCLCPP_INFO(logger, "%s uses SCHED_FIFO/%d", thread_name, priority);
}

enum class UpdateBlock
{
  Position,
  LinearVelocity,
  Orientation,
  AngularVelocity
};

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

Eigen::Isometry3d pose(const geometry_msgs::msg::Pose & message)
{
  const Eigen::Vector3d position(
    message.position.x, message.position.y, message.position.z);
  Eigen::Quaterniond orientation(
    message.orientation.w, message.orientation.x,
    message.orientation.y, message.orientation.z);
  if (!position.allFinite() || !orientation.coeffs().allFinite() ||
    orientation.norm() < 1e-9)
  {
    throw std::runtime_error("invalid pose");
  }
  return pose_transform(position, orientation.normalized());
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
      covariance(row, column) = values[6 * row + column];
    }
  }
  return covariance;
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

Eigen::Matrix<double, 6, 6> pose_covariance_at_base(
  const Eigen::Matrix<double, 6, 6> & source_covariance,
  const Eigen::Isometry3d & odom_from_source,
  const Eigen::Isometry3d & source_from_base)
{
  const Eigen::Vector3d lever_arm_odom =
    odom_from_source.linear() * source_from_base.translation();
  Eigen::Matrix<double, 6, 6> jacobian =
    Eigen::Matrix<double, 6, 6>::Identity();
  jacobian.block<3, 3>(0, 3) = -skew(lever_arm_odom);
  const Eigen::Matrix<double, 6, 6> result =
    jacobian * source_covariance * jacobian.transpose();
  return symmetrized_covariance<6>(result);
}

Eigen::Matrix<double, 6, 6> pose_covariance_in_filter_coordinates(
  const Eigen::Matrix<double, 6, 6> & fixed_axis_covariance,
  const Eigen::Quaterniond & orientation)
{
  Eigen::Matrix<double, 6, 6> jacobian =
    Eigen::Matrix<double, 6, 6>::Identity();
  jacobian.block<3, 3>(3, 3) = orientation.conjugate().toRotationMatrix();
  const Eigen::Matrix<double, 6, 6> result =
    jacobian * fixed_axis_covariance * jacobian.transpose();
  return symmetrized_covariance<6>(result);
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

void prune_rate_window(std::deque<std::int64_t> & samples, std::int64_t now_ns)
{
  while (!samples.empty() && now_ns - samples.front() > kRateWindowNs) {
    samples.pop_front();
  }
}

double window_rate_hz(
  const std::deque<std::int64_t> & samples, std::int64_t now_ns)
{
  if (samples.size() < 2U || now_ns < samples.back() ||
    now_ns - samples.back() > kRateWindowNs)
  {
    return 0.0;
  }
  const auto duration_ns = samples.back() - samples.front();
  return duration_ns > 0 ?
         static_cast<double>(samples.size() - 1U) * 1e9 /
         static_cast<double>(duration_ns) : 0.0;
}

bool passes_three_dof_gate(
  const Eigen::Vector3d & residual,
  const Eigen::Matrix3d & covariance,
  double gate_chi2)
{
  if (!residual.allFinite() || !covariance.allFinite() ||
    !std::isfinite(gate_chi2) || gate_chi2 <= 0.0)
  {
    return false;
  }
  const Eigen::Matrix3d symmetric = symmetrized_covariance<3>(covariance);
  const Eigen::LDLT<Eigen::Matrix3d> decomposition(symmetric);
  if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
    return false;
  }
  const double nis = residual.dot(decomposition.solve(residual));
  return std::isfinite(nis) && nis <= gate_chi2;
}

struct EkfState
{
  std::int64_t stamp_ns{};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  // Zero-order-held IMU input in odom coordinates. It is part of each replay
  // checkpoint so delayed VIO corrections are re-propagated through the same
  // acceleration history that was available in real time.
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d linear_acceleration_covariance{Eigen::Matrix3d::Zero()};
  Matrix12d covariance{Matrix12d::Identity()};
};

struct EkfNoise
{
  double linear_acceleration{0.30};
  double angular_acceleration{0.20};
};

class VioTagEkf
{
public:
  explicit VioTagEkf(EkfNoise noise = {})
  : noise_(noise) {}

  bool initialize(const EkfState & state)
  {
    state_ = state;
    state_.orientation.normalize();
    initialized_ = stabilize_state_covariance();
    return initialized_;
  }

  bool propagate_to(std::int64_t stamp)
  {
    if (!initialized_ || stamp < state_.stamp_ns) {return false;}
    if (stamp == state_.stamp_ns) {return true;}
    const double dt = static_cast<double>(stamp - state_.stamp_ns) * 1e-9;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {return false;}

    const double dt2 = dt * dt;
    state_.position += state_.velocity * dt +
      0.5 * state_.linear_acceleration * dt2;
    state_.velocity += state_.linear_acceleration * dt;
    state_.orientation =
      (state_.orientation * exp_quaternion(state_.angular_velocity * dt)).normalized();

    Matrix12d dynamics = Matrix12d::Zero();
    dynamics.block<3, 3>(0, 3).setIdentity();
    dynamics.block<3, 3>(6, 6) = -skew(state_.angular_velocity);
    dynamics.block<3, 3>(6, 9).setIdentity();
    const Matrix12d transition =
      Matrix12d::Identity() + dynamics * dt + 0.5 * dynamics * dynamics * dt * dt;

    Matrix12d process_covariance = Matrix12d::Zero();
    const double linear_variance =
      noise_.linear_acceleration * noise_.linear_acceleration;
    const double angular_variance =
      noise_.angular_acceleration * noise_.angular_acceleration;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    process_covariance.block<3, 3>(0, 0).diagonal().setConstant(
      linear_variance * dt3 / 3.0);
    process_covariance.block<3, 3>(0, 3).diagonal().setConstant(
      linear_variance * dt2 / 2.0);
    process_covariance.block<3, 3>(3, 0) =
      process_covariance.block<3, 3>(0, 3).transpose();
    process_covariance.block<3, 3>(3, 3).diagonal().setConstant(
      linear_variance * dt);
    // The current accelerometer sample is held between IMU events. Its
    // reported covariance is discrete input uncertainty, distinct from the
    // continuous unmodelled-acceleration noise above.
    process_covariance.block<3, 3>(0, 0) +=
      0.25 * state_.linear_acceleration_covariance * dt4;
    process_covariance.block<3, 3>(0, 3) +=
      0.5 * state_.linear_acceleration_covariance * dt3;
    process_covariance.block<3, 3>(3, 0) +=
      0.5 * state_.linear_acceleration_covariance * dt3;
    process_covariance.block<3, 3>(3, 3) +=
      state_.linear_acceleration_covariance * dt2;
    process_covariance.block<3, 3>(6, 6).diagonal().setConstant(
      angular_variance * dt3 / 3.0);
    process_covariance.block<3, 3>(6, 9).diagonal().setConstant(
      angular_variance * dt2 / 2.0);
    process_covariance.block<3, 3>(9, 6) =
      process_covariance.block<3, 3>(6, 9).transpose();
    process_covariance.block<3, 3>(9, 9).diagonal().setConstant(
      angular_variance * dt);

    const Matrix12d propagated_covariance =
      (transition * state_.covariance * transition.transpose() +
      process_covariance).eval();
    state_.covariance = propagated_covariance;
    state_.stamp_ns = stamp;
    initialized_ = stabilize_state_covariance();
    return initialized_;
  }

  bool update_position(
    const Eigen::Vector3d & position,
    const Eigen::Matrix3d & covariance,
    double gate_chi2)
  {
    Eigen::Matrix<double, 3, 12> observation =
      Eigen::Matrix<double, 3, 12>::Zero();
    observation.block<3, 3>(0, 0).setIdentity();
    const bool accepted = update<3>(
      position - state_.position, observation, covariance, gate_chi2,
      UpdateBlock::Position);
    last_pose_nis_ = last_nis_;
    return accepted;
  }

  bool update_orientation(
    const Eigen::Quaterniond & orientation,
    const Eigen::Matrix3d & covariance,
    double gate_chi2)
  {
    Eigen::Matrix<double, 3, 12> observation =
      Eigen::Matrix<double, 3, 12>::Zero();
    observation.block<3, 3>(0, 6).setIdentity();
    return update<3>(
      log_quaternion(state_.orientation.conjugate() * orientation.normalized()),
      observation, covariance, gate_chi2, UpdateBlock::Orientation);
  }

  bool update_linear_velocity(
    const Eigen::Vector3d & body_velocity,
    const Eigen::Matrix3d & covariance,
    double gate_chi2,
    double correction_limit_mps,
    double innovation_rejection_limit_mps)
  {
    last_linear_velocity_innovation_mps_ = NAN;
    last_linear_velocity_correction_mps_ = NAN;
    last_linear_velocity_correction_limited_ = false;
    const Eigen::Matrix3d body_from_odom =
      state_.orientation.conjugate().toRotationMatrix();
    const Eigen::Vector3d predicted_linear = body_from_odom * state_.velocity;
    const Eigen::Vector3d innovation = body_velocity - predicted_linear;
    last_linear_velocity_innovation_mps_ = innovation.norm();
    if (!std::isfinite(correction_limit_mps) || correction_limit_mps <= 0.0 ||
      !std::isfinite(innovation_rejection_limit_mps) ||
      innovation_rejection_limit_mps <= 0.0 ||
      last_linear_velocity_innovation_mps_ > innovation_rejection_limit_mps)
    {
      last_twist_nis_ = INFINITY;
      return false;
    }
    Eigen::Matrix<double, 3, 12> observation =
      Eigen::Matrix<double, 3, 12>::Zero();
    observation.block<3, 3>(0, 3) = body_from_odom;
    observation.block<3, 3>(0, 6) = skew(predicted_linear);
    const bool accepted = update<3>(
      innovation, observation, covariance, gate_chi2, UpdateBlock::LinearVelocity,
      correction_limit_mps);
    last_twist_nis_ = last_nis_;
    return accepted;
  }

  bool update_angular_velocity(
    const Eigen::Vector3d & angular_velocity,
    const Eigen::Matrix3d & covariance,
    double gate_chi2)
  {
    Eigen::Matrix<double, 3, 12> observation =
      Eigen::Matrix<double, 3, 12>::Zero();
    observation.block<3, 3>(0, 9).setIdentity();
    return update<3>(
      angular_velocity - state_.angular_velocity,
      observation, covariance, gate_chi2, UpdateBlock::AngularVelocity);
  }

  bool set_linear_acceleration(
    const Eigen::Vector3d & linear_acceleration,
    const Eigen::Matrix3d & covariance)
  {
    if (!initialized_ || !linear_acceleration.allFinite() ||
      !valid_covariance<3>(covariance))
    {
      return false;
    }
    state_.linear_acceleration = linear_acceleration;
    state_.linear_acceleration_covariance = covariance;
    return true;
  }

  const EkfState & state() const {return state_;}
  bool initialized() const {return initialized_;}
  double last_pose_nis() const {return last_pose_nis_;}
  double last_twist_nis() const {return last_twist_nis_;}
  double last_linear_velocity_innovation_mps() const
  {
    return last_linear_velocity_innovation_mps_;
  }
  double last_linear_velocity_correction_mps() const
  {
    return last_linear_velocity_correction_mps_;
  }
  bool last_linear_velocity_correction_limited() const
  {
    return last_linear_velocity_correction_limited_;
  }

private:
  template<int Size>
  bool update(
    const Eigen::Matrix<double, Size, 1> & innovation,
    const Eigen::Matrix<double, Size, 12> & observation,
    const Eigen::Matrix<double, Size, Size> & measurement_covariance,
    double gate_chi2,
    UpdateBlock update_block,
    double velocity_correction_limit_mps = INFINITY)
  {
    if (!initialized_ || !innovation.allFinite() ||
      !measurement_covariance.allFinite() ||
      !std::isfinite(gate_chi2) || gate_chi2 <= 0.0)
    {
      return false;
    }
    const Eigen::LDLT<Eigen::Matrix<double, Size, Size>> measurement_decomposition(
      measurement_covariance);
    if (measurement_decomposition.info() != Eigen::Success ||
      !measurement_decomposition.isPositive())
    {
      return false;
    }
    const Eigen::Matrix<double, Size, Size> raw_innovation_covariance =
      observation * state_.covariance * observation.transpose() +
      measurement_covariance;
    const Eigen::Matrix<double, Size, Size> innovation_covariance =
      symmetrized_covariance<Size>(raw_innovation_covariance);
    const Eigen::LDLT<Eigen::Matrix<double, Size, Size>> decomposition(
      innovation_covariance);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
      return false;
    }
    last_nis_ = innovation.dot(decomposition.solve(innovation));
    if (!std::isfinite(last_nis_) || last_nis_ > gate_chi2) {return false;}

    Eigen::Matrix<double, 12, Size> gain =
      state_.covariance * observation.transpose() *
      decomposition.solve(Eigen::Matrix<double, Size, Size>::Identity());
    // Pose and twist blocks in one ZED odometry sample are correlated, but
    // nav_msgs/Odometry carries no pose/twist cross-covariance.  Each observed
    // block therefore corrects only its matching state block.
    switch (update_block) {
      case UpdateBlock::Position:
        gain.template block<9, Size>(3, 0).setZero();
        break;
      case UpdateBlock::LinearVelocity:
        gain.template block<3, Size>(0, 0).setZero();
        gain.template block<6, Size>(6, 0).setZero();
        break;
      case UpdateBlock::Orientation:
        gain.template block<6, Size>(0, 0).setZero();
        gain.template block<3, Size>(9, 0).setZero();
        break;
      case UpdateBlock::AngularVelocity:
        gain.template block<9, Size>(0, 0).setZero();
        break;
    }
    Eigen::Matrix<double, 12, 1> correction = gain * innovation;
    if (std::isfinite(velocity_correction_limit_mps)) {
      const double velocity_correction_mps = correction.segment<3>(3).norm();
      last_linear_velocity_correction_mps_ = velocity_correction_mps;
      if (velocity_correction_mps > velocity_correction_limit_mps) {
        const double bounded_influence =
          velocity_correction_limit_mps / velocity_correction_mps;
        gain *= bounded_influence;
        correction *= bounded_influence;
        last_linear_velocity_correction_mps_ = velocity_correction_limit_mps;
        last_linear_velocity_correction_limited_ = true;
      }
    }
    const Matrix12d identity = Matrix12d::Identity();
    const Matrix12d residual = identity - gain * observation;
    const Matrix12d updated_covariance =
      (residual * state_.covariance * residual.transpose() +
      gain * measurement_covariance * gain.transpose()).eval();
    state_.covariance = updated_covariance;
    inject(correction);
    initialized_ = stabilize_state_covariance();
    return initialized_;
  }

  void inject(const Eigen::Matrix<double, 12, 1> & correction)
  {
    state_.position += correction.segment<3>(0);
    state_.velocity += correction.segment<3>(3);
    state_.orientation =
      (state_.orientation * exp_quaternion(correction.segment<3>(6))).normalized();
    state_.angular_velocity += correction.segment<3>(9);
    Matrix12d reset = Matrix12d::Identity();
    reset.block<3, 3>(6, 6) -= 0.5 * skew(correction.segment<3>(6));
    const Matrix12d reset_covariance =
      (reset * state_.covariance * reset.transpose()).eval();
    state_.covariance = reset_covariance;
  }

  bool stabilize_state_covariance()
  {
    return robotcore_sensors::stabilize_covariance<12>(state_.covariance);
  }

  EkfState state_;
  EkfNoise noise_;
  bool initialized_{false};
  double last_nis_{NAN};
  double last_pose_nis_{NAN};
  double last_twist_nis_{NAN};
  double last_linear_velocity_innovation_mps_{NAN};
  double last_linear_velocity_correction_mps_{NAN};
  bool last_linear_velocity_correction_limited_{false};
};

class RosClockOffsetJumpDetector
{
public:
  bool update(std::int64_t ros_ns, std::int64_t steady_ns)
  {
    if (!initialized_) {
      initialized_ = true;
      last_ros_ns_ = ros_ns;
      last_steady_ns_ = steady_ns;
      return false;
    }
    const auto offset_change =
      (ros_ns - last_ros_ns_) - (steady_ns - last_steady_ns_);
    last_ros_ns_ = ros_ns;
    last_steady_ns_ = steady_ns;
    return std::llabs(offset_change) > 100000000LL;
  }

private:
  std::int64_t last_ros_ns_{};
  std::int64_t last_steady_ns_{};
  bool initialized_{false};
};
}  // namespace

class VioTagFusionComponent final : public rclcpp::Node
{
public:
  explicit VioTagFusionComponent(const rclcpp::NodeOptions & options)
  : Node("ekf", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    updater_(this)
  {
    const auto realtime_priority = declare_parameter<std::int64_t>(
      "executor_realtime_priority", 0);
    if (realtime_priority < 0 || realtime_priority > 99) {
      throw std::invalid_argument("executor_realtime_priority must be in [0, 99]");
    }
    executor_realtime_priority_ = static_cast<int>(realtime_priority);
    history_duration_s_ = declare_parameter<double>("history_duration_s", 3.0);
    vio_arrival_timeout_s_ = declare_parameter<double>("vio_arrival_timeout_s", 0.40);
    vio_prediction_horizon_s_ = declare_parameter<double>(
      "vio_prediction_horizon_s", 0.40);
    tag_fresh_s_ = declare_parameter<double>("tag_fresh_s", 0.35);
    zed_status_timeout_s_ = declare_parameter<double>("zed_status_timeout_s", 0.20);
    // Every live correction below is three-dimensional.  16.266 is the
    // 99.9-percent chi-square threshold for three degrees of freedom.
    pose_gate_chi2_ = declare_parameter<double>("vio_pose_gate_chi2", 16.266);
    twist_gate_chi2_ = declare_parameter<double>("vio_twist_gate_chi2", 16.266);
    imu_angular_velocity_gate_chi2_ = declare_parameter<double>(
      "imu_angular_velocity_gate_chi2", 16.266);
    use_imu_linear_acceleration_ = declare_parameter<bool>(
      "use_imu_linear_acceleration", false);
    gravity_mps2_ = declare_parameter<double>("gravity_mps2", 9.80665);
    maximum_imu_linear_acceleration_mps2_ = declare_parameter<double>(
      "maximum_imu_linear_acceleration_mps2", 6.0);
    imu_linear_acceleration_stddev_floor_mps2_ = declare_parameter<double>(
      "imu_linear_acceleration_stddev_floor_mps2", 0.15);
    linear_velocity_stddev_floor_mps_ = declare_parameter<double>(
      "vio_linear_velocity_stddev_floor_mps", 0.10);
    linear_velocity_correction_limit_mps_ = declare_parameter<double>(
      "vio_linear_velocity_correction_limit_mps", 0.04);
    linear_velocity_innovation_limit_mps_ = declare_parameter<double>(
      "vio_linear_velocity_innovation_limit_mps", 0.25);
    tag_gate_chi2_ = declare_parameter<double>("tag_pose_gate_chi2", 16.266);
    const auto alignment_candidate_count = declare_parameter<std::int64_t>(
      "alignment_candidate_count", 4);
    alignment_candidate_window_s_ = declare_parameter<double>(
      "alignment_candidate_window_s", 8.0);
    alignment_translation_tolerance_m_ = declare_parameter<double>(
      "alignment_translation_tolerance_m", 0.20);
    alignment_rotation_tolerance_rad_ = declare_parameter<double>(
      "alignment_rotation_tolerance_deg", 12.0) * M_PI / 180.0;
    const auto minimum_alignment_tag_count = declare_parameter<std::int64_t>(
      "minimum_alignment_tag_count", 2);
    alignment_update_gain_ = declare_parameter<double>("alignment_update_gain", 0.02);
    alignment_max_translation_step_m_ = declare_parameter<double>(
      "alignment_max_translation_step_m", 0.002);
    alignment_max_rotation_step_rad_ = declare_parameter<double>(
      "alignment_max_rotation_step_deg", 0.1) * M_PI / 180.0;
    require_zed_tracking_ok_ = declare_parameter<bool>("require_zed_tracking_ok", true);
    initial_velocity_stddev_ = declare_parameter<double>(
      "initial_velocity_stddev_mps", 0.25);
    initial_angular_velocity_stddev_ = declare_parameter<double>(
      "initial_angular_velocity_stddev_rps", 0.25);
    noise_.linear_acceleration = declare_parameter<double>(
      "linear_acceleration_noise_mps2_sqrt_hz", 0.30);
    noise_.angular_acceleration = declare_parameter<double>(
      "angular_acceleration_noise_rps2_sqrt_hz", 0.20);
    filter_ = VioTagEkf(noise_);

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    if (history_duration_s_ <= 0.5 || vio_arrival_timeout_s_ <= 0.0 ||
      vio_prediction_horizon_s_ <= 0.0 || tag_fresh_s_ <= 0.0 ||
      zed_status_timeout_s_ <= 0.0 || pose_gate_chi2_ <= 0.0 ||
      twist_gate_chi2_ <= 0.0 || imu_angular_velocity_gate_chi2_ <= 0.0 ||
      !std::isfinite(gravity_mps2_) || gravity_mps2_ <= 0.0 ||
      !std::isfinite(maximum_imu_linear_acceleration_mps2_) ||
      maximum_imu_linear_acceleration_mps2_ <= 0.0 ||
      !std::isfinite(imu_linear_acceleration_stddev_floor_mps2_) ||
      imu_linear_acceleration_stddev_floor_mps2_ <= 0.0 ||
      tag_gate_chi2_ <= 0.0 ||
      linear_velocity_stddev_floor_mps_ <= 0.0 ||
      linear_velocity_correction_limit_mps_ <= 0.0 ||
      linear_velocity_innovation_limit_mps_ <= 0.0 ||
      linear_velocity_correction_limit_mps_ >
      linear_velocity_innovation_limit_mps_ ||
      alignment_candidate_count < 2 || alignment_candidate_count > 32 ||
      !std::isfinite(alignment_candidate_window_s_) ||
      alignment_candidate_window_s_ < 1.0 || alignment_candidate_window_s_ > 30.0 ||
      !std::isfinite(alignment_translation_tolerance_m_) ||
      alignment_translation_tolerance_m_ <= 0.0 ||
      alignment_translation_tolerance_m_ > 2.0 ||
      !std::isfinite(alignment_rotation_tolerance_rad_) ||
      alignment_rotation_tolerance_rad_ <= 0.0 ||
      alignment_rotation_tolerance_rad_ > M_PI / 2.0 ||
      minimum_alignment_tag_count < 2 || minimum_alignment_tag_count > 32 ||
      !std::isfinite(alignment_update_gain_) || alignment_update_gain_ <= 0.0 ||
      alignment_update_gain_ > 1.0 ||
      !std::isfinite(alignment_max_translation_step_m_) ||
      alignment_max_translation_step_m_ <= 0.0 ||
      !std::isfinite(alignment_max_rotation_step_rad_) ||
      alignment_max_rotation_step_rad_ <= 0.0 ||
      alignment_max_rotation_step_rad_ > M_PI ||
      initial_velocity_stddev_ <= 0.0 || initial_angular_velocity_stddev_ <= 0.0 ||
      noise_.linear_acceleration <= 0.0 || noise_.angular_acceleration <= 0.0)
    {
      throw std::runtime_error("VIO/Tag EKF timing, gate, or noise parameters are invalid");
    }
    alignment_candidate_count_ = static_cast<std::size_t>(alignment_candidate_count);
    minimum_alignment_tag_count_ = static_cast<std::size_t>(minimum_alignment_tag_count);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto tag_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("vio_topic", "/zedx/zed_node/odom"), sensor_qos,
      std::bind(&VioTagFusionComponent::on_vio, this, std::placeholders::_1));
    tag_sub_ = create_subscription<robotcore_interfaces::msg::AprilTagPoseEstimate>(
      declare_parameter<std::string>("tag_topic", "/localization/apriltag_pose"), tag_qos,
      std::bind(&VioTagFusionComponent::on_tag, this, std::placeholders::_1));
    zed_status_sub_ = create_subscription<zed_msgs::msg::PosTrackStatus>(
      declare_parameter<std::string>(
        "zed_tracking_status_topic", "/zedx/zed_node/pose/status"), sensor_qos,
      std::bind(&VioTagFusionComponent::on_zed_status, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/sensors/external_imu"),
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&VioTagFusionComponent::on_imu, this, std::placeholders::_1));

    body_pub_ = create_publisher<robotcore_interfaces::msg::BodyState>(
      "/robot/body_state", rclcpp::QoS(1).reliable());
    status_pub_ = create_publisher<robotcore_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(1).reliable());

    const double output_hz = declare_parameter<double>("output_rate_hz", 60.0);
    output_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, output_hz)),
      std::bind(&VioTagFusionComponent::publish, this));
    updater_.setHardwareID("vio-tag-ekf");
    updater_.add("Localization estimator", this, &VioTagFusionComponent::diagnose);
  }

private:
  void configure_executor_thread_once() const
  {
    thread_local const VioTagFusionComponent * configured_component = nullptr;
    if (configured_component == this) {return;}
    configure_fifo_thread(
      get_logger(), "VIO/Tag EKF executor", executor_realtime_priority_);
    configured_component = this;
  }

  struct Measurement
  {
    std::uint64_t id{};
    std::int64_t stamp_ns{};
    bool has_pose{false};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Matrix<double, 6, 6> pose_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
    bool has_twist{false};
    Eigen::Matrix<double, 6, 1> body_twist{
      Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 6> twist_covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
    bool has_external_angular_velocity{false};
    Eigen::Vector3d external_angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d external_angular_velocity_covariance{
      Eigen::Matrix3d::Identity()};
    bool has_external_linear_acceleration{false};
    Eigen::Vector3d external_specific_force{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d external_specific_force_covariance{
      Eigen::Matrix3d::Identity()};
  };

  struct MeasurementResult
  {
    bool pose_accepted{false};
    bool twist_accepted{false};
    bool angular_velocity_accepted{false};
    bool linear_acceleration_accepted{false};
    bool linear_acceleration_limited{false};
    bool linear_velocity_accepted{false};
    bool linear_velocity_correction_limited{false};
    double linear_velocity_innovation_mps{NAN};
    double linear_velocity_correction_mps{NAN};
    bool any() const
    {
      return pose_accepted || twist_accepted || linear_acceleration_accepted;
    }
  };

  struct AlignmentCandidate
  {
    std::int64_t stamp_ns{};
    Eigen::Isometry3d map_from_odom{Eigen::Isometry3d::Identity()};
    Eigen::Matrix<double, 6, 6> covariance{
      Eigen::Matrix<double, 6, 6>::Identity()};
  };

  static std::int64_t steady_now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool current_epoch(std::int64_t measurement_ns, std::int64_t arrival_ns) const
  {
    constexpr std::int64_t maximum_age_ns = 2000000000LL;
    return measurement_ns > 0 &&
           measurement_ns >= arrival_ns - maximum_age_ns &&
           measurement_ns <= arrival_ns + maximum_age_ns;
  }

  Eigen::Matrix3d regularized_linear_velocity_covariance(
    const Eigen::Matrix3d & covariance) const
  {
    const Eigen::Matrix3d symmetric = symmetrized_covariance<3>(covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
    if (solver.info() != Eigen::Success) {return symmetric;}
    const double variance_floor = linear_velocity_stddev_floor_mps_ *
      linear_velocity_stddev_floor_mps_;
    const Eigen::Vector3d eigenvalues =
      solver.eigenvalues().cwiseMax(variance_floor);
    const Eigen::Matrix3d regularized =
      (solver.eigenvectors() * eigenvalues.asDiagonal() *
      solver.eigenvectors().transpose()).eval();
    return symmetrized_covariance<3>(regularized);
  }

  bool reset_on_clock_jump(std::int64_t ros_now_ns)
  {
    if (!clock_jump_detector_.update(ros_now_ns, steady_now_ns())) {return false;}
    clear_filter_and_alignment();
    ++clock_discontinuities_;
    RCLCPP_WARN(
      get_logger(), "ROS clock discontinuity detected; VIO/Tag EKF history reset");
    return true;
  }

  void clear_filter_and_alignment()
  {
    filter_ = VioTagEkf(noise_);
    anchor_state_.reset();
    measurements_.clear();
    measurement_filters_.clear();
    map_from_odom_.reset();
    alignment_covariance_.setZero();
    alignment_candidates_.clear();
    alignment_candidate_cluster_size_ = 0U;
    last_vio_measurement_stamp_ns_ = 0;
    last_vio_accepted_stamp_ns_ = 0;
    last_vio_accepted_arrival_ns_ = 0;
    last_tag_candidate_stamp_ns_ = 0;
    last_tag_stamp_ns_ = 0;
    last_tag_arrival_ns_ = 0;
    last_absolute_stamp_ns_ = 0;
    last_tag_translation_residual_ = NAN;
    last_tag_angle_residual_deg_ = NAN;
    last_zed_status_arrival_ns_ = 0;
    have_zed_status_ = false;
    last_vio_linear_velocity_innovation_mps_ = NAN;
    last_vio_linear_velocity_correction_mps_ = NAN;
    vio_measurement_stamps_.clear();
    imu_measurement_stamps_.clear();
    tag_measurement_stamps_.clear();
    vio_arrivals_.clear();
    tag_arrivals_.clear();
    tag_frame_arrivals_.clear();
    body_state_arrivals_.clear();
    last_imu_measurement_stamp_ns_ = 0;
  }

  void reset_for_new_tag_map()
  {
    map_from_odom_.reset();
    alignment_covariance_.setZero();
    alignment_candidates_.clear();
    alignment_candidate_cluster_size_ = 0U;
    last_tag_candidate_stamp_ns_ = 0;
    last_tag_stamp_ns_ = 0;
    last_tag_arrival_ns_ = 0;
    last_absolute_stamp_ns_ = 0;
    last_tag_translation_residual_ = NAN;
    last_tag_angle_residual_deg_ = NAN;
    tag_measurement_stamps_.clear();
    tag_arrivals_.clear();
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

  void on_zed_status(const zed_msgs::msg::PosTrackStatus::SharedPtr message)
  {
    configure_executor_thread_once();
    std::lock_guard<std::mutex> lock(mutex_);
    last_zed_status_arrival_ns_ = now().nanoseconds();
    last_zed_odometry_status_ = message->odometry_status;
    have_zed_status_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    configure_executor_thread_once();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto arrival_ns = now().nanoseconds();
    reset_on_clock_jump(arrival_ns);
    const auto measurement_ns = stamp_ns(message->header.stamp);
    if (!current_epoch(measurement_ns, arrival_ns)) {
      ++invalid_imu_drops_;
      return;
    }
    if (measurement_ns <= last_imu_measurement_stamp_ns_) {
      ++old_imu_drops_;
      return;
    }
    if (message->header.frame_id != base_frame_) {
      ++invalid_imu_drops_;
      return;
    }

    const Eigen::Vector3d angular_velocity(
      message->angular_velocity.x,
      message->angular_velocity.y,
      message->angular_velocity.z);
    Eigen::Matrix3d covariance;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        covariance(row, column) =
          message->angular_velocity_covariance[3 * row + column];
      }
    }
    covariance = symmetrized_covariance<3>(covariance);
    if (!angular_velocity.allFinite() || !valid_covariance<3>(covariance)) {
      ++invalid_imu_drops_;
      return;
    }
    last_imu_measurement_stamp_ns_ = measurement_ns;
    if (!filter_.initialized() || !anchor_state_) {return;}
    if (measurement_ns <= anchor_state_->stamp_ns) {
      ++imu_outside_history_drops_;
      return;
    }

    Measurement event;
    event.id = ++measurement_sequence_;
    event.stamp_ns = measurement_ns;
    event.has_external_angular_velocity = true;
    event.external_angular_velocity = angular_velocity;
    event.external_angular_velocity_covariance = covariance;
    if (use_imu_linear_acceleration_) {
      event.external_specific_force = Eigen::Vector3d(
        message->linear_acceleration.x,
        message->linear_acceleration.y,
        message->linear_acceleration.z);
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          event.external_specific_force_covariance(row, column) =
            message->linear_acceleration_covariance[3 * row + column];
        }
      }
      event.external_specific_force_covariance =
        symmetrized_covariance<3>(event.external_specific_force_covariance);
      const double variance_floor =
        imu_linear_acceleration_stddev_floor_mps2_ *
        imu_linear_acceleration_stddev_floor_mps2_;
      for (int axis = 0; axis < 3; ++axis) {
        event.external_specific_force_covariance(axis, axis) = std::max(
          event.external_specific_force_covariance(axis, axis), variance_floor);
      }
      event.has_external_linear_acceleration =
        event.external_specific_force.allFinite() &&
        valid_covariance<3>(event.external_specific_force_covariance);
      if (!event.has_external_linear_acceleration) {
        ++invalid_imu_linear_acceleration_drops_;
      }
    }
    const bool delayed = measurement_ns < filter_.state().stamp_ns;
    const std::size_t insertion_index = insert_measurement(event);
    const MeasurementResult result = replay_from(insertion_index, event.id);
    if (delayed) {
      ++delayed_measurement_replays_;
    }
    if (!result.angular_velocity_accepted) {
      ++imu_angular_velocity_gate_rejections_;
    }
    if (!result.angular_velocity_accepted &&
      !result.linear_acceleration_accepted)
    {
      const auto erased_index = erase_measurement(event.id);
      if (erased_index) {replay_from(*erased_index, 0U);}
      return;
    }
    if (result.linear_acceleration_limited) {
      ++imu_linear_acceleration_limits_;
    }

    ++external_imu_updates_;
    imu_measurement_stamps_.push_back(measurement_ns);
    prune_rate_window(imu_measurement_stamps_, measurement_ns);
    prune_history();
  }

  bool zed_tracking_allows_measurement(std::int64_t arrival_ns) const
  {
    if (!require_zed_tracking_ok_) {return true;}
    return have_zed_status_ &&
           last_zed_odometry_status_ == zed_msgs::msg::PosTrackStatus::OK &&
           arrival_ns >= last_zed_status_arrival_ns_ &&
           (arrival_ns - last_zed_status_arrival_ns_) * 1e-9 <= zed_status_timeout_s_;
  }

  bool initialize_filter(const Measurement & vio)
  {
    EkfState state;
    state.stamp_ns = vio.stamp_ns;
    state.position = vio.position;
    state.orientation = vio.orientation;
    if (vio.has_twist) {
      state.velocity = state.orientation * vio.body_twist.head<3>();
      state.angular_velocity = vio.body_twist.tail<3>();
    }
    state.covariance.setZero();

    state.covariance.block<3, 3>(0, 0) = vio.pose_covariance.block<3, 3>(0, 0);
    state.covariance.block<3, 3>(6, 6) = vio.pose_covariance.block<3, 3>(3, 3);
    if (vio.has_twist) {
      const Eigen::Matrix3d rotation = state.orientation.toRotationMatrix();
      state.covariance.block<3, 3>(3, 3) = rotation *
        regularized_linear_velocity_covariance(
        vio.twist_covariance.block<3, 3>(0, 0)) *
        rotation.transpose();
      state.covariance.block<3, 3>(9, 9) =
        vio.twist_covariance.block<3, 3>(3, 3);
    } else {
      state.covariance.block<3, 3>(3, 3).diagonal().setConstant(
        initial_velocity_stddev_ * initial_velocity_stddev_);
      state.covariance.block<3, 3>(9, 9).diagonal().setConstant(
        initial_angular_velocity_stddev_ * initial_angular_velocity_stddev_);
    }
    filter_ = VioTagEkf(noise_);
    if (!filter_.initialize(state)) {
      anchor_state_.reset();
      measurements_.clear();
      measurement_filters_.clear();
      ++covariance_validation_failures_;
      return false;
    }
    anchor_state_ = filter_.state();
    measurements_.clear();
    measurement_filters_.clear();
    return true;
  }

  void on_vio(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    configure_executor_thread_once();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto arrival_ns = now().nanoseconds();
    reset_on_clock_jump(arrival_ns);
    const auto measurement_ns = stamp_ns(message->header.stamp);
    if (!current_epoch(measurement_ns, arrival_ns)) {
      ++timestamp_epoch_rejections_;
      return;
    }
    if (measurement_ns <= last_vio_measurement_stamp_ns_) {
      ++old_vio_drops_;
      return;
    }
    last_vio_measurement_stamp_ns_ = measurement_ns;
    if (!zed_tracking_allows_measurement(arrival_ns)) {
      ++zed_status_rejections_;
      return;
    }
    if (message->child_frame_id.empty() ||
      !update_cached_extrinsic(message->child_frame_id))
    {
      return;
    }

    Eigen::Isometry3d odom_from_source;
    Eigen::Isometry3d odom_from_base;
    try {
      odom_from_source = pose(message->pose.pose);
      odom_from_base = odom_from_source * base_from_vio_source_->inverse();
    } catch (...) {
      ++invalid_vio_drops_;
      return;
    }
    Eigen::Matrix<double, 6, 1> source_twist;
    source_twist <<
      message->twist.twist.linear.x,
      message->twist.twist.linear.y,
      message->twist.twist.linear.z,
      message->twist.twist.angular.x,
      message->twist.twist.angular.y,
      message->twist.twist.angular.z;
    const Eigen::Matrix<double, 6, 6> twist_transform =
      adjoint(*base_from_vio_source_);
    const Eigen::Matrix<double, 6, 1> body_twist =
      twist_transform * source_twist;
    if (!body_twist.allFinite()) {
      ++invalid_vio_drops_;
      return;
    }

    Eigen::Matrix<double, 6, 6> source_pose_covariance =
      covariance6(message->pose.covariance);
    if (!normalize_measurement_covariance<6>(source_pose_covariance)) {
      ++invalid_vio_pose_covariance_drops_;
      return;
    }
    const Eigen::Matrix<double, 6, 6> pose_covariance_fixed =
      pose_covariance_at_base(
      source_pose_covariance, odom_from_source, base_from_vio_source_->inverse());
    if (!valid_covariance<6>(pose_covariance_fixed)) {
      ++invalid_vio_pose_covariance_drops_;
      return;
    }
    Eigen::Matrix<double, 6, 6> source_twist_covariance =
      covariance6(message->twist.covariance);
    Eigen::Matrix<double, 6, 6> body_twist_covariance =
      Eigen::Matrix<double, 6, 6>::Zero();
    bool body_twist_covariance_valid =
      normalize_measurement_covariance<6>(source_twist_covariance);
    if (body_twist_covariance_valid) {
      const Eigen::Matrix<double, 6, 6> transformed_twist_covariance =
        (twist_transform * source_twist_covariance *
        twist_transform.transpose()).eval();
      body_twist_covariance = symmetrized_covariance<6>(transformed_twist_covariance);
      constexpr double angular_velocity_stddev_rps = 0.10;
      body_twist_covariance.block<3, 3>(3, 3).diagonal().array() +=
        angular_velocity_stddev_rps * angular_velocity_stddev_rps;
      body_twist_covariance_valid = valid_covariance<6>(body_twist_covariance);
    }

    Measurement event;
    event.id = ++measurement_sequence_;
    event.stamp_ns = measurement_ns;
    event.has_pose = true;
    event.position = odom_from_base.translation();
    event.orientation = Eigen::Quaterniond(odom_from_base.linear()).normalized();
    event.pose_covariance = pose_covariance_in_filter_coordinates(
      pose_covariance_fixed, event.orientation);
    event.has_twist = body_twist_covariance_valid;
    event.body_twist = body_twist;
    if (event.has_twist) {
      event.twist_covariance = body_twist_covariance;
      twist_covariance_source_ = "ZED SDK";
    } else {
      ++invalid_vio_twist_covariance_drops_;
      twist_covariance_source_ = "unavailable; twist update skipped";
    }
    if (!filter_.initialized()) {
      if (!initialize_filter(event)) {return;}
      record_accepted_vio(measurement_ns, arrival_ns);
      publish_vio_correction_locked();
      return;
    }
    if (!anchor_state_ || measurement_ns <= anchor_state_->stamp_ns) {
      ++measurement_outside_history_drops_;
      return;
    }
    if (measurement_ns - filter_.state().stamp_ns > 1000000000LL) {
      if (!initialize_filter(event)) {return;}
      record_accepted_vio(measurement_ns, arrival_ns);
      publish_vio_correction_locked();
      return;
    }

    const bool delayed = measurement_ns < filter_.state().stamp_ns;
    const std::size_t insertion_index = insert_measurement(event);
    const auto result = replay_from(insertion_index, event.id);
    if (delayed) {++delayed_measurement_replays_;}
    if (event.has_twist) {
      last_vio_linear_velocity_innovation_mps_ =
        result.linear_velocity_innovation_mps;
      last_vio_linear_velocity_correction_mps_ =
        result.linear_velocity_correction_mps;
      if (result.linear_velocity_correction_limited) {
        ++vio_linear_velocity_correction_limits_;
      }
      if (!result.linear_velocity_accepted) {
        ++vio_linear_velocity_rejections_;
      }
    }
    if (!result.any()) {
      const auto erased_index = erase_measurement(event.id);
      if (erased_index) {replay_from(*erased_index, 0U);}
      ++vio_pose_gate_rejections_;
      if (event.has_twist) {++vio_twist_gate_rejections_;}
      return;
    }
    if (!result.pose_accepted) {++vio_pose_gate_rejections_;}
    if (event.has_twist && !result.twist_accepted) {++vio_twist_gate_rejections_;}
    record_accepted_vio(measurement_ns, arrival_ns);
    prune_history();
    publish_vio_correction_locked();
  }

  void record_accepted_vio(std::int64_t measurement_ns, std::int64_t arrival_ns)
  {
    last_vio_accepted_stamp_ns_ = measurement_ns;
    last_vio_accepted_arrival_ns_ = arrival_ns;
    vio_transport_s_ = (arrival_ns - measurement_ns) * 1e-9;
    vio_measurement_stamps_.push_back(measurement_ns);
    prune_rate_window(vio_measurement_stamps_, measurement_ns);
    vio_arrivals_.push_back(arrival_ns);
    prune_rate_window(vio_arrivals_, arrival_ns);
  }

  MeasurementResult apply_measurement(VioTagEkf & filter, const Measurement & event) const
  {
    MeasurementResult result;
    if (event.has_pose) {
      const bool position_accepted = filter.update_position(
        event.position, event.pose_covariance.block<3, 3>(0, 0), pose_gate_chi2_);
      const bool orientation_accepted = filter.update_orientation(
        event.orientation, event.pose_covariance.block<3, 3>(3, 3), pose_gate_chi2_);
      result.pose_accepted = position_accepted || orientation_accepted;
    }
    if (event.has_twist) {
      const Eigen::Matrix3d linear_velocity_covariance =
        regularized_linear_velocity_covariance(
        event.twist_covariance.block<3, 3>(0, 0));
      const bool linear_velocity_accepted = filter.update_linear_velocity(
        event.body_twist.head<3>(), linear_velocity_covariance,
        twist_gate_chi2_, linear_velocity_correction_limit_mps_,
        linear_velocity_innovation_limit_mps_);
      result.linear_velocity_accepted = linear_velocity_accepted;
      result.linear_velocity_innovation_mps =
        filter.last_linear_velocity_innovation_mps();
      result.linear_velocity_correction_mps =
        filter.last_linear_velocity_correction_mps();
      result.linear_velocity_correction_limited =
        filter.last_linear_velocity_correction_limited();
      const bool angular_velocity_accepted = filter.update_angular_velocity(
        event.body_twist.tail<3>(), event.twist_covariance.block<3, 3>(3, 3),
        twist_gate_chi2_);
      result.angular_velocity_accepted = angular_velocity_accepted;
      result.twist_accepted = linear_velocity_accepted || angular_velocity_accepted;
    }
    if (event.has_external_angular_velocity) {
      result.angular_velocity_accepted = filter.update_angular_velocity(
        event.external_angular_velocity,
        event.external_angular_velocity_covariance,
        imu_angular_velocity_gate_chi2_);
      result.twist_accepted =
        result.twist_accepted || result.angular_velocity_accepted;
    }
    if (event.has_external_linear_acceleration) {
      const Eigen::Matrix3d odom_from_body =
        filter.state().orientation.toRotationMatrix();
      Eigen::Vector3d linear_acceleration =
        odom_from_body * event.external_specific_force -
        Eigen::Vector3d(0.0, 0.0, gravity_mps2_);
      const double acceleration_norm = linear_acceleration.norm();
      if (acceleration_norm > maximum_imu_linear_acceleration_mps2_) {
        linear_acceleration *=
          maximum_imu_linear_acceleration_mps2_ / acceleration_norm;
        result.linear_acceleration_limited = true;
      }
      const Eigen::Matrix3d covariance = symmetrized_covariance<3>(
        odom_from_body * event.external_specific_force_covariance *
        odom_from_body.transpose());
      result.linear_acceleration_accepted =
        filter.set_linear_acceleration(linear_acceleration, covariance);
    }
    return result;
  }

  std::size_t insert_measurement(const Measurement & event)
  {
    const auto insertion = std::upper_bound(
      measurements_.begin(), measurements_.end(), event,
      [](const Measurement & left, const Measurement & right) {
        return std::tie(left.stamp_ns, left.id) < std::tie(right.stamp_ns, right.id);
      });
    const auto index = static_cast<std::size_t>(
      std::distance(measurements_.begin(), insertion));
    measurements_.insert(insertion, event);
    measurement_filters_.insert(
      measurement_filters_.begin() + static_cast<std::ptrdiff_t>(index),
      VioTagEkf(noise_));
    history_high_water_ = std::max(history_high_water_, measurements_.size());
    return index;
  }

  std::optional<std::size_t> erase_measurement(std::uint64_t id)
  {
    const auto event = std::find_if(
      measurements_.begin(), measurements_.end(),
      [id](const Measurement & candidate) {return candidate.id == id;});
    if (event == measurements_.end()) {return std::nullopt;}
    const auto index = static_cast<std::size_t>(
      std::distance(measurements_.begin(), event));
    measurements_.erase(event);
    measurement_filters_.erase(
      measurement_filters_.begin() + static_cast<std::ptrdiff_t>(index));
    return index;
  }

  MeasurementResult replay_from(std::size_t start_index, std::uint64_t tracked_id)
  {
    MeasurementResult tracked;
    if (!anchor_state_ || start_index > measurements_.size() ||
      measurement_filters_.size() != measurements_.size())
    {
      ++checkpoint_consistency_failures_;
      return tracked;
    }

    VioTagEkf replay_filter(noise_);
    if (start_index == 0U && !replay_filter.initialize(*anchor_state_)) {
      ++covariance_validation_failures_;
      return tracked;
    }
    if (start_index > 0U) {
      replay_filter = measurement_filters_[start_index - 1U];
    }

    std::vector<VioTagEkf> rebuilt_filters;
    rebuilt_filters.reserve(measurements_.size() - start_index);
    for (std::size_t index = start_index; index < measurements_.size(); ++index) {
      const auto & event = measurements_[index];
      if (!replay_filter.propagate_to(event.stamp_ns)) {break;}
      const auto result = apply_measurement(replay_filter, event);
      if (event.id == tracked_id) {tracked = result;}
      if (!replay_filter.initialized()) {break;}
      rebuilt_filters.push_back(replay_filter);
    }

    if (rebuilt_filters.size() != measurements_.size() - start_index) {
      ++checkpoint_consistency_failures_;
      return tracked;
    }
    for (std::size_t offset = 0; offset < rebuilt_filters.size(); ++offset) {
      measurement_filters_[start_index + offset] = std::move(rebuilt_filters[offset]);
    }
    filter_ = replay_filter;
    replayed_measurement_events_ += rebuilt_filters.size();
    max_incremental_replay_events_ = std::max(
      max_incremental_replay_events_, rebuilt_filters.size());
    return tracked;
  }

  std::optional<EkfState> state_at(std::int64_t target_ns) const
  {
    if (!anchor_state_ || target_ns < anchor_state_->stamp_ns ||
      target_ns > filter_.state().stamp_ns)
    {
      return std::nullopt;
    }
    const auto after_target = std::upper_bound(
      measurements_.begin(), measurements_.end(), target_ns,
      [](std::int64_t stamp, const Measurement & event) {
        return stamp < event.stamp_ns;
      });
    const auto event_count = static_cast<std::size_t>(
      std::distance(measurements_.begin(), after_target));
    if (measurement_filters_.size() != measurements_.size()) {return std::nullopt;}
    VioTagEkf temporary(noise_);
    if (event_count == 0U && !temporary.initialize(*anchor_state_)) {
      return std::nullopt;
    }
    if (event_count > 0U) {temporary = measurement_filters_[event_count - 1U];}
    if (!temporary.propagate_to(target_ns)) {return std::nullopt;}
    return temporary.state();
  }

  void prune_history()
  {
    if (!anchor_state_ || measurements_.empty()) {return;}
    const auto cutoff_ns = filter_.state().stamp_ns -
      static_cast<std::int64_t>(history_duration_s_ * 1e9);
    if (cutoff_ns <= anchor_state_->stamp_ns) {return;}
    while (!measurements_.empty() && measurements_.front().stamp_ns <= cutoff_ns) {
      if (measurement_filters_.empty()) {
        ++checkpoint_consistency_failures_;
        return;
      }
      anchor_state_ = measurement_filters_.front().state();
      measurements_.pop_front();
      measurement_filters_.pop_front();
    }
    if (anchor_state_->stamp_ns < cutoff_ns) {
      VioTagEkf anchor_filter(noise_);
      if (!anchor_filter.initialize(*anchor_state_) ||
        !anchor_filter.propagate_to(cutoff_ns))
      {
        ++covariance_validation_failures_;
        return;
      }
      anchor_state_ = anchor_filter.state();
    }
    history_high_water_ = std::max(history_high_water_, measurements_.size());
  }

  Eigen::Matrix<double, 6, 6> state_pose_covariance_fixed(
    const EkfState & state) const
  {
    Eigen::Matrix<double, 6, 12> jacobian =
      Eigen::Matrix<double, 6, 12>::Zero();
    jacobian.block<3, 3>(0, 0).setIdentity();
    jacobian.block<3, 3>(3, 6) = state.orientation.toRotationMatrix();
    const Eigen::Matrix<double, 6, 6> result =
      jacobian * state.covariance * jacobian.transpose();
    return symmetrized_covariance<6>(result);
  }

  void on_tag(const robotcore_interfaces::msg::AprilTagPoseEstimate::SharedPtr message)
  {
    configure_executor_thread_once();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto arrival_ns = now().nanoseconds();
    reset_on_clock_jump(arrival_ns);
    if (message->map_generation < last_tag_map_generation_) {return;}
    const bool map_changed = last_tag_map_generation_ != 0U &&
      message->map_generation > last_tag_map_generation_;
    if (message->relocalization_requested || map_changed) {reset_for_new_tag_map();}
    last_tag_map_generation_ = message->map_generation;
    last_tag_estimate_ = *message;
    last_tag_frame_arrival_ns_ = arrival_ns;
    tag_frame_arrivals_.push_back(arrival_ns);
    prune_rate_window(tag_frame_arrivals_, arrival_ns);
    if (!message->pose_valid || message->header.frame_id != map_frame_ ||
      !filter_.initialized())
    {
      return;
    }
    if (message->inlier_tag_count < 0 ||
      static_cast<std::size_t>(message->inlier_tag_count) < minimum_alignment_tag_count_)
    {
      ++insufficient_alignment_tag_drops_;
      return;
    }

    const auto tag_stamp_ns = stamp_ns(message->header.stamp);
    if (!current_epoch(tag_stamp_ns, arrival_ns)) {
      ++timestamp_epoch_rejections_;
      return;
    }
    if (tag_stamp_ns <= last_tag_candidate_stamp_ns_) {
      ++old_tag_drops_;
      return;
    }
    last_tag_candidate_stamp_ns_ = tag_stamp_ns;
    const auto local_state = state_at(tag_stamp_ns);
    if (!local_state) {
      ++measurement_outside_history_drops_;
      return;
    }

    Eigen::Isometry3d map_from_base;
    try {
      map_from_base = pose(message->pose.pose);
    } catch (...) {
      ++invalid_tag_drops_;
      return;
    }
    Eigen::Matrix<double, 6, 6> tag_covariance_map =
      covariance6(message->pose.covariance);
    if (!normalize_measurement_covariance<6>(tag_covariance_map)) {
      ++invalid_tag_covariance_drops_;
      return;
    }
    const Eigen::Isometry3d odom_from_base = pose_transform(
      local_state->position, local_state->orientation);
    const Eigen::Isometry3d candidate = map_from_base * odom_from_base.inverse();
    const Eigen::Matrix<double, 6, 6> state_covariance_odom =
      state_pose_covariance_fixed(*local_state);
    if (!valid_covariance<6>(state_covariance_odom)) {
      ++covariance_validation_failures_;
      return;
    }
    const Eigen::Matrix<double, 6, 6> candidate_covariance =
      alignment_candidate_covariance(
      tag_covariance_map, state_covariance_odom, odom_from_base, candidate);
    if (!valid_covariance<6>(candidate_covariance)) {
      ++covariance_validation_failures_;
      return;
    }

    if (!map_from_odom_) {
      if (!add_alignment_candidate(candidate, candidate_covariance, tag_stamp_ns)) {return;}
      if (!establish_alignment()) {return;}
      const Eigen::Isometry3d aligned_map_from_base = *map_from_odom_ * odom_from_base;
      last_tag_translation_residual_ =
        (map_from_base.translation() - aligned_map_from_base.translation()).norm();
      last_tag_angle_residual_deg_ = rotation_distance(
        map_from_base.linear(), aligned_map_from_base.linear()) * 180.0 / M_PI;
      record_accepted_tag(tag_stamp_ns, arrival_ns);
      return;
    }

    const Eigen::Isometry3d predicted_map_from_base =
      *map_from_odom_ * odom_from_base;
    last_tag_translation_residual_ =
      (map_from_base.translation() - predicted_map_from_base.translation()).norm();
    last_tag_angle_residual_deg_ = rotation_distance(
      map_from_base.linear(), predicted_map_from_base.linear()) * 180.0 / M_PI;
    if (!update_alignment(candidate, candidate_covariance)) {
      ++tag_gate_rejections_;
      return;
    }
    record_accepted_tag(tag_stamp_ns, arrival_ns);
  }

  void record_accepted_tag(std::int64_t measurement_ns, std::int64_t arrival_ns)
  {
    last_tag_stamp_ns_ = measurement_ns;
    last_tag_arrival_ns_ = arrival_ns;
    last_absolute_stamp_ns_ = measurement_ns;
    tag_transport_s_ = (arrival_ns - measurement_ns) * 1e-9;
    tag_measurement_stamps_.push_back(measurement_ns);
    prune_rate_window(tag_measurement_stamps_, measurement_ns);
    tag_arrivals_.push_back(arrival_ns);
    prune_rate_window(tag_arrivals_, arrival_ns);
  }

  bool add_alignment_candidate(
    const Eigen::Isometry3d & transform,
    const Eigen::Matrix<double, 6, 6> & covariance,
    std::int64_t stamp)
  {
    const auto window_ns = static_cast<std::int64_t>(
      std::llround(alignment_candidate_window_s_ * 1e9));
    const auto cutoff_ns = stamp - window_ns;
    while (!alignment_candidates_.empty() &&
      alignment_candidates_.front().stamp_ns < cutoff_ns)
    {
      alignment_candidates_.pop_front();
    }
    alignment_candidates_.push_back({stamp, transform, covariance});
    constexpr std::size_t maximum_buffer_size = 256U;
    while (alignment_candidates_.size() > maximum_buffer_size) {
      alignment_candidates_.pop_front();
    }

    std::vector<Eigen::Isometry3d> transforms;
    transforms.reserve(alignment_candidates_.size());
    for (const auto & candidate : alignment_candidates_) {
      transforms.push_back(candidate.map_from_odom);
    }
    const auto selected_indices = tightest_consistent_pose_cluster(
      transforms, alignment_candidate_count_, alignment_translation_tolerance_m_,
      alignment_rotation_tolerance_rad_);
    alignment_candidate_cluster_size_ = selected_indices.size();
    if (selected_indices.size() < alignment_candidate_count_) {return false;}

    std::deque<AlignmentCandidate> selected;
    for (const auto index : selected_indices) {
      selected.push_back(alignment_candidates_[index]);
    }
    alignment_candidates_ = std::move(selected);
    return true;
  }

  bool establish_alignment()
  {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation(alignment_candidates_.front().map_from_odom.linear());
    for (std::size_t index = 0; index < alignment_candidates_.size(); ++index) {
      translation += alignment_candidates_[index].map_from_odom.translation();
      if (index > 0U) {
        orientation = orientation.slerp(
          1.0 / static_cast<double>(index + 1U),
          Eigen::Quaterniond(alignment_candidates_[index].map_from_odom.linear())).normalized();
      }
    }
    const Eigen::Isometry3d map_from_odom = pose_transform(
      translation / static_cast<double>(alignment_candidates_.size()), orientation);
    Eigen::Matrix<double, 6, 6> alignment_covariance =
      Eigen::Matrix<double, 6, 6>::Zero();
    const double count = static_cast<double>(alignment_candidates_.size());
    for (const auto & candidate : alignment_candidates_) {
      // Consecutive candidates share the VIO trajectory and Tag map.  Dividing
      // by count squared would assume independence and become overconfident.
      // The average reported covariance is a conservative upper bound for the
      // mean when those cross-correlations are unknown.
      alignment_covariance += candidate.covariance / count;
      Eigen::Matrix<double, 6, 1> residual;
      residual.head<3>() = candidate.map_from_odom.translation() -
        map_from_odom.translation();
      // Candidate covariance is expressed with fixed axes in map, so the
      // orientation residual must use the matching left perturbation.
      residual.tail<3>() = log_quaternion(
        Eigen::Quaterniond(
          candidate.map_from_odom.linear() * map_from_odom.linear().transpose()));
      alignment_covariance += residual * residual.transpose() / (count - 1.0);
    }
    alignment_covariance = symmetrized_covariance<6>(alignment_covariance);
    const auto candidate_count = alignment_candidates_.size();
    alignment_candidates_.clear();
    alignment_candidate_cluster_size_ = 0U;
    if (!valid_covariance<6>(alignment_covariance)) {
      ++covariance_validation_failures_;
      return false;
    }
    map_from_odom_ = map_from_odom;
    alignment_covariance_ = alignment_covariance;
    last_alignment_candidate_count_ = candidate_count;
    ++alignment_establishments_;
    RCLCPP_INFO(
      get_logger(), "AprilTag map alignment established from %zu consistent poses",
      candidate_count);
    return true;
  }

  bool update_alignment(
    const Eigen::Isometry3d & candidate,
    const Eigen::Matrix<double, 6, 6> & candidate_covariance)
  {
    if (!map_from_odom_) {return false;}

    Eigen::Matrix<double, 6, 6> prior_covariance = alignment_covariance_;
    Eigen::Matrix<double, 6, 6> observation_covariance = candidate_covariance;
    if (!stabilize_covariance<6>(prior_covariance) ||
      !stabilize_covariance<6>(observation_covariance))
    {
      ++covariance_validation_failures_;
      return false;
    }

    Eigen::Matrix<double, 6, 1> innovation;
    innovation.head<3>() =
      candidate.translation() - map_from_odom_->translation();
    innovation.tail<3>() = log_quaternion(
      Eigen::Quaterniond(candidate.linear() * map_from_odom_->linear().transpose()));
    const Eigen::Matrix<double, 6, 6> innovation_covariance =
      symmetrized_covariance<6>(prior_covariance + observation_covariance);
    if (!passes_three_dof_gate(
        innovation.head<3>(), innovation_covariance.block<3, 3>(0, 0),
        tag_gate_chi2_) ||
      !passes_three_dof_gate(
        innovation.tail<3>(), innovation_covariance.block<3, 3>(3, 3),
        tag_gate_chi2_))
    {
      return false;
    }

    const Eigen::Matrix<double, 6, 6> identity =
      Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> prior_decomposition(
      prior_covariance);
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> observation_decomposition(
      observation_covariance);
    if (prior_decomposition.info() != Eigen::Success ||
      !prior_decomposition.isPositive() ||
      observation_decomposition.info() != Eigen::Success ||
      !observation_decomposition.isPositive())
    {
      ++covariance_validation_failures_;
      return false;
    }

    // The new Tag candidate and the existing alignment share VIO history and
    // map data.  Equal-weight covariance intersection remains consistent
    // without pretending that their unknown cross-covariance is zero.
    constexpr double prior_weight = 0.5;
    const Eigen::Matrix<double, 6, 6> prior_information =
      prior_decomposition.solve(identity);
    const Eigen::Matrix<double, 6, 6> observation_information =
      observation_decomposition.solve(identity);
    const Eigen::Matrix<double, 6, 6> fused_information =
      symmetrized_covariance<6>(
      prior_weight * prior_information +
      (1.0 - prior_weight) * observation_information);
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> fused_decomposition(
      fused_information);
    if (fused_decomposition.info() != Eigen::Success ||
      !fused_decomposition.isPositive())
    {
      ++covariance_validation_failures_;
      return false;
    }
    Eigen::Matrix<double, 6, 6> fused_covariance =
      fused_decomposition.solve(identity);
    const Eigen::Matrix<double, 6, 1> covariance_intersection_correction =
      fused_covariance * (1.0 - prior_weight) *
      observation_information * innovation;
    if (!covariance_intersection_correction.allFinite() ||
      !stabilize_covariance<6>(fused_covariance))
    {
      ++covariance_validation_failures_;
      return false;
    }

    // Tag poses in the underwater image can jump between locally plausible
    // PnP solutions.  Use them only as a slow global-alignment correction;
    // the ZED VIO trajectory remains the continuous high-rate motion source.
    const Eigen::Vector3d translation_correction = scaled_vector_with_norm_limit(
      covariance_intersection_correction.head<3>(), alignment_update_gain_,
      alignment_max_translation_step_m_);
    const Eigen::Vector3d rotation_correction = scaled_vector_with_norm_limit(
      covariance_intersection_correction.tail<3>(), alignment_update_gain_,
      alignment_max_rotation_step_rad_);
    if (alignment_update_gain_ * covariance_intersection_correction.head<3>().norm() >
      alignment_max_translation_step_m_)
    {
      ++alignment_translation_step_limits_;
    }
    if (alignment_update_gain_ * covariance_intersection_correction.tail<3>().norm() >
      alignment_max_rotation_step_rad_)
    {
      ++alignment_rotation_step_limits_;
    }

    const Eigen::Quaterniond prior_orientation(map_from_odom_->linear());
    const Eigen::Quaterniond fused_orientation =
      (exp_quaternion(rotation_correction) * prior_orientation).normalized();
    map_from_odom_ = pose_transform(
      map_from_odom_->translation() + translation_correction, fused_orientation);
    alignment_covariance_ = fused_covariance;
    ++alignment_updates_;
    return true;
  }

  std::string localization_source(bool absolute) const
  {
    if (absolute) {return "AprilTag+ZED VIO EKF";}
    if (map_from_odom_) {return "ZED VIO EKF after Tag loss";}
    return "ZED VIO EKF (local odom)";
  }

  bool fill_covariances(
    nav_msgs::msg::Odometry & odometry, const EkfState & state) const
  {
    Eigen::Matrix3d odom_to_output_rotation = Eigen::Matrix3d::Identity();
    if (map_from_odom_) {odom_to_output_rotation = map_from_odom_->linear();}
    const Eigen::Matrix3d output_from_body =
      odom_to_output_rotation * state.orientation.toRotationMatrix();
    Eigen::Matrix<double, 6, 12> pose_jacobian =
      Eigen::Matrix<double, 6, 12>::Zero();
    pose_jacobian.block<3, 3>(0, 0) = odom_to_output_rotation;
    pose_jacobian.block<3, 3>(3, 6) = output_from_body;
    const Eigen::Matrix<double, 6, 6> raw_state_pose_covariance =
      (pose_jacobian * state.covariance * pose_jacobian.transpose()).eval();
    const Eigen::Matrix<double, 6, 6> state_pose_covariance =
      symmetrized_covariance<6>(raw_state_pose_covariance);
    Eigen::Matrix<double, 6, 6> pose_covariance = state_pose_covariance;
    if (map_from_odom_) {
      if (!valid_covariance<6>(alignment_covariance_)) {return false;}
      Eigen::Matrix<double, 6, 6> alignment_jacobian =
        Eigen::Matrix<double, 6, 6>::Identity();
      alignment_jacobian.block<3, 3>(0, 3) = -skew(
        map_from_odom_->linear() * state.position);
      const Eigen::Matrix<double, 6, 6> raw_alignment_contribution =
        (alignment_jacobian * alignment_covariance_ *
        alignment_jacobian.transpose()).eval();
      const Eigen::Matrix<double, 6, 6> alignment_contribution =
        symmetrized_covariance<6>(raw_alignment_contribution);
      // Alignment was estimated from the same VIO history as the current
      // state, but the replay buffer does not retain their cross-covariance.
      // Use a guaranteed upper bound instead of assuming independence.
      pose_covariance = covariance_sum_with_unknown_correlation<6>(
        state_pose_covariance, alignment_contribution);
    }

    const Eigen::Vector3d body_velocity =
      state.orientation.conjugate() * state.velocity;
    Eigen::Matrix<double, 6, 12> twist_jacobian =
      Eigen::Matrix<double, 6, 12>::Zero();
    twist_jacobian.block<3, 3>(0, 3) =
      state.orientation.conjugate().toRotationMatrix();
    twist_jacobian.block<3, 3>(0, 6) = skew(body_velocity);
    twist_jacobian.block<3, 3>(3, 9).setIdentity();
    const Eigen::Matrix<double, 6, 6> raw_twist_covariance =
      (twist_jacobian * state.covariance * twist_jacobian.transpose()).eval();
    const Eigen::Matrix<double, 6, 6> twist_covariance =
      symmetrized_covariance<6>(raw_twist_covariance);
    if (!valid_covariance<6>(pose_covariance) ||
      !valid_covariance<6>(twist_covariance))
    {
      return false;
    }
    store_covariance(odometry.pose.covariance, pose_covariance);
    store_covariance(odometry.twist.covariance, twist_covariance);
    return true;
  }

  void publish()
  {
    configure_executor_thread_once();
    std::lock_guard<std::mutex> lock(mutex_);
    publish_locked(now());
  }

  void publish_vio_correction_locked()
  {
    publish_locked(now());
    // The fixed-rate timer is a prediction heartbeat. Restart its period after
    // an event-driven correction so a timer edge cannot immediately duplicate
    // the fresh BodyState publication.
    if (output_timer_) {output_timer_->reset();}
  }

  void publish_locked(const rclcpp::Time & now_time)
  {
    const auto now_ns = now_time.nanoseconds();
    if (reset_on_clock_jump(now_ns) || !filter_.initialized()) {return;}

    VioTagEkf output_filter = filter_;
    const bool prediction_ok = output_filter.propagate_to(now_ns);
    if (!prediction_ok) {return;}
    const auto & state = output_filter.state();
    const double vio_measurement_age_s = last_vio_accepted_stamp_ns_ > 0 ?
      (now_ns - last_vio_accepted_stamp_ns_) * 1e-9 : INFINITY;
    const double vio_arrival_age_s = last_vio_accepted_arrival_ns_ > 0 ?
      (now_ns - last_vio_accepted_arrival_ns_) * 1e-9 : INFINITY;
    const double tag_age_s = last_tag_stamp_ns_ > 0 ?
      (now_ns - last_tag_stamp_ns_) * 1e-9 : INFINITY;
    const double tag_arrival_age_s = last_tag_arrival_ns_ > 0 ?
      (now_ns - last_tag_arrival_ns_) * 1e-9 : INFINITY;
    const bool vio_usable = vio_arrival_age_s <= vio_arrival_timeout_s_ &&
      vio_measurement_age_s <= vio_prediction_horizon_s_;
    const bool tag_usable = tag_age_s <= tag_fresh_s_ &&
      tag_arrival_age_s <= tag_fresh_s_;
    const bool absolute_valid = vio_usable && map_from_odom_ &&
      tag_usable;
    const bool estimated = vio_usable && !absolute_valid;

    const Eigen::Isometry3d odom_from_base = pose_transform(
      state.position, state.orientation);
    const Eigen::Isometry3d output_from_base = map_from_odom_ ?
      *map_from_odom_ * odom_from_base : odom_from_base;
    const Eigen::Vector3d body_velocity =
      state.orientation.conjugate() * state.velocity;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = rclcpp::Time(state.stamp_ns, RCL_ROS_TIME);
    odometry.header.frame_id = map_from_odom_ ? map_frame_ : odom_frame_;
    odometry.child_frame_id = base_frame_;
    set_pose(odometry.pose.pose, output_from_base);
    odometry.twist.twist.linear.x = body_velocity.x();
    odometry.twist.twist.linear.y = body_velocity.y();
    odometry.twist.twist.linear.z = body_velocity.z();
    odometry.twist.twist.angular.x = state.angular_velocity.x();
    odometry.twist.twist.angular.y = state.angular_velocity.y();
    odometry.twist.twist.angular.z = state.angular_velocity.z();
    if (!fill_covariances(odometry, state)) {
      ++invalid_output_covariance_drops_;
      RCLCPP_ERROR(
        get_logger(), "invalid localization covariance; resetting estimator and map alignment");
      clear_filter_and_alignment();
      return;
    }
    robotcore_interfaces::msg::BodyState body;
    body.header = odometry.header;
    body.pose = odometry.pose.pose;
    body.twist = odometry.twist.twist;
    body.linear_velocity_valid = vio_usable;
    body.state_valid = absolute_valid;
    body.position_estimated = estimated;
    body.localization_source = localization_source(absolute_valid);
    body_pub_->publish(body);
    body_state_arrivals_.push_back(now_ns);
    prune_rate_window(body_state_arrivals_, now_ns);
    publish_status(
      now_time, odometry, vio_measurement_age_s, tag_age_s,
      vio_usable, tag_usable, absolute_valid, estimated);
  }

  void publish_status(
    const rclcpp::Time & now_time,
    const nav_msgs::msg::Odometry & odometry,
    double vio_age_s,
    double tag_age_s,
    bool vio_usable,
    bool tag_usable,
    bool absolute_valid,
    bool estimated)
  {
    robotcore_interfaces::msg::LocalizationStatus status;
    status.header.stamp = now_time;
    status.header.frame_id = odometry.header.frame_id;
    if (last_vio_accepted_stamp_ns_ > 0) {
      status.last_vio_stamp = rclcpp::Time(last_vio_accepted_stamp_ns_, RCL_ROS_TIME);
    }
    if (last_tag_stamp_ns_ > 0) {
      status.last_tag_stamp = rclcpp::Time(last_tag_stamp_ns_, RCL_ROS_TIME);
    }
    if (last_absolute_stamp_ns_ > 0) {
      status.last_absolute_fix_stamp =
        rclcpp::Time(last_absolute_stamp_ns_, RCL_ROS_TIME);
    }
    status.vio_age_s = vio_age_s;
    status.tag_age_s = tag_age_s;
    status.absolute_fix_age_s = last_absolute_stamp_ns_ > 0 ?
      (now_time.nanoseconds() - last_absolute_stamp_ns_) * 1e-9 : INFINITY;
    status.apriltag_frame_age_s = last_tag_frame_arrival_ns_ > 0 ?
      (now_time.nanoseconds() - last_tag_frame_arrival_ns_) * 1e-9 : INFINITY;
    status.vio_rate_hz = window_rate_hz(
      vio_measurement_stamps_, now_time.nanoseconds());
    status.tag_rate_hz = window_rate_hz(
      tag_measurement_stamps_, now_time.nanoseconds());
    status.body_state_rate_hz =
      window_rate_hz(body_state_arrivals_, now_time.nanoseconds());
    status.apriltag_frame_rate_hz =
      window_rate_hz(tag_frame_arrivals_, now_time.nanoseconds());
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
    status.tag_fresh = tag_usable;
    status.tag_consistent = std::isfinite(last_tag_translation_residual_) &&
      std::isfinite(last_tag_angle_residual_deg_) &&
      last_tag_translation_residual_ <= 0.50 && last_tag_angle_residual_deg_ <= 10.0;
    status.absolute_fix_valid = absolute_valid;
    status.position_estimated = estimated;
    status.localization_source = localization_source(absolute_valid);
    status.tag_observation_class = last_tag_estimate_.pose_valid ? "primary" : "";
    status.apriltag_rejection_reason = last_tag_estimate_.rejection_reason;
    if (!vio_usable) {
      status.rejection_reason = "ZED VIO/EKF state is stale";
    } else if (!map_from_odom_) {
      status.rejection_reason = "waiting for Tag map alignment";
    } else if (!tag_usable) {
      status.rejection_reason = "AprilTag absolute fix is stale";
    } else {status.rejection_reason = "";}
    status_pub_->publish(status);
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    configure_executor_thread_once();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now_ns = now().nanoseconds();
    const bool tracking_ok = zed_tracking_allows_measurement(now_ns);
    const int level = filter_.initialized() && tracking_ok ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.summary(
      level,
      map_from_odom_ ? "AprilTag/ZED VIO EKF estimating in map" :
      "ZED VIO EKF waiting for Tag map alignment");
    status.add("filter_initialized", filter_.initialized());
    status.add("map_alignment_initialized", static_cast<bool>(map_from_odom_));
    status.add("alignment_candidate_buffer_size", alignment_candidates_.size());
    status.add("alignment_candidate_cluster_size", alignment_candidate_cluster_size_);
    status.add("alignment_candidate_required", alignment_candidate_count_);
    status.add("alignment_candidate_window_s", alignment_candidate_window_s_);
    status.add("last_alignment_candidate_count", last_alignment_candidate_count_);
    status.add("alignment_establishments", alignment_establishments_);
    status.add("alignment_updates", alignment_updates_);
    status.add("minimum_alignment_tag_count", minimum_alignment_tag_count_);
    status.add("insufficient_alignment_tag_drops", insufficient_alignment_tag_drops_);
    status.add("alignment_update_gain", alignment_update_gain_);
    status.add("alignment_max_translation_step_m", alignment_max_translation_step_m_);
    status.add(
      "alignment_max_rotation_step_deg",
      alignment_max_rotation_step_rad_ * 180.0 / M_PI);
    status.add("alignment_translation_step_limits", alignment_translation_step_limits_);
    status.add("alignment_rotation_step_limits", alignment_rotation_step_limits_);
    status.add("vio_measurement_rate_hz", window_rate_hz(vio_measurement_stamps_, now_ns));
    status.add("vio_arrival_rate_hz", window_rate_hz(vio_arrivals_, now_ns));
    status.add("external_imu_update_rate_hz", window_rate_hz(imu_measurement_stamps_, now_ns));
    status.add("external_imu_updates", external_imu_updates_);
    status.add("use_imu_linear_acceleration", use_imu_linear_acceleration_);
    status.add(
      "maximum_imu_linear_acceleration_mps2",
      maximum_imu_linear_acceleration_mps2_);
    status.add(
      "invalid_imu_linear_acceleration_drops",
      invalid_imu_linear_acceleration_drops_);
    status.add("imu_linear_acceleration_limits", imu_linear_acceleration_limits_);
    status.add("old_imu_drops", old_imu_drops_);
    status.add("invalid_imu_drops", invalid_imu_drops_);
    status.add("imu_outside_history_drops", imu_outside_history_drops_);
    status.add(
      "imu_angular_velocity_gate_rejections",
      imu_angular_velocity_gate_rejections_);
    status.add("tag_measurement_rate_hz", window_rate_hz(tag_measurement_stamps_, now_ns));
    status.add("tag_arrival_rate_hz", window_rate_hz(tag_arrivals_, now_ns));
    status.add("zed_tracking_status_received", have_zed_status_);
    status.add("zed_odometry_status", static_cast<int>(last_zed_odometry_status_));
    status.add("zed_tracking_measurement_allowed", tracking_ok);
    status.add("twist_covariance_source", twist_covariance_source_);
    status.add("last_pose_nis", filter_.last_pose_nis());
    status.add("last_twist_nis", filter_.last_twist_nis());
    status.add("delayed_measurement_replays", delayed_measurement_replays_);
    status.add("measurement_history_size", measurements_.size());
    status.add("measurement_checkpoint_count", measurement_filters_.size());
    status.add("replayed_measurement_events", replayed_measurement_events_);
    status.add("max_incremental_replay_events", max_incremental_replay_events_);
    status.add("checkpoint_consistency_failures", checkpoint_consistency_failures_);
    status.add("history_high_water", history_high_water_);
    status.add("measurement_outside_history_drops", measurement_outside_history_drops_);
    status.add("invalid_vio_pose_covariance_drops", invalid_vio_pose_covariance_drops_);
    status.add("invalid_vio_twist_covariance_drops", invalid_vio_twist_covariance_drops_);
    status.add("covariance_validation_failures", covariance_validation_failures_);
    status.add("invalid_output_covariance_drops", invalid_output_covariance_drops_);
    status.add("vio_linear_velocity_stddev_floor_mps", linear_velocity_stddev_floor_mps_);
    status.add(
      "vio_linear_velocity_correction_limit_mps",
      linear_velocity_correction_limit_mps_);
    status.add(
      "vio_linear_velocity_innovation_limit_mps",
      linear_velocity_innovation_limit_mps_);
    status.add(
      "last_vio_linear_velocity_innovation_mps",
      last_vio_linear_velocity_innovation_mps_);
    status.add(
      "last_vio_linear_velocity_correction_mps",
      last_vio_linear_velocity_correction_mps_);
    status.add(
      "vio_linear_velocity_correction_limits",
      vio_linear_velocity_correction_limits_);
    status.add(
      "vio_linear_velocity_rejections",
      vio_linear_velocity_rejections_);
    status.add("invalid_tag_covariance_drops", invalid_tag_covariance_drops_);
    status.add("vio_pose_gate_rejections", vio_pose_gate_rejections_);
    status.add("vio_twist_gate_rejections", vio_twist_gate_rejections_);
    status.add("tag_gate_rejections", tag_gate_rejections_);
    status.add("timestamp_epoch_rejections", timestamp_epoch_rejections_);
    status.add("old_vio_drops", old_vio_drops_);
    status.add("old_tag_drops", old_tag_drops_);
    status.add("zed_status_rejections", zed_status_rejections_);
    status.add("transform_failures", transform_failures_);
    status.add("clock_discontinuities", clock_discontinuities_);
  }

  std::mutex mutex_;
  EkfNoise noise_;
  VioTagEkf filter_;
  std::optional<EkfState> anchor_state_;
  std::deque<Measurement> measurements_;
  // Exact posterior filter after each corresponding measurement. A delayed
  // sample invalidates only checkpoints at and after its insertion point, so
  // replay cost follows measurement latency instead of the full history size.
  std::deque<VioTagEkf> measurement_filters_;
  std::optional<Eigen::Isometry3d> map_from_odom_;
  Eigen::Matrix<double, 6, 6> alignment_covariance_{
    Eigen::Matrix<double, 6, 6>::Zero()};
  std::deque<AlignmentCandidate> alignment_candidates_;
  std::optional<Eigen::Isometry3d> base_from_vio_source_;
  std::string cached_source_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double history_duration_s_{};
  int executor_realtime_priority_{};
  double vio_arrival_timeout_s_{};
  double vio_prediction_horizon_s_{};
  double tag_fresh_s_{};
  double zed_status_timeout_s_{};
  double pose_gate_chi2_{};
  double twist_gate_chi2_{};
  double imu_angular_velocity_gate_chi2_{};
  double gravity_mps2_{};
  double maximum_imu_linear_acceleration_mps2_{};
  double imu_linear_acceleration_stddev_floor_mps2_{};
  double linear_velocity_stddev_floor_mps_{};
  double linear_velocity_correction_limit_mps_{};
  double linear_velocity_innovation_limit_mps_{};
  double tag_gate_chi2_{};
  std::size_t alignment_candidate_count_{};
  std::size_t alignment_candidate_cluster_size_{};
  std::size_t last_alignment_candidate_count_{};
  double alignment_candidate_window_s_{};
  double alignment_translation_tolerance_m_{};
  double alignment_rotation_tolerance_rad_{};
  std::size_t minimum_alignment_tag_count_{};
  double alignment_update_gain_{};
  double alignment_max_translation_step_m_{};
  double alignment_max_rotation_step_rad_{};
  double initial_velocity_stddev_{};
  double initial_angular_velocity_stddev_{};
  bool require_zed_tracking_ok_{true};
  bool use_imu_linear_acceleration_{false};
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string twist_covariance_source_{"not received"};

  std::int64_t last_vio_measurement_stamp_ns_{};
  std::int64_t last_imu_measurement_stamp_ns_{};
  std::int64_t last_vio_accepted_stamp_ns_{};
  std::int64_t last_vio_accepted_arrival_ns_{};
  std::int64_t last_zed_status_arrival_ns_{};
  std::int64_t last_tag_candidate_stamp_ns_{};
  std::int64_t last_tag_stamp_ns_{};
  std::int64_t last_tag_arrival_ns_{};
  std::int64_t last_absolute_stamp_ns_{};
  std::int64_t last_tag_frame_arrival_ns_{};
  std::uint8_t last_zed_odometry_status_{zed_msgs::msg::PosTrackStatus::UNAVAILABLE};
  bool have_zed_status_{false};
  double vio_transport_s_{NAN};
  double tag_transport_s_{NAN};
  double last_tag_translation_residual_{NAN};
  double last_tag_angle_residual_deg_{NAN};
  double last_vio_linear_velocity_innovation_mps_{NAN};
  double last_vio_linear_velocity_correction_mps_{NAN};
  std::uint64_t last_tag_map_generation_{};
  std::uint64_t measurement_sequence_{};
  std::uint64_t delayed_measurement_replays_{};
  std::uint64_t replayed_measurement_events_{};
  std::size_t max_incremental_replay_events_{};
  std::uint64_t checkpoint_consistency_failures_{};
  std::uint64_t alignment_establishments_{};
  std::uint64_t alignment_updates_{};
  std::uint64_t insufficient_alignment_tag_drops_{};
  std::uint64_t alignment_translation_step_limits_{};
  std::uint64_t alignment_rotation_step_limits_{};
  std::uint64_t timestamp_epoch_rejections_{};
  std::uint64_t old_vio_drops_{};
  std::uint64_t old_imu_drops_{};
  std::uint64_t invalid_imu_drops_{};
  std::uint64_t imu_outside_history_drops_{};
  std::uint64_t imu_angular_velocity_gate_rejections_{};
  std::uint64_t invalid_imu_linear_acceleration_drops_{};
  std::uint64_t imu_linear_acceleration_limits_{};
  std::uint64_t external_imu_updates_{};
  std::uint64_t zed_status_rejections_{};
  std::uint64_t invalid_vio_drops_{};
  std::uint64_t invalid_vio_pose_covariance_drops_{};
  std::uint64_t invalid_vio_twist_covariance_drops_{};
  std::uint64_t measurement_outside_history_drops_{};
  std::uint64_t old_tag_drops_{};
  std::uint64_t invalid_tag_drops_{};
  std::uint64_t invalid_tag_covariance_drops_{};
  std::uint64_t covariance_validation_failures_{};
  std::uint64_t invalid_output_covariance_drops_{};
  std::uint64_t vio_pose_gate_rejections_{};
  std::uint64_t vio_twist_gate_rejections_{};
  std::uint64_t vio_linear_velocity_correction_limits_{};
  std::uint64_t vio_linear_velocity_rejections_{};
  std::uint64_t tag_gate_rejections_{};
  std::uint64_t transform_failures_{};
  std::uint64_t clock_discontinuities_{};
  std::size_t history_high_water_{};
  RosClockOffsetJumpDetector clock_jump_detector_;

  std::deque<std::int64_t> vio_measurement_stamps_;
  std::deque<std::int64_t> imu_measurement_stamps_;
  std::deque<std::int64_t> tag_measurement_stamps_;
  std::deque<std::int64_t> vio_arrivals_;
  std::deque<std::int64_t> tag_arrivals_;
  std::deque<std::int64_t> tag_frame_arrivals_;
  std::deque<std::int64_t> body_state_arrivals_;
  robotcore_interfaces::msg::AprilTagPoseEstimate last_tag_estimate_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<robotcore_interfaces::msg::AprilTagPoseEstimate>::SharedPtr tag_sub_;
  rclcpp::Subscription<zed_msgs::msg::PosTrackStatus>::SharedPtr zed_status_sub_;
  rclcpp::Publisher<robotcore_interfaces::msg::BodyState>::SharedPtr body_pub_;
  rclcpp::Publisher<robotcore_interfaces::msg::LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace robotcore_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(robotcore_sensors::VioTagFusionComponent)
