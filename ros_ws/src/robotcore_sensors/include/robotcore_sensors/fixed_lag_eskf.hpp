#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

namespace robotcore_sensors
{
using Matrix15d = Eigen::Matrix<double, 15, 15>;

struct EskfState
{
  std::int64_t stamp_ns{};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Matrix15d covariance{Matrix15d::Identity()};
};

struct ImuSample
{
  std::int64_t stamp_ns{};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
  bool accel_valid{false};
};

struct EskfNoise
{
  double gyro_noise{0.015};
  double accel_noise{0.20};
  double gyro_bias_walk{0.0005};
  double accel_bias_walk{0.01};
};

struct AcceptedMeasurementFreshness
{
  bool record(bool accepted, std::int64_t candidate_stamp_ns, std::int64_t candidate_arrival_ns)
  {
    if (!accepted || candidate_stamp_ns <= stamp_ns) {return false;}
    stamp_ns = candidate_stamp_ns;
    arrival_ns = candidate_arrival_ns;
    return true;
  }

  std::int64_t stamp_ns{};
  std::int64_t arrival_ns{};
};

inline bool timestamp_in_current_epoch(
  std::int64_t stamp_ns, std::int64_t arrival_ns,
  std::int64_t maximum_absolute_age_ns = 1000000000LL)
{
  return stamp_ns > 0 && arrival_ns > 0 && maximum_absolute_age_ns >= 0 &&
         stamp_ns >= arrival_ns - maximum_absolute_age_ns &&
         stamp_ns <= arrival_ns + maximum_absolute_age_ns;
}

inline bool vio_bridge_required(
  bool imu_fresh, std::int64_t measurement_stamp_ns,
  std::int64_t filter_stamp_ns,
  std::int64_t maximum_direct_imu_gap_ns = 200000000LL)
{
  if (measurement_stamp_ns <= filter_stamp_ns) {return false;}
  return !imu_fresh ||
         measurement_stamp_ns - filter_stamp_ns > maximum_direct_imu_gap_ns;
}

class RosClockOffsetJumpDetector
{
public:
  explicit RosClockOffsetJumpDetector(std::int64_t threshold_ns = 100000000LL)
  : threshold_ns_(threshold_ns) {}

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
    return offset_change > threshold_ns_ || offset_change < -threshold_ns_;
  }

  void reset()
  {
    initialized_ = false;
    last_ros_ns_ = 0;
    last_steady_ns_ = 0;
  }

private:
  std::int64_t threshold_ns_;
  std::int64_t last_ros_ns_{};
  std::int64_t last_steady_ns_{};
  bool initialized_{false};
};

class FixedLagEskf
{
public:
  explicit FixedLagEskf(EskfNoise noise = {});
  void initialize(const EskfState & state, const ImuSample & first_imu);
  bool propagate(const ImuSample & sample);
  bool update_pose(
    const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation,
    const Eigen::Matrix<double, 6, 6> & covariance, double gate_chi2 = 22.458);
  bool update_velocity(
    const Eigen::Vector3d & velocity, const Eigen::Matrix3d & covariance,
    double gate_chi2 = 16.266);
  const EskfState & state() const {return state_;}
  const ImuSample & last_imu() const {return last_imu_;}
  void set_state(const EskfState & state, const ImuSample & last_imu);
  bool initialized() const {return initialized_;}
  double last_nis() const {return last_nis_;}

private:
  template<int M>
  bool update(
    const Eigen::Matrix<double, M, 1> & innovation,
    const Eigen::Matrix<double, M, 15> & h,
    const Eigen::Matrix<double, M, M> & r, double gate_chi2);
  void inject(const Eigen::Matrix<double, 15, 1> & error);
  void stabilize_covariance();

  EskfState state_;
  ImuSample last_imu_;
  EskfNoise noise_;
  bool initialized_{false};
  double last_nis_{0.0};
};
}  // namespace robotcore_sensors
