#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

namespace eup_sensors
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
}  // namespace eup_sensors
