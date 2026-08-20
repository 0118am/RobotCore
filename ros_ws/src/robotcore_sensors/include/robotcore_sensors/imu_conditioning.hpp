#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robotcore_sensors
{
inline Eigen::Vector3d rotate_imu_vector_to_base(
  const Eigen::Vector3d & imu_vector, const Eigen::Matrix3d & base_from_imu)
{
  return base_from_imu * imu_vector;
}

class TimestampedVectorLowPass
{
public:
  TimestampedVectorLowPass(double cutoff_hz, double reset_gap_s)
  : cutoff_hz_(cutoff_hz), reset_gap_s_(reset_gap_s) {}

  Eigen::Vector3d update(const Eigen::Vector3d & input, std::int64_t stamp_ns)
  {
    if (!initialized_) {
      reset(input, stamp_ns);
      return state_;
    }
    const double dt = static_cast<double>(stamp_ns - last_stamp_ns_) * 1e-9;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > reset_gap_s_) {
      reset(input, stamp_ns);
      return state_;
    }
    const double alpha = 1.0 - std::exp(-2.0 * std::acos(-1.0) * cutoff_hz_ * dt);
    state_ += alpha * (input - state_);
    last_stamp_ns_ = stamp_ns;
    return state_;
  }

  void reset(const Eigen::Vector3d & input, std::int64_t stamp_ns)
  {
    state_ = input;
    last_stamp_ns_ = stamp_ns;
    initialized_ = true;
  }

private:
  double cutoff_hz_{};
  double reset_gap_s_{};
  Eigen::Vector3d state_{Eigen::Vector3d::Zero()};
  std::int64_t last_stamp_ns_{};
  bool initialized_{false};
};

template<typename CovarianceArray>
Eigen::Matrix3d condition_imu_covariance(
  const CovarianceArray & raw_covariance,
  const Eigen::Matrix3d & base_from_imu,
  double standard_deviation_floor)
{
  const double variance_floor = standard_deviation_floor * standard_deviation_floor;
  Eigen::Matrix3d sensor_covariance = Eigen::Matrix3d::Zero();
  bool input_valid = raw_covariance[0] >= 0.0;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      const double value = raw_covariance[3 * row + column];
      input_valid = input_valid && std::isfinite(value);
      sensor_covariance(row, column) = value;
    }
  }
  if (!input_valid) {sensor_covariance.setZero();}

  Eigen::Matrix3d result =
    base_from_imu * sensor_covariance * base_from_imu.transpose();
  result = 0.5 * (result + result.transpose());
  if (!result.allFinite()) {return Eigen::Matrix3d::Identity() * variance_floor;}

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(result);
  if (solver.info() != Eigen::Success) {
    return Eigen::Matrix3d::Identity() * variance_floor;
  }
  const Eigen::Vector3d eigenvalues =
    solver.eigenvalues().cwiseMax(std::max(0.0, variance_floor));
  return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}
}  // namespace robotcore_sensors
