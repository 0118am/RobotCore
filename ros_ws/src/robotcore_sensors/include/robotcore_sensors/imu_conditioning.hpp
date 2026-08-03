#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace robotcore_sensors
{
inline bool persistent_imu_calibration_valid(
  const Eigen::Matrix3d & accel_matrix,
  const Eigen::Matrix3d & gyro_matrix,
  const Eigen::Vector3d & accel_bias,
  const Eigen::Vector3d & gyro_bias)
{
  return accel_matrix.allFinite() && gyro_matrix.allFinite() &&
         accel_bias.allFinite() && gyro_bias.allFinite() &&
         accel_matrix.determinant() > 1e-6 && gyro_matrix.determinant() > 1e-6;
}

inline bool acceleration_fusion_enabled(
  bool persistent_calibration_valid, double startup_residual_mps2,
  double maximum_residual_mps2)
{
  return persistent_calibration_valid && std::isfinite(startup_residual_mps2) &&
         startup_residual_mps2 < maximum_residual_mps2;
}

inline Eigen::Vector3d calibrate_sensor_vector(
  const Eigen::Vector3d & raw, const Eigen::Vector3d & bias,
  const Eigen::Matrix3d & calibration)
{
  return calibration * (raw - bias);
}

inline Eigen::Vector3d rotate_imu_vector_to_base(
  const Eigen::Vector3d & imu_vector, const Eigen::Matrix3d & base_from_imu)
{
  return base_from_imu * imu_vector;
}

template<typename CovarianceArray>
Eigen::Matrix3d condition_imu_covariance(
  const CovarianceArray & raw_covariance,
  const Eigen::Matrix3d & calibration,
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

  const Eigen::Matrix3d jacobian = base_from_imu * calibration;
  Eigen::Matrix3d result = jacobian * sensor_covariance * jacobian.transpose();
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
