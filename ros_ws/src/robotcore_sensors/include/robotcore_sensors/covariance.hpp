#pragma once

#include "robotcore_sensors/geometry.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <limits>

namespace robotcore_sensors
{
using Matrix6d = Eigen::Matrix<double, 6, 6>;

inline constexpr double kCovarianceSymmetryTolerance = 1e-10;
inline constexpr double kCovariancePsdTolerance = 1e-12;
inline constexpr double kCovarianceRepairTolerance = 1e-10;
inline constexpr double kCovarianceEigenvalueFloor = 1e-12;
inline constexpr double kMeasurementCovarianceRoundoffTolerance =
  64.0 * std::numeric_limits<float>::epsilon();

template<int Size>
using FixedCovariance = Eigen::Matrix<double, Size, Size>;

template<int Size>
FixedCovariance<Size> symmetrized_covariance(
  const FixedCovariance<Size> & covariance)
{
  // Eigen transpose expressions alias their source.  Returning a concrete
  // object forces the complete right-hand side to be evaluated before any
  // caller assigns it back to the source matrix.
  const FixedCovariance<Size> symmetric =
    (0.5 * (covariance + covariance.transpose())).eval();
  return symmetric;
}

template<int Size>
bool valid_covariance(const FixedCovariance<Size> & covariance)
{
  if (!covariance.allFinite()) {return false;}
  const double scale = std::max(1.0, covariance.cwiseAbs().maxCoeff());
  const double asymmetry =
    (covariance - covariance.transpose()).cwiseAbs().maxCoeff();
  if (asymmetry > kCovarianceSymmetryTolerance * scale) {return false;}

  const FixedCovariance<Size> symmetric = symmetrized_covariance<Size>(covariance);
  Eigen::SelfAdjointEigenSolver<FixedCovariance<Size>> solver(
    symmetric, Eigen::EigenvaluesOnly);
  return solver.info() == Eigen::Success &&
         solver.eigenvalues().minCoeff() >= -kCovariancePsdTolerance * scale;
}

// ROS drivers commonly promote float covariance entries to double.  Such a
// matrix can acquire float-roundoff asymmetry or a correspondingly small
// negative eigenvalue even when the producer formed a valid covariance.  At
// the measurement boundary, repair only errors at that scale; internal EKF
// covariances remain subject to valid_covariance() without this relaxation.
template<int Size>
bool normalize_measurement_covariance(FixedCovariance<Size> & covariance)
{
  if (!covariance.allFinite()) {return false;}
  const double magnitude = std::max(
    covariance.cwiseAbs().maxCoeff(), std::numeric_limits<double>::min());
  const double tolerance = kMeasurementCovarianceRoundoffTolerance * magnitude;
  const double asymmetry =
    (covariance - covariance.transpose()).cwiseAbs().maxCoeff();
  if (asymmetry > tolerance) {return false;}

  covariance = symmetrized_covariance<Size>(covariance);
  Eigen::SelfAdjointEigenSolver<FixedCovariance<Size>> solver(covariance);
  if (solver.info() != Eigen::Success ||
    solver.eigenvalues().minCoeff() < -tolerance)
  {
    return false;
  }
  if (solver.eigenvalues().minCoeff() < 0.0) {
    const Eigen::Matrix<double, Size, 1> eigenvalues =
      solver.eigenvalues().cwiseMax(kCovarianceEigenvalueFloor * magnitude);
    covariance = symmetrized_covariance<Size>(
      (solver.eigenvectors() * eigenvalues.asDiagonal() *
      solver.eigenvectors().transpose()).eval());
  }
  return valid_covariance<Size>(covariance);
}

template<int Size>
bool stabilize_covariance(FixedCovariance<Size> & covariance)
{
  if (!covariance.allFinite()) {return false;}
  const FixedCovariance<Size> symmetric = symmetrized_covariance<Size>(covariance);
  const double scale = std::max(1.0, symmetric.cwiseAbs().maxCoeff());
  Eigen::SelfAdjointEigenSolver<FixedCovariance<Size>> solver(symmetric);
  if (solver.info() != Eigen::Success ||
    solver.eigenvalues().minCoeff() < -kCovarianceRepairTolerance * scale)
  {
    return false;
  }

  const Eigen::Matrix<double, Size, 1> eigenvalues =
    solver.eigenvalues().cwiseMax(kCovarianceEigenvalueFloor * scale);
  const FixedCovariance<Size> projected =
    (solver.eigenvectors() * eigenvalues.asDiagonal() *
    solver.eigenvectors().transpose()).eval();
  covariance = symmetrized_covariance<Size>(projected);
  return valid_covariance<Size>(covariance);
}

// If two error terms may be correlated but their cross-covariance is not
// available, simply adding their marginal covariances assumes independence.
// Young's inequality gives the correlation-independent upper bound
// Cov(x + y) <= 2 * (Cov(x) + Cov(y)).
template<int Size>
FixedCovariance<Size> covariance_sum_with_unknown_correlation(
  const FixedCovariance<Size> & first,
  const FixedCovariance<Size> & second)
{
  const FixedCovariance<Size> upper_bound = (2.0 * (first + second)).eval();
  return symmetrized_covariance<Size>(upper_bound);
}

// First-order covariance for T_map_odom = T_map_base * inverse(T_odom_base).
// All three 6-vectors use the ROS fixed-axis convention [position, rotation]
// in their respective parent frames.  The full Jacobians include the
// translation/rotation lever arm introduced by pose inversion and composition.
inline Matrix6d alignment_candidate_covariance(
  const Matrix6d & map_from_base_covariance,
  const Matrix6d & odom_from_base_covariance,
  const Eigen::Isometry3d & odom_from_base,
  const Eigen::Isometry3d & map_from_odom)
{
  const Eigen::Matrix3d map_from_odom_rotation = map_from_odom.linear();
  const Eigen::Vector3d base_position_odom = odom_from_base.translation();
  const Eigen::Vector3d rotated_base_position =
    map_from_odom_rotation * base_position_odom;

  Matrix6d map_from_base_jacobian = Matrix6d::Identity();
  map_from_base_jacobian.block<3, 3>(0, 3) = skew(rotated_base_position);

  Matrix6d odom_from_base_jacobian = Matrix6d::Zero();
  odom_from_base_jacobian.block<3, 3>(0, 0) = -map_from_odom_rotation;
  odom_from_base_jacobian.block<3, 3>(0, 3) =
    -map_from_odom_rotation * skew(base_position_odom);
  odom_from_base_jacobian.block<3, 3>(3, 3) = -map_from_odom_rotation;

  const Matrix6d covariance =
    (map_from_base_jacobian * map_from_base_covariance *
    map_from_base_jacobian.transpose() +
    odom_from_base_jacobian * odom_from_base_covariance *
    odom_from_base_jacobian.transpose()).eval();
  return symmetrized_covariance<6>(covariance);
}
}  // namespace robotcore_sensors
