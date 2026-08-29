#include "robotcore_sensors/covariance.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>

namespace
{
using robotcore_sensors::Matrix6d;

Eigen::Isometry3d transform(
  const Eigen::Vector3d & translation, const Eigen::Vector3d & rotation_vector)
{
  return robotcore_sensors::pose_transform(
    translation, robotcore_sensors::exp_quaternion(rotation_vector));
}

Eigen::Isometry3d perturb_fixed_axes(
  const Eigen::Isometry3d & value, const Eigen::Matrix<double, 6, 1> & delta)
{
  Eigen::Isometry3d perturbed = value;
  perturbed.translation() += delta.head<3>();
  perturbed.linear() =
    robotcore_sensors::exp_quaternion(delta.tail<3>()).toRotationMatrix() *
    value.linear();
  return perturbed;
}

Eigen::Matrix<double, 6, 1> fixed_axis_residual(
  const Eigen::Isometry3d & value, const Eigen::Isometry3d & reference)
{
  Eigen::Matrix<double, 6, 1> residual;
  residual.head<3>() = value.translation() - reference.translation();
  residual.tail<3>() = robotcore_sensors::log_quaternion(
    Eigen::Quaterniond(value.linear() * reference.linear().transpose()));
  return residual;
}

Matrix6d positive_definite_covariance(double scale, double offset)
{
  Matrix6d factor;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      factor(row, column) = scale *
        static_cast<double>((row + 1) * (column + 2)) / 37.0;
    }
  }
  return factor * factor.transpose() + offset * Matrix6d::Identity();
}
}  // namespace

TEST(Covariance, SymmetrizationEvaluatesBeforeAssignment)
{
  Matrix6d input;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      input(row, column) = static_cast<double>(10 * row + column);
    }
  }
  Matrix6d expected;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      expected(row, column) = 0.5 * (input(row, column) + input(column, row));
    }
  }

  Matrix6d actual = input;
  actual = robotcore_sensors::symmetrized_covariance<6>(actual);

  EXPECT_TRUE(actual.isApprox(expected, 1e-15));
  EXPECT_DOUBLE_EQ((actual - actual.transpose()).cwiseAbs().maxCoeff(), 0.0);
}

TEST(Covariance, ValidationRejectsMalformedMatrices)
{
  const Matrix6d valid = positive_definite_covariance(0.02, 0.001);
  EXPECT_TRUE(robotcore_sensors::valid_covariance<6>(valid));

  Matrix6d asymmetric = valid;
  asymmetric(0, 1) += 1e-4;
  EXPECT_FALSE(robotcore_sensors::valid_covariance<6>(asymmetric));

  Matrix6d indefinite = valid;
  indefinite(5, 5) = -0.1;
  EXPECT_FALSE(robotcore_sensors::valid_covariance<6>(indefinite));

  Matrix6d nonfinite = valid;
  nonfinite(2, 2) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(robotcore_sensors::valid_covariance<6>(nonfinite));
}

TEST(Covariance, MeasurementNormalizationAcceptsFloatRoundoffOnly)
{
  Matrix6d rounded = positive_definite_covariance(0.02, 0.001);
  rounded = rounded.cast<float>().cast<double>();
  rounded(0, 1) = std::nextafter(
    static_cast<float>(rounded(1, 0)), std::numeric_limits<float>::infinity());
  ASSERT_NE(rounded(0, 1), rounded(1, 0));

  EXPECT_TRUE(robotcore_sensors::normalize_measurement_covariance<6>(rounded));
  EXPECT_DOUBLE_EQ((rounded - rounded.transpose()).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_TRUE(robotcore_sensors::valid_covariance<6>(rounded));

  Matrix6d slightly_indefinite = 0.01 * Matrix6d::Identity();
  slightly_indefinite(5, 5) = -1e-9;
  EXPECT_TRUE(
    robotcore_sensors::normalize_measurement_covariance<6>(slightly_indefinite));
  EXPECT_TRUE(robotcore_sensors::valid_covariance<6>(slightly_indefinite));

  Matrix6d materially_asymmetric = positive_definite_covariance(0.02, 0.001);
  materially_asymmetric(0, 1) += 1e-4;
  EXPECT_FALSE(
    robotcore_sensors::normalize_measurement_covariance<6>(materially_asymmetric));

  Matrix6d materially_indefinite = Matrix6d::Identity();
  materially_indefinite(5, 5) = -1e-3;
  EXPECT_FALSE(
    robotcore_sensors::normalize_measurement_covariance<6>(materially_indefinite));

  Matrix6d nonfinite = Matrix6d::Identity();
  nonfinite(2, 2) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(robotcore_sensors::normalize_measurement_covariance<6>(nonfinite));
}

TEST(Covariance, StabilizationRepairsOnlyRoundoffScaleErrors)
{
  Matrix6d roundoff = Matrix6d::Identity();
  roundoff(5, 5) = -1e-13;
  ASSERT_TRUE(robotcore_sensors::stabilize_covariance<6>(roundoff));
  EXPECT_TRUE(robotcore_sensors::valid_covariance<6>(roundoff));
  EXPECT_GT(roundoff(5, 5), 0.0);

  Matrix6d materially_indefinite = Matrix6d::Identity();
  materially_indefinite(5, 5) = -1e-3;
  EXPECT_FALSE(robotcore_sensors::stabilize_covariance<6>(materially_indefinite));
  EXPECT_DOUBLE_EQ(materially_indefinite(5, 5), -1e-3);
}

TEST(Covariance, UnknownCorrelationSumBoundsExactCovariance)
{
  Matrix6d first_factor;
  Matrix6d second_factor;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      first_factor(row, column) =
        static_cast<double>((row + 2) * (column + 1)) / 31.0;
      second_factor(row, column) =
        static_cast<double>((row + 1) - 2 * column) / 23.0;
    }
  }
  const Matrix6d first = first_factor * first_factor.transpose();
  const Matrix6d second = second_factor * second_factor.transpose();
  // Both errors are driven by the same latent vector, which realizes a valid
  // nonzero cross-covariance absent from their two marginal covariances.
  const Matrix6d exact =
    (first_factor + second_factor) *
    (first_factor + second_factor).transpose();
  const Matrix6d upper_bound =
    robotcore_sensors::covariance_sum_with_unknown_correlation<6>(first, second);
  const Matrix6d margin =
    robotcore_sensors::symmetrized_covariance<6>(upper_bound - exact);

  EXPECT_TRUE(robotcore_sensors::valid_covariance<6>(upper_bound));
  EXPECT_TRUE(robotcore_sensors::valid_covariance<6>(margin));
}

TEST(Covariance, AlignmentPropagationMatchesFiniteDifferenceJacobians)
{
  const Eigen::Isometry3d map_from_base = transform(
    Eigen::Vector3d(1.2, -0.8, 0.4), Eigen::Vector3d(0.18, -0.09, 0.27));
  const Eigen::Isometry3d odom_from_base = transform(
    Eigen::Vector3d(2.3, 0.6, -0.2), Eigen::Vector3d(-0.12, 0.21, 0.08));
  const Eigen::Isometry3d nominal = map_from_base * odom_from_base.inverse();
  const Matrix6d map_covariance = positive_definite_covariance(0.03, 0.002);
  const Matrix6d odom_covariance = positive_definite_covariance(0.02, 0.001);

  Eigen::Matrix<double, 6, 6> map_jacobian;
  Eigen::Matrix<double, 6, 6> odom_jacobian;
  constexpr double epsilon = 1e-7;
  for (int column = 0; column < 6; ++column) {
    Eigen::Matrix<double, 6, 1> delta =
      Eigen::Matrix<double, 6, 1>::Zero();
    delta(column) = epsilon;
    const Eigen::Isometry3d map_plus =
      perturb_fixed_axes(map_from_base, delta) * odom_from_base.inverse();
    const Eigen::Isometry3d map_minus =
      perturb_fixed_axes(map_from_base, -delta) * odom_from_base.inverse();
    map_jacobian.col(column) =
      (fixed_axis_residual(map_plus, nominal) -
      fixed_axis_residual(map_minus, nominal)) / (2.0 * epsilon);

    const Eigen::Isometry3d odom_plus =
      map_from_base * perturb_fixed_axes(odom_from_base, delta).inverse();
    const Eigen::Isometry3d odom_minus =
      map_from_base * perturb_fixed_axes(odom_from_base, -delta).inverse();
    odom_jacobian.col(column) =
      (fixed_axis_residual(odom_plus, nominal) -
      fixed_axis_residual(odom_minus, nominal)) / (2.0 * epsilon);
  }

  const Matrix6d expected = robotcore_sensors::symmetrized_covariance<6>(
    (map_jacobian * map_covariance * map_jacobian.transpose() +
    odom_jacobian * odom_covariance * odom_jacobian.transpose()).eval());
  const Matrix6d actual = robotcore_sensors::alignment_candidate_covariance(
    map_covariance, odom_covariance, odom_from_base, nominal);

  EXPECT_TRUE(actual.isApprox(expected, 1e-8));
  EXPECT_TRUE(robotcore_sensors::valid_covariance<6>(actual));
}
