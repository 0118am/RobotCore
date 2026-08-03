#include "robotcore_sensors/imu_conditioning.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

TEST(ImuConditioning, MountingRotationMapsSensorVectorsIntoBaseFrame)
{
  const Eigen::Matrix3d base_from_imu =
    Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d base = robotcore_sensors::rotate_imu_vector_to_base(
    Eigen::Vector3d::UnitX(), base_from_imu);

  EXPECT_NEAR(base.x(), 0.0, 1e-12);
  EXPECT_NEAR(base.y(), 1.0, 1e-12);
  EXPECT_NEAR(base.z(), 0.0, 1e-12);
}

TEST(ImuConditioning, CovarianceFollowsCalibrationAndMountingRotation)
{
  const std::array<double, 9> raw_covariance{
    1.0, 0.0, 0.0,
    0.0, 4.0, 0.0,
    0.0, 0.0, 9.0};
  Eigen::Matrix3d calibration = Eigen::Matrix3d::Identity();
  calibration(0, 0) = 2.0;
  const Eigen::Matrix3d base_from_imu =
    Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const Eigen::Matrix3d conditioned = robotcore_sensors::condition_imu_covariance(
    raw_covariance, calibration, base_from_imu, 0.01);

  EXPECT_NEAR(conditioned(0, 0), 4.0, 1e-12);
  EXPECT_NEAR(conditioned(1, 1), 4.0, 1e-12);
  EXPECT_NEAR(conditioned(2, 2), 9.0, 1e-12);
  EXPECT_LT((conditioned - conditioned.transpose()).norm(), 1e-12);
}

TEST(ImuConditioning, UnknownCovarianceUsesConfiguredNoiseFloor)
{
  std::array<double, 9> unavailable{};
  unavailable[0] = -1.0;
  const Eigen::Matrix3d conditioned = robotcore_sensors::condition_imu_covariance(
    unavailable, Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), 0.2);

  EXPECT_TRUE(conditioned.isApprox(Eigen::Matrix3d::Identity() * 0.04, 1e-12));
}

TEST(ImuConditioning, InvalidPersistentCalibrationCannotEnableAccelerationFusion)
{
  Eigen::Matrix3d singular_gyro = Eigen::Matrix3d::Identity();
  singular_gyro.row(2).setZero();
  EXPECT_FALSE(robotcore_sensors::persistent_imu_calibration_valid(
    Eigen::Matrix3d::Identity(), singular_gyro,
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  EXPECT_FALSE(robotcore_sensors::acceleration_fusion_enabled(false, 0.0, 1.5));
  EXPECT_TRUE(robotcore_sensors::acceleration_fusion_enabled(true, 0.2, 1.5));
}

TEST(ImuConditioning, NonFinitePersistentBiasIsRejected)
{
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  gyro_bias.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(robotcore_sensors::persistent_imu_calibration_valid(
    Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(),
    Eigen::Vector3d::Zero(), gyro_bias));
}
