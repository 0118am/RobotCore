#include "robotcore_sensors/imu_conditioning.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <cmath>

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

TEST(ImuConditioning, CovarianceFollowsMountingRotation)
{
  const std::array<double, 9> raw_covariance{
    1.0, 0.0, 0.0,
    0.0, 4.0, 0.0,
    0.0, 0.0, 9.0};
  const Eigen::Matrix3d base_from_imu =
    Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const Eigen::Matrix3d conditioned = robotcore_sensors::condition_imu_covariance(
    raw_covariance, base_from_imu, 0.01);

  EXPECT_NEAR(conditioned(0, 0), 4.0, 1e-12);
  EXPECT_NEAR(conditioned(1, 1), 1.0, 1e-12);
  EXPECT_NEAR(conditioned(2, 2), 9.0, 1e-12);
  EXPECT_LT((conditioned - conditioned.transpose()).norm(), 1e-12);
}

TEST(ImuConditioning, UnknownCovarianceUsesConfiguredNoiseFloor)
{
  std::array<double, 9> unavailable{};
  unavailable[0] = -1.0;
  const Eigen::Matrix3d conditioned = robotcore_sensors::condition_imu_covariance(
    unavailable, Eigen::Matrix3d::Identity(), 0.2);

  EXPECT_TRUE(conditioned.isApprox(Eigen::Matrix3d::Identity() * 0.04, 1e-12));
}

TEST(ImuConditioning, TimestampedLowPassUsesMeasuredSampleInterval)
{
  robotcore_sensors::TimestampedVectorLowPass filter(10.0, 0.2);
  EXPECT_TRUE(filter.update(Eigen::Vector3d::Zero(), 1000000000LL).isZero());
  const Eigen::Vector3d filtered = filter.update(
    Eigen::Vector3d::Ones(), 1010000000LL);
  const double expected = 1.0 - std::exp(-2.0 * std::acos(-1.0) * 10.0 * 0.01);
  EXPECT_TRUE(filtered.isApprox(Eigen::Vector3d::Constant(expected), 1e-12));
}

TEST(ImuConditioning, TimestampedLowPassResetsAfterInputGap)
{
  robotcore_sensors::TimestampedVectorLowPass filter(10.0, 0.2);
  filter.update(Eigen::Vector3d::Zero(), 1000000000LL);
  const Eigen::Vector3d after_gap = filter.update(
    Eigen::Vector3d(1.0, 2.0, 3.0), 1300000000LL);
  EXPECT_TRUE(after_gap.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0), 1e-12));
}
