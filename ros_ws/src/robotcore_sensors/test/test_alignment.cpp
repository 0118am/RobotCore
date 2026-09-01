#include "robotcore_sensors/geometry.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace
{
Eigen::Isometry3d pose(double x, double yaw_deg = 0.0)
{
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = Eigen::Vector3d(x, 2.0, 0.8);
  value.linear() = Eigen::AngleAxisd(
    yaw_deg * std::acos(-1.0) / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return value;
}
}  // namespace

TEST(Alignment, FindsFourConsistentPosesAcrossGapsAndOutliers)
{
  const std::vector<Eigen::Isometry3d> poses{
    pose(1.00), pose(1.65, 25.0), pose(1.03, 1.0),
    pose(0.55, -30.0), pose(0.98, -1.0), pose(1.04, 0.5)};

  const auto cluster = robotcore_sensors::tightest_consistent_pose_cluster(
    poses, 4U, 0.20, 12.0 * std::acos(-1.0) / 180.0);

  EXPECT_EQ(cluster, (std::vector<std::size_t>{0U, 2U, 4U, 5U}));
}

TEST(Alignment, RequiresEverySelectedPairToBeConsistent)
{
  const std::vector<Eigen::Isometry3d> chained{
    pose(0.00), pose(0.15), pose(0.30), pose(0.45)};

  const auto cluster = robotcore_sensors::tightest_consistent_pose_cluster(
    chained, 4U, 0.20, 12.0 * std::acos(-1.0) / 180.0);

  EXPECT_LT(cluster.size(), 4U);
}

TEST(Alignment, RejectsAnOrientationOutlierInsideTheTranslationGate)
{
  const std::vector<Eigen::Isometry3d> poses{
    pose(1.00), pose(1.01, 22.0), pose(1.02, 1.0),
    pose(0.99, -1.0), pose(1.03, 0.5)};

  const auto cluster = robotcore_sensors::tightest_consistent_pose_cluster(
    poses, 4U, 0.20, 12.0 * std::acos(-1.0) / 180.0);

  EXPECT_EQ(cluster, (std::vector<std::size_t>{0U, 2U, 3U, 4U}));
}

TEST(Alignment, AppliesLowGainBelowTheCorrectionLimit)
{
  const Eigen::Vector3d correction =
    robotcore_sensors::scaled_vector_with_norm_limit(
    Eigen::Vector3d(0.03, -0.04, 0.0), 0.02, 0.002);

  EXPECT_NEAR(correction.x(), 0.0006, 1e-12);
  EXPECT_NEAR(correction.y(), -0.0008, 1e-12);
  EXPECT_NEAR(correction.norm(), 0.001, 1e-12);
}

TEST(Alignment, CapsCorrectionWithoutChangingItsDirection)
{
  const Eigen::Vector3d raw(0.30, 0.40, 0.0);
  const Eigen::Vector3d correction =
    robotcore_sensors::scaled_vector_with_norm_limit(raw, 0.02, 0.002);

  EXPECT_NEAR(correction.norm(), 0.002, 1e-12);
  EXPECT_NEAR(correction.normalized().dot(raw.normalized()), 1.0, 1e-12);
}
