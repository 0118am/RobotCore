#include "robotcore_sensors/fixed_lag_eskf.hpp"
#include "robotcore_sensors/geometry.hpp"
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
#include <vector>

namespace
{
robotcore_sensors::FixedLagEskf stationary_filter()
{
  robotcore_sensors::FixedLagEskf filter;
  robotcore_sensors::EskfState state;
  state.covariance.setIdentity();
  robotcore_sensors::ImuSample first;
  first.stamp_ns = 1000000000LL;
  first.accel = Eigen::Vector3d(0.0, 0.0, 9.80665);
  first.accel_valid = true;
  filter.initialize(state, first);
  return filter;
}
}

TEST(FixedLagEskf, StationaryPropagationDoesNotDrift)
{
  auto filter = stationary_filter();
  for (int i = 1; i <= 100; ++i) {
    robotcore_sensors::ImuSample sample;
    sample.stamp_ns = 1000000000LL + i * 10000000LL;
    sample.accel = Eigen::Vector3d(0.0, 0.0, 9.80665);
    sample.accel_valid = true;
    ASSERT_TRUE(filter.propagate(sample));
  }
  EXPECT_LT(filter.state().position.norm(), 1e-8);
  EXPECT_LT(filter.state().velocity.norm(), 1e-8);
}

TEST(FixedLagEskf, JosephUpdateKeepsCovariancePositive)
{
  auto filter = stationary_filter();
  Eigen::Matrix<double, 6, 6> r = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  ASSERT_TRUE(filter.update_pose(
    Eigen::Vector3d(0.1, -0.1, 0.02), Eigen::Quaterniond::Identity(), r));
  Eigen::SelfAdjointEigenSolver<robotcore_sensors::Matrix15d> solver(filter.state().covariance);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
  EXPECT_LT((filter.state().covariance - filter.state().covariance.transpose()).norm(), 1e-10);
}

TEST(FixedLagEskf, RejectsOutlierByNis)
{
  auto filter = stationary_filter();
  Eigen::Matrix<double, 6, 6> r = Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
  EXPECT_FALSE(filter.update_pose(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Quaterniond::Identity(), r));
}

TEST(FixedLagEskf, So3ExponentialAndLogarithmRoundTrip)
{
  const Eigen::Vector3d rotation_vector(0.12, -0.07, 0.21);
  const auto recovered = robotcore_sensors::log_quaternion(
    robotcore_sensors::exp_quaternion(rotation_vector));
  EXPECT_LT((recovered - rotation_vector).norm(), 1e-12);
}

TEST(FixedLagEskf, DelayedUpdateReplayMatchesChronologicalProcessing)
{
  auto chronological = stationary_filter();
  std::vector<robotcore_sensors::ImuSample> samples;
  samples.reserve(50);
  robotcore_sensors::EskfState rewind_state;
  robotcore_sensors::ImuSample rewind_imu;
  Eigen::Matrix<double, 6, 6> measurement_covariance =
    Eigen::Matrix<double, 6, 6>::Identity() * 0.02;
  for (int i = 1; i <= 50; ++i) {
    robotcore_sensors::ImuSample sample;
    sample.stamp_ns = 1000000000LL + i * 10000000LL;
    sample.gyro = Eigen::Vector3d(0.0, 0.0, 0.01);
    sample.accel = Eigen::Vector3d(0.0, 0.0, 9.80665);
    sample.accel_valid = true;
    samples.push_back(sample);
    ASSERT_TRUE(chronological.propagate(sample));
    if (i == 25) {
      rewind_state = chronological.state();
      rewind_imu = chronological.last_imu();
      ASSERT_TRUE(chronological.update_pose(
        Eigen::Vector3d(0.01, -0.02, 0.0),
        Eigen::Quaterniond(Eigen::AngleAxisd(0.002, Eigen::Vector3d::UnitZ())),
        measurement_covariance));
    }
  }

  auto delayed = stationary_filter();
  for (const auto & sample : samples) {ASSERT_TRUE(delayed.propagate(sample));}
  delayed.set_state(rewind_state, rewind_imu);
  ASSERT_TRUE(delayed.update_pose(
    Eigen::Vector3d(0.01, -0.02, 0.0),
    Eigen::Quaterniond(Eigen::AngleAxisd(0.002, Eigen::Vector3d::UnitZ())),
    measurement_covariance));
  for (std::size_t i = 25; i < samples.size(); ++i) {ASSERT_TRUE(delayed.propagate(samples[i]));}

  EXPECT_LT((delayed.state().position - chronological.state().position).norm(), 1e-12);
  EXPECT_LT((delayed.state().velocity - chronological.state().velocity).norm(), 1e-12);
  EXPECT_LT((delayed.state().covariance - chronological.state().covariance).norm(), 1e-11);
}

TEST(FixedLagEskf, FreshnessAdvancesOnlyForAcceptedNewerMeasurements)
{
  robotcore_sensors::AcceptedMeasurementFreshness freshness;
  EXPECT_FALSE(freshness.record(false, 1000, 1100));
  EXPECT_EQ(freshness.stamp_ns, 0);

  EXPECT_TRUE(freshness.record(true, 1000, 1100));
  EXPECT_EQ(freshness.stamp_ns, 1000);
  EXPECT_EQ(freshness.arrival_ns, 1100);

  EXPECT_FALSE(freshness.record(false, 2000, 2100));
  EXPECT_FALSE(freshness.record(true, 900, 2200));
  EXPECT_EQ(freshness.stamp_ns, 1000);
  EXPECT_EQ(freshness.arrival_ns, 1100);
}
