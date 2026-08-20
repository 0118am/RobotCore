#include "robotcore_sensors/apriltag_map.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <fstream>

namespace
{
nlohmann::json empty_map()
{
  return {
    {"schema_version", 1},
    {"frame", "map"},
    {"tags", nlohmann::json::object()},
  };
}

nlohmann::json synthetic_valid_map()
{
  auto document = empty_map();
  document["tags"]["12"] = {
    {"position_m", {1.0, 1.0, 0.0}},
    {"rpy_deg", {0.0, 0.0, 20.0}},
    {"size_m", 0.2},
  };
  document["tags"]["31"] = {
    {"position_m", {2.0, 0.0, 0.6}},
    {"rpy_deg", {90.0, 0.0, 180.0}},
    {"size_m", 0.2},
  };
  return document;
}
}  // namespace

TEST(AprilTagMap, ExplicitMeasuredSizesAndPoolSurfacesAreAccepted)
{
  const auto tags = robotcore_sensors::parse_apriltag_map(
    synthetic_valid_map(), "map", robotcore_sensors::CuboidPoolGeometry{});
  ASSERT_EQ(tags.size(), 2U);
  EXPECT_DOUBLE_EQ(tags.at(12).size, 0.2);
  EXPECT_DOUBLE_EQ(tags.at(31).size, 0.2);
  for (const auto & corner : tags.at(31).corners) {
    EXPECT_NEAR(corner.y(), 0.0, 1e-12);
  }
}

TEST(AprilTagMap, SchemaFrameAndCanonicalIdsAreStrict)
{
  auto wrong_schema = empty_map();
  wrong_schema["schema_version"] = 2;
  EXPECT_THROW(
    robotcore_sensors::parse_apriltag_map(
      wrong_schema, "map", robotcore_sensors::CuboidPoolGeometry{}),
    std::exception);

  auto wrong_frame = empty_map();
  wrong_frame["frame"] = "odom";
  EXPECT_THROW(
    robotcore_sensors::parse_apriltag_map(
      wrong_frame, "map", robotcore_sensors::CuboidPoolGeometry{}),
    std::exception);

  auto noncanonical_id = synthetic_valid_map();
  noncanonical_id["tags"]["01"] = noncanonical_id["tags"]["12"];
  EXPECT_THROW(
    robotcore_sensors::parse_apriltag_map(
      noncanonical_id, "map", robotcore_sensors::CuboidPoolGeometry{}),
    std::exception);
}

TEST(AprilTagMap, MissingMeasuredSizeIsRejected)
{
  auto document = synthetic_valid_map();
  document["tags"]["12"].erase("size_m");
  EXPECT_THROW(
    robotcore_sensors::parse_apriltag_map(
      document, "map", robotcore_sensors::CuboidPoolGeometry{}),
    std::exception);
}

TEST(AprilTagMap, OffSurfaceOrOutwardFacingTagsAreRejected)
{
  auto off_surface = synthetic_valid_map();
  off_surface["tags"]["12"]["position_m"] = {1.0, 1.0, 0.5};
  EXPECT_THROW(
    robotcore_sensors::parse_apriltag_map(
      off_surface, "map", robotcore_sensors::CuboidPoolGeometry{}),
    std::exception);

  auto outward = synthetic_valid_map();
  outward["tags"]["31"]["rpy_deg"] = {90.0, 0.0, 0.0};
  EXPECT_THROW(
    robotcore_sensors::parse_apriltag_map(
      outward, "map", robotcore_sensors::CuboidPoolGeometry{}),
    std::exception);
}

TEST(AprilTagMap, DeployedSurveyCanBeValidatedExplicitly)
{
  const char * path = std::getenv("ROBOTCORE_TEST_DEPLOYED_TAG_MAP");
  if (path == nullptr || std::string(path).empty()) {
    GTEST_SKIP() << "set ROBOTCORE_TEST_DEPLOYED_TAG_MAP for the on-robot survey acceptance check";
  }
  std::ifstream stream(path);
  ASSERT_TRUE(stream.good()) << path;
  nlohmann::json document;
  ASSERT_NO_THROW(stream >> document);
  const auto tags = robotcore_sensors::parse_apriltag_map(
    document, "map", robotcore_sensors::CuboidPoolGeometry{});
  EXPECT_FALSE(tags.empty()) << "deployed survey must contain measured Tags";
}

TEST(AprilTagQuality, AcceptsLargeConvexObservationsAndRanksLessObliqueTagsFirst)
{
  const std::array<Eigen::Vector2d, 4> square{
    Eigen::Vector2d(100.0, 100.0), Eigen::Vector2d(160.0, 100.0),
    Eigen::Vector2d(160.0, 160.0), Eigen::Vector2d(100.0, 160.0)};
  const std::array<Eigen::Vector2d, 4> oblique{
    Eigen::Vector2d(100.0, 100.0), Eigen::Vector2d(160.0, 104.0),
    Eigen::Vector2d(150.0, 132.0), Eigen::Vector2d(105.0, 130.0)};
  const auto square_quality = robotcore_sensors::assess_tag_image_quality(
    square, 960U, 600U, 20.0, 2.5);
  const auto oblique_quality = robotcore_sensors::assess_tag_image_quality(
    oblique, 960U, 600U, 20.0, 2.5);
  ASSERT_TRUE(square_quality.accepted);
  ASSERT_TRUE(oblique_quality.accepted);
  EXPECT_GT(square_quality.score, oblique_quality.score);
  EXPECT_DOUBLE_EQ(square_quality.minimum_edge_px, 60.0);
}

TEST(AprilTagQuality, RejectsSmallNonConvexAndOutOfFrameObservations)
{
  const std::array<Eigen::Vector2d, 4> small{
    Eigen::Vector2d(100.0, 100.0), Eigen::Vector2d(110.0, 100.0),
    Eigen::Vector2d(110.0, 110.0), Eigen::Vector2d(100.0, 110.0)};
  const std::array<Eigen::Vector2d, 4> nonconvex{
    Eigen::Vector2d(100.0, 100.0), Eigen::Vector2d(160.0, 100.0),
    Eigen::Vector2d(120.0, 120.0), Eigen::Vector2d(100.0, 160.0)};
  auto out_of_frame = small;
  out_of_frame[0].x() = -1.0;
  EXPECT_FALSE(robotcore_sensors::assess_tag_image_quality(
    small, 960U, 600U, 20.0, 2.5).accepted);
  EXPECT_FALSE(robotcore_sensors::assess_tag_image_quality(
    nonconvex, 960U, 600U, 20.0, 2.5).accepted);
  EXPECT_FALSE(robotcore_sensors::assess_tag_image_quality(
    out_of_frame, 960U, 600U, 5.0, 2.5).accepted);
}

TEST(AprilTagInliers, KeepsEveryTagWithIndependentCornerSupport)
{
  const std::vector<int> point_inliers{
    0, 1, 2, 3,
    4, 5, 6,
    8, 9,
  };
  const auto supported = robotcore_sensors::independently_supported_tag_indices(
    point_inliers, 3U, 4U, 3);
  EXPECT_EQ(supported, (std::vector<std::size_t>{0U, 1U}));
}

TEST(AprilTagInliers, IgnoresDuplicateAndOutOfRangePointIndices)
{
  const std::vector<int> point_inliers{0, 1, 1, 2, -1, 12};
  const auto supported = robotcore_sensors::independently_supported_tag_indices(
    point_inliers, 3U, 4U, 3);
  EXPECT_EQ(supported, (std::vector<std::size_t>{0U}));
}
