#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace robotcore_sensors
{
struct TagDefinition
{
  std::array<Eigen::Vector3d, 4> corners;
  double size{};
};

struct CuboidPoolGeometry
{
  bool enforce{true};
  double length_m{5.42};
  double width_m{3.73};
  double surface_tolerance_m{0.02};
  double orientation_tolerance_rad{2.0 * std::acos(-1.0) / 180.0};
};

inline Eigen::Matrix3d fixed_axis_rpy_rotation(const Eigen::Vector3d & rpy)
{
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

inline int canonical_tag_id(const std::string & raw_id)
{
  if (raw_id.empty() || !std::all_of(raw_id.begin(), raw_id.end(), [](unsigned char value) {
      return std::isdigit(value) != 0;
    }))
  {
    throw std::runtime_error("tag id must be a canonical unsigned integer: " + raw_id);
  }
  std::size_t consumed = 0U;
  const unsigned long parsed = std::stoul(raw_id, &consumed, 10);
  if (consumed != raw_id.size() || parsed > 65535UL || std::to_string(parsed) != raw_id) {
    throw std::runtime_error("tag id must be a canonical unsigned integer: " + raw_id);
  }
  return static_cast<int>(parsed);
}

inline void validate_tag_on_pool(
  int id, const Eigen::Vector3d & center, const Eigen::Matrix3d & rotation,
  const std::array<Eigen::Vector3d, 4> & corners, const CuboidPoolGeometry & pool)
{
  if (!pool.enforce) {return;}
  if (!std::isfinite(pool.length_m) || !std::isfinite(pool.width_m) ||
    !std::isfinite(pool.surface_tolerance_m) || !std::isfinite(pool.orientation_tolerance_rad) ||
    pool.length_m <= 0.0 || pool.width_m <= 0.0 || pool.surface_tolerance_m < 0.0 ||
    pool.orientation_tolerance_rad < 0.0 || pool.orientation_tolerance_rad > std::acos(-1.0))
  {
    throw std::runtime_error("invalid cuboid pool geometry parameters");
  }

  struct Surface
  {
    int axis;
    double coordinate;
    Eigen::Vector3d inward_normal;
    bool wall;
  };
  std::vector<Surface> candidates;
  const double tolerance = pool.surface_tolerance_m;
  if (std::abs(center.z()) <= tolerance) {
    candidates.push_back({2, 0.0, Eigen::Vector3d::UnitZ(), false});
  }
  if (std::abs(center.x()) <= tolerance) {
    candidates.push_back({0, 0.0, Eigen::Vector3d::UnitX(), true});
  }
  if (std::abs(center.x() - pool.length_m) <= tolerance) {
    candidates.push_back({0, pool.length_m, -Eigen::Vector3d::UnitX(), true});
  }
  if (std::abs(center.y()) <= tolerance) {
    candidates.push_back({1, 0.0, Eigen::Vector3d::UnitY(), true});
  }
  if (std::abs(center.y() - pool.width_m) <= tolerance) {
    candidates.push_back({1, pool.width_m, -Eigen::Vector3d::UnitY(), true});
  }

  const Eigen::Vector3d printed_top = rotation * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d face_normal = rotation * Eigen::Vector3d::UnitZ();
  const double minimum_alignment = std::cos(pool.orientation_tolerance_rad);
  for (const auto & surface : candidates) {
    if (face_normal.dot(surface.inward_normal) < minimum_alignment) {continue;}
    if (surface.wall && printed_top.dot(Eigen::Vector3d::UnitZ()) < minimum_alignment) {continue;}
    const bool corners_valid = std::all_of(corners.begin(), corners.end(), [&](const auto & corner) {
      return corner.allFinite() && corner.x() >= -tolerance &&
             corner.x() <= pool.length_m + tolerance && corner.y() >= -tolerance &&
             corner.y() <= pool.width_m + tolerance && corner.z() >= -tolerance &&
             std::abs(corner[surface.axis] - surface.coordinate) <= tolerance;
    });
    if (corners_valid) {return;}
  }
  throw std::runtime_error(
          "tag " + std::to_string(id) +
          " is not on a pool surface with inward normal, upright wall axes, and in-bounds corners");
}

inline std::map<int, TagDefinition> parse_apriltag_map(
  const nlohmann::json & root, const std::string & expected_frame,
  const CuboidPoolGeometry & pool)
{
  if (!root.is_object() || !root.contains("schema_version") ||
    !root.at("schema_version").is_number_integer() || root.at("schema_version").get<int>() != 1)
  {
    throw std::runtime_error("tag map schema_version must be integer 1");
  }
  if (!root.contains("frame") || !root.at("frame").is_string() ||
    root.at("frame").get<std::string>() != expected_frame)
  {
    throw std::runtime_error("tag map frame must match " + expected_frame);
  }
  if (!root.contains("tags") || !root.at("tags").is_object()) {
    throw std::runtime_error("tag map tags must be a JSON object");
  }

  std::map<int, TagDefinition> result;
  for (auto iterator = root.at("tags").begin(); iterator != root.at("tags").end(); ++iterator) {
    const int id = canonical_tag_id(iterator.key());
    if (!iterator.value().is_object() || !iterator.value().contains("size_m")) {
      throw std::runtime_error("tag " + iterator.key() + " requires an explicit measured size_m");
    }
    const auto position_values = iterator.value().at("position_m").get<std::vector<double>>();
    const auto rpy_values = iterator.value().at("rpy_deg").get<std::vector<double>>();
    const double size = iterator.value().at("size_m").get<double>();
    if (position_values.size() != 3U || rpy_values.size() != 3U ||
      !std::isfinite(size) || size < 0.01 || size > 2.0)
    {
      throw std::runtime_error("invalid tag definition " + iterator.key());
    }
    const Eigen::Vector3d center(position_values[0], position_values[1], position_values[2]);
    const Eigen::Vector3d rpy =
      Eigen::Vector3d(rpy_values[0], rpy_values[1], rpy_values[2]) * std::acos(-1.0) / 180.0;
    const bool position_in_range = std::all_of(
      position_values.begin(), position_values.end(), [](double value) {return std::abs(value) <= 100.0;});
    const bool rpy_in_range = std::all_of(
      rpy_values.begin(), rpy_values.end(), [](double value) {return std::abs(value) <= 360.0;});
    if (!center.allFinite() || !rpy.allFinite() || !position_in_range || !rpy_in_range) {
      throw std::runtime_error("out-of-range or non-finite tag definition " + iterator.key());
    }
    const Eigen::Matrix3d rotation = fixed_axis_rpy_rotation(rpy);
    const double half = 0.5 * size;
    const std::array<Eigen::Vector3d, 4> local = {
      Eigen::Vector3d(-half, half, 0.0), Eigen::Vector3d(half, half, 0.0),
      Eigen::Vector3d(half, -half, 0.0), Eigen::Vector3d(-half, -half, 0.0)};
    TagDefinition definition;
    definition.size = size;
    for (std::size_t index = 0; index < local.size(); ++index) {
      definition.corners[index] = rotation * local[index] + center;
    }
    validate_tag_on_pool(id, center, rotation, definition.corners, pool);
    if (!result.emplace(id, definition).second) {
      throw std::runtime_error("duplicate numeric tag id " + std::to_string(id));
    }
  }
  return result;
}
}  // namespace robotcore_sensors
