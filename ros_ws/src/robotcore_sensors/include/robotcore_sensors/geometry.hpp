#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace robotcore_sensors
{
inline Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

inline Eigen::Quaterniond exp_quaternion(const Eigen::Vector3d & theta)
{
  const double angle = theta.norm();
  if (angle < 1e-10) {
    return Eigen::Quaterniond(1.0, 0.5 * theta.x(), 0.5 * theta.y(), 0.5 * theta.z()).normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, theta / angle));
}

inline Eigen::Vector3d log_quaternion(Eigen::Quaterniond q)
{
  q.normalize();
  if (q.w() < 0.0) {q.coeffs() *= -1.0;}
  const double n = q.vec().norm();
  if (n < 1e-10) {return 2.0 * q.vec();}
  return 2.0 * std::atan2(n, std::clamp(q.w(), -1.0, 1.0)) * q.vec() / n;
}

inline Eigen::Isometry3d pose_transform(
  const Eigen::Vector3d & p, const Eigen::Quaterniond & q)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.normalized().toRotationMatrix();
  transform.translation() = p;
  return transform;
}

inline double rotation_distance(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
{
  const double cosine = std::clamp((a.transpose() * b).trace() * 0.5 - 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

inline Eigen::Vector3d scaled_vector_with_norm_limit(
  const Eigen::Vector3d & value, double gain, double maximum_norm)
{
  const Eigen::Vector3d scaled = gain * value;
  const double norm = scaled.norm();
  if (norm <= maximum_norm || norm < 1e-12) {return scaled;}
  return scaled * (maximum_norm / norm);
}

inline double normalized_pose_distance(
  const Eigen::Isometry3d & first, const Eigen::Isometry3d & second,
  double translation_tolerance_m, double rotation_tolerance_rad)
{
  if (!std::isfinite(translation_tolerance_m) ||
    !std::isfinite(rotation_tolerance_rad) ||
    translation_tolerance_m <= 0.0 || rotation_tolerance_rad <= 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(
    (first.translation() - second.translation()).norm() / translation_tolerance_m,
    rotation_distance(first.linear(), second.linear()) / rotation_tolerance_rad);
}

inline std::vector<std::size_t> tightest_consistent_pose_cluster(
  const std::vector<Eigen::Isometry3d> & poses, std::size_t maximum_count,
  double translation_tolerance_m, double rotation_tolerance_rad)
{
  if (maximum_count == 0U || poses.empty() ||
    !std::isfinite(translation_tolerance_m) ||
    !std::isfinite(rotation_tolerance_rad) ||
    translation_tolerance_m <= 0.0 || rotation_tolerance_rad <= 0.0)
  {
    return {};
  }

  std::vector<std::size_t> best;
  double best_spread = std::numeric_limits<double>::infinity();
  std::size_t best_newest_index = 0U;
  for (std::size_t reverse_seed = 0U; reverse_seed < poses.size(); ++reverse_seed) {
    const std::size_t seed = poses.size() - 1U - reverse_seed;
    std::vector<std::pair<double, std::size_t>> ranked;
    ranked.reserve(poses.size());
    for (std::size_t index = 0U; index < poses.size(); ++index) {
      const double distance = normalized_pose_distance(
        poses[seed], poses[index], translation_tolerance_m, rotation_tolerance_rad);
      if (distance <= 1.0) {ranked.emplace_back(distance, index);}
    }
    std::sort(ranked.begin(), ranked.end(), [](const auto & left, const auto & right) {
      if (left.first != right.first) {return left.first < right.first;}
      return left.second > right.second;
    });

    std::vector<std::size_t> cluster;
    cluster.reserve(std::min(maximum_count, ranked.size()));
    for (const auto & entry : ranked) {
      const bool mutually_consistent = std::all_of(
        cluster.begin(), cluster.end(), [&](std::size_t selected) {
          return normalized_pose_distance(
            poses[selected], poses[entry.second],
            translation_tolerance_m, rotation_tolerance_rad) <= 1.0;
        });
      if (!mutually_consistent) {continue;}
      cluster.push_back(entry.second);
      if (cluster.size() == maximum_count) {break;}
    }

    double spread = 0.0;
    std::size_t newest_index = 0U;
    for (std::size_t left = 0U; left < cluster.size(); ++left) {
      newest_index = std::max(newest_index, cluster[left]);
      for (std::size_t right = left + 1U; right < cluster.size(); ++right) {
        spread += normalized_pose_distance(
          poses[cluster[left]], poses[cluster[right]],
          translation_tolerance_m, rotation_tolerance_rad);
      }
    }
    const bool better = cluster.size() > best.size() ||
      (cluster.size() == best.size() && spread < best_spread - 1e-12) ||
      (cluster.size() == best.size() && std::abs(spread - best_spread) <= 1e-12 &&
      newest_index > best_newest_index);
    if (better) {
      best = std::move(cluster);
      best_spread = spread;
      best_newest_index = newest_index;
    }
  }
  std::sort(best.begin(), best.end());
  return best;
}

}  // namespace robotcore_sensors
