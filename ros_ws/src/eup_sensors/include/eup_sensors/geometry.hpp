#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace eup_sensors
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

inline Eigen::Isometry3d blend_transform(
  const Eigen::Isometry3d & a, const Eigen::Isometry3d & b, double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.translation() = (1.0 - alpha) * a.translation() + alpha * b.translation();
  output.linear() = Eigen::Quaterniond(a.linear()).slerp(alpha, Eigen::Quaterniond(b.linear())).normalized().toRotationMatrix();
  return output;
}
}  // namespace eup_sensors
