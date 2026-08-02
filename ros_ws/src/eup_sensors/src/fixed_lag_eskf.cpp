#include "eup_sensors/fixed_lag_eskf.hpp"
#include "eup_sensors/geometry.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace eup_sensors
{
FixedLagEskf::FixedLagEskf(EskfNoise noise) : noise_(noise) {}

void FixedLagEskf::initialize(const EskfState & state, const ImuSample & first_imu)
{
  state_ = state;
  state_.orientation.normalize();
  last_imu_ = first_imu;
  state_.stamp_ns = first_imu.stamp_ns;
  stabilize_covariance();
  initialized_ = true;
}

void FixedLagEskf::set_state(const EskfState & state, const ImuSample & last_imu)
{
  state_ = state;
  last_imu_ = last_imu;
  initialized_ = true;
}

bool FixedLagEskf::propagate(const ImuSample & sample)
{
  if (!initialized_ || sample.stamp_ns <= state_.stamp_ns) {return false;}
  const double dt = static_cast<double>(sample.stamp_ns - state_.stamp_ns) * 1e-9;
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.20) {return false;}

  const Eigen::Vector3d omega = 0.5 * (last_imu_.gyro + sample.gyro) - state_.gyro_bias;
  const Eigen::Vector3d measured_accel = 0.5 * (last_imu_.accel + sample.accel);
  const bool use_accel = last_imu_.accel_valid && sample.accel_valid;
  const Eigen::Quaterniond q_mid =
    (state_.orientation * exp_quaternion(0.5 * omega * dt)).normalized();
  const Eigen::Vector3d specific_force = use_accel
    ? measured_accel - state_.accel_bias
    : q_mid.conjugate() * Eigen::Vector3d(0.0, 0.0, 9.80665);
  const Eigen::Vector3d acceleration =
    q_mid * specific_force + Eigen::Vector3d(0.0, 0.0, -9.80665);
  state_.position += state_.velocity * dt + 0.5 * acceleration * dt * dt;
  state_.velocity += acceleration * dt;
  state_.orientation = (state_.orientation * exp_quaternion(omega * dt)).normalized();

  Matrix15d f = Matrix15d::Zero();
  f.block<3, 3>(0, 3).setIdentity();
  if (use_accel) {
    f.block<3, 3>(3, 6) = -q_mid.toRotationMatrix() * skew(specific_force);
    f.block<3, 3>(3, 12) = -q_mid.toRotationMatrix();
  }
  f.block<3, 3>(6, 6) = -skew(omega);
  f.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 15, 12> g = Eigen::Matrix<double, 15, 12>::Zero();
  if (use_accel) {g.block<3, 3>(3, 3) = -q_mid.toRotationMatrix();}
  g.block<3, 3>(6, 0) = -Eigen::Matrix3d::Identity();
  g.block<3, 3>(9, 6).setIdentity();
  g.block<3, 3>(12, 9).setIdentity();
  Eigen::Matrix<double, 12, 12> qc = Eigen::Matrix<double, 12, 12>::Zero();
  qc.block<3, 3>(0, 0).diagonal().setConstant(noise_.gyro_noise * noise_.gyro_noise);
  qc.block<3, 3>(3, 3).diagonal().setConstant(
    use_accel ? noise_.accel_noise * noise_.accel_noise : 0.0);
  qc.block<3, 3>(6, 6).diagonal().setConstant(noise_.gyro_bias_walk * noise_.gyro_bias_walk);
  qc.block<3, 3>(9, 9).diagonal().setConstant(noise_.accel_bias_walk * noise_.accel_bias_walk);
  const Matrix15d phi = Matrix15d::Identity() + f * dt + 0.5 * f * f * dt * dt;
  state_.covariance = phi * state_.covariance * phi.transpose() + g * qc * g.transpose() * dt;
  state_.stamp_ns = sample.stamp_ns;
  last_imu_ = sample;
  stabilize_covariance();
  return true;
}

bool FixedLagEskf::update_pose(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation,
  const Eigen::Matrix<double, 6, 6> & covariance, double gate_chi2)
{
  Eigen::Matrix<double, 6, 1> innovation;
  innovation.head<3>() = position - state_.position;
  innovation.tail<3>() = log_quaternion(state_.orientation.conjugate() * orientation.normalized());
  Eigen::Matrix<double, 6, 15> h = Eigen::Matrix<double, 6, 15>::Zero();
  h.block<3, 3>(0, 0).setIdentity();
  h.block<3, 3>(3, 6).setIdentity();
  return update<6>(innovation, h, covariance, gate_chi2);
}

bool FixedLagEskf::update_velocity(
  const Eigen::Vector3d & velocity, const Eigen::Matrix3d & covariance, double gate_chi2)
{
  Eigen::Matrix<double, 3, 15> h = Eigen::Matrix<double, 3, 15>::Zero();
  h.block<3, 3>(0, 3).setIdentity();
  return update<3>(velocity - state_.velocity, h, covariance, gate_chi2);
}

template<int M>
bool FixedLagEskf::update(
  const Eigen::Matrix<double, M, 1> & innovation,
  const Eigen::Matrix<double, M, 15> & h,
  const Eigen::Matrix<double, M, M> & r, double gate_chi2)
{
  if (!innovation.allFinite() || !r.allFinite()) {return false;}
  const Eigen::Matrix<double, M, M> s = h * state_.covariance * h.transpose() + r;
  const Eigen::LDLT<Eigen::Matrix<double, M, M>> ldlt(s);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {return false;}
  last_nis_ = innovation.dot(ldlt.solve(innovation));
  if (!std::isfinite(last_nis_) || last_nis_ > gate_chi2) {return false;}
  const Eigen::Matrix<double, 15, M> k =
    state_.covariance * h.transpose() * ldlt.solve(Eigen::Matrix<double, M, M>::Identity());
  const Eigen::Matrix<double, 15, 1> error = k * innovation;
  const Matrix15d identity = Matrix15d::Identity();
  const Matrix15d correction = identity - k * h;
  state_.covariance = correction * state_.covariance * correction.transpose() + k * r * k.transpose();
  inject(error);
  stabilize_covariance();
  return true;
}

void FixedLagEskf::inject(const Eigen::Matrix<double, 15, 1> & error)
{
  state_.position += error.segment<3>(0);
  state_.velocity += error.segment<3>(3);
  state_.orientation = (state_.orientation * exp_quaternion(error.segment<3>(6))).normalized();
  state_.gyro_bias += error.segment<3>(9);
  state_.accel_bias += error.segment<3>(12);
  Matrix15d reset = Matrix15d::Identity();
  reset.block<3, 3>(6, 6) -= 0.5 * skew(error.segment<3>(6));
  state_.covariance = reset * state_.covariance * reset.transpose();
}

void FixedLagEskf::stabilize_covariance()
{
  state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix15d> solver(state_.covariance);
  if (solver.info() != Eigen::Success) {
    state_.covariance = Matrix15d::Identity();
    return;
  }
  const auto values = solver.eigenvalues().cwiseMax(1e-12);
  state_.covariance = solver.eigenvectors() * values.asDiagonal() * solver.eigenvectors().transpose();
}

template bool FixedLagEskf::update<3>(
  const Eigen::Matrix<double, 3, 1> &, const Eigen::Matrix<double, 3, 15> &,
  const Eigen::Matrix<double, 3, 3> &, double);
template bool FixedLagEskf::update<6>(
  const Eigen::Matrix<double, 6, 1> &, const Eigen::Matrix<double, 6, 15> &,
  const Eigen::Matrix<double, 6, 6> &, double);
}  // namespace eup_sensors
