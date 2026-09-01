#include "the_eyes/fusion/ekf.hpp"

#include <cmath>

#include "the_eyes/fusion/so3.hpp"

namespace the_eyes::fusion {
namespace {

// Error-state block offsets within the 9-vector.
constexpr int kP = 0;  // position
constexpr int kV = 3;  // velocity
constexpr int kT = 6;  // orientation (theta)

}  // namespace

ObjectEkf::ObjectEkf(const EkfConfig& cfg, const Eigen::Isometry3d& initial_pose)
    : cfg_(cfg),
      position_(initial_pose.translation()),
      velocity_(Eigen::Vector3d::Zero()),
      orientation_(Eigen::Quaterniond(initial_pose.rotation()).normalized()),
      covariance_(StateMatrix::Zero()) {
  covariance_.block<3, 3>(kP, kP) =
      Eigen::Matrix3d::Identity() * std::pow(cfg_.initial_position_sigma_m, 2);
  covariance_.block<3, 3>(kV, kV) =
      Eigen::Matrix3d::Identity() * std::pow(cfg_.initial_velocity_sigma_mps, 2);
  covariance_.block<3, 3>(kT, kT) =
      Eigen::Matrix3d::Identity() * std::pow(cfg_.initial_orientation_sigma_rad, 2);
}

void ObjectEkf::Predict(double dt_s) {
  if (!(dt_s > 0.0)) return;

  // Nominal: constant velocity, orientation held (random walk adds only noise).
  position_ += velocity_ * dt_s;

  StateMatrix f = StateMatrix::Identity();
  f.block<3, 3>(kP, kV) = Eigen::Matrix3d::Identity() * dt_s;

  // Random-walk process noise scales with elapsed time, so a long gap between
  // frames widens the covariance and the next measurement is trusted more.
  StateMatrix q = StateMatrix::Zero();
  q.block<3, 3>(kP, kP) =
      Eigen::Matrix3d::Identity() * std::pow(cfg_.process_noise_position_m, 2) * dt_s;
  q.block<3, 3>(kV, kV) =
      Eigen::Matrix3d::Identity() * std::pow(cfg_.process_noise_velocity_mps, 2) * dt_s;
  q.block<3, 3>(kT, kT) =
      Eigen::Matrix3d::Identity() * std::pow(cfg_.process_noise_orientation_rad, 2) * dt_s;

  covariance_ = f * covariance_ * f.transpose() + q;
}

void ObjectEkf::Update(const Eigen::Isometry3d& measured_pose, double confidence) {
  const double conf = std::min(1.0, std::max(1e-3, confidence));

  // H selects position and orientation error; velocity is observed only
  // indirectly, through its correlation with position built up in Predict.
  Eigen::Matrix<double, 6, 9> h = Eigen::Matrix<double, 6, 9>::Zero();
  h.block<3, 3>(0, kP) = Eigen::Matrix3d::Identity();
  h.block<3, 3>(3, kT) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 6> r = Eigen::Matrix<double, 6, 6>::Zero();
  r.block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() * std::pow(cfg_.measurement_noise_position_m, 2) / conf;
  r.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * std::pow(cfg_.measurement_noise_orientation_rad, 2) / conf;

  const Eigen::Quaterniond q_meas = Eigen::Quaterniond(measured_pose.rotation()).normalized();

  Eigen::Matrix<double, 6, 1> innovation;
  innovation.head<3>() = measured_pose.translation() - position_;
  // Orientation innovation lives in the tangent space at the nominal attitude:
  // the rotation that carries the estimate onto the measurement.
  innovation.tail<3>() = so3::Log(orientation_.conjugate() * q_meas);

  const Eigen::Matrix<double, 6, 6> s = h * covariance_ * h.transpose() + r;
  const Eigen::Matrix<double, 9, 6> k = covariance_ * h.transpose() * s.inverse();
  const StateVector dx = k * innovation;

  // Inject the error into the nominal state, then reset it to zero -- which is
  // what makes the linearisation valid: the error stays small by construction.
  position_ += dx.segment<3>(kP);
  velocity_ += dx.segment<3>(kV);
  orientation_ = (orientation_ * so3::Exp(dx.segment<3>(kT))).normalized();

  // Joseph form. The textbook (I - KH)P is algebraically equivalent but loses
  // symmetry to rounding over a long run, and an asymmetric covariance
  // eventually yields a negative-definite S and a filter that diverges silently.
  const StateMatrix ikh = StateMatrix::Identity() - k * h;
  covariance_ = ikh * covariance_ * ikh.transpose() + k * r * k.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

Eigen::Isometry3d ObjectEkf::pose() const {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = orientation_.toRotationMatrix();
  t.translation() = position_;
  return t;
}

double ObjectEkf::position_sigma_m() const {
  return std::sqrt(covariance_.block<3, 3>(kP, kP).trace() / 3.0);
}

double ObjectEkf::orientation_sigma_rad() const {
  return std::sqrt(covariance_.block<3, 3>(kT, kT).trace() / 3.0);
}

}  // namespace the_eyes::fusion
