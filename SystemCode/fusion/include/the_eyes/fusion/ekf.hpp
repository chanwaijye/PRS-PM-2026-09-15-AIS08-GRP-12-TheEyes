// Per-object error-state EKF for temporal smoothing.
//
// Written from first principles (CLAUDE.md constraint 2 rules out a packaged
// EKF). Formulation is the standard multiplicative / error-state filter:
//
//   nominal state   p (position), v (velocity), q (orientation)
//   error state     dx = [dp, dv, dtheta] in R^9, covariance P
//
// Orientation is carried as a unit quaternion in the nominal state and its
// error as a 3-vector in the tangent space at q. Keeping the error minimal
// avoids the two failure modes of filtering a 4-vector quaternion directly: a
// singular covariance from the unit-norm constraint, and drift off the manifold
// as updates accumulate.
//
// Process model: constant velocity in position, random walk in orientation.
// There is no angular-velocity state -- tabletop objects are moved by a robot
// in discrete pick-and-place steps rather than tumbling continuously, so a
// rotation-rate estimate would have almost nothing to lock onto between events.
#ifndef THE_EYES_FUSION_EKF_HPP
#define THE_EYES_FUSION_EKF_HPP

#include <Eigen/Geometry>

#include "the_eyes/fusion/config.hpp"

namespace the_eyes::fusion {

class ObjectEkf {
 public:
  using StateVector = Eigen::Matrix<double, 9, 1>;
  using StateMatrix = Eigen::Matrix<double, 9, 9>;

  ObjectEkf(const EkfConfig& cfg, const Eigen::Isometry3d& initial_pose);

  // Advance the nominal state and inflate the covariance. dt_s <= 0 is a no-op:
  // two views of the same frame arrive with identical timestamps, and rewinding
  // on a jittered stamp would corrupt the state.
  void Predict(double dt_s);

  // Fold in one fused measurement. `confidence` in (0,1] scales the measurement
  // noise as R/confidence, so a low-confidence fuse moves the filter less.
  void Update(const Eigen::Isometry3d& measured_pose, double confidence);

  Eigen::Isometry3d pose() const;
  const Eigen::Vector3d& position() const { return position_; }
  const Eigen::Vector3d& velocity() const { return velocity_; }
  const Eigen::Quaterniond& orientation() const { return orientation_; }
  const StateMatrix& covariance() const { return covariance_; }

  // RMS position uncertainty, metres -- a scalar summary for logging and for
  // deriving the confidence reported on ObjectState.
  double position_sigma_m() const;
  double orientation_sigma_rad() const;

 private:
  EkfConfig cfg_;
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Quaterniond orientation_;
  StateMatrix covariance_;
};

}  // namespace the_eyes::fusion

#endif  // THE_EYES_FUSION_EKF_HPP
