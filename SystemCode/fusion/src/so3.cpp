#include "the_eyes/fusion/so3.hpp"

#include <cmath>

namespace the_eyes::fusion::so3 {

Eigen::Quaterniond Exp(const Eigen::Vector3d& omega) {
  const double theta = omega.norm();
  Eigen::Quaterniond q;
  if (theta < 1e-8) {
    // sin(theta/2)/theta -> 1/2 as theta -> 0. Taking the limit directly avoids
    // the 0/0 that would otherwise produce NaN for a stationary object, which
    // is the common case every frame the EKF is converged.
    q.w() = 1.0;
    q.vec() = 0.5 * omega;
  } else {
    q.w() = std::cos(0.5 * theta);
    q.vec() = (std::sin(0.5 * theta) / theta) * omega;
  }
  q.normalize();
  return q;
}

Eigen::Vector3d Log(const Eigen::Quaterniond& q_in) {
  Eigen::Quaterniond q = q_in.normalized();
  // q and -q are the same rotation; pick w >= 0 so the result is the shortest
  // path rather than its 2*pi complement.
  if (q.w() < 0.0) q.coeffs() *= -1.0;

  const double n = q.vec().norm();
  if (n < 1e-8) {
    // Near identity, theta ~ 2 * |vec| / w.
    return 2.0 * q.vec() / std::max(q.w(), 1e-12);
  }
  const double theta = 2.0 * std::atan2(n, q.w());
  return (theta / n) * q.vec();
}

double AngularDistance(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  const double d = std::abs(a.normalized().dot(b.normalized()));
  // Clamp: floating-point dot products stray outside [-1,1] and acos returns
  // NaN there, which would silently disable the disagreement gate.
  return 2.0 * std::acos(std::min(1.0, std::max(-1.0, d)));
}

Eigen::Quaterniond CanonicalizeSign(const Eigen::Quaterniond& q,
                                    const Eigen::Quaterniond& reference) {
  if (q.dot(reference) < 0.0) {
    return Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
  }
  return q;
}

}  // namespace the_eyes::fusion::so3
