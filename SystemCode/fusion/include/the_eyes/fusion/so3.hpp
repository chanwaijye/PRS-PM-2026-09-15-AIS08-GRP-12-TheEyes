// Minimal SO(3) helpers: exponential/logarithm maps and angular distance.
//
// Written out rather than pulled from Sophus/manif deliberately -- CLAUDE.md
// constraint 2 rules out a packaged estimation stack for this module, and the
// error-state EKF needs the tangent-space maps to be ours.
#ifndef THE_EYES_FUSION_SO3_HPP
#define THE_EYES_FUSION_SO3_HPP

#include <Eigen/Geometry>

namespace the_eyes::fusion::so3 {

// Exponential map: rotation vector (axis * angle, radians) -> unit quaternion.
// Uses the small-angle series near zero, where sin(t/2)/t is 0/0.
Eigen::Quaterniond Exp(const Eigen::Vector3d& omega);

// Logarithm map: unit quaternion -> rotation vector. Returns the shortest
// rotation, so the result always has norm <= pi. Sign-canonicalised, since q
// and -q are the same rotation but log to vectors differing by 2*pi.
Eigen::Vector3d Log(const Eigen::Quaterniond& q);

// Geodesic angle between two rotations, radians, in [0, pi].
double AngularDistance(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b);

// Returns q or -q, whichever lies in the same hemisphere as `reference`.
// Averaging is sign-invariant, but interpolation and logging are not.
Eigen::Quaterniond CanonicalizeSign(const Eigen::Quaterniond& q,
                                    const Eigen::Quaterniond& reference);

}  // namespace the_eyes::fusion::so3

#endif  // THE_EYES_FUSION_SO3_HPP
