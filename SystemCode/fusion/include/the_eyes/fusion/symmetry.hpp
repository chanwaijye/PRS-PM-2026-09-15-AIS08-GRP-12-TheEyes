// Orientation modulo an object's symmetry.
//
// A cube looks the same after any of 24 rotations about its centre, a bowl
// after any turn about its axis. A pose estimator has no way to prefer one of
// those orientations, so two views of one static object -- or one view on two
// frames -- legitimately return rotations that differ by a group element and
// still describe the same pose.
//
// MEASURED on the rig, 2026-10-04: the two cameras reported the printed cube
// 88 deg apart in one scene and 178 deg apart in another. Fusion already knew
// not to AVERAGE such a pair and took the best-scoring view's orientation
// instead, but the choice of best view changes whenever a camera loses the
// object, so the orientation handed to the filter jumped 88 deg on 43 of 300
// pairs. The EKF smoothed across each jump, through orientations that are not
// the cube at all: gate + EKF scored 8.5 mm ADD-S and 58% inside the
// threshold, where the gate alone scored 3.0 mm and 99.7% on the same frames.
//
// The remedy is not to tolerate the difference but to remove it: map every
// orientation to the member of its symmetry class nearest a reference, and
// only then average, gate or filter. Written here from first principles
// (CLAUDE.md constraint 2), on top of so3.hpp.
#ifndef THE_EYES_FUSION_SYMMETRY_HPP
#define THE_EYES_FUSION_SYMMETRY_HPP

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "the_eyes/types.hpp"

namespace the_eyes::fusion {

// Symmetry group by class, from the mesh registry. A class that is absent has
// no declared group.
using SymmetryRegistry = std::map<std::string, SymmetryGroup>;

// The 24 proper rotations that map a cube onto itself, as unit quaternions:
// every signed permutation of the axes with determinant +1. The identity is
// element 0.
const std::vector<Eigen::Quaterniond>& CubicRotations();

// The orientation equivalent to `q` under `group` that is nearest `reference`.
//
// The group acts in the OBJECT's frame, so the equivalents of q are q * g.
//   kNone    q itself.
//   kCubic   the q * g, over the 24 rotations, closest to the reference.
//   kAxialZ  q * Rz(phi) for the phi that brings it closest, in closed form:
//            with d = q^-1 * reference, phi = 2 atan2(d.z, d.w) -- the twist
//            part of d about z. What is left over is the tilt of the axis,
//            which no turn about the axis can remove.
Eigen::Quaterniond NearestEquivalent(const Eigen::Quaterniond& q,
                                     const Eigen::Quaterniond& reference, SymmetryGroup group);

// Geodesic angle between two orientations once the symmetry is removed,
// radians: how far apart two poses of the object really are. For kNone this is
// so3::AngularDistance.
double AngularDistanceModulo(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b,
                             SymmetryGroup group);

}  // namespace the_eyes::fusion

#endif  // THE_EYES_FUSION_SYMMETRY_HPP
