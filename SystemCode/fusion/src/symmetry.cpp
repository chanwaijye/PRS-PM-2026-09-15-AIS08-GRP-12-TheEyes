#include "the_eyes/fusion/symmetry.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "the_eyes/fusion/so3.hpp"

namespace the_eyes::fusion {
namespace {

std::vector<Eigen::Quaterniond> BuildCubicRotations() {
  // A rotation that maps the cube onto itself sends each axis to plus or minus
  // another axis: a signed permutation matrix. There are 6 * 8 = 48 of those,
  // and exactly half have determinant +1 -- the proper rotations.
  std::vector<Eigen::Quaterniond> out;
  std::array<int, 3> perm = {0, 1, 2};
  do {
    for (int signs = 0; signs < 8; ++signs) {
      Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
      for (int col = 0; col < 3; ++col) {
        m(perm[static_cast<size_t>(col)], col) = (signs & (1 << col)) ? -1.0 : 1.0;
      }
      if (m.determinant() < 0.0) continue;
      out.emplace_back(Eigen::Quaterniond(m).normalized());
    }
  } while (std::next_permutation(perm.begin(), perm.end()));

  // Identity first, so element 0 means "no change" to a reader of a log.
  const auto identity = std::find_if(out.begin(), out.end(), [](const Eigen::Quaterniond& q) {
    return std::abs(std::abs(q.w()) - 1.0) < 1e-12;
  });
  std::iter_swap(out.begin(), identity);
  return out;
}

}  // namespace

const std::vector<Eigen::Quaterniond>& CubicRotations() {
  static const std::vector<Eigen::Quaterniond> group = BuildCubicRotations();
  return group;
}

Eigen::Quaterniond NearestEquivalent(const Eigen::Quaterniond& q,
                                     const Eigen::Quaterniond& reference, SymmetryGroup group) {
  const Eigen::Quaterniond qn = q.normalized();
  const Eigen::Quaterniond ref = reference.normalized();
  switch (group) {
    case SymmetryGroup::kNone:
      return qn;
    case SymmetryGroup::kCubic: {
      // |dot| is monotone in the geodesic angle and blind to the q / -q sign,
      // so the largest one is the nearest candidate.
      Eigen::Quaterniond best = qn;
      double best_dot = -1.0;
      for (const Eigen::Quaterniond& g : CubicRotations()) {
        const Eigen::Quaterniond candidate = qn * g;
        const double dot = std::abs(candidate.dot(ref));
        if (dot > best_dot) {
          best_dot = dot;
          best = candidate;
        }
      }
      return so3::CanonicalizeSign(best.normalized(), ref);
    }
    case SymmetryGroup::kAxialZ: {
      const Eigen::Quaterniond d = qn.conjugate() * ref;
      // Rz(phi) = (cos(phi/2), 0, 0, sin(phi/2)); its dot with d is
      // d.w cos(phi/2) + d.z sin(phi/2), largest at phi/2 = atan2(d.z, d.w).
      // When d.w and d.z are both zero the reference is a half-turn about an
      // axis in the xy plane and every phi is equally near: atan2(0, 0) is 0,
      // which leaves q alone, as good an answer as any.
      const double half = std::atan2(d.z(), d.w());
      const Eigen::Quaterniond twist(std::cos(half), 0.0, 0.0, std::sin(half));
      return so3::CanonicalizeSign((qn * twist).normalized(), ref);
    }
  }
  return qn;
}

double AngularDistanceModulo(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b,
                             SymmetryGroup group) {
  return so3::AngularDistance(NearestEquivalent(a, b, group), b);
}

}  // namespace the_eyes::fusion
