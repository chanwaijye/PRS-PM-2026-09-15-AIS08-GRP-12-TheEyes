// Orientation modulo symmetry. The group has to be exactly the cube's, and the
// nearest-equivalent map has to undo exactly a group element and nothing else,
// or fusion would be averaging the wrong rotations with more confidence than
// before.
#include "the_eyes/fusion/symmetry.hpp"

#include <cmath>

#include <gtest/gtest.h>

#include "the_eyes/fusion/so3.hpp"

namespace the_eyes::fusion {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

Eigen::Quaterniond Rot(double deg, const Eigen::Vector3d& axis) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(deg * kDeg, axis.normalized()));
}

// Angles here are compared to 1e-6 rad, not tighter: AngularDistance is
// 2 acos(dot), and acos cannot resolve below about 3e-8 rad next to 1.

// An arbitrary, non-special reference orientation.
Eigen::Quaterniond Ref() { return Rot(37.0, {0.3, -0.5, 0.8}) * Rot(-64.0, {1, 0.2, 0.1}); }

TEST(Symmetry, TheCubicGroupIsTheCubesTwentyFourRotations) {
  const auto& g = CubicRotations();
  ASSERT_EQ(g.size(), 24u);
  EXPECT_NEAR(std::abs(g[0].w()), 1.0, 1e-12) << "identity first";

  for (size_t i = 0; i < g.size(); ++i) {
    EXPECT_NEAR(g[i].norm(), 1.0, 1e-12);
    // Each one maps the cube's vertices onto the cube's vertices.
    const Eigen::Vector3d v = g[i] * Eigen::Vector3d(1, 1, 1);
    EXPECT_NEAR(std::abs(v.x()), 1.0, 1e-12);
    EXPECT_NEAR(std::abs(v.y()), 1.0, 1e-12);
    EXPECT_NEAR(std::abs(v.z()), 1.0, 1e-12);
    // All distinct.
    for (size_t j = i + 1; j < g.size(); ++j) {
      EXPECT_GT(so3::AngularDistance(g[i], g[j]), 1.0 * kDeg) << i << " vs " << j;
    }
    // Closed: the product of two members is a member.
    for (size_t j = 0; j < g.size(); ++j) {
      const Eigen::Quaterniond p = g[i] * g[j];
      bool found = false;
      for (const auto& h : g) found = found || so3::AngularDistance(p, h) < 1e-6;
      EXPECT_TRUE(found) << i << " * " << j << " left the group";
    }
  }
}

TEST(Symmetry, NearestEquivalentUndoesEveryCubeRotation) {
  const Eigen::Quaterniond ref = Ref();
  for (const Eigen::Quaterniond& g : CubicRotations()) {
    const Eigen::Quaterniond same_pose = ref * g;
    const Eigen::Quaterniond back = NearestEquivalent(same_pose, ref, SymmetryGroup::kCubic);
    EXPECT_NEAR(so3::AngularDistance(back, ref), 0.0, 1e-6);
    EXPECT_NEAR(AngularDistanceModulo(same_pose, ref, SymmetryGroup::kCubic), 0.0, 1e-6);
    EXPECT_GT(back.dot(ref), 0.0) << "returned in the reference's hemisphere";
  }
}

TEST(Symmetry, TheCubeCaseSeenOnTheRig) {
  // Two views of one static cube, 88.3 deg apart: a quarter turn that is the
  // same pose, plus 1.7 deg that is real disagreement. Only the 1.7 survives.
  const Eigen::Quaterniond front = Ref();
  const Eigen::Quaterniond over = front * Rot(88.3, Eigen::Vector3d::UnitZ());
  EXPECT_NEAR(so3::AngularDistance(front, over) / kDeg, 88.3, 1e-6);
  EXPECT_NEAR(AngularDistanceModulo(over, front, SymmetryGroup::kCubic) / kDeg, 1.7, 1e-6);

  const Eigen::Quaterniond mapped = NearestEquivalent(over, front, SymmetryGroup::kCubic);
  EXPECT_NEAR(so3::AngularDistance(mapped, front) / kDeg, 1.7, 1e-6);
  // The mapped orientation is still the SAME pose of the cube as `over`.
  EXPECT_NEAR(AngularDistanceModulo(mapped, over, SymmetryGroup::kCubic), 0.0, 1e-6);
}

TEST(Symmetry, ARealRotationOfACubeIsNotExplainedAway) {
  // 40 deg about a face axis is 40 deg from the nearest equivalent pose: the
  // group must not absorb error that is not a group element.
  const Eigen::Quaterniond ref = Ref();
  const Eigen::Quaterniond turned = ref * Rot(40.0, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(AngularDistanceModulo(turned, ref, SymmetryGroup::kCubic) / kDeg, 40.0, 1e-6);
  // Past 45 deg the next quarter-turn is nearer.
  const Eigen::Quaterniond further = ref * Rot(60.0, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(AngularDistanceModulo(further, ref, SymmetryGroup::kCubic) / kDeg, 30.0, 1e-6);
}

TEST(Symmetry, AxialSymmetryRemovesTheTwistAndKeepsTheTilt) {
  // A bowl turned 137 deg about its own axis is the same bowl. Tipped 6 deg, it
  // is not, and the 6 deg must come through untouched.
  const Eigen::Quaterniond ref = Ref();
  for (double twist : {0.0, 30.0, 137.0, -170.0, 180.0}) {
    const Eigen::Quaterniond spun = ref * Rot(twist, Eigen::Vector3d::UnitZ());
    EXPECT_NEAR(AngularDistanceModulo(spun, ref, SymmetryGroup::kAxialZ), 0.0, 1e-6) << twist;

    const Eigen::Quaterniond tipped = ref * Rot(6.0, Eigen::Vector3d::UnitX()) *
                                      Rot(twist, Eigen::Vector3d::UnitZ());
    EXPECT_NEAR(AngularDistanceModulo(tipped, ref, SymmetryGroup::kAxialZ) / kDeg, 6.0, 1e-6)
        << twist;
    // The equivalent it returns has the same axis as the input.
    const Eigen::Quaterniond back = NearestEquivalent(tipped, ref, SymmetryGroup::kAxialZ);
    EXPECT_NEAR(((back * Eigen::Vector3d::UnitZ()) - (tipped * Eigen::Vector3d::UnitZ())).norm(),
                0.0, 1e-6);
  }
}

TEST(Symmetry, NoGroupChangesNothing) {
  const Eigen::Quaterniond ref = Ref();
  const Eigen::Quaterniond q = ref * Rot(90.0, Eigen::Vector3d::UnitZ());
  EXPECT_NEAR(so3::AngularDistance(NearestEquivalent(q, ref, SymmetryGroup::kNone), q), 0.0, 1e-12);
  EXPECT_NEAR(AngularDistanceModulo(q, ref, SymmetryGroup::kNone) / kDeg, 90.0, 1e-6);
}

}  // namespace
}  // namespace the_eyes::fusion
