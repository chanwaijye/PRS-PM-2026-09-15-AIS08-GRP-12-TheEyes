#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "the_eyes/fusion/quaternion_average.hpp"
#include "the_eyes/fusion/so3.hpp"

namespace the_eyes::fusion {
namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Quaterniond Rot(double deg, const Eigen::Vector3d& axis) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(deg * kPi / 180.0, axis.normalized()));
}

TEST(So3, ExpLogRoundTrip) {
  std::mt19937 rng(20260819);
  std::uniform_real_distribution<double> axis(-1.0, 1.0);
  std::uniform_real_distribution<double> angle(-kPi + 1e-3, kPi - 1e-3);

  for (int i = 0; i < 200; ++i) {
    Eigen::Vector3d a(axis(rng), axis(rng), axis(rng));
    if (a.norm() < 1e-6) continue;
    const Eigen::Vector3d omega = a.normalized() * angle(rng);
    EXPECT_LT((so3::Log(so3::Exp(omega)) - omega).norm(), 1e-9) << "i=" << i;
  }
}

TEST(So3, ExpIsFiniteAtZero) {
  // The 0/0 in sin(t/2)/t: a stationary object hits this every frame.
  const Eigen::Quaterniond q = so3::Exp(Eigen::Vector3d::Zero());
  EXPECT_TRUE(q.coeffs().allFinite());
  EXPECT_NEAR(so3::AngularDistance(q, Eigen::Quaterniond::Identity()), 0.0, 1e-12);

  const Eigen::Vector3d tiny(1e-12, -2e-12, 3e-13);
  EXPECT_TRUE(so3::Exp(tiny).coeffs().allFinite());
  EXPECT_TRUE(so3::Log(so3::Exp(tiny)).allFinite());
}

TEST(So3, LogTakesTheShortPathRegardlessOfSign) {
  const Eigen::Quaterniond q = Rot(170.0, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond neg(-q.w(), -q.x(), -q.y(), -q.z());
  EXPECT_LT((so3::Log(q) - so3::Log(neg)).norm(), 1e-9);
  EXPECT_LE(so3::Log(q).norm(), kPi + 1e-9);
}

TEST(So3, AngularDistanceIsClampedAndSymmetric) {
  const Eigen::Quaterniond a = Rot(30.0, Eigen::Vector3d::UnitX());
  const Eigen::Quaterniond b = Rot(75.0, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(so3::AngularDistance(a, b) * 180.0 / kPi, 45.0, 1e-9);
  EXPECT_NEAR(so3::AngularDistance(a, b), so3::AngularDistance(b, a), 1e-12);
  // Identical inputs are where acos(1 + eps) would go NaN.
  EXPECT_NEAR(so3::AngularDistance(a, a), 0.0, 1e-7);
  EXPECT_TRUE(std::isfinite(so3::AngularDistance(a, a)));
}

TEST(QuaternionAverage, SingleInputIsReturnedUnchanged) {
  const Eigen::Quaterniond q = Rot(37.0, Eigen::Vector3d(1, 2, 3));
  const Eigen::Quaterniond avg = WeightedQuaternionAverage({q}, {0.7});
  EXPECT_NEAR(so3::AngularDistance(q, avg), 0.0, 1e-9);
}

TEST(QuaternionAverage, IdenticalInputsAverageToThemselves) {
  const Eigen::Quaterniond q = Rot(80.0, Eigen::Vector3d(0, 1, 1));
  const Eigen::Quaterniond avg = WeightedQuaternionAverage({q, q, q}, {1.0, 0.4, 2.0});
  EXPECT_NEAR(so3::AngularDistance(q, avg), 0.0, 1e-9);
}

TEST(QuaternionAverage, IsInvariantToInputSign) {
  // The whole reason for the outer-product formulation: two views reporting q
  // and -q describe the SAME rotation, and componentwise averaging cancels them.
  const Eigen::Quaterniond q = Rot(80.0, Eigen::Vector3d::UnitY());
  const Eigen::Quaterniond neg(-q.w(), -q.x(), -q.y(), -q.z());

  const Eigen::Quaterniond avg = WeightedQuaternionAverage({q, neg}, {1.0, 1.0});
  EXPECT_NEAR(so3::AngularDistance(q, avg), 0.0, 1e-9);

  const Eigen::Vector4d naive = 0.5 * (q.coeffs() + neg.coeffs());
  EXPECT_LT(naive.norm(), 1e-12) << "sanity: naive averaging really does cancel";
}

TEST(QuaternionAverage, EqualWeightsGiveTheMidpoint) {
  const Eigen::Quaterniond a = Rot(0.0, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond b = Rot(60.0, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond avg = WeightedQuaternionAverage({a, b}, {1.0, 1.0});
  EXPECT_NEAR(so3::AngularDistance(a, avg) * 180.0 / kPi, 30.0, 1e-6);
  EXPECT_NEAR(so3::AngularDistance(b, avg) * 180.0 / kPi, 30.0, 1e-6);
}

TEST(QuaternionAverage, WeightPullsTheResultTowardTheHeavierInput) {
  const Eigen::Quaterniond a = Rot(0.0, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond b = Rot(60.0, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond avg = WeightedQuaternionAverage({a, b}, {9.0, 1.0});
  const double to_a = so3::AngularDistance(a, avg) * 180.0 / kPi;
  EXPECT_LT(to_a, 30.0);
  EXPECT_GT(to_a, 0.0);
}

TEST(QuaternionAverage, RecoversTruthFromNoisyViews) {
  // The property fusion actually relies on: averaging noisy per-view rotations
  // lands closer to truth than a single noisy view does.
  const Eigen::Quaterniond truth = Rot(25.0, Eigen::Vector3d(0.3, 1.0, 0.2));
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 6.0 * kPi / 180.0);

  double single_err_sum = 0.0;
  double fused_err_sum = 0.0;
  constexpr int kTrials = 400;
  for (int t = 0; t < kTrials; ++t) {
    std::vector<Eigen::Quaterniond> views;
    for (int v = 0; v < 4; ++v) {
      const Eigen::Vector3d perturb(noise(rng), noise(rng), noise(rng));
      views.push_back(so3::Exp(perturb) * truth);
    }
    single_err_sum += so3::AngularDistance(truth, views.front());
    fused_err_sum += so3::AngularDistance(
        truth, WeightedQuaternionAverage(views, std::vector<double>(views.size(), 1.0)));
  }
  EXPECT_LT(fused_err_sum / kTrials, single_err_sum / kTrials);
}

TEST(QuaternionAverage, IsDeterministicAcrossRepeatedCalls) {
  const std::vector<Eigen::Quaterniond> q = {Rot(10.0, Eigen::Vector3d::UnitX()),
                                             Rot(50.0, Eigen::Vector3d::UnitY()),
                                             Rot(95.0, Eigen::Vector3d::UnitZ())};
  const std::vector<double> w = {0.9, 0.5, 0.7};
  const Eigen::Quaterniond first = WeightedQuaternionAverage(q, w);
  for (int i = 0; i < 20; ++i) {
    EXPECT_TRUE(WeightedQuaternionAverage(q, w).coeffs().isApprox(first.coeffs(), 1e-15));
  }
}

TEST(QuaternionAverage, RejectsMalformedInput) {
  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  EXPECT_THROW(WeightedQuaternionAverage({}, {}), std::invalid_argument);
  EXPECT_THROW(WeightedQuaternionAverage({q, q}, {1.0}), std::invalid_argument);
  EXPECT_THROW(WeightedQuaternionAverage({q, q}, {0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(WeightedQuaternionAverage({q, q}, {1.0, -1.0}), std::invalid_argument);
}

}  // namespace
}  // namespace the_eyes::fusion
