#include "the_eyes/fusion/ekf.hpp"

#include <cmath>
#include <random>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "the_eyes/fusion/so3.hpp"

namespace the_eyes::fusion {
namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d Pose(const Eigen::Vector3d& t, double deg = 0.0,
                       const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ()) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = Eigen::AngleAxisd(deg * kPi / 180.0, axis.normalized()).toRotationMatrix();
  p.translation() = t;
  return p;
}

EkfConfig Cfg() { return EkfConfig{}; }

TEST(Ekf, InitialisesOnTheFirstPose) {
  const Eigen::Isometry3d initial = Pose({0.1, 0.2, 0.3}, 20.0);
  const ObjectEkf ekf(Cfg(), initial);
  EXPECT_LT((ekf.position() - initial.translation()).norm(), 1e-12);
  EXPECT_NEAR(so3::AngularDistance(ekf.orientation(), Eigen::Quaterniond(initial.rotation())),
              0.0, 1e-9);
  EXPECT_LT(ekf.velocity().norm(), 1e-12);
}

TEST(Ekf, PredictIsANoOpForNonPositiveDt) {
  // Two views of the same frame carry identical timestamps, and a jittered
  // stamp can go backwards. Neither may rewind the filter.
  ObjectEkf ekf(Cfg(), Pose({0.0, 0.0, 0.0}));
  ekf.Update(Pose({0.10, 0.0, 0.0}), 1.0);
  const auto before = ekf.covariance();
  const Eigen::Vector3d p_before = ekf.position();

  ekf.Predict(0.0);
  ekf.Predict(-0.5);
  EXPECT_TRUE(ekf.covariance().isApprox(before, 1e-15));
  EXPECT_LT((ekf.position() - p_before).norm(), 1e-15);
}

TEST(Ekf, ConvergesToAStaticMeasurement) {
  ObjectEkf ekf(Cfg(), Pose({0.0, 0.0, 0.0}));
  const Eigen::Isometry3d truth = Pose({0.25, -0.10, 0.05}, 40.0, Eigen::Vector3d(1, 1, 0));

  for (int i = 0; i < 100; ++i) {
    ekf.Predict(1.0 / 30.0);
    ekf.Update(truth, 1.0);
  }
  EXPECT_LT((ekf.position() - truth.translation()).norm(), 1e-3);
  EXPECT_LT(so3::AngularDistance(ekf.orientation(), Eigen::Quaterniond(truth.rotation())),
            1.0 * kPi / 180.0);
}

TEST(Ekf, SmoothsMeasurementNoiseBelowTheRawInput) {
  // This is the whole point of the temporal filter: report less error than the
  // per-frame measurement carries.
  const Eigen::Isometry3d truth = Pose({0.30, 0.10, 0.05}, 15.0);
  std::mt19937 rng(20260819);
  std::normal_distribution<double> noise(0.0, 0.01);  // 10 mm per-axis

  ObjectEkf ekf(Cfg(), truth);
  double raw_err_sum = 0.0;
  double filtered_err_sum = 0.0;
  constexpr int kFrames = 300;
  for (int i = 0; i < kFrames; ++i) {
    Eigen::Isometry3d measured = truth;
    measured.translation() += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));

    ekf.Predict(1.0 / 30.0);
    ekf.Update(measured, 1.0);

    raw_err_sum += (measured.translation() - truth.translation()).norm();
    filtered_err_sum += (ekf.position() - truth.translation()).norm();
  }
  EXPECT_LT(filtered_err_sum / kFrames, raw_err_sum / kFrames);
}

TEST(Ekf, TracksConstantVelocityMotion) {
  const Eigen::Vector3d velocity(0.20, 0.0, 0.0);  // 0.2 m/s along +x
  const double dt = 1.0 / 30.0;

  ObjectEkf ekf(Cfg(), Pose({0.0, 0.0, 0.0}));
  for (int i = 1; i <= 120; ++i) {
    ekf.Predict(dt);
    ekf.Update(Pose(velocity * (i * dt)), 1.0);
  }
  // The constant-velocity model should have learned the rate, not just chased.
  EXPECT_LT((ekf.velocity() - velocity).norm(), 0.05);
}

TEST(Ekf, CovarianceStaysSymmetricAndPositiveDefinite) {
  // Joseph-form update exists to keep this true over a long run; the textbook
  // (I-KH)P form drifts asymmetric and eventually diverges silently.
  std::mt19937 rng(3);
  std::normal_distribution<double> noise(0.0, 0.02);

  ObjectEkf ekf(Cfg(), Pose({0.0, 0.0, 0.0}));
  for (int i = 0; i < 2000; ++i) {
    ekf.Predict(1.0 / 30.0);
    Eigen::Isometry3d m = Pose({0.1, 0.1, 0.1});
    m.translation() += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
    ekf.Update(m, 0.8);

    const auto& p = ekf.covariance();
    ASSERT_TRUE(p.allFinite()) << "iteration " << i;
    ASSERT_LT((p - p.transpose()).norm(), 1e-12) << "iteration " << i;
  }
  const Eigen::SelfAdjointEigenSolver<ObjectEkf::StateMatrix> solver(ekf.covariance());
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(Ekf, LowConfidenceMeasurementMovesTheFilterLess) {
  const Eigen::Isometry3d start = Pose({0.0, 0.0, 0.0});
  const Eigen::Isometry3d jump = Pose({0.20, 0.0, 0.0});

  ObjectEkf confident(Cfg(), start);
  ObjectEkf unsure(Cfg(), start);
  // Settle both so the initial covariance is not doing the work.
  for (int i = 0; i < 30; ++i) {
    confident.Predict(1.0 / 30.0);
    confident.Update(start, 1.0);
    unsure.Predict(1.0 / 30.0);
    unsure.Update(start, 1.0);
  }
  confident.Predict(1.0 / 30.0);
  confident.Update(jump, 1.0);
  unsure.Predict(1.0 / 30.0);
  unsure.Update(jump, 0.05);

  EXPECT_GT(confident.position().x(), unsure.position().x());
}

TEST(Ekf, UncertaintyGrowsWhileCoastingAndShrinksOnUpdate) {
  ObjectEkf ekf(Cfg(), Pose({0.0, 0.0, 0.0}));
  ekf.Update(Pose({0.0, 0.0, 0.0}), 1.0);
  const double settled = ekf.position_sigma_m();

  ekf.Predict(0.5);  // half a second with no measurement
  const double coasted = ekf.position_sigma_m();
  EXPECT_GT(coasted, settled);

  ekf.Update(Pose({0.0, 0.0, 0.0}), 1.0);
  EXPECT_LT(ekf.position_sigma_m(), coasted);
}

}  // namespace
}  // namespace the_eyes::fusion
