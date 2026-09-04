// Fusion acceptance tests. AGENTS.md §5 lists these as binary criteria;
// every one of them runs against synthetic dual-view streams with no hardware,
// which is the module's primary development mode.
#include "the_eyes/fusion/fusion.hpp"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "the_eyes/fusion/config.hpp"
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

// A synthetic two-camera rig roughly matching the intended physical layout:
// one camera looking across the table, one looking straight down.
ExtrinsicsSet Rig(double rmse_px = 0.1) {
  ExtrinsicsSet ex;
  ex.world_frame = "table_origin";

  CameraExtrinsics front;
  front.camera_id = "cam_front";
  front.serial = "FRONT001";
  front.T_world_cam.linear() =
      (Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  front.T_world_cam.translation() = Eigen::Vector3d(0.60, 0.0, 0.40);
  front.reprojection_rmse_px = rmse_px;
  ex.cameras.emplace(front.camera_id, front);

  CameraExtrinsics over;
  over.camera_id = "cam_over";
  over.serial = "OVER002";
  over.T_world_cam.linear() =
      Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  over.T_world_cam.translation() = Eigen::Vector3d(0.0, 0.0, 0.90);
  over.reprojection_rmse_px = rmse_px;
  ex.cameras.emplace(over.camera_id, over);
  return ex;
}

// Builds the per-view estimate a camera would report for a given world pose.
PoseEstimate Observe(const ExtrinsicsSet& ex, const std::string& camera_id,
                     const std::string& cls, const Eigen::Isometry3d& world_pose,
                     double score = 0.9, bool symmetric = false) {
  PoseEstimate e;
  e.camera_id = camera_id;
  e.cls = cls;
  e.stamp_ns = 0;
  e.pose_cam = ex.at(camera_id).T_world_cam.inverse() * world_pose;
  e.score = score;
  e.symmetric = symmetric;
  e.extent = Eigen::Vector3d(0.025, 0.025, 0.025);
  return e;
}

FusionConfig Cfg() { return FusionConfig{}; }

// ---------------------------------------------------------------------------
// Acceptance: refuses to start on extrinsics failing the quality gate
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Belief output: the filter's own uncertainty reaches the contract
// ---------------------------------------------------------------------------

TEST(Fusion, EmitsFilterUncertaintyAndVelocityOnTheContract) {
  // The EKF has always computed these; before this they were computed and
  // thrown away every frame. A consumer deciding whether a geometric predicate
  // over this pose is safe needs the sigma, and `confidence` cannot answer that
  // -- it is a detector score, not a distance.
  Fusion fusion(Cfg(), Rig());
  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03}, 15.0);

  const SceneState first = fusion.Process(
      {Observe(Rig(), "cam_front", "cube", truth), Observe(Rig(), "cam_over", "cube", truth)},
      1'000'000'000ULL);
  ASSERT_EQ(first.objects.size(), 1u);
  EXPECT_TRUE(has_uncertainty(first.objects[0]));
  EXPECT_GT(first.objects[0].position_sigma_m, 0.0);
  EXPECT_GT(first.objects[0].orientation_sigma_rad, 0.0);

  // Repeated consistent observations must tighten the belief. A filter whose
  // covariance did not shrink would be reporting a constant, which is worse
  // than reporting nothing because it looks like information.
  SceneState latest = first;
  for (int i = 1; i < 30; ++i) {
    latest = fusion.Process(
        {Observe(Rig(), "cam_front", "cube", truth), Observe(Rig(), "cam_over", "cube", truth)},
        1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL);
  }
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_LT(latest.objects[0].position_sigma_m, first.objects[0].position_sigma_m);
  EXPECT_LT(latest.objects[0].orientation_sigma_rad, first.objects[0].orientation_sigma_rad);

  // A static object on a table: the constant-velocity state must settle near
  // zero rather than chasing measurement noise.
  EXPECT_LT(latest.objects[0].velocity_w.norm(), 0.02);
}

TEST(Fusion, VelocityFollowsAMovingObject) {
  Fusion fusion(Cfg(), Rig());
  const Eigen::Vector3d velocity(0.05, 0.0, 0.0);  // 5 cm/s along +x

  SceneState latest;
  for (int i = 0; i < 60; ++i) {
    const double t = static_cast<double>(i) * 0.033;
    const Eigen::Isometry3d truth = Pose(Eigen::Vector3d(0.0, 0.05, 0.03) + velocity * t);
    latest = fusion.Process(
        {Observe(Rig(), "cam_front", "cube", truth), Observe(Rig(), "cam_over", "cube", truth)},
        1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL);
  }
  ASSERT_EQ(latest.objects.size(), 1u);
  EXPECT_LT((latest.objects[0].velocity_w - velocity).norm(), 0.02);
}

TEST(Fusion, ADefaultObjectStateReportsNoUncertainty) {
  // The sentinel is the whole safety property: a producer that predates the
  // field, or the naive ablation arm that runs no filter, must not read as
  // "certain to zero metres".
  const ObjectState fresh;
  EXPECT_FALSE(has_uncertainty(fresh));
  EXPECT_LT(fresh.position_sigma_m, 0.0);
  EXPECT_LT(fresh.orientation_sigma_rad, 0.0);
}

TEST(Fusion, NaiveModeEmitsTheRawFuseAndNoUncertainty) {
  // The ablation baseline is defined as having no temporal filter, so it must
  // not inherit one through the back door -- neither in the pose it emits nor
  // in the belief it claims.
  FusionConfig cfg = Cfg();
  cfg.mode = FusionMode::kNaive;
  Fusion naive(cfg, Rig());

  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03});
  SceneState scene;
  for (int i = 0; i < 10; ++i) {
    scene = naive.Process(
        {Observe(Rig(), "cam_front", "cube", truth), Observe(Rig(), "cam_over", "cube", truth)},
        1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL);
  }
  ASSERT_EQ(scene.objects.size(), 1u);
  EXPECT_FALSE(has_uncertainty(scene.objects[0]));
  // Noise-free agreeing views: the unweighted mean is exactly the truth, with
  // no filter lag between it and the measurement.
  EXPECT_LT((scene.objects[0].pose_w.translation() - truth.translation()).norm(), 1e-9);
}

TEST(Fusion, NaiveModeBelievesAViewTheGateWouldReject) {
  // This is what the baseline is FOR. Same input, two policies: the gate drops
  // the 8 cm outlier and keeps the good view; the naive mean splits the
  // difference and lands 4 cm from the object.
  const ExtrinsicsSet ex = Rig();
  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03});
  Eigen::Isometry3d wrong = truth;
  wrong.translation() += Eigen::Vector3d(0.08, 0.0, 0.0);
  const std::vector<PoseEstimate> estimates = {
      Observe(ex, "cam_front", "cube", truth, 0.95),
      Observe(ex, "cam_over", "cube", wrong, 0.60)};

  FusionConfig naive_cfg = Cfg();
  naive_cfg.mode = FusionMode::kNaive;
  const SceneState naive = Fusion(naive_cfg, ex).Process(estimates, 1'000'000'000ULL);
  const SceneState gated = Fusion(Cfg(), ex).Process(estimates, 1'000'000'000ULL);

  ASSERT_EQ(naive.objects.size(), 1u);
  ASSERT_EQ(gated.objects.size(), 1u);
  const double naive_error = (naive.objects[0].pose_w.translation() - truth.translation()).norm();
  const double gated_error = (gated.objects[0].pose_w.translation() - truth.translation()).norm();

  EXPECT_NEAR(naive_error, 0.04, 5e-3) << "unweighted mean of a good and an 8 cm-wrong view";
  EXPECT_LT(gated_error, naive_error);
  // The gate must also SAY it dropped a view, not silently improve.
  EXPECT_EQ(gated.objects[0].visible_views, kViewCamFront);
}

TEST(Fusion, RefusesToConstructOnBadCalibration) {
  FusionConfig cfg = Cfg();
  cfg.calibration.max_reprojection_rmse_px = 0.5;
  EXPECT_THROW(Fusion(cfg, Rig(/*rmse_px=*/1.4)), std::runtime_error);
  EXPECT_NO_THROW(Fusion(cfg, Rig(/*rmse_px=*/0.2)));
}

// ---------------------------------------------------------------------------
// Acceptance: single-view objects pass through with the correct visible_views
// ---------------------------------------------------------------------------

TEST(Fusion, SingleViewObjectCarriesOnlyItsOwnViewBit) {
  const ExtrinsicsSet ex = Rig();
  Fusion fusion(Cfg(), ex);

  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03});
  const SceneState scene =
      fusion.Process({Observe(ex, "cam_front", "cube", truth)}, 1'000'000'000ULL);

  ASSERT_EQ(scene.objects.size(), 1u);
  EXPECT_EQ(scene.objects[0].visible_views, kViewCamFront);
  EXPECT_FALSE(scene.objects[0].visible_views & kViewCamOver);
  EXPECT_LT((scene.objects[0].pose_w.translation() - truth.translation()).norm(), 1e-6);
}

TEST(Fusion, DualViewObjectCarriesBothViewBits) {
  const ExtrinsicsSet ex = Rig();
  Fusion fusion(Cfg(), ex);

  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03});
  const SceneState scene = fusion.Process({Observe(ex, "cam_front", "cube", truth),
                                           Observe(ex, "cam_over", "cube", truth)},
                                          1'000'000'000ULL);

  ASSERT_EQ(scene.objects.size(), 1u);
  EXPECT_EQ(scene.objects[0].visible_views, kViewCamFront | kViewCamOver);
}

TEST(Fusion, NeverEmitsAnObjectWithNoViews) {
  const ExtrinsicsSet ex = Rig();
  Fusion fusion(Cfg(), ex);
  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03});

  fusion.Process({Observe(ex, "cam_front", "cube", truth)}, 1'000'000'000ULL);
  // Next frame: nothing observed. The track survives for re-identification but
  // must not be emitted, since visible_views would be 0.
  const SceneState scene = fusion.Process({}, 1'033'000'000ULL);
  EXPECT_TRUE(scene.objects.empty());
  EXPECT_EQ(fusion.active_track_count(), 1u);
  for (const ObjectState& o : scene.objects) EXPECT_NE(o.visible_views, 0u);
}

// ---------------------------------------------------------------------------
// Acceptance: synthetic recovery -- fused error below single-view error
// ---------------------------------------------------------------------------

TEST(Fusion, FusedErrorIsBelowSingleViewErrorUnderGaussianNoise) {
  const ExtrinsicsSet ex = Rig();
  const Eigen::Isometry3d truth = Pose({0.12, -0.04, 0.03}, 22.0, Eigen::Vector3d(0.2, 1, 0.3));

  std::mt19937 rng(20260819);
  std::normal_distribution<double> pos_noise(0.0, 0.008);   // 8 mm
  std::normal_distribution<double> rot_noise(0.0, 4.0 * kPi / 180.0);

  const auto perturb = [&](const Eigen::Isometry3d& p) {
    Eigen::Isometry3d out = p;
    out.translation() += Eigen::Vector3d(pos_noise(rng), pos_noise(rng), pos_noise(rng));
    out.linear() = (so3::Exp({rot_noise(rng), rot_noise(rng), rot_noise(rng)}) *
                    Eigen::Quaterniond(p.rotation()))
                       .toRotationMatrix();
    return out;
  };

  double single_err = 0.0;
  double fused_err = 0.0;
  constexpr int kTrials = 500;
  for (int i = 0; i < kTrials; ++i) {
    const Eigen::Isometry3d a = perturb(truth);
    const Eigen::Isometry3d b = perturb(truth);

    PoseEstimate ea = Observe(ex, "cam_front", "cube", a);
    PoseEstimate eb = Observe(ex, "cam_over", "cube", b);

    // FuseFrame is stateless, so this measures the fuse itself with no EKF
    // smoothing confounding the comparison.
    FusionDiagnostics diag;
    const auto single = FuseFrame({ea}, ex, Cfg(), {}, &diag);
    const auto fused = FuseFrame({ea, eb}, ex, Cfg(), {}, &diag);
    ASSERT_EQ(single.size(), 1u);
    ASSERT_EQ(fused.size(), 1u);

    single_err += (single[0].pose_w.translation() - truth.translation()).norm();
    fused_err += (fused[0].pose_w.translation() - truth.translation()).norm();
  }
  single_err /= kTrials;
  fused_err /= kTrials;

  EXPECT_LT(fused_err, single_err);
  // Averaging two independent equal-variance views should approach a 1/sqrt(2)
  // reduction; anything near parity means the fuse is not actually combining.
  EXPECT_LT(fused_err, 0.85 * single_err);
}

// ---------------------------------------------------------------------------
// Acceptance: disagreement gate rejects an injected outlier view
// ---------------------------------------------------------------------------

TEST(Fusion, ThreeViewGateRejectsAnOutlierWithoutCorruptingTheEstimate) {
  // Three views so there is a majority to appeal to. The third camera is
  // synthetic -- the physical rig has two, but the gate logic must generalise.
  ExtrinsicsSet ex = Rig();
  CameraExtrinsics side;
  side.camera_id = "cam_side";
  side.serial = "SIDE003";
  side.T_world_cam.translation() = Eigen::Vector3d(0.0, 0.6, 0.4);
  side.reprojection_rmse_px = 0.1;
  ex.cameras.emplace(side.camera_id, side);

  FusionConfig cfg = Cfg();
  cfg.view_names = {"cam_front", "cam_over", "cam_side"};

  const Eigen::Isometry3d truth = Pose({0.10, 0.0, 0.03});
  // 10 cm off: inside the association radius, so the cluster forms and the
  // gate is actually reachable -- but well outside the 5 cm agreement gate.
  const Eigen::Isometry3d outlier = Pose({0.10, 0.0, 0.03 + 0.10});

  FusionDiagnostics diag;
  const auto fused = FuseFrame({Observe(ex, "cam_front", "cube", truth, 0.9),
                                Observe(ex, "cam_over", "cube", truth, 0.9),
                                Observe(ex, "cam_side", "cube", outlier, 0.9)},
                               ex, cfg, {}, &diag);

  ASSERT_EQ(fused.size(), 1u);
  EXPECT_EQ(diag.views_rejected, 1);
  EXPECT_TRUE(fused[0].rejected_views & (1u << 2)) << "cam_side should be the rejected bit";
  EXPECT_FALSE(fused[0].visible_views & (1u << 2));
  EXPECT_EQ(fused[0].visible_views, kViewCamFront | kViewCamOver);
  // The surviving estimate must be clean, not dragged toward the outlier.
  EXPECT_LT((fused[0].pose_w.translation() - truth.translation()).norm(), 1e-6);
}

TEST(Fusion, TwoViewConflictKeepsTheHigherScoringViewAndPenalisesConfidence) {
  // With two cameras there is no majority, so the gate cannot identify which
  // view is wrong. It keeps the higher-scoring one and says so.
  const ExtrinsicsSet ex = Rig();
  const Eigen::Isometry3d good = Pose({0.10, 0.0, 0.03});
  const Eigen::Isometry3d bad = Pose({0.10, 0.0, 0.03 + 0.10});

  FusionDiagnostics diag;
  const auto fused = FuseFrame({Observe(ex, "cam_front", "cube", good, 0.95),
                                Observe(ex, "cam_over", "cube", bad, 0.40)},
                               ex, Cfg(), {}, &diag);

  ASSERT_EQ(fused.size(), 1u);
  EXPECT_EQ(diag.two_view_conflicts, 1);
  EXPECT_TRUE(fused[0].two_view_conflict);
  EXPECT_EQ(fused[0].visible_views, kViewCamFront);
  EXPECT_EQ(fused[0].rejected_views, kViewCamOver);
  EXPECT_LT((fused[0].pose_w.translation() - good.translation()).norm(), 1e-6);
  // Confidence is knocked down: we guessed, and the output should admit it.
  EXPECT_LT(fused[0].confidence, 0.95 * Cfg().disagreement_gate
                                            .two_view_conflict_confidence_penalty +
                                    1e-9);
}

TEST(Fusion, AgreeingViewsAreNotGated) {
  const ExtrinsicsSet ex = Rig();
  const Eigen::Isometry3d truth = Pose({0.10, 0.0, 0.03});
  Eigen::Isometry3d nudged = truth;
  nudged.translation().x() += 0.01;  // 1 cm apart, well inside the 8 cm gate

  FusionDiagnostics diag;
  const auto fused = FuseFrame({Observe(ex, "cam_front", "cube", truth),
                                Observe(ex, "cam_over", "cube", nudged)},
                               ex, Cfg(), {}, &diag);
  ASSERT_EQ(fused.size(), 1u);
  EXPECT_EQ(diag.two_view_conflicts, 0);
  EXPECT_EQ(fused[0].rejected_views, 0u);
  EXPECT_EQ(fused[0].visible_views, kViewCamFront | kViewCamOver);
}

// ---------------------------------------------------------------------------
// Acceptance: symmetric objects are not naively orientation-averaged
// ---------------------------------------------------------------------------

TEST(Fusion, SymmetricObjectOrientationIsTakenNotAveraged) {
  const ExtrinsicsSet ex = Rig();
  // A cylinder seen from two views: the reported yaws differ by a symmetry
  // element, not by error. Averaging them lands on a rotation neither view saw.
  const Eigen::Isometry3d a = Pose({0.10, 0.0, 0.03}, 0.0);
  const Eigen::Isometry3d b = Pose({0.10, 0.0, 0.03}, 90.0);

  FusionDiagnostics diag;
  const auto fused = FuseFrame({Observe(ex, "cam_front", "cylinder", a, 0.95, true),
                                Observe(ex, "cam_over", "cylinder", b, 0.60, true)},
                               ex, Cfg(), {}, &diag);

  ASSERT_EQ(fused.size(), 1u);
  EXPECT_TRUE(fused[0].symmetric);
  const Eigen::Quaterniond result(fused[0].pose_w.rotation());
  // Must equal the higher-scoring view exactly, not the 45-degree midpoint.
  EXPECT_NEAR(so3::AngularDistance(result, Eigen::Quaterniond(a.rotation())), 0.0, 1e-9);
  EXPECT_GT(so3::AngularDistance(result, Eigen::Quaterniond(b.rotation())), 80.0 * kPi / 180.0);
  // ...and both views still count as having seen it.
  EXPECT_EQ(fused[0].visible_views, kViewCamFront | kViewCamOver);
}

TEST(Fusion, SymmetricOrientationDisagreementDoesNotTripTheGate) {
  const ExtrinsicsSet ex = Rig();
  const Eigen::Isometry3d a = Pose({0.10, 0.0, 0.03}, 0.0);
  const Eigen::Isometry3d b = Pose({0.10, 0.0, 0.03}, 170.0);  // way past 30 deg

  FusionDiagnostics diag;
  const auto fused = FuseFrame({Observe(ex, "cam_front", "cylinder", a, 0.9, true),
                                Observe(ex, "cam_over", "cylinder", b, 0.9, true)},
                               ex, Cfg(), {}, &diag);
  ASSERT_EQ(fused.size(), 1u);
  EXPECT_EQ(diag.two_view_conflicts, 0) << "symmetry is not disagreement";
  EXPECT_EQ(fused[0].visible_views, kViewCamFront | kViewCamOver);
}

TEST(Fusion, AsymmetricObjectOrientationIsAveraged) {
  const ExtrinsicsSet ex = Rig();
  const Eigen::Isometry3d a = Pose({0.10, 0.0, 0.03}, 0.0);
  const Eigen::Isometry3d b = Pose({0.10, 0.0, 0.03}, 20.0);

  FusionDiagnostics diag;
  const auto fused = FuseFrame({Observe(ex, "cam_front", "cube", a, 0.9, false),
                                Observe(ex, "cam_over", "cube", b, 0.9, false)},
                               ex, Cfg(), {}, &diag);
  ASSERT_EQ(fused.size(), 1u);
  const Eigen::Quaterniond result(fused[0].pose_w.rotation());
  EXPECT_NEAR(so3::AngularDistance(result, Eigen::Quaterniond(a.rotation())) * 180.0 / kPi,
              10.0, 0.5);
}

// ---------------------------------------------------------------------------
// Association and tracking
// ---------------------------------------------------------------------------

TEST(Fusion, TwoInstancesOfOneClassDoNotMergeAcrossViews) {
  const ExtrinsicsSet ex = Rig();
  Fusion fusion(Cfg(), ex);

  const Eigen::Isometry3d left = Pose({-0.20, 0.0, 0.03});
  const Eigen::Isometry3d right = Pose({0.20, 0.0, 0.03});

  const SceneState scene = fusion.Process({Observe(ex, "cam_front", "cube", left),
                                           Observe(ex, "cam_over", "cube", left),
                                           Observe(ex, "cam_front", "cube", right),
                                           Observe(ex, "cam_over", "cube", right)},
                                          1'000'000'000ULL);
  ASSERT_EQ(scene.objects.size(), 2u);
  EXPECT_NE(scene.objects[0].id, scene.objects[1].id);
  for (const ObjectState& o : scene.objects) {
    EXPECT_EQ(o.visible_views, kViewCamFront | kViewCamOver);
  }
}

TEST(Fusion, TwoDetectionsFromOneCameraAreTwoInstances) {
  // A single view cannot see one object twice, so same-camera detections must
  // never land in the same cluster however close they are.
  const ExtrinsicsSet ex = Rig();
  const Eigen::Isometry3d a = Pose({0.10, 0.0, 0.03});
  const Eigen::Isometry3d b = Pose({0.11, 0.0, 0.03});  // 1 cm apart

  FusionDiagnostics diag;
  const auto fused = FuseFrame(
      {Observe(ex, "cam_front", "cube", a), Observe(ex, "cam_front", "cube", b)}, ex, Cfg(), {},
      &diag);
  EXPECT_EQ(fused.size(), 2u);
}

TEST(Fusion, ObjectKeepsItsIdAcrossFramesAndThroughBriefOcclusion) {
  const ExtrinsicsSet ex = Rig();
  Fusion fusion(Cfg(), ex);
  const Eigen::Isometry3d truth = Pose({0.10, 0.0, 0.03});

  const SceneState first =
      fusion.Process({Observe(ex, "cam_front", "cube", truth)}, 1'000'000'000ULL);
  ASSERT_EQ(first.objects.size(), 1u);
  const std::string id = first.objects[0].id;

  // Occluded for a frame, then back.
  fusion.Process({}, 1'033'000'000ULL);
  const SceneState third =
      fusion.Process({Observe(ex, "cam_front", "cube", truth)}, 1'066'000'000ULL);
  ASSERT_EQ(third.objects.size(), 1u);
  EXPECT_EQ(third.objects[0].id, id);
}

TEST(Fusion, TrackIsRetiredAfterMaxCoast) {
  const ExtrinsicsSet ex = Rig();
  Fusion fusion(Cfg(), ex);
  const Eigen::Isometry3d truth = Pose({0.10, 0.0, 0.03});

  fusion.Process({Observe(ex, "cam_front", "cube", truth)}, 1'000'000'000ULL);
  EXPECT_EQ(fusion.active_track_count(), 1u);
  // Default max_coast_s is 1.0; jump two seconds.
  fusion.Process({}, 3'000'000'000ULL);
  EXPECT_EQ(fusion.active_track_count(), 0u);
  EXPECT_EQ(fusion.diagnostics().tracks_retired, 1);
}

TEST(Fusion, OutputIsDeterministicUnderInputPermutation) {
  const ExtrinsicsSet ex = Rig();
  std::vector<PoseEstimate> estimates = {
      Observe(ex, "cam_front", "cube", Pose({-0.20, 0.0, 0.03})),
      Observe(ex, "cam_over", "cube", Pose({-0.20, 0.0, 0.03})),
      Observe(ex, "cam_front", "cube", Pose({0.20, 0.0, 0.03})),
      Observe(ex, "cam_over", "bowl", Pose({0.0, 0.25, 0.04})),
  };

  Fusion reference_fusion(Cfg(), ex);
  const SceneState reference = reference_fusion.Process(estimates, 1'000'000'000ULL);
  ASSERT_FALSE(reference.objects.empty());

  std::mt19937 rng(11);
  for (int trial = 0; trial < 20; ++trial) {
    std::shuffle(estimates.begin(), estimates.end(), rng);
    Fusion fusion(Cfg(), ex);
    const SceneState scene = fusion.Process(estimates, 1'000'000'000ULL);
    ASSERT_EQ(scene.objects.size(), reference.objects.size()) << "trial " << trial;
    for (size_t i = 0; i < scene.objects.size(); ++i) {
      EXPECT_EQ(scene.objects[i].id, reference.objects[i].id);
      EXPECT_EQ(scene.objects[i].visible_views, reference.objects[i].visible_views);
      EXPECT_LT((scene.objects[i].pose_w.translation() -
                 reference.objects[i].pose_w.translation())
                    .norm(),
                1e-12);
    }
  }
}

TEST(Fusion, EstimateFromAnUncalibratedCameraIsDroppedWithAWarning) {
  const ExtrinsicsSet ex = Rig();
  Fusion fusion(Cfg(), ex);

  PoseEstimate rogue = Observe(ex, "cam_front", "cube", Pose({0.1, 0.0, 0.03}));
  rogue.camera_id = "cam_ghost";

  const SceneState scene = fusion.Process({rogue}, 1'000'000'000ULL);
  EXPECT_TRUE(scene.objects.empty());
  ASSERT_FALSE(fusion.diagnostics().warnings.empty());
  EXPECT_NE(fusion.diagnostics().warnings[0].find("cam_ghost"), std::string::npos);
}

TEST(Fusion, ExtentComesFromTheEstimateOrTheRegistry) {
  const ExtrinsicsSet ex = Rig();
  const ExtentRegistry registry = {{"bowl", Eigen::Vector3d(0.07, 0.07, 0.04)}};
  Fusion fusion(Cfg(), ex, registry);

  PoseEstimate no_extent = Observe(ex, "cam_front", "bowl", Pose({0.0, 0.25, 0.04}));
  no_extent.extent = Eigen::Vector3d::Zero();  // clustering path supplied none

  const SceneState scene = fusion.Process({no_extent}, 1'000'000'000ULL);
  ASSERT_EQ(scene.objects.size(), 1u);
  EXPECT_LT((scene.objects[0].extent - Eigen::Vector3d(0.07, 0.07, 0.04)).norm(), 1e-12);
  EXPECT_TRUE(fusion.diagnostics().warnings.empty());
}

TEST(Fusion, ZeroExtentWithNoRegistryEntryIsWarnedAbout) {
  const ExtrinsicsSet ex = Rig();
  Fusion fusion(Cfg(), ex);

  PoseEstimate no_extent = Observe(ex, "cam_front", "mystery", Pose({0.0, 0.25, 0.04}));
  no_extent.extent = Eigen::Vector3d::Zero();

  const SceneState scene = fusion.Process({no_extent}, 1'000'000'000ULL);
  ASSERT_EQ(scene.objects.size(), 1u);
  ASSERT_FALSE(fusion.diagnostics().warnings.empty());
  EXPECT_NE(fusion.diagnostics().warnings[0].find("zero extent"), std::string::npos);
}

TEST(Fusion, RelationsAreLeftEmptyForTheScenegraphModule) {
  const ExtrinsicsSet ex = Rig();
  Fusion fusion(Cfg(), ex);
  const SceneState scene =
      fusion.Process({Observe(ex, "cam_front", "cube", Pose({0.1, 0.0, 0.03}))},
                     1'000'000'000ULL);
  ASSERT_EQ(scene.objects.size(), 1u);
  EXPECT_TRUE(scene.objects[0].relations.empty());
}

}  // namespace
}  // namespace the_eyes::fusion
