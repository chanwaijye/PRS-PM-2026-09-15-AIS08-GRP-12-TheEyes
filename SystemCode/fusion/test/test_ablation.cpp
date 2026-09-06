// The occlusion ablation harness. CLAUDE.md: this result is never cut -- it is
// the project's entire technical claim -- so the harness itself is tested.
#include "the_eyes/fusion/metrics.hpp"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "the_eyes/fusion/so3.hpp"

namespace the_eyes::fusion {
namespace {

constexpr double kPi = 3.14159265358979323846;
const Eigen::Vector3d kHalfExtent(0.025, 0.025, 0.025);

Eigen::Isometry3d Pose(const Eigen::Vector3d& t, double deg = 0.0) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = Eigen::AngleAxisd(deg * kPi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p.translation() = t;
  return p;
}

ExtrinsicsSet Rig() {
  ExtrinsicsSet ex;
  ex.world_frame = "table_origin";
  CameraExtrinsics front;
  front.camera_id = "cam_front";
  front.serial = "FRONT001";
  front.T_world_cam.translation() = Eigen::Vector3d(0.60, 0.0, 0.40);
  front.reprojection_rmse_px = 0.1;
  ex.cameras.emplace(front.camera_id, front);

  CameraExtrinsics over;
  over.camera_id = "cam_over";
  over.serial = "OVER002";
  over.T_world_cam.linear() =
      Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  over.T_world_cam.translation() = Eigen::Vector3d(0.0, 0.0, 0.90);
  over.reprojection_rmse_px = 0.1;
  ex.cameras.emplace(over.camera_id, over);
  return ex;
}

PoseEstimate Observe(const ExtrinsicsSet& ex, const std::string& camera_id,
                     const std::string& cls, const Eigen::Isometry3d& world_pose,
                     double score = 0.9) {
  PoseEstimate e;
  e.camera_id = camera_id;
  e.cls = cls;
  e.pose_cam = ex.at(camera_id).T_world_cam.inverse() * world_pose;
  e.score = score;
  e.extent = kHalfExtent;
  return e;
}

ModelRegistry Models() {
  ModelRegistry m;
  m["cube"] = ModelEntry{BoxModelPoints(kHalfExtent, 4), BoxDiameter(kHalfExtent)};
  return m;
}

const AblationRow& Row(const AblationResult& r, const std::string& label) {
  for (const AblationRow& row : r.rows) {
    if (row.label == label) return row;
  }
  throw std::runtime_error("no ablation row labelled '" + label + "'");
}

// --------------------------------------------------------------------------
// Metric primitives
// --------------------------------------------------------------------------

TEST(Metrics, AddAndAddSAreZeroForAnExactPose) {
  const ModelPoints model = BoxModelPoints(kHalfExtent, 4);
  const Eigen::Isometry3d p = Pose({0.1, 0.2, 0.03}, 33.0);
  EXPECT_NEAR(Add(p, p, model), 0.0, 1e-12);
  EXPECT_NEAR(AddS(p, p, model), 0.0, 1e-12);
}

TEST(Metrics, PureTranslationErrorEqualsTheOffset) {
  const ModelPoints model = BoxModelPoints(kHalfExtent, 4);
  const Eigen::Isometry3d gt = Pose({0.0, 0.0, 0.0});
  const Eigen::Isometry3d est = Pose({0.01, 0.0, 0.0});
  EXPECT_NEAR(Add(est, gt, model), 0.01, 1e-12);
}

TEST(Metrics, AddSForgivesASymmetryWhereAddDoesNot) {
  // A cube rotated 90 degrees about z is the same solid. ADD punishes it
  // because corresponding points moved; ADD-S does not, because the nearest
  // point is still on the surface. This is exactly why the symmetric flag
  // exists in the mesh registry.
  const ModelPoints model = BoxModelPoints(kHalfExtent, 4);
  const Eigen::Isometry3d gt = Pose({0.0, 0.0, 0.0}, 0.0);
  const Eigen::Isometry3d rotated = Pose({0.0, 0.0, 0.0}, 90.0);

  EXPECT_GT(Add(rotated, gt, model), 0.01);
  EXPECT_LT(AddS(rotated, gt, model), 1e-9);
}

TEST(Metrics, EmptyModelThrowsRatherThanReturningZeroError) {
  const Eigen::Isometry3d p = Pose({0.0, 0.0, 0.0});
  EXPECT_THROW(Add(p, p, {}), std::invalid_argument);
  EXPECT_THROW(AddS(p, p, {}), std::invalid_argument);
}

TEST(Metrics, BoxModelPointsLieOnTheSurface) {
  const ModelPoints model = BoxModelPoints(kHalfExtent, 4);
  ASSERT_FALSE(model.empty());
  for (const Eigen::Vector3d& p : model) {
    const Eigen::Vector3d d = p.cwiseAbs() - kHalfExtent;
    EXPECT_LT(d.maxCoeff(), 1e-12) << "point outside the box";
    EXPECT_GT(d.maxCoeff(), -1e-12) << "point strictly inside, not on the surface";
  }
}

// --------------------------------------------------------------------------
// The headline result
// --------------------------------------------------------------------------

TEST(Ablation, FusedBeatsEitherSingleViewUnderSymmetricNoise) {
  const ExtrinsicsSet ex = Rig();
  const FusionConfig cfg;
  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03}, 15.0);

  std::mt19937 rng(20260819);
  std::normal_distribution<double> noise(0.0, 0.010);  // 10 mm per axis, per view

  std::vector<AblationFrame> frames;
  for (int i = 0; i < 120; ++i) {
    AblationFrame f;
    f.stamp_ns = static_cast<uint64_t>(i) * 33'000'000ULL + 1'000'000'000ULL;
    for (const std::string& cam : {"cam_front", "cam_over"}) {
      Eigen::Isometry3d measured = truth;
      measured.translation() += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
      f.estimates.push_back(Observe(ex, cam, "cube", measured));
    }
    f.ground_truth.push_back(GroundTruthObject{"cube", truth});
    frames.push_back(std::move(f));
  }

  const AblationResult result = RunOcclusionAblation(frames, ex, cfg, {}, Models());
  ASSERT_EQ(result.rows.size(), 8u);

  // Within each filter tier, two views beat one: averaging two independent
  // noisy measurements halves the variance whoever writes the average.
  EXPECT_LT(Row(result, "naive").mean_add_s_m, Row(result, "cam_front").mean_add_s_m);
  EXPECT_LT(Row(result, "naive").mean_add_s_m, Row(result, "cam_over").mean_add_s_m);
  EXPECT_LT(Row(result, "fused").mean_add_s_m, Row(result, "cam_front+ekf").mean_add_s_m);
  EXPECT_LT(Row(result, "fused").mean_add_s_m, Row(result, "cam_over+ekf").mean_add_s_m);
  // Every configuration sees the object here; the gain is accuracy, not recall.
  for (const AblationRow& row : result.rows) EXPECT_EQ(row.missed, 0) << row.label;
}

TEST(Ablation, TheFilterAndThePolicyAreNotConfounded) {
  // The 2026-09-05 review finding, made structural. Under symmetric Gaussian
  // noise with no outliers the gate has nothing to reject and equal scores make
  // the weighted mean the plain mean, so the POLICY contributes nothing -- and
  // the harness must say so at BOTH filter settings. Meanwhile the EKF, which
  // is a running average over frames of a static object, contributes a lot.
  //
  // Before the filter became its own switch, `naive` ran without the EKF and
  // `fused` ran with it, so this exact input reported a large fused-minus-naive
  // gap that was credited to the policy. That is what this test forbids.
  const ExtrinsicsSet ex = Rig();
  const FusionConfig cfg;
  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03}, 15.0);

  std::mt19937 rng(20260905);
  std::normal_distribution<double> noise(0.0, 0.010);

  std::vector<AblationFrame> frames;
  for (int i = 0; i < 120; ++i) {
    AblationFrame f;
    f.stamp_ns = static_cast<uint64_t>(i) * 33'000'000ULL + 1'000'000'000ULL;
    for (const std::string& cam : {"cam_front", "cam_over"}) {
      Eigen::Isometry3d measured = truth;
      measured.translation() += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
      f.estimates.push_back(Observe(ex, cam, "cube", measured));
    }
    f.ground_truth.push_back(GroundTruthObject{"cube", truth});
    frames.push_back(std::move(f));
  }

  const AblationResult result = RunOcclusionAblation(frames, ex, cfg, {}, Models());
  const double naive_raw = Row(result, "naive").mean_add_s_m;
  const double gated_raw = Row(result, "gated").mean_add_s_m;
  const double naive_ekf = Row(result, "naive+ekf").mean_add_s_m;
  const double fused = Row(result, "fused").mean_add_s_m;

  // Policy delta is negligible at each filter setting: nothing for the policy
  // to do. Not exactly zero -- with 10 mm/axis per view the two views' 3D
  // disagreement occasionally exceeds the 5 cm gate on noise alone, and the
  // two-view tie-break then keeps one view instead of the mean. That is a few % of
  // the error here and is itself a property worth knowing about the gate.
  EXPECT_NEAR(gated_raw, naive_raw, 0.05 * naive_raw);
  EXPECT_NEAR(fused, naive_ekf, 0.05 * naive_ekf);
  // Filter delta is large at each policy setting. Not the 1/sqrt(120) of a
  // true running mean: the configured process noise gives the EKF a finite
  // memory, and it settles near 0.6x on this input. 0.8x is the bound because
  // the point is "clearly non-zero", not a model of the filter's steady state.
  EXPECT_LT(naive_ekf, 0.8 * naive_raw);
  EXPECT_LT(fused, 0.8 * gated_raw);
  // And so the old headline gap -- fused vs the unfiltered naive -- is real but
  // belongs to the filter column, not the policy column.
  EXPECT_LT(fused, 0.8 * naive_raw);
}

TEST(Ablation, FusionRecoversRecallWhenOneViewIsOccluded) {
  // The occlusion case the two-camera rig exists for: cam_over cannot see the
  // object at all for a stretch of frames. Single-view cam_over reports misses;
  // the fused configuration does not.
  const ExtrinsicsSet ex = Rig();
  const FusionConfig cfg;
  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03});

  std::vector<AblationFrame> frames;
  for (int i = 0; i < 60; ++i) {
    AblationFrame f;
    f.stamp_ns = static_cast<uint64_t>(i) * 33'000'000ULL + 1'000'000'000ULL;
    f.estimates.push_back(Observe(ex, "cam_front", "cube", truth));
    const bool occluded_from_over = (i % 3 != 0);
    if (!occluded_from_over) f.estimates.push_back(Observe(ex, "cam_over", "cube", truth));
    f.ground_truth.push_back(GroundTruthObject{"cube", truth});
    frames.push_back(std::move(f));
  }

  const AblationResult result = RunOcclusionAblation(frames, ex, cfg, {}, Models());
  const AblationRow& over = Row(result, "cam_over");
  const AblationRow& fused = Row(result, "fused");

  EXPECT_GT(over.missed, 0);
  EXPECT_EQ(fused.missed, 0);
  EXPECT_GT(fused.recall_at_0p1_diameter, over.recall_at_0p1_diameter);
}

TEST(Ablation, RowsCoverEveryViewPlusFusedAndFormatCleanly) {
  const ExtrinsicsSet ex = Rig();
  const FusionConfig cfg;
  const Eigen::Isometry3d truth = Pose({0.10, 0.0, 0.03});

  AblationFrame f;
  f.stamp_ns = 1'000'000'000ULL;
  f.estimates = {Observe(ex, "cam_front", "cube", truth), Observe(ex, "cam_over", "cube", truth)};
  f.ground_truth = {GroundTruthObject{"cube", truth}};

  const AblationResult result = RunOcclusionAblation({f}, ex, cfg, {}, Models());
  ASSERT_EQ(result.rows.size(), 8u);
  // Raw tier, then EKF tier; within each, single views, then naive, then the
  // policy row, so the table is read top to bottom as "what each thing buys".
  const std::vector<std::string> expected = {"cam_front", "cam_over",         "naive",
                                             "gated",     "cam_front+ekf",    "cam_over+ekf",
                                             "naive+ekf", "fused"};
  for (size_t i = 0; i < expected.size(); ++i) EXPECT_EQ(result.rows[i].label, expected[i]);
  for (size_t i = 0; i < 4; ++i) EXPECT_FALSE(result.rows[i].temporal_filter) << i;
  for (size_t i = 4; i < 8; ++i) EXPECT_TRUE(result.rows[i].temporal_filter) << i;
  EXPECT_EQ(Row(result, "naive").mode, FusionMode::kNaive);
  EXPECT_EQ(Row(result, "naive+ekf").mode, FusionMode::kNaive);
  EXPECT_EQ(Row(result, "gated").mode, FusionMode::kGated);
  EXPECT_EQ(Row(result, "fused").mode, FusionMode::kGated);
  EXPECT_EQ(Row(result, "cam_front").views, std::vector<std::string>{"cam_front"});
  EXPECT_EQ(Row(result, "fused").views.size(), 2u);

  const std::string table = result.Format();
  EXPECT_NE(table.find("cam_front"), std::string::npos);
  EXPECT_NE(table.find("naive"), std::string::npos);
  EXPECT_NE(table.find("gated"), std::string::npos);
  EXPECT_NE(table.find("fused"), std::string::npos);
  EXPECT_NE(table.find("ADD-S"), std::string::npos);
}

// --------------------------------------------------------------------------
// The baseline arm -- what makes the headline claim falsifiable
// --------------------------------------------------------------------------

TEST(Ablation, GatedFusionBeatsTheNaiveBaselineWhenOneViewIsAnOutlier) {
  // The scenario the disagreement gate exists for, and the only kind of
  // scenario in which this module can out-perform a two-line average: one view
  // is confidently wrong. cam_over reports the object 8 cm from where it is,
  // which is past the 5 cm gate. The gate drops it; the naive baseline averages
  // it in and lands halfway to nowhere.
  //
  // If this test ever passes only marginally on real captures, that is the
  // finding -- it means FoundationPose rarely produces gate-tripping outliers
  // at this rig geometry, which is worth reporting rather than hiding.
  const ExtrinsicsSet ex = Rig();
  const FusionConfig cfg;
  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03});
  Eigen::Isometry3d wrong = truth;
  wrong.translation() += Eigen::Vector3d(0.08, 0.0, 0.0);

  std::vector<AblationFrame> frames;
  for (int i = 0; i < 40; ++i) {
    AblationFrame f;
    f.stamp_ns = static_cast<uint64_t>(i) * 33'000'000ULL + 1'000'000'000ULL;
    // cam_front is right and scores higher; cam_over is the outlier.
    f.estimates.push_back(Observe(ex, "cam_front", "cube", truth, 0.95));
    f.estimates.push_back(Observe(ex, "cam_over", "cube", wrong, 0.60));
    f.ground_truth.push_back(GroundTruthObject{"cube", truth});
    frames.push_back(std::move(f));
  }

  const AblationResult result = RunOcclusionAblation(frames, ex, cfg, {}, Models());

  // The policy's contribution, measured with the filter held fixed -- both
  // ways. Comparing `fused` (EKF) against `naive` (raw) would fold the filter
  // into the policy's credit; see TheFilterAndThePolicyAreNotConfounded.
  const AblationRow& naive = Row(result, "naive");
  const AblationRow& gated = Row(result, "gated");
  EXPECT_LT(gated.mean_add_s_m, naive.mean_add_s_m)
      << "the disagreement gate must buy something over an unweighted mean, or "
         "the module's headline claim is just 'two cameras beat one'";
  EXPECT_GT(gated.recall_at_0p1_diameter, naive.recall_at_0p1_diameter);

  const AblationRow& naive_ekf = Row(result, "naive+ekf");
  const AblationRow& fused = Row(result, "fused");
  EXPECT_LT(fused.mean_add_s_m, naive_ekf.mean_add_s_m)
      << "a constant 4 cm bias is not noise: the filter cannot average it away, "
         "so the gate's gain must survive the EKF";
  EXPECT_GT(fused.recall_at_0p1_diameter, naive_ekf.recall_at_0p1_diameter);
}

TEST(Ablation, NaiveBaselineStillSeesWhatBothViewsSee) {
  // The baseline is a baseline, not a strawman: with both views agreeing it
  // must recover the object just as well as the full policy. A naive arm that
  // failed here would flatter the fused arm for the wrong reason.
  const ExtrinsicsSet ex = Rig();
  const Eigen::Isometry3d truth = Pose({0.10, 0.05, 0.03});

  std::vector<AblationFrame> frames;
  for (int i = 0; i < 20; ++i) {
    AblationFrame f;
    f.stamp_ns = static_cast<uint64_t>(i) * 33'000'000ULL + 1'000'000'000ULL;
    f.estimates = {Observe(ex, "cam_front", "cube", truth),
                   Observe(ex, "cam_over", "cube", truth)};
    f.ground_truth = {GroundTruthObject{"cube", truth}};
    frames.push_back(std::move(f));
  }

  const AblationResult result = RunOcclusionAblation(frames, ex, FusionConfig{}, {}, Models());
  const AblationRow& naive = Row(result, "naive");
  EXPECT_EQ(naive.missed, 0);
  EXPECT_EQ(naive.matched, 20);
  EXPECT_LT(naive.mean_add_s_m, 1e-6) << "noise-free input: the mean is exact";
}

TEST(Ablation, MissingModelRegistryEntryThrows) {
  // ADD-S cannot be normalised without diameter_m, so this must fail loudly
  // rather than reporting an unnormalised number into the report.
  const ExtrinsicsSet ex = Rig();
  const Eigen::Isometry3d truth = Pose({0.10, 0.0, 0.03});

  AblationFrame f;
  f.stamp_ns = 1'000'000'000ULL;
  f.estimates = {Observe(ex, "cam_front", "cube", truth)};
  f.ground_truth = {GroundTruthObject{"cube", truth}};

  EXPECT_THROW(RunOcclusionAblation({f}, ex, FusionConfig{}, {}, ModelRegistry{}),
               std::runtime_error);
}

TEST(Ablation, ConfigurationsDoNotShareTemporalState) {
  // Each configuration must get a fresh Fusion, or the EKF from the previous
  // run would leak into the next and the comparison would be meaningless.
  const ExtrinsicsSet ex = Rig();
  const Eigen::Isometry3d truth = Pose({0.10, 0.0, 0.03});

  std::vector<AblationFrame> frames;
  for (int i = 0; i < 10; ++i) {
    AblationFrame f;
    f.stamp_ns = static_cast<uint64_t>(i) * 33'000'000ULL + 1'000'000'000ULL;
    f.estimates = {Observe(ex, "cam_front", "cube", truth),
                   Observe(ex, "cam_over", "cube", truth)};
    f.ground_truth = {GroundTruthObject{"cube", truth}};
    frames.push_back(std::move(f));
  }

  const AblationResult a = RunOcclusionAblation(frames, ex, FusionConfig{}, {}, Models());
  const AblationResult b = RunOcclusionAblation(frames, ex, FusionConfig{}, {}, Models());
  ASSERT_EQ(a.rows.size(), b.rows.size());
  for (size_t i = 0; i < a.rows.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.rows[i].mean_add_s_m, b.rows[i].mean_add_s_m) << "row " << i;
    EXPECT_EQ(a.rows[i].matched, b.rows[i].matched);
  }
}

}  // namespace
}  // namespace the_eyes::fusion
