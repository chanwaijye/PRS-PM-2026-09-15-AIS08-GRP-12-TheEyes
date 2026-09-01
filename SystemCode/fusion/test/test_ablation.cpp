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
  ASSERT_EQ(result.rows.size(), 3u);

  const AblationRow& front = Row(result, "cam_front");
  const AblationRow& over = Row(result, "cam_over");
  const AblationRow& fused = Row(result, "fused");

  EXPECT_LT(fused.mean_add_s_m, front.mean_add_s_m);
  EXPECT_LT(fused.mean_add_s_m, over.mean_add_s_m);
  // Every configuration sees the object here; the gain is accuracy, not recall.
  EXPECT_EQ(fused.missed, 0);
  EXPECT_EQ(front.missed, 0);
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
  ASSERT_EQ(result.rows.size(), 3u);
  EXPECT_EQ(result.rows[0].label, "cam_front");
  EXPECT_EQ(result.rows[1].label, "cam_over");
  EXPECT_EQ(result.rows[2].label, "fused");

  const std::string table = result.Format();
  EXPECT_NE(table.find("cam_front"), std::string::npos);
  EXPECT_NE(table.find("fused"), std::string::npos);
  EXPECT_NE(table.find("ADD-S"), std::string::npos);
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
