// Ground truth is the yardstick every ADD-S number is read against, so the
// arithmetic that produces it is tested harder than the thing it measures.
#include "the_eyes/runtime/marker_trays.hpp"

#include <unistd.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "the_eyes/runtime/metrics_log.hpp"

using namespace the_eyes;
using namespace the_eyes::runtime;

namespace {

// mkdtemp/mkstemp, matching the rest of the suite -- tmpnam races and warns.
std::string TempDir() {
  char tmpl[] = "/tmp/the_eyes_trays_XXXXXX";
  const char* d = mkdtemp(tmpl);
  if (d == nullptr) throw std::runtime_error("mkdtemp failed");
  return std::string(d);
}

std::string WriteTemp(const std::string& name, const std::string& body) {
  const std::string path = TempDir() + "/" + name;
  std::ofstream f(path);
  f << body;
  return path;
}

Eigen::Isometry3d Pose(double x, double y, double z, double roll_deg, double pitch_deg,
                       double yaw_deg) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  const double d = M_PI / 180.0;
  p.linear() = (Eigen::AngleAxisd(yaw_deg * d, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch_deg * d, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll_deg * d, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  p.translation() = Eigen::Vector3d(x, y, z);
  return p;
}

}  // namespace

TEST(MarkerTrays, LoadsTheShippedConfig) {
  const TrayConfig cfg = LoadTrayConfig(std::string(THE_EYES_CONFIG_DIR) + "/marker_trays.yaml");
  EXPECT_EQ(cfg.dictionary, "DICT_5X5_250");
  EXPECT_NEAR(cfg.marker_length_m, 0.040, 1e-9);
  EXPECT_EQ(cfg.trays.size(), 3u);
  EXPECT_EQ(cfg.trays.at("red_cube").markers.size(), 2u);
  EXPECT_EQ(cfg.trays.at("bowl").markers.size(), 4u);
  // Every id resolves to exactly one class.
  EXPECT_EQ(cfg.cls_of_marker.size(), 8u);
  EXPECT_EQ(cfg.cls_of_marker.at(100), "red_cube");
  EXPECT_EQ(cfg.cls_of_marker.at(107), "bowl");
}

TEST(MarkerTrays, RefusesADuplicateMarkerId) {
  // The failure this guards against is not a crash: it is two objects both
  // receiving plausible ground truth from one observation.
  const std::string path = WriteTemp("dup.yaml", R"(
dictionary: "DICT_5X5_250"
marker_length_m: 0.04
trays:
  red_cube:
    t_tray_object: [0, 0, 0.017]
    markers:
      - {id: 100, t_tray_marker: [-0.057, 0, -0.0004]}
  bowl:
    t_tray_object: [0, 0, 0.0215]
    markers:
      - {id: 100, t_tray_marker: [0.072, 0, -0.0004]}
)");
  EXPECT_THROW(LoadTrayConfig(path), std::runtime_error);
}

TEST(MarkerTrays, RefusesAMarkerLengthThatWouldRescaleEveryPose) {
  const std::string path = WriteTemp("len.yaml", R"(
dictionary: "DICT_5X5_250"
marker_length_m: 0.0
trays:
  red_cube:
    t_tray_object: [0, 0, 0.017]
    markers:
      - {id: 100, t_tray_marker: [0, 0, 0]}
)");
  EXPECT_THROW(LoadTrayConfig(path), std::runtime_error);
}

TEST(MarkerTrays, RefusesATrayWithNoMarkers) {
  const std::string path = WriteTemp("nom.yaml", R"(
dictionary: "DICT_5X5_250"
marker_length_m: 0.04
trays:
  red_cube:
    t_tray_object: [0, 0, 0.017]
    markers: []
)");
  EXPECT_THROW(LoadTrayConfig(path), std::runtime_error);
}

TEST(MarkerTrays, MarkerToObjectIsTheTranslationDifference) {
  const TrayConfig cfg = LoadTrayConfig(std::string(THE_EYES_CONFIG_DIR) + "/marker_trays.yaml");
  const Tray& cube = cfg.trays.at("red_cube");
  // t_tray_object [0,0,0.017] - t_tray_marker [-0.057,0,-0.0004]
  const Eigen::Isometry3d T = MarkerToObject(cube, 100);
  EXPECT_NEAR(T.translation().x(), 0.057, 1e-12);
  EXPECT_NEAR(T.translation().y(), 0.000, 1e-12);
  EXPECT_NEAR(T.translation().z(), 0.0174, 1e-12);
  EXPECT_TRUE(T.linear().isApprox(Eigen::Matrix3d::Identity()));
  // The other marker is the mirror image, so the two must agree on the object.
  const Eigen::Isometry3d T2 = MarkerToObject(cube, 101);
  EXPECT_NEAR(T2.translation().x(), -0.057, 1e-12);
  EXPECT_THROW(MarkerToObject(cube, 999), std::runtime_error);
}

TEST(MarkerTrays, AverageOfNothingReportsNothing) {
  const PoseAverage a = AveragePoses({});
  EXPECT_EQ(a.n, 0);
}

TEST(MarkerTrays, IdenticalPosesAverageToThemselvesWithZeroSpread) {
  const Eigen::Isometry3d p = Pose(0.1, -0.2, 0.03, 5, -10, 30);
  const PoseAverage a = AveragePoses({p, p, p, p});
  EXPECT_EQ(a.n, 4);
  EXPECT_TRUE(a.mean.translation().isApprox(p.translation(), 1e-12));
  EXPECT_NEAR(a.translation_rms_m, 0.0, 1e-12);
  EXPECT_NEAR(a.rotation_rms_deg, 0.0, 1e-9);
  EXPECT_NEAR(a.rotation_max_deg, 0.0, 1e-9);
}

TEST(MarkerTrays, SurvivesQuaternionSignFlips) {
  // q and -q are the same rotation. Averaging quaternion components naively
  // would cancel them toward zero and produce a meaningless mean with a large
  // apparent spread; Markley's method sees one rotation and no scatter.
  Eigen::Isometry3d p = Pose(0.0, 0.0, 0.0, 0, 0, 120);
  Eigen::Quaterniond q(p.linear());
  Eigen::Isometry3d flipped = Eigen::Isometry3d::Identity();
  flipped.linear() = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z()).toRotationMatrix();

  const PoseAverage a = AveragePoses({p, flipped, p, flipped});
  EXPECT_EQ(a.n, 4);
  EXPECT_NEAR(a.rotation_rms_deg, 0.0, 1e-6);
  EXPECT_NEAR(a.rotation_max_deg, 0.0, 1e-6);
  const Eigen::Quaterniond qm(a.mean.linear());
  EXPECT_NEAR(std::abs(qm.dot(q)), 1.0, 1e-9);
}

TEST(MarkerTrays, ScatterIsReportedNotHidden) {
  // Two samples 20 mm apart and 4 degrees apart in yaw: the mean sits between
  // them and the spread reports the half-separation.
  const Eigen::Isometry3d a = Pose(0.00, 0, 0, 0, 0, -2);
  const Eigen::Isometry3d b = Pose(0.02, 0, 0, 0, 0, +2);
  const PoseAverage r = AveragePoses({a, b});
  EXPECT_NEAR(r.mean.translation().x(), 0.01, 1e-12);
  EXPECT_NEAR(r.translation_rms_m, 0.01, 1e-12);
  EXPECT_NEAR(r.translation_max_m, 0.01, 1e-12);
  EXPECT_NEAR(r.rotation_rms_deg, 2.0, 1e-6);
  EXPECT_NEAR(r.rotation_max_deg, 2.0, 1e-6);
}

TEST(MarkerTrays, GroundTruthRoundTripsThroughTheLogAndReplacesOnRewrite) {
  const std::string db = TempDir() + "/gt.sqlite";
  int64_t run = -1;
  {
    MetricsLog log(db);
    RunInfo info;
    info.source = "replay";
    info.detector_backend = "mock:none";
    info.pose_backend = "mock:identity";
    info.config_dir = "cfg";
    run = log.BeginRun(info);

    PipelineResult r;
    r.pair_index = 7;
    r.stamp_ns = 1;
    log.WriteFrame(run, r);

    GroundTruthRow g;
    g.pair_index = 7;
    g.cls = "red_cube";
    g.pose_w = Pose(0.1, 0.2, 0.03, 0, 0, 45);
    g.source = "aruco_tray";
    log.WriteGroundTruth(run, {g});
    // Re-running the tool must correct, not accumulate.
    log.WriteGroundTruth(run, {g});
    log.EndRun(run, "ok", 1);
  }
  const auto frames = LoadAblationFrames(db, run);
  ASSERT_EQ(frames.size(), 1u);
  ASSERT_EQ(frames[0].ground_truth.size(), 1u) << "a second write must replace, not append";
  EXPECT_EQ(frames[0].ground_truth[0].cls, "red_cube");
  EXPECT_NEAR(frames[0].ground_truth[0].pose_w.translation().x(), 0.1, 1e-9);
}

TEST(MarkerTrays, PairIndicesReturnsWhatTheRunActuallyLogged) {
  // Ground truth solved from separate stills has no session to enumerate, so
  // the run's own pair list is the only source of the indices to write to. It
  // must be the LOGGED indices, not 0..n: a run's indices start after the
  // discarded warm-up frames.
  const std::string db = TempDir() + "/pairs.sqlite";
  int64_t run = -1;
  {
    MetricsLog log(db);
    RunInfo info;
    info.source = "live";
    info.detector_backend = "mock:none";
    info.pose_backend = "mock:identity";
    info.config_dir = "cfg";
    run = log.BeginRun(info);
    for (uint64_t p : {30u, 31u, 33u}) {  // 32 dropped, as a real run would
      PipelineResult r;
      r.pair_index = p;
      r.stamp_ns = p;
      log.WriteFrame(run, r);
    }
    log.EndRun(run, "ok", 3);
  }
  const std::vector<uint64_t> pairs = PairIndices(db, run);
  ASSERT_EQ(pairs.size(), 3u);
  EXPECT_EQ(pairs[0], 30u);
  EXPECT_EQ(pairs[1], 31u);
  EXPECT_EQ(pairs[2], 33u);
  EXPECT_TRUE(PairIndices(db, run + 99).empty());
}

TEST(MarkerTrays, GroundTruthWithoutASourceIsRefused) {
  const std::string db = TempDir() + "/gt2.sqlite";
  MetricsLog log(db);
  RunInfo info;
  info.source = "replay";
  info.detector_backend = "mock:none";
  info.pose_backend = "mock:identity";
  info.config_dir = "cfg";
  const int64_t run = log.BeginRun(info);
  GroundTruthRow g;
  g.cls = "red_cube";
  g.source = "";
  EXPECT_THROW(log.WriteGroundTruth(run, {g}), std::runtime_error);
}
