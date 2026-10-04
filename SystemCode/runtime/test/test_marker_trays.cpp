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
  // The table-plane solve stands on these three numbers.
  EXPECT_NEAR(cfg.trays.at("red_cube").plate_thickness_m, 0.0105, 1e-9);
  EXPECT_NEAR(cfg.trays.at("mustard_bottle").plate_thickness_m, 0.0145, 1e-9);
  EXPECT_NEAR(cfg.trays.at("bowl").plate_thickness_m, 0.0085, 1e-9);
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

namespace {

// A cluster shaped like the real 2026-09-20 tray stills: per-marker samples up
// to ~17 mm and ~2.8 deg from their mean (bowl, cam_over, the worst class).
std::vector<Eigen::Isometry3d> RealisticCluster() {
  std::vector<Eigen::Isometry3d> out;
  const double dx[] = {-0.012, -0.006, 0.0, 0.004, 0.009, 0.015, -0.003, 0.007};
  const double dy[] = {0.005, -0.008, 0.002, 0.010, -0.004, 0.001, -0.011, 0.006};
  const double yaw[] = {-1.4, 0.6, 2.2, -0.8, 1.1, -2.1, 0.3, 1.7};
  for (int i = 0; i < 8; ++i) {
    out.push_back(Pose(-0.16 + dx[i], -0.04 + dy[i], 0.030, yaw[i] * 0.5, -yaw[i] * 0.3, 30 + yaw[i]));
  }
  return out;
}

}  // namespace

TEST(MarkerTrays, ScreenKeepsAnHonestClusterWhole) {
  // Everything the real stills produced must pass: the gate is for failures an
  // order of magnitude outside the measured scatter, not for trimming noise.
  const OutlierScreen s = ScreenPoseOutliers(RealisticCluster(), 10.0, 0.040);
  EXPECT_EQ(s.rejected, 0);
  EXPECT_GE(s.medoid, 0);
}

TEST(MarkerTrays, ScreenDropsAPlanarFlipThatWouldDragTheMean) {
  // A marker that resolved to its other planar solution: same place, normal
  // tilted ~120 deg. Listed FIRST so the medoid cannot come from input order.
  std::vector<Eigen::Isometry3d> poses;
  poses.push_back(Pose(-0.16, -0.04, 0.030, 120, 0, 30));
  poses.push_back(Pose(-0.15, -0.05, 0.035, 115, 5, 32));
  for (const auto& p : RealisticCluster()) poses.push_back(p);

  const OutlierScreen s = ScreenPoseOutliers(poses, 10.0, 0.040);
  EXPECT_EQ(s.rejected, 2);
  EXPECT_FALSE(s.keep[0]);
  EXPECT_FALSE(s.keep[1]);
  for (size_t i = 2; i < poses.size(); ++i) EXPECT_TRUE(s.keep[i]) << i;
  EXPECT_GE(s.medoid, 2) << "the medoid must come from the majority";

  // And the reason it matters: averaged in, the two flips move the mean
  // rotation by degrees; screened out, they move nothing.
  std::vector<Eigen::Isometry3d> kept;
  for (size_t i = 0; i < poses.size(); ++i) if (s.keep[i]) kept.push_back(poses[i]);
  const PoseAverage clean = AveragePoses(RealisticCluster());
  const PoseAverage dirty = AveragePoses(poses);
  const PoseAverage screened = AveragePoses(kept);
  EXPECT_GT(RotationAngleDeg(dirty.mean.linear(), clean.mean.linear()), 5.0);
  EXPECT_NEAR(RotationAngleDeg(screened.mean.linear(), clean.mean.linear()), 0.0, 1e-9);
}

TEST(MarkerTrays, ScreenDropsACardInTheWrongRecess) {
  // Right orientation, wrong place: the card sits in the recess 144 mm along,
  // so its implied object centre is 144 mm off. Rotation alone would pass it.
  std::vector<Eigen::Isometry3d> poses = RealisticCluster();
  poses.push_back(Pose(-0.16 + 0.144, -0.04, 0.030, 0, 0, 30));
  const OutlierScreen s = ScreenPoseOutliers(poses, 10.0, 0.040);
  EXPECT_EQ(s.rejected, 1);
  EXPECT_FALSE(s.keep.back());
}

TEST(MarkerTrays, ScreenOfNothingIsEmpty) {
  const OutlierScreen s = ScreenPoseOutliers({}, 10.0, 0.040);
  EXPECT_EQ(s.medoid, -1);
  EXPECT_EQ(s.rejected, 0);
  EXPECT_TRUE(s.keep.empty());
  EXPECT_THROW(ScreenPoseOutliers({}, 0.0, 0.040), std::invalid_argument);
}

namespace {

// A camera at `c` looking at `target`, image y pointing down.
Eigen::Isometry3d LookAt(const Eigen::Vector3d& c, const Eigen::Vector3d& target) {
  const Eigen::Vector3d z = (target - c).normalized();
  const Eigen::Vector3d x = z.cross(Eigen::Vector3d::UnitZ()).normalized();
  const Eigen::Vector3d y = z.cross(x);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear().col(0) = x;
  T.linear().col(1) = y;
  T.linear().col(2) = z;
  T.translation() = c;
  return T;
}

Eigen::Matrix3d PlaneTestK() {
  Eigen::Matrix3d K;
  K << 1365.0, 0, 970.0, 0, 1365.0, 546.0, 0, 0, 1;
  return K;
}

Eigen::Vector2d Project(const Eigen::Vector3d& pw, const Eigen::Matrix3d& K,
                        const Eigen::Isometry3d& T_world_cam) {
  const Eigen::Vector3d pc = T_world_cam.inverse() * pw;
  return (K * pc).hnormalized();
}

}  // namespace

TEST(MarkerTrays, PixelOnWorldPlaneInvertsProjection) {
  const Eigen::Isometry3d T = LookAt({-0.41, -0.37, 0.40}, {0, 0, 0});
  const Eigen::Matrix3d K = PlaneTestK();
  for (const Eigen::Vector3d& p : {Eigen::Vector3d(0.03, -0.02, 0.0101),
                                   Eigen::Vector3d(-0.20, 0.15, 0.0141),
                                   Eigen::Vector3d(0.22, -0.18, 0.0)}) {
    const Eigen::Vector3d back = PixelOnWorldPlane(Project(p, K, T), K, T, p.z());
    EXPECT_TRUE(back.isApprox(p, 1e-9)) << back.transpose() << " vs " << p.transpose();
  }
  // A pixel above the horizon never meets the table.
  EXPECT_THROW(PixelOnWorldPlane({970.0, -4000.0}, K, T, 0.0), std::runtime_error);
}

TEST(MarkerTrays, AWrongPlaneHeightMovesThePointAlongTheView) {
  // The solve's one physical assumption, priced: at ~36 deg of elevation a
  // millimetre of height error is about 1.4 mm on the table, directed away
  // from the camera. This is why the plate thickness has to be right.
  const Eigen::Isometry3d T = LookAt({-0.41, -0.37, 0.40}, {0, 0, 0});
  const Eigen::Matrix3d K = PlaneTestK();
  const Eigen::Vector3d p(0.0, 0.0, 0.010);
  const Eigen::Vector3d wrong = PixelOnWorldPlane(Project(p, K, T), K, T, 0.009);
  const double shift = (wrong - p).head<2>().norm();
  EXPECT_GT(shift, 0.0012);
  EXPECT_LT(shift, 0.0016);
}

TEST(MarkerTrays, PlanarFitRecoversYawAndTranslationExactly) {
  const auto m = MarkerCornersXY(0.040);
  EXPECT_TRUE(m[0].isApprox(Eigen::Vector2d(-0.020, 0.020)));  // top-left first
  EXPECT_TRUE(m[2].isApprox(Eigen::Vector2d(0.020, -0.020)));
  const Eigen::Rotation2Dd R(33.0 * M_PI / 180.0);
  const Eigen::Vector2d t(-0.048, 0.025);
  std::vector<Eigen::Vector2d> model(m.begin(), m.end()), world;
  for (const auto& p : model) world.push_back(R * p + t);
  const PlanarFit f = FitPlanarRigid(model, world);
  EXPECT_NEAR(f.yaw_rad, 33.0 * M_PI / 180.0, 1e-12);
  EXPECT_TRUE(f.t.isApprox(t, 1e-12));
  EXPECT_NEAR(f.rms_m, 0.0, 1e-12);
  EXPECT_EQ(f.n, 4);

  const Eigen::Isometry3d T = PoseOnPlane(f, 0.0101);
  EXPECT_NEAR(T.translation().z(), 0.0101, 1e-12);
  EXPECT_NEAR(RotationAngleDeg(T.linear(), Pose(0, 0, 0, 0, 0, 33).linear()), 0.0, 1e-9);
}

TEST(MarkerTrays, PlanarFitReportsALayoutThatDoesNotFit) {
  // One corner 2 mm out: the fit cannot absorb it rigidly, and says so.
  const auto m = MarkerCornersXY(0.040);
  std::vector<Eigen::Vector2d> model(m.begin(), m.end()), world(m.begin(), m.end());
  world[1] += Eigen::Vector2d(0.002, 0.0);
  const PlanarFit f = FitPlanarRigid(model, world);
  EXPECT_GT(f.rms_m, 0.0005);
  EXPECT_LT(f.rms_m, 0.0020);
  EXPECT_THROW(FitPlanarRigid({model[0]}, {world[0]}), std::invalid_argument);
  EXPECT_THROW(FitPlanarRigid(model, {world[0], world[1]}), std::invalid_argument);
}

TEST(MarkerTrays, RotationAngleIsShortestArc) {
  EXPECT_NEAR(RotationAngleDeg(Pose(0, 0, 0, 0, 0, 10).linear(), Pose(0, 0, 0, 0, 0, -20).linear()),
              30.0, 1e-9);
  EXPECT_NEAR(RotationAngleDeg(Pose(0, 0, 0, 0, 0, 170).linear(), Pose(0, 0, 0, 0, 0, -170).linear()),
              20.0, 1e-9);
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

TEST(MarkerTrays, ReadGroundTruthKeepsPairInstanceAndSource) {
  const std::string db = TempDir() + "/gt_read.sqlite";
  int64_t run = -1;
  {
    MetricsLog log(db);
    RunInfo info;
    info.source = "replay";
    info.detector_backend = "mock:none";
    info.pose_backend = "mock:identity";
    info.config_dir = "cfg";
    run = log.BeginRun(info);
    std::vector<GroundTruthRow> rows;
    for (uint64_t p : {40u, 41u}) {
      GroundTruthRow a;
      a.pair_index = p;
      a.cls = "red_cube";
      a.pose_w = Pose(0.1, 0.2, 0.03, 0, 0, 45);
      a.source = "aruco_tray";
      rows.push_back(a);
      GroundTruthRow b = a;
      b.cls = "bowl";
      b.instance = 1;
      b.pose_w = Pose(-0.1, 0.0, 0.03, 0, 0, -10);
      rows.push_back(b);
    }
    log.WriteGroundTruth(run, rows);
  }
  const std::vector<GroundTruthRow> got = ReadGroundTruth(db, run);
  ASSERT_EQ(got.size(), 4u);
  EXPECT_EQ(got[0].pair_index, 40u);
  EXPECT_EQ(got[0].cls, "bowl");  // ordered by pair, then class
  EXPECT_EQ(got[0].instance, 1);
  EXPECT_EQ(got[0].source, "aruco_tray");
  EXPECT_TRUE(got[0].pose_w.isApprox(Pose(-0.1, 0.0, 0.03, 0, 0, -10), 1e-9));
  EXPECT_EQ(got[3].pair_index, 41u);
  EXPECT_EQ(got[3].cls, "red_cube");
  EXPECT_TRUE(ReadGroundTruth(db, run + 1).empty());
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
