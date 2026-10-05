#include <cstdlib>
#include <filesystem>

#include <gtest/gtest.h>
#include <sqlite3.h>

#include "the_eyes/runtime/metrics_log.hpp"

namespace {

using namespace the_eyes;
using namespace the_eyes::runtime;

std::string TempDb() {
  char tmpl[] = "/tmp/the_eyes_metrics_XXXXXX";
  const char* d = mkdtemp(tmpl);
  if (d == nullptr) throw std::runtime_error("mkdtemp failed");
  return std::string(d) + "/metrics.sqlite";
}

int64_t Count(const std::string& db_path, const std::string& table) {
  sqlite3* db = nullptr;
  EXPECT_EQ(sqlite3_open(db_path.c_str(), &db), SQLITE_OK);
  sqlite3_stmt* stmt = nullptr;
  const std::string sql = "SELECT COUNT(*) FROM " + table;
  EXPECT_EQ(sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr), SQLITE_OK);
  EXPECT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  const int64_t n = sqlite3_column_int64(stmt, 0);
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return n;
}

// frame_views.brightness_gain for one camera of one pair; -1 where it is NULL.
double GainOf(const std::string& db_path, const std::string& camera_id, int64_t pair_index) {
  sqlite3* db = nullptr;
  EXPECT_EQ(sqlite3_open(db_path.c_str(), &db), SQLITE_OK);
  sqlite3_stmt* stmt = nullptr;
  const std::string sql = "SELECT brightness_gain FROM frame_views WHERE camera_id = '" +
                          camera_id + "' AND pair_index = " + std::to_string(pair_index);
  EXPECT_EQ(sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr), SQLITE_OK);
  EXPECT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  const double g = sqlite3_column_type(stmt, 0) == SQLITE_NULL ? -1.0 : sqlite3_column_double(stmt, 0);
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return g;
}

Eigen::Isometry3d Pose(double x, double y, double z, double yaw_deg) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = (Eigen::AngleAxisd(yaw_deg * 3.14159265358979323846 / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  p.translation() = Eigen::Vector3d(x, y, z);
  return p;
}

PipelineResult Result(uint64_t pair_index) {
  PipelineResult r;
  r.pair_index = pair_index;
  r.stamp_ns = 1'000'000'000ULL + pair_index * 33'000'000ULL;
  r.skew_ms = 11.0;
  r.frames_dropped_since_last = 1;
  r.view_stamps["cam_front"] = {r.stamp_ns, 100 + pair_index};
  r.view_stamps["cam_over"] = {r.stamp_ns - 11'000'000ULL, 200 + pair_index};
  // The night bottle of 2026-10-05: cam_front bright enough, cam_over at 23.
  r.brightness_gain = {{"cam_front", 1.0}, {"cam_over", 45.0 / 23.0}};

  DetectionSet front;
  front.camera_id = "cam_front";
  front.stamp_ns = r.stamp_ns;
  front.detections.push_back({"mustard_bottle", 0.9f, cv::Rect2i(10, 20, 30, 40), 1});
  front.detections.push_back({"bowl", 0.8f, cv::Rect2i(50, 60, 70, 80), 2});
  DetectionSet over;
  over.camera_id = "cam_over";
  over.stamp_ns = r.stamp_ns - 11'000'000ULL;
  over.detections.push_back({"bowl", 0.7f, cv::Rect2i(1, 2, 3, 4), 1});
  r.detections = {{"cam_front", front}, {"cam_over", over}};

  PoseEstimate e1;
  e1.camera_id = "cam_front";
  e1.cls = "mustard_bottle";
  e1.track_id = 1;
  e1.stamp_ns = front.stamp_ns;
  e1.pose_cam = Pose(0.1, -0.2, 0.6, 35.0);
  e1.score = 0.82;
  e1.symmetric = false;
  e1.extent = Eigen::Vector3d(0.048576, 0.033309, 0.095704);
  PoseEstimate e2 = e1;
  e2.camera_id = "cam_over";
  e2.cls = "bowl";
  e2.symmetric = true;
  e2.pose_cam = Pose(-0.05, 0.02, 0.8, -120.0);
  e2.extent = Eigen::Vector3d(0.080732, 0.080582, 0.027505);
  r.estimates = {{"cam_front", {e1}}, {"cam_over", {e2}}};

  ObjectState o1;
  o1.id = "mustard_bottle_01";
  o1.cls = "mustard_bottle";
  o1.pose_w = Pose(0.3, 0.1, 0.09, 10.0);
  o1.confidence = 0.82;
  o1.visible_views = 0b01;
  o1.extent = e1.extent;
  o1.stamp_ns = r.stamp_ns;
  o1.position_sigma_m = 0.004;
  o1.orientation_sigma_rad = 0.02;
  o1.velocity_w = Eigen::Vector3d(0.01, 0, 0);
  ObjectState table;
  table.id = "table_01";
  table.cls = "table";
  table.confidence = 1.0;
  table.visible_views = 0b11;
  table.extent = Eigen::Vector3d(0.4, 0.3, 0.01);
  table.stamp_ns = r.stamp_ns;
  r.scene.stamp_ns = r.stamp_ns;
  r.scene.objects = {o1, table};
  r.static_ids = {"table_01"};
  r.pddl.facts = {"(clear mustard_bottle_01)", "(on mustard_bottle_01 table_01)",
                  "(occluded_from mustard_bottle_01 cam_over)"};
  r.pddl.init_block = "(:init ...)";
  r.timings.detect_ms = {{"cam_front", 6.5}, {"cam_over", 6.1}};
  r.timings.pose_ms = {{"cam_front", 12.0}, {"cam_over", 11.0}};
  r.timings.fusion_ms = 0.2;
  r.timings.scenegraph_ms = 0.05;
  r.timings.total_ms = 36.0;
  r.fusion_diag.views_rejected = 1;
  r.fusion_diag.tracks_created = 1;
  r.pose_diag["cam_over"].declined_by_estimator = 1;
  r.warnings = {{"fusion", "a warning"}};
  return r;
}

}  // namespace

TEST(MetricsLog, WritesEveryTableAndReadsBackEstimatesExactly) {
  const std::string db = TempDb();
  int64_t run_id = -1;
  {
    MetricsLog log(db);
    RunInfo info;
    info.source = "replay";
    info.source_path = "/nowhere";
    info.detector_backend = "mock:fixture";
    info.detector_provider = "";
    info.pose_backend = "mock:identity";
    info.config_dir = "config";
    info.git_rev = "abc123";
    run_id = log.BeginRun(info);
    EXPECT_GE(run_id, 1);
    log.WriteFrame(run_id, Result(0));
    log.WriteFrame(run_id, Result(1));
    log.EndRun(run_id, "ok", 2);
  }
  EXPECT_EQ(Count(db, "schema_version"), 1);
  EXPECT_EQ(Count(db, "runs"), 1);
  EXPECT_EQ(Count(db, "frames"), 2);
  EXPECT_EQ(Count(db, "frame_views"), 4);
  EXPECT_EQ(Count(db, "detections"), 6);
  EXPECT_EQ(Count(db, "estimates"), 4);
  EXPECT_EQ(Count(db, "objects"), 4);
  EXPECT_EQ(Count(db, "facts"), 6);
  EXPECT_EQ(Count(db, "warnings"), 2);
  EXPECT_EQ(Count(db, "ground_truth"), 0);
  EXPECT_EQ(LatestRunId(db), run_id);
  // What the detector was shown is part of the record, per view and per pair.
  EXPECT_DOUBLE_EQ(GainOf(db, "cam_front", 1), 1.0);
  EXPECT_DOUBLE_EQ(GainOf(db, "cam_over", 1), 45.0 / 23.0);

  const std::vector<fusion::AblationFrame> frames = LoadAblationFrames(db, run_id);
  ASSERT_EQ(frames.size(), 2u);
  for (size_t i = 0; i < frames.size(); ++i) {
    const PipelineResult expected = Result(i);
    EXPECT_EQ(frames[i].stamp_ns, expected.stamp_ns);
    ASSERT_EQ(frames[i].estimates.size(), 2u);
    EXPECT_TRUE(frames[i].ground_truth.empty());
    for (const PoseEstimate& got : frames[i].estimates) {
      const PoseEstimate& want = expected.estimates.at(got.camera_id)[0];
      EXPECT_EQ(got.cls, want.cls);
      EXPECT_EQ(got.track_id, want.track_id);
      EXPECT_EQ(got.stamp_ns, want.stamp_ns);
      EXPECT_DOUBLE_EQ(got.score, want.score);
      EXPECT_EQ(got.symmetric, want.symmetric);
      EXPECT_TRUE(got.extent.isApprox(want.extent, 0.0));
      EXPECT_TRUE(got.pose_cam.matrix().isApprox(want.pose_cam.matrix(), 1e-12))
          << got.camera_id << "\n" << got.pose_cam.matrix() << "\nvs\n" << want.pose_cam.matrix();
    }
  }

  // Reopening does not recreate anything or complain about the version.
  {
    MetricsLog again(db);
    RunInfo info;
    info.source = "live";
    info.detector_backend = "onnx";
    info.pose_backend = "foundationpose";
    info.config_dir = "config";
    const int64_t second = again.BeginRun(info);
    EXPECT_EQ(second, run_id + 1);
  }
  EXPECT_EQ(Count(db, "schema_version"), 1);
  EXPECT_EQ(Count(db, "runs"), 2);
  EXPECT_EQ(Count(db, "frames"), 2);
}

TEST(MetricsLog, ALogFromBeforeTheGainColumnGainsItAndKeepsItsRows) {
  // The scored logs of 2026-10-05 were written before frame_views recorded the
  // brightness gain. Reprocessing adds a run to the same file, so it has to
  // open: the old rows read NULL ("not recorded"), the new ones carry the gain.
  const std::string db = TempDb();
  int64_t first = -1;
  {
    MetricsLog log(db);
    RunInfo info;
    info.source = "replay";
    info.detector_backend = "mock:fixture";
    info.pose_backend = "mock:identity";
    info.config_dir = "config";
    first = log.BeginRun(info);
    log.WriteFrame(first, Result(0));
    log.EndRun(first, "ok", 1);
  }
  sqlite3* raw = nullptr;
  ASSERT_EQ(sqlite3_open(db.c_str(), &raw), SQLITE_OK);
  ASSERT_EQ(sqlite3_exec(raw, "ALTER TABLE frame_views DROP COLUMN brightness_gain", nullptr,
                         nullptr, nullptr),
            SQLITE_OK)
      << sqlite3_errmsg(raw);
  sqlite3_close(raw);

  {
    MetricsLog log(db);
    RunInfo info;
    info.source = "replay";
    info.detector_backend = "mock:fixture";
    info.pose_backend = "mock:identity";
    info.config_dir = "config";
    const int64_t second = log.BeginRun(info);
    EXPECT_EQ(second, first + 1);
    log.WriteFrame(second, Result(1));
    log.EndRun(second, "ok", 1);
  }
  EXPECT_EQ(Count(db, "frame_views"), 4);
  EXPECT_DOUBLE_EQ(GainOf(db, "cam_over", 0), -1.0) << "written before the column: not recorded";
  EXPECT_DOUBLE_EQ(GainOf(db, "cam_over", 1), 45.0 / 23.0);
  // The older run still rebuilds for the ablation harness.
  EXPECT_EQ(LoadAblationFrames(db, first).size(), 1u);
}

TEST(MetricsLog, RejectsAForeignSchemaVersion) {
  const std::string db = TempDb();
  { MetricsLog log(db); }
  sqlite3* raw = nullptr;
  ASSERT_EQ(sqlite3_open(db.c_str(), &raw), SQLITE_OK);
  ASSERT_EQ(sqlite3_exec(raw, "UPDATE schema_version SET version = 99", nullptr, nullptr, nullptr),
            SQLITE_OK);
  sqlite3_close(raw);
  EXPECT_THROW(MetricsLog{db}, std::runtime_error);
}
