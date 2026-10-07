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

// objects.supported for one object of one pair; -1 where it is NULL.
int SupportedOf(const std::string& db_path, const std::string& object_id, int64_t run_id) {
  sqlite3* db = nullptr;
  EXPECT_EQ(sqlite3_open(db_path.c_str(), &db), SQLITE_OK);
  sqlite3_stmt* stmt = nullptr;
  const std::string sql = "SELECT supported FROM objects WHERE object_id = '" + object_id +
                          "' AND run_id = " + std::to_string(run_id);
  EXPECT_EQ(sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr), SQLITE_OK);
  EXPECT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  const int v = sqlite3_column_type(stmt, 0) == SQLITE_NULL ? -1 : sqlite3_column_int(stmt, 0);
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return v;
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
  // t3_bottle_over_b's clear view: well supported, not seen through.
  e1.depth_support = 0.9;
  e1.seen_through = 0.0;
  PoseEstimate e2 = e1;
  e2.camera_id = "cam_over";
  e2.cls = "bowl";
  e2.symmetric = true;
  e2.pose_cam = Pose(-0.05, 0.02, 0.8, -120.0);
  e2.extent = Eigen::Vector3d(0.080732, 0.080582, 0.027505);
  // ...and an estimate nobody measured: "not supplied" has to survive the log.
  e2.depth_support = -1.0;
  e2.seen_through = -1.0;
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
      // The depth evidence is what the harness's gate decides on: it has to
      // come back exactly, and "not supplied" has to come back as that.
      EXPECT_DOUBLE_EQ(got.depth_support, want.depth_support) << got.camera_id;
      EXPECT_DOUBLE_EQ(got.seen_through, want.seen_through) << got.camera_id;
      EXPECT_EQ(has_view_evidence(got), got.camera_id == "cam_front");
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
  // ...and before objects recorded what the support check made of them.
  ASSERT_EQ(sqlite3_exec(raw, "ALTER TABLE objects DROP COLUMN supported", nullptr, nullptr, nullptr),
            SQLITE_OK)
      << sqlite3_errmsg(raw);
  // ...and before the estimates carried depth evidence (2026-10-07).
  for (const char* sql : {"ALTER TABLE estimates DROP COLUMN depth_support",
                          "ALTER TABLE estimates DROP COLUMN seen_through",
                          "ALTER TABLE frames DROP COLUMN evidence_ms",
                          "ALTER TABLE frames DROP COLUMN views_refuted"}) {
    ASSERT_EQ(sqlite3_exec(raw, sql, nullptr, nullptr, nullptr), SQLITE_OK) << sqlite3_errmsg(raw);
  }
  sqlite3_close(raw);

  // Read back as it stands, by the harness, with no writer having reopened it:
  // every estimate of the old run has no evidence, and nothing is refused.
  {
    const auto old_frames = LoadAblationFrames(db, first);
    ASSERT_EQ(old_frames.size(), 1u);
    for (const PoseEstimate& e : old_frames[0].estimates) EXPECT_FALSE(has_view_evidence(e));
  }

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
  EXPECT_EQ(SupportedOf(db, "table_01", first), -1) << "written before the column: not recorded";
  // The older run still rebuilds for the ablation harness, without evidence;
  // the new one carries it.
  const auto old_frames = LoadAblationFrames(db, first);
  ASSERT_EQ(old_frames.size(), 1u);
  for (const PoseEstimate& e : old_frames[0].estimates) EXPECT_FALSE(has_view_evidence(e));
  const auto new_frames = LoadAblationFrames(db, first + 1);
  ASSERT_EQ(new_frames.size(), 1u);
  int with_evidence = 0;
  for (const PoseEstimate& e : new_frames[0].estimates) with_evidence += has_view_evidence(e) ? 1 : 0;
  EXPECT_EQ(with_evidence, 1);
}

TEST(MetricsLog, RecordsWhichObjectsTheSupportCheckLeftOut) {
  // t3_cube_over, 2026-10-05: fusion tracked a second cube at the occluder's
  // range. It stays in the objects table -- that fusion produced it is the
  // finding -- and the column says the scene graph did not report it.
  PipelineResult r = Result(0);
  ObjectState phantom = r.scene.objects[0];
  phantom.id = "red_cube_02";
  phantom.cls = "red_cube";
  phantom.pose_w.translation() = Eigen::Vector3d(0.0265, -0.1042, 0.1202);
  r.scene.objects.push_back(phantom);

  const std::string db = TempDb();
  MetricsLog log(db);
  RunInfo info;
  info.source = "replay";
  info.detector_backend = "mock:fixture";
  info.pose_backend = "mock:identity";
  info.config_dir = "config";

  r.support_checked = true;
  r.unsupported_ids = {"red_cube_02"};
  const int64_t checked = log.BeginRun(info);
  log.WriteFrame(checked, r);
  log.EndRun(checked, "ok", 1);
  EXPECT_EQ(SupportedOf(db, "red_cube_02", checked), 0);
  EXPECT_EQ(SupportedOf(db, "mustard_bottle_01", checked), 1);
  EXPECT_EQ(SupportedOf(db, "table_01", checked), 1);

  // The check off is not the same as every object passing it.
  r.support_checked = false;
  r.unsupported_ids.clear();
  const int64_t unchecked = log.BeginRun(info);
  log.WriteFrame(unchecked, r);
  log.EndRun(unchecked, "ok", 1);
  EXPECT_EQ(SupportedOf(db, "red_cube_02", unchecked), -1);
  EXPECT_EQ(SupportedOf(db, "table_01", unchecked), -1);
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
