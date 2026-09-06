// The whole runner chain with zero hardware: a synthetic recorded session,
// the fixture detector, the identity pose mock, and the REAL extrinsics,
// fusion, predicate and scene configs, through Pipeline into the metrics log.
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>

#include <gtest/gtest.h>
#include <sqlite3.h>

#include "the_eyes/fusion/config.hpp"
#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/pose/mesh_registry.hpp"
#include "the_eyes/runtime/backends.hpp"
#include "the_eyes/runtime/metrics_log.hpp"
#include "the_eyes/runtime/pipeline.hpp"
#include "the_eyes/runtime/recorder.hpp"
#include "the_eyes/runtime/replay_frame_source.hpp"
#include "the_eyes/runtime/startup_checks.hpp"

namespace {

using namespace the_eyes;
using namespace the_eyes::runtime;
namespace fs = std::filesystem;

constexpr int kW = 160;
constexpr int kH = 120;
constexpr int kPairs = 6;

std::string TempDirPath() {
  char tmpl[] = "/tmp/the_eyes_replay_pipe_XXXXXX";
  const char* d = mkdtemp(tmpl);
  if (d == nullptr) throw std::runtime_error("mkdtemp failed");
  return d;
}

uint64_t FrontStamp(int i) { return 1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL; }
uint64_t OverStamp(int i) { return FrontStamp(i) - 11'000'000ULL; }

camera::CameraFrame Flat(const std::string& id, const std::string& serial, uint64_t stamp,
                         uint64_t number) {
  camera::CameraFrame f;
  f.camera_id = id;
  f.serial = serial;
  f.stamp_ns = stamp;
  f.frame_number = number;
  f.rgb = cv::Mat(kH, kW, CV_8UC3, cv::Scalar(90, 100, 110));
  f.depth = cv::Mat(kH, kW, CV_16UC1, cv::Scalar(700));
  f.depth_scale_m = 0.001;
  f.K << 120.0, 0, kW / 2.0, 0, 120.0, kH / 2.0, 0, 0, 1;
  return f;
}

// Records a session and writes a detector fixture keyed to its stamps.
struct Fixture {
  fs::path session;
  std::string detector_fixture;
};

Fixture MakeSession(const std::string& root) {
  SessionMeta meta;
  meta.session_id = "synthetic";
  meta.created_utc = "2026-09-06T00:00:00Z";
  meta.source = "test";
  meta.camera_ids = {"cam_front", "cam_over"};
  meta.width = kW;
  meta.height = kH;
  meta.fps = 30;
  CameraMeta front;
  front.serial = "261722073945";
  front.K = Flat("cam_front", front.serial, 0, 0).K;
  CameraMeta over = front;
  over.serial = "261822074567";
  meta.cameras = {{"cam_front", front}, {"cam_over", over}};

  RecorderConfig rc;
  rc.session_dir = fs::path(root) / "session";
  Recorder rec(rc, meta);
  for (int i = 0; i < kPairs; ++i) {
    FramePair p;
    p.pair_index = static_cast<uint64_t>(i);
    p.stamp_ns = FrontStamp(i);
    p.skew_ms = 11.0;
    p.frames.emplace("cam_front", Flat("cam_front", front.serial, FrontStamp(i), 10 + i));
    p.frames.emplace("cam_over", Flat("cam_over", over.serial, OverStamp(i), 20 + i));
    rec.Enqueue(p);
  }
  rec.Close();

  std::ofstream fx(root + "/fixture.yaml");
  fx << "frames:\n";
  for (int i = 0; i < kPairs; ++i) {
    fx << "  - camera_id: cam_front\n    stamp_ns: " << FrontStamp(i) << "\n    detections:\n"
       << "      - {cls: mustard_bottle, score: 0.9, bbox: [20, 20, 40, 60]}\n"
       << "      - {cls: bowl, score: 0.8, bbox: [90, 40, 50, 40]}\n"
       << "      - {cls: unicorn, score: 0.5, bbox: [10, 90, 20, 20]}\n";
    fx << "  - camera_id: cam_over\n    stamp_ns: " << OverStamp(i) << "\n    detections:\n"
       << "      - {cls: bowl, score: 0.7, bbox: [60, 30, 50, 50]}\n";
  }
  return {rc.session_dir, root + "/fixture.yaml"};
}

std::unique_ptr<Pipeline> BuildPipeline(const Fixture& fx) {
  LoadedConfigs c = LoadAllConfigs(THE_EYES_CONFIG_DIR, THE_EYES_REPO_ROOT);
  c.runtime.detector.backend = "mock";
  c.runtime.detector.mock_fixture = fx.detector_fixture;
  c.pose_backend.backend = "mock";

  DetectorHandle det = MakeDetector(c.runtime.detector, c.detector, THE_EYES_REPO_ROOT);
  PoseHandle pose = MakePoseEstimator(c.pose_backend);
  std::map<std::string, std::unique_ptr<pose::PoseAgent>> agents;
  for (const std::string& v : c.fusion.view_names) {
    agents[v] = std::make_unique<pose::PoseAgent>(c.meshes, c.pose_agent, pose.estimator);
  }
  auto fusion = std::make_unique<fusion::Fusion>(c.fusion, c.extrinsics, ExtentsFrom(c.meshes));
  return std::make_unique<Pipeline>(std::move(det), std::move(pose), std::move(agents),
                                    std::move(fusion), c.predicates, c.scene, c.fusion.view_names);
}

std::vector<std::string> FactsOfRun(const std::string& db, int64_t run_id) {
  sqlite3* raw = nullptr;
  EXPECT_EQ(sqlite3_open(db.c_str(), &raw), SQLITE_OK);
  sqlite3_stmt* stmt = nullptr;
  EXPECT_EQ(sqlite3_prepare_v2(raw, "SELECT pair_index, fact FROM facts WHERE run_id=? ORDER BY pair_index, fact",
                               -1, &stmt, nullptr),
            SQLITE_OK);
  sqlite3_bind_int64(stmt, 1, run_id);
  std::vector<std::string> out;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    out.push_back(std::to_string(sqlite3_column_int64(stmt, 0)) + " " +
                  reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1)));
  }
  sqlite3_finalize(stmt);
  sqlite3_close(raw);
  return out;
}

int64_t CountRows(const std::string& db, const std::string& sql) {
  sqlite3* raw = nullptr;
  EXPECT_EQ(sqlite3_open(db.c_str(), &raw), SQLITE_OK);
  sqlite3_stmt* stmt = nullptr;
  EXPECT_EQ(sqlite3_prepare_v2(raw, sql.c_str(), -1, &stmt, nullptr), SQLITE_OK);
  EXPECT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  const int64_t n = sqlite3_column_int64(stmt, 0);
  sqlite3_finalize(stmt);
  sqlite3_close(raw);
  return n;
}

}  // namespace

TEST(ReplayPipeline, RunsEndToEndWithoutHardwareAndIsDeterministic) {
  const std::string root = TempDirPath();
  const Fixture fx = MakeSession(root);
  const std::string db = root + "/metrics.sqlite";

  std::vector<int64_t> run_ids;
  PipelineResult last;
  for (int run = 0; run < 2; ++run) {
    std::unique_ptr<Pipeline> pipeline = BuildPipeline(fx);
    ReplayFrameSource source(fx.session);
    MetricsLog log(db);
    RunInfo info;
    info.source = "replay";
    info.source_path = fx.session.string();
    info.detector_backend = pipeline->detector().label;
    info.pose_backend = pipeline->pose().label;
    info.config_dir = THE_EYES_CONFIG_DIR;
    const int64_t run_id = log.BeginRun(info);
    run_ids.push_back(run_id);
    uint64_t n = 0;
    while (auto pair = source.NextPair(0)) {
      last = pipeline->Process(*pair);
      log.WriteFrame(run_id, last);
      ++n;
    }
    log.EndRun(run_id, "ok", n);
    EXPECT_EQ(n, static_cast<uint64_t>(kPairs));
    EXPECT_EQ(pipeline->latency().at("total").count(), static_cast<size_t>(kPairs));
    EXPECT_EQ(pipeline->latency().at("pose:cam_over").count(), static_cast<size_t>(kPairs));
  }

  // The final frame: the unicorn was dropped at pose_agent (no mesh), the
  // table came from scene.yaml, and at least one detected object survived.
  ASSERT_EQ(last.detections.at("cam_front").detections.size(), 3u);
  EXPECT_EQ(last.pose_diag.at("cam_front").dropped_no_mesh, 1);
  EXPECT_EQ(last.estimates.at("cam_front").size(), 2u);
  EXPECT_EQ(last.estimates.at("cam_over").size(), 1u);
  EXPECT_EQ(last.static_ids, std::vector<std::string>{"table_01"});
  int fused = 0;
  for (const ObjectState& o : last.scene.objects) {
    EXPECT_NE(o.visible_views, 0u) << o.id;
    if (o.id != "table_01") ++fused;
  }
  EXPECT_GE(fused, 1);
  EXPECT_NE(last.pddl.init_block.find("table_01"), std::string::npos) << last.pddl.init_block;
  EXPECT_FALSE(last.pddl.facts.empty());
  EXPECT_GT(last.timings.total_ms, 0.0);

  // Same input, same modules, same facts -- twice.
  ASSERT_EQ(run_ids.size(), 2u);
  const std::vector<std::string> a = FactsOfRun(db, run_ids[0]);
  const std::vector<std::string> b = FactsOfRun(db, run_ids[1]);
  EXPECT_FALSE(a.empty());
  EXPECT_EQ(a, b);
  EXPECT_EQ(CountRows(db, "SELECT COUNT(*) FROM frames WHERE run_id=" + std::to_string(run_ids[0])),
            kPairs);
  EXPECT_EQ(CountRows(db, "SELECT COUNT(*) FROM estimates WHERE run_id=" + std::to_string(run_ids[0])),
            3 * kPairs);
  EXPECT_EQ(CountRows(db, "SELECT COUNT(*) FROM objects WHERE run_id=" + std::to_string(run_ids[0]) +
                              " AND is_static=1"),
            kPairs);

  // And the log rebuilds ablation frames the fusion harness could consume.
  const auto frames = LoadAblationFrames(db, run_ids[0]);
  ASSERT_EQ(frames.size(), static_cast<size_t>(kPairs));
  EXPECT_EQ(frames.back().estimates.size(), 3u);
  EXPECT_TRUE(frames.back().ground_truth.empty());
}
