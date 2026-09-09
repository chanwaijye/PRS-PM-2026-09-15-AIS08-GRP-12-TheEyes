// SQLite metric log (CLAUDE.md: "Log metrics to SQLite; the report needs
// reproducible numbers"). One database per run directory; one row per pair
// in `frames`, with everything that pair produced in the child tables. The
// `estimates` table carries full camera-frame PoseEstimates so the occlusion
// ablation can be re-run from a log against ground truth added later
// (LoadAblationFrames); `ground_truth` is empty until the ArUco-tray tool
// fills it.
//
// Uses the sqlite3 C API directly: this is logging, not the graded fusion or
// scene-graph work, so a library is fine here -- but there is no need for a
// wrapper on top of it either.
#ifndef THE_EYES_RUNTIME_METRICS_LOG_HPP
#define THE_EYES_RUNTIME_METRICS_LOG_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "the_eyes/fusion/metrics.hpp"
#include "the_eyes/runtime/pipeline_result.hpp"

namespace the_eyes::runtime {

constexpr int kMetricsSchemaVersion = 1;

struct RunInfo {
  std::string source;             // "live" | "replay"
  std::string source_path;        // session dir, or serials for live
  std::string detector_backend;   // "onnx" | "mock:<fixture>"
  std::string detector_provider;  // OnnxDetector::active_provider(); flagged on fallback
  std::string pose_backend;       // "foundationpose" | "mock:identity"
  std::string config_dir;
  std::string git_rev;
  std::string cameras_json;       // serial / K / depth scale / sync per camera
  std::string notes;              // startup report text
};

// One ground-truth pose in the WORLD frame. Filled by the ArUco-tray tool, and
// read back by LoadAblationFrames as fusion::GroundTruthObject.
struct GroundTruthRow {
  uint64_t pair_index = 0;
  std::string cls;
  int instance = 0;   // which instance of `cls`, when a scene holds more than one
  Eigen::Isometry3d pose_w = Eigen::Isometry3d::Identity();
  std::string source; // how it was obtained, e.g. "aruco_tray" -- never blank,
                      // so a reader can tell measured truth from anything else
};

class MetricsLog {
 public:
  // Opens or creates the database, applies the schema, prepares statements.
  // Throws std::runtime_error on an incompatible schema version.
  explicit MetricsLog(const std::string& db_path);
  ~MetricsLog();
  MetricsLog(const MetricsLog&) = delete;
  MetricsLog& operator=(const MetricsLog&) = delete;

  int64_t BeginRun(const RunInfo& info);
  // One transaction per frame. Throws on a write failure; the caller decides
  // whether a dead log should stop the run.
  void WriteFrame(int64_t run_id, const PipelineResult& result);
  void EndRun(int64_t run_id, const std::string& exit_status, uint64_t pairs_processed);

  // REPLACES this run's ground truth: every existing row for run_id is deleted
  // first, in the same transaction. Appending would be the wrong default --
  // LoadAblationFrames reads every row for a run, so re-running the tray tool
  // after a fix would silently double the ground truth rather than correct it.
  void WriteGroundTruth(int64_t run_id, const std::vector<GroundTruthRow>& rows);

  const std::string& path() const { return path_; }

 private:
  struct Impl;
  std::string path_;
  std::unique_ptr<Impl> impl_;
};

// Rebuilds per-pair AblationFrames from the `estimates` and `ground_truth`
// tables of one run, ordered by pair_index. Ground truth is whatever the
// table holds -- empty if nothing has filled it, in which case the ablation
// cannot be scored and the caller must say so rather than invent numbers.
std::vector<fusion::AblationFrame> LoadAblationFrames(const std::string& db_path, int64_t run_id);

// Newest run_id in the database, or -1 if it holds none.
int64_t LatestRunId(const std::string& db_path);

// Every pair_index the run actually logged, ascending.
//
// Ground truth for a static scene is frozen once and written to every pair, so
// the writer needs the run's pair list. When ground truth is solved from
// separate high-resolution stills rather than from the run's own frames there
// is no session to enumerate, and this is the only place the answer exists.
std::vector<uint64_t> PairIndices(const std::string& db_path, int64_t run_id);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_METRICS_LOG_HPP
