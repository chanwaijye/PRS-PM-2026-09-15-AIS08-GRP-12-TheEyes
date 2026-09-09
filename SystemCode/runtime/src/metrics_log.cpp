#include "the_eyes/runtime/metrics_log.hpp"

#include <stdexcept>

#include <sqlite3.h>

#include "the_eyes/runtime/clock.hpp"

namespace the_eyes::runtime {
namespace {

const char* kSchema = R"SQL(
CREATE TABLE IF NOT EXISTS schema_version(version INTEGER NOT NULL);
CREATE TABLE IF NOT EXISTS runs(
  run_id INTEGER PRIMARY KEY,
  started_utc TEXT NOT NULL,
  finished_utc TEXT,
  source TEXT NOT NULL,
  source_path TEXT,
  detector_backend TEXT NOT NULL,
  detector_provider TEXT,
  pose_backend TEXT NOT NULL,
  config_dir TEXT NOT NULL,
  git_rev TEXT,
  cameras_json TEXT,
  notes TEXT,
  pairs_processed INTEGER,
  exit_status TEXT);
CREATE TABLE IF NOT EXISTS frames(
  run_id INTEGER NOT NULL REFERENCES runs(run_id),
  pair_index INTEGER NOT NULL,
  stamp_ns INTEGER NOT NULL,
  skew_ms REAL NOT NULL,
  frames_dropped_since_last INTEGER NOT NULL,
  fusion_ms REAL, scenegraph_ms REAL, total_ms REAL,
  n_objects INTEGER, n_facts INTEGER,
  views_rejected INTEGER, two_view_conflicts INTEGER,
  tracks_created INTEGER, tracks_retired INTEGER,
  PRIMARY KEY(run_id, pair_index));
CREATE TABLE IF NOT EXISTS frame_views(
  run_id INTEGER NOT NULL, pair_index INTEGER NOT NULL, camera_id TEXT NOT NULL,
  stamp_ns INTEGER NOT NULL, frame_number INTEGER NOT NULL,
  detect_ms REAL, pose_ms REAL, n_detections INTEGER, n_estimates INTEGER,
  dropped_no_mesh INTEGER, dropped_roi_too_small INTEGER,
  dropped_low_score INTEGER, declined_by_estimator INTEGER,
  PRIMARY KEY(run_id, pair_index, camera_id));
CREATE TABLE IF NOT EXISTS detections(
  run_id INTEGER NOT NULL, pair_index INTEGER NOT NULL, camera_id TEXT NOT NULL,
  stamp_ns INTEGER NOT NULL, cls TEXT NOT NULL, score REAL NOT NULL,
  track_id INTEGER NOT NULL, x INTEGER, y INTEGER, w INTEGER, h INTEGER);
CREATE TABLE IF NOT EXISTS estimates(
  run_id INTEGER NOT NULL, pair_index INTEGER NOT NULL, camera_id TEXT NOT NULL,
  stamp_ns INTEGER NOT NULL, cls TEXT NOT NULL, track_id INTEGER NOT NULL,
  score REAL NOT NULL, symmetric INTEGER NOT NULL,
  tx REAL, ty REAL, tz REAL, qw REAL, qx REAL, qy REAL, qz REAL,
  ex REAL, ey REAL, ez REAL);
CREATE TABLE IF NOT EXISTS objects(
  run_id INTEGER NOT NULL, pair_index INTEGER NOT NULL, object_id TEXT NOT NULL,
  cls TEXT NOT NULL, visible_views INTEGER NOT NULL, confidence REAL NOT NULL,
  tx REAL, ty REAL, tz REAL, qw REAL, qx REAL, qy REAL, qz REAL,
  ex REAL, ey REAL, ez REAL,
  position_sigma_m REAL, orientation_sigma_rad REAL, vx REAL, vy REAL, vz REAL,
  stamp_ns INTEGER NOT NULL, is_static INTEGER NOT NULL DEFAULT 0,
  PRIMARY KEY(run_id, pair_index, object_id));
CREATE TABLE IF NOT EXISTS facts(
  run_id INTEGER NOT NULL, pair_index INTEGER NOT NULL, fact TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS warnings(
  run_id INTEGER NOT NULL, pair_index INTEGER, stage TEXT NOT NULL, message TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS ground_truth(
  run_id INTEGER NOT NULL, pair_index INTEGER NOT NULL, cls TEXT NOT NULL,
  instance INTEGER NOT NULL DEFAULT 0,
  tx REAL, ty REAL, tz REAL, qw REAL, qx REAL, qy REAL, qz REAL,
  source TEXT NOT NULL);
CREATE INDEX IF NOT EXISTS idx_estimates_run_pair ON estimates(run_id, pair_index);
CREATE INDEX IF NOT EXISTS idx_objects_run_pair ON objects(run_id, pair_index);
CREATE INDEX IF NOT EXISTS idx_facts_run_pair ON facts(run_id, pair_index);
)SQL";

std::string ErrorOf(sqlite3* db, const std::string& what) {
  return "metrics log: " + what + ": " + (db ? sqlite3_errmsg(db) : "no database");
}

void Exec(sqlite3* db, const char* sql) {
  char* err = nullptr;
  if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
    const std::string msg = err ? err : "unknown";
    sqlite3_free(err);
    throw std::runtime_error("metrics log: " + msg);
  }
}

class Stmt {
 public:
  Stmt() = default;
  Stmt(sqlite3* db, const char* sql) : db_(db) {
    if (sqlite3_prepare_v2(db, sql, -1, &stmt_, nullptr) != SQLITE_OK) {
      throw std::runtime_error(ErrorOf(db, std::string("prepare '") + sql + "'"));
    }
  }
  ~Stmt() {
    if (stmt_) sqlite3_finalize(stmt_);
  }
  Stmt(const Stmt&) = delete;
  Stmt& operator=(const Stmt&) = delete;
  Stmt(Stmt&& o) noexcept : db_(o.db_), stmt_(o.stmt_) { o.stmt_ = nullptr; }
  Stmt& operator=(Stmt&& o) noexcept {
    if (this != &o) {
      if (stmt_) sqlite3_finalize(stmt_);
      db_ = o.db_;
      stmt_ = o.stmt_;
      o.stmt_ = nullptr;
    }
    return *this;
  }

  Stmt& Reset() {
    sqlite3_reset(stmt_);
    sqlite3_clear_bindings(stmt_);
    col_ = 1;
    return *this;
  }
  Stmt& Bind(int64_t v) {
    sqlite3_bind_int64(stmt_, col_++, v);
    return *this;
  }
  Stmt& Bind(uint64_t v) { return Bind(static_cast<int64_t>(v)); }
  Stmt& Bind(int v) { return Bind(static_cast<int64_t>(v)); }
  Stmt& Bind(uint32_t v) { return Bind(static_cast<int64_t>(v)); }
  Stmt& Bind(double v) {
    sqlite3_bind_double(stmt_, col_++, v);
    return *this;
  }
  Stmt& Bind(const std::string& v) {
    sqlite3_bind_text(stmt_, col_++, v.c_str(), -1, SQLITE_TRANSIENT);
    return *this;
  }
  Stmt& BindNull() {
    sqlite3_bind_null(stmt_, col_++);
    return *this;
  }
  Stmt& Bind(const Eigen::Vector3d& v) { return Bind(v.x()).Bind(v.y()).Bind(v.z()); }
  Stmt& Bind(const Eigen::Isometry3d& p) {
    const Eigen::Quaterniond q(p.linear());
    return Bind(p.translation()).Bind(q.w()).Bind(q.x()).Bind(q.y()).Bind(q.z());
  }
  void Run() {
    const int rc = sqlite3_step(stmt_);
    if (rc != SQLITE_DONE) throw std::runtime_error(ErrorOf(db_, "step"));
  }
  bool Row() {
    const int rc = sqlite3_step(stmt_);
    if (rc == SQLITE_ROW) return true;
    if (rc == SQLITE_DONE) return false;
    throw std::runtime_error(ErrorOf(db_, "step"));
  }
  int64_t Int(int col) const { return sqlite3_column_int64(stmt_, col); }
  double Dbl(int col) const { return sqlite3_column_double(stmt_, col); }
  std::string Str(int col) const {
    const unsigned char* s = sqlite3_column_text(stmt_, col);
    return s ? reinterpret_cast<const char*>(s) : "";
  }
  bool IsNull(int col) const { return sqlite3_column_type(stmt_, col) == SQLITE_NULL; }

 private:
  sqlite3* db_ = nullptr;
  sqlite3_stmt* stmt_ = nullptr;
  int col_ = 1;
};

sqlite3* Open(const std::string& path, bool create) {
  sqlite3* db = nullptr;
  const int flags = SQLITE_OPEN_READWRITE | (create ? SQLITE_OPEN_CREATE : 0);
  if (sqlite3_open_v2(path.c_str(), &db, flags, nullptr) != SQLITE_OK) {
    const std::string msg = db ? sqlite3_errmsg(db) : "cannot open";
    if (db) sqlite3_close(db);
    throw std::runtime_error("metrics log: cannot open '" + path + "': " + msg);
  }
  return db;
}

Eigen::Isometry3d PoseFrom(const Stmt& s, int first_col) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(s.Dbl(first_col), s.Dbl(first_col + 1), s.Dbl(first_col + 2));
  const Eigen::Quaterniond q(s.Dbl(first_col + 3), s.Dbl(first_col + 4), s.Dbl(first_col + 5),
                             s.Dbl(first_col + 6));
  p.linear() = q.normalized().toRotationMatrix();
  return p;
}

}  // namespace

struct MetricsLog::Impl {
  sqlite3* db = nullptr;
  Stmt begin_run, end_run, frame, frame_view, detection, estimate, object, fact, warning;

  ~Impl() {
    if (db) sqlite3_close(db);
  }
};

MetricsLog::MetricsLog(const std::string& db_path) : path_(db_path), impl_(std::make_unique<Impl>()) {
  impl_->db = Open(db_path, true);
  sqlite3* db = impl_->db;
  Exec(db, "PRAGMA journal_mode=WAL;");
  Exec(db, "PRAGMA synchronous=NORMAL;");
  Exec(db, "PRAGMA foreign_keys=ON;");
  Exec(db, kSchema);

  {
    Stmt v(db, "SELECT version FROM schema_version LIMIT 1");
    if (v.Row()) {
      const int64_t version = v.Int(0);
      if (version != kMetricsSchemaVersion) {
        throw std::runtime_error("metrics log: '" + db_path + "' has schema version " +
                                 std::to_string(version) + ", this build writes " +
                                 std::to_string(kMetricsSchemaVersion));
      }
    } else {
      Stmt ins(db, "INSERT INTO schema_version(version) VALUES (?)");
      ins.Reset().Bind(kMetricsSchemaVersion).Run();
    }
  }

  impl_->begin_run = Stmt(db,
      "INSERT INTO runs(started_utc, source, source_path, detector_backend, detector_provider, "
      "pose_backend, config_dir, git_rev, cameras_json, notes) VALUES (?,?,?,?,?,?,?,?,?,?)");
  impl_->end_run = Stmt(db,
      "UPDATE runs SET finished_utc=?, exit_status=?, pairs_processed=? WHERE run_id=?");
  impl_->frame = Stmt(db,
      "INSERT INTO frames(run_id, pair_index, stamp_ns, skew_ms, frames_dropped_since_last, "
      "fusion_ms, scenegraph_ms, total_ms, n_objects, n_facts, views_rejected, "
      "two_view_conflicts, tracks_created, tracks_retired) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)");
  impl_->frame_view = Stmt(db,
      "INSERT INTO frame_views(run_id, pair_index, camera_id, stamp_ns, frame_number, detect_ms, "
      "pose_ms, n_detections, n_estimates, dropped_no_mesh, dropped_roi_too_small, "
      "dropped_low_score, declined_by_estimator) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)");
  impl_->detection = Stmt(db,
      "INSERT INTO detections(run_id, pair_index, camera_id, stamp_ns, cls, score, track_id, "
      "x, y, w, h) VALUES (?,?,?,?,?,?,?,?,?,?,?)");
  impl_->estimate = Stmt(db,
      "INSERT INTO estimates(run_id, pair_index, camera_id, stamp_ns, cls, track_id, score, "
      "symmetric, tx, ty, tz, qw, qx, qy, qz, ex, ey, ez) "
      "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)");
  impl_->object = Stmt(db,
      "INSERT INTO objects(run_id, pair_index, object_id, cls, visible_views, confidence, "
      "tx, ty, tz, qw, qx, qy, qz, ex, ey, ez, position_sigma_m, orientation_sigma_rad, "
      "vx, vy, vz, stamp_ns, is_static) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)");
  impl_->fact = Stmt(db, "INSERT INTO facts(run_id, pair_index, fact) VALUES (?,?,?)");
  impl_->warning = Stmt(db,
      "INSERT INTO warnings(run_id, pair_index, stage, message) VALUES (?,?,?,?)");
}

MetricsLog::~MetricsLog() = default;

int64_t MetricsLog::BeginRun(const RunInfo& info) {
  impl_->begin_run.Reset()
      .Bind(NowUtcString())
      .Bind(info.source)
      .Bind(info.source_path)
      .Bind(info.detector_backend)
      .Bind(info.detector_provider)
      .Bind(info.pose_backend)
      .Bind(info.config_dir)
      .Bind(info.git_rev)
      .Bind(info.cameras_json)
      .Bind(info.notes)
      .Run();
  return sqlite3_last_insert_rowid(impl_->db);
}

void MetricsLog::EndRun(int64_t run_id, const std::string& exit_status, uint64_t pairs_processed) {
  impl_->end_run.Reset().Bind(NowUtcString()).Bind(exit_status).Bind(pairs_processed).Bind(run_id).Run();
}

void MetricsLog::WriteFrame(int64_t run_id, const PipelineResult& r) {
  Impl& s = *impl_;
  Exec(s.db, "BEGIN");
  try {
    s.frame.Reset()
        .Bind(run_id).Bind(r.pair_index).Bind(r.stamp_ns).Bind(r.skew_ms)
        .Bind(r.frames_dropped_since_last)
        .Bind(r.timings.fusion_ms).Bind(r.timings.scenegraph_ms).Bind(r.timings.total_ms)
        .Bind(static_cast<int64_t>(r.scene.objects.size()))
        .Bind(static_cast<int64_t>(r.pddl.facts.size()))
        .Bind(r.fusion_diag.views_rejected).Bind(r.fusion_diag.two_view_conflicts)
        .Bind(r.fusion_diag.tracks_created).Bind(r.fusion_diag.tracks_retired)
        .Run();

    for (const auto& [camera_id, stamps] : r.view_stamps) {
      const auto det = r.detections.find(camera_id);
      const auto est = r.estimates.find(camera_id);
      const auto diag = r.pose_diag.find(camera_id);
      const pose::PoseAgentDiagnostics none;
      const pose::PoseAgentDiagnostics& d = diag != r.pose_diag.end() ? diag->second : none;
      auto ms = [&](const std::map<std::string, double>& m) {
        auto it = m.find(camera_id);
        return it != m.end() ? it->second : 0.0;
      };
      s.frame_view.Reset()
          .Bind(run_id).Bind(r.pair_index).Bind(camera_id)
          .Bind(stamps.first).Bind(stamps.second)
          .Bind(ms(r.timings.detect_ms)).Bind(ms(r.timings.pose_ms))
          .Bind(static_cast<int64_t>(det != r.detections.end() ? det->second.detections.size() : 0))
          .Bind(static_cast<int64_t>(est != r.estimates.end() ? est->second.size() : 0))
          .Bind(d.dropped_no_mesh).Bind(d.dropped_roi_too_small)
          .Bind(d.dropped_low_score).Bind(d.declined_by_estimator)
          .Run();
    }

    for (const auto& [camera_id, set] : r.detections) {
      for (const Detection& d : set.detections) {
        s.detection.Reset()
            .Bind(run_id).Bind(r.pair_index).Bind(camera_id).Bind(set.stamp_ns)
            .Bind(d.cls).Bind(static_cast<double>(d.score)).Bind(d.track_id)
            .Bind(d.bbox.x).Bind(d.bbox.y).Bind(d.bbox.width).Bind(d.bbox.height)
            .Run();
      }
    }

    for (const auto& [camera_id, list] : r.estimates) {
      for (const PoseEstimate& e : list) {
        s.estimate.Reset()
            .Bind(run_id).Bind(r.pair_index).Bind(e.camera_id).Bind(e.stamp_ns)
            .Bind(e.cls).Bind(e.track_id).Bind(e.score).Bind(e.symmetric ? 1 : 0)
            .Bind(e.pose_cam).Bind(e.extent)
            .Run();
      }
    }

    for (const ObjectState& o : r.scene.objects) {
      bool is_static = false;
      for (const std::string& id : r.static_ids) is_static = is_static || (id == o.id);
      s.object.Reset()
          .Bind(run_id).Bind(r.pair_index).Bind(o.id).Bind(o.cls)
          .Bind(o.visible_views).Bind(o.confidence)
          .Bind(o.pose_w).Bind(o.extent)
          .Bind(o.position_sigma_m).Bind(o.orientation_sigma_rad)
          .Bind(o.velocity_w).Bind(o.stamp_ns).Bind(is_static ? 1 : 0)
          .Run();
    }

    for (const std::string& f : r.pddl.facts) {
      s.fact.Reset().Bind(run_id).Bind(r.pair_index).Bind(f).Run();
    }
    for (const auto& [stage, message] : r.warnings) {
      s.warning.Reset().Bind(run_id).Bind(r.pair_index).Bind(stage).Bind(message).Run();
    }
    Exec(s.db, "COMMIT");
  } catch (...) {
    Exec(s.db, "ROLLBACK");
    throw;
  }
}

void MetricsLog::WriteGroundTruth(int64_t run_id, const std::vector<GroundTruthRow>& rows) {
  Impl& s = *impl_;
  Exec(s.db, "BEGIN");
  try {
    {
      Stmt del(s.db, "DELETE FROM ground_truth WHERE run_id=?");
      del.Reset().Bind(run_id).Run();
    }
    Stmt ins(s.db,
             "INSERT INTO ground_truth(run_id,pair_index,cls,instance,"
             "tx,ty,tz,qw,qx,qy,qz,source) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)");
    for (const GroundTruthRow& g : rows) {
      if (g.source.empty()) {
        throw std::runtime_error("metrics log: ground truth for '" + g.cls +
                                 "' has no source; measured truth must be distinguishable "
                                 "from anything else that fills this table");
      }
      ins.Reset()
          .Bind(run_id).Bind(g.pair_index).Bind(g.cls).Bind(g.instance)
          .Bind(g.pose_w).Bind(g.source)
          .Run();
    }
    Exec(s.db, "COMMIT");
  } catch (...) {
    Exec(s.db, "ROLLBACK");
    throw;
  }
}

std::vector<fusion::AblationFrame> LoadAblationFrames(const std::string& db_path, int64_t run_id) {
  sqlite3* db = Open(db_path, false);
  std::vector<fusion::AblationFrame> frames;
  try {
    std::map<uint64_t, size_t> index_of;
    {
      Stmt q(db, "SELECT pair_index, stamp_ns FROM frames WHERE run_id=? ORDER BY pair_index");
      q.Reset().Bind(run_id);
      while (q.Row()) {
        fusion::AblationFrame f;
        f.stamp_ns = static_cast<uint64_t>(q.Int(1));
        index_of[static_cast<uint64_t>(q.Int(0))] = frames.size();
        frames.push_back(std::move(f));
      }
    }
    {
      Stmt q(db,
             "SELECT pair_index, camera_id, stamp_ns, cls, track_id, score, symmetric, "
             "tx, ty, tz, qw, qx, qy, qz, ex, ey, ez FROM estimates WHERE run_id=? "
             "ORDER BY pair_index, camera_id, cls, track_id");
      q.Reset().Bind(run_id);
      while (q.Row()) {
        auto it = index_of.find(static_cast<uint64_t>(q.Int(0)));
        if (it == index_of.end()) continue;
        PoseEstimate e;
        e.camera_id = q.Str(1);
        e.stamp_ns = static_cast<uint64_t>(q.Int(2));
        e.cls = q.Str(3);
        e.track_id = static_cast<uint32_t>(q.Int(4));
        e.score = q.Dbl(5);
        e.symmetric = q.Int(6) != 0;
        e.pose_cam = PoseFrom(q, 7);
        e.extent = Eigen::Vector3d(q.Dbl(14), q.Dbl(15), q.Dbl(16));
        frames[it->second].estimates.push_back(std::move(e));
      }
    }
    {
      Stmt q(db,
             "SELECT pair_index, cls, tx, ty, tz, qw, qx, qy, qz FROM ground_truth "
             "WHERE run_id=? ORDER BY pair_index, cls, instance");
      q.Reset().Bind(run_id);
      while (q.Row()) {
        auto it = index_of.find(static_cast<uint64_t>(q.Int(0)));
        if (it == index_of.end()) continue;
        fusion::GroundTruthObject g;
        g.cls = q.Str(1);
        g.pose_w = PoseFrom(q, 2);
        frames[it->second].ground_truth.push_back(std::move(g));
      }
    }
  } catch (...) {
    sqlite3_close(db);
    throw;
  }
  sqlite3_close(db);
  return frames;
}

std::vector<uint64_t> PairIndices(const std::string& db_path, int64_t run_id) {
  sqlite3* db = Open(db_path, false);
  std::vector<uint64_t> out;
  try {
    Stmt q(db, "SELECT pair_index FROM frames WHERE run_id=? ORDER BY pair_index");
    q.Reset().Bind(run_id);
    while (q.Row()) out.push_back(static_cast<uint64_t>(q.Int(0)));
  } catch (...) {
    sqlite3_close(db);
    throw;
  }
  sqlite3_close(db);
  return out;
}

int64_t LatestRunId(const std::string& db_path) {
  sqlite3* db = Open(db_path, false);
  int64_t id = -1;
  try {
    Stmt q(db, "SELECT MAX(run_id) FROM runs");
    if (q.Row() && !q.IsNull(0)) id = q.Int(0);
  } catch (...) {
    sqlite3_close(db);
    throw;
  }
  sqlite3_close(db);
  return id;
}

}  // namespace the_eyes::runtime
