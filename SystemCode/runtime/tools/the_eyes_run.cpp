// The Eyes -- live end-to-end runner.
//
//   the_eyes_run check  [--live]                      startup checks only
//   the_eyes_run live   [--record [NAME] | --record-only [NAME]] [--frames N] [--show|--show=console]
//   the_eyes_run replay <session_dir> [--realtime] [--start N] [--max-pairs N] [--show|--show=console]
//
// Common: --config-dir DIR (default: config)  --repo-root DIR (default: parent
// of config dir)  --detector onnx|mock:<fixture>|fixed:<cam>=<cls>,<x>,<y>,<w>,<h>[;...]
// --pose foundationpose|mock
// --db PATH  --no-log  --print-every N
//
// One process, no ROS: cameras -> RGB-D YOLOv8 -> FoundationPose -> fusion ->
// scene graph -> PDDL, with every pair logged to SQLite. INTERFACES.md §1/§6
// name ROS2 topics; the payloads here are the same structs, in-process.
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "the_eyes/fusion/fusion.hpp"
#include "the_eyes/runtime/backends.hpp"
#include "the_eyes/runtime/console.hpp"
#include "the_eyes/runtime/clock.hpp"
#include "the_eyes/runtime/metrics_log.hpp"
#include "the_eyes/runtime/pipeline.hpp"
#include "the_eyes/runtime/recorder.hpp"
#include "the_eyes/runtime/replay_frame_source.hpp"
#include "the_eyes/runtime/startup_checks.hpp"
#include "the_eyes/runtime/viewer.hpp"
#if THE_EYES_WITH_ONNXRUNTIME
#include "the_eyes/detector/onnx_detector.hpp"
#endif
#if THE_EYES_WITH_REALSENSE
#include "the_eyes/runtime/live_frame_source.hpp"
#endif

namespace {

using namespace the_eyes;
using namespace the_eyes::runtime;
namespace fs = std::filesystem;

volatile std::sig_atomic_t g_stop = 0;
void OnSignal(int) { g_stop = 1; }

constexpr int kExitOk = 0;
constexpr int kExitRuntime = 1;
constexpr int kExitStartup = 2;
constexpr int kExitInterrupted = 130;

struct Options {
  std::string command;
  std::string config_dir = "config";
  std::string repo_root;
  std::string session_dir;       // replay
  bool live_checks = false;      // check --live
  std::string record_name;       // "" = no record; "-" = record with no name
  bool record = false;
  bool record_only = false;
  uint64_t frames = 0;           // live: stop after N processed pairs; 0 = until Ctrl-C
  bool show = false;
  bool console = false;
  bool realtime = false;
  uint64_t start_pair = 0;
  uint64_t max_pairs = 0;
  std::string db_path;
  bool no_log = false;
  int print_every = -1;
  std::string detector_override;
  std::string pose_override;
};

void Usage(std::ostream& os) {
  os << "usage:\n"
        "  the_eyes_run check  [--live]\n"
        "  the_eyes_run live   [--record [NAME] | --record-only [NAME]] [--frames N] [--show|--show=console]\n"
        "  the_eyes_run replay <session_dir> [--realtime] [--start N] [--max-pairs N] [--show|--show=console]\n"
        "                      [--record [NAME]]\n"
        "common: --config-dir DIR  --repo-root DIR\n"
        "        --detector onnx | mock:<fixture> | fixed:<cam>=<cls>,<x>,<y>,<w>,<h>[;<cam>=...]\n"
        "        --pose foundationpose|mock  --db PATH  --no-log  --print-every N\n";
}

Options Parse(int argc, char** argv) {
  Options o;
  if (argc < 2) throw std::runtime_error("missing command");
  o.command = argv[1];
  std::vector<std::string> args(argv + 2, argv + argc);
  auto value = [&](size_t& i, const std::string& flag) -> std::string {
    if (i + 1 >= args.size()) throw std::runtime_error(flag + " needs a value");
    return args[++i];
  };
  auto optional_value = [&](size_t& i) -> std::string {
    if (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0) return args[++i];
    return "";
  };
  for (size_t i = 0; i < args.size(); ++i) {
    const std::string& a = args[i];
    if (a == "--config-dir") o.config_dir = value(i, a);
    else if (a == "--repo-root") o.repo_root = value(i, a);
    else if (a == "--live") o.live_checks = true;
    else if (a == "--record") { o.record = true; o.record_name = optional_value(i); }
    else if (a == "--record-only") { o.record = true; o.record_only = true; o.record_name = optional_value(i); }
    else if (a == "--frames") o.frames = std::stoull(value(i, a));
    else if (a == "--show") o.show = true;
    // `--show` keeps exactly the behaviour it has always had. `--show=console`
    // is the richer window (docs/VIZ_PLAN.md); it is a separate value rather
    // than a replacement so an existing command line renders what it always did.
    else if (a == "--show=console") { o.show = true; o.console = true; }
    else if (a == "--realtime") o.realtime = true;
    else if (a == "--start") o.start_pair = std::stoull(value(i, a));
    else if (a == "--max-pairs") o.max_pairs = std::stoull(value(i, a));
    else if (a == "--db") o.db_path = value(i, a);
    else if (a == "--no-log") o.no_log = true;
    else if (a == "--print-every") o.print_every = std::stoi(value(i, a));
    else if (a == "--detector") o.detector_override = value(i, a);
    else if (a == "--pose") o.pose_override = value(i, a);
    else if (a.rfind("--", 0) == 0) throw std::runtime_error("unknown option " + a);
    else if (o.command == "replay" && o.session_dir.empty()) o.session_dir = a;
    else throw std::runtime_error("unexpected argument " + a);
  }
  if (o.command != "check" && o.command != "live" && o.command != "replay") {
    throw std::runtime_error("unknown command '" + o.command + "'");
  }
  if (o.command == "replay" && o.session_dir.empty()) {
    throw std::runtime_error("replay needs a session directory");
  }
  o.config_dir = fs::absolute(o.config_dir).lexically_normal().string();
  if (o.repo_root.empty()) o.repo_root = fs::path(o.config_dir).parent_path().string();
  o.repo_root = fs::absolute(o.repo_root).lexically_normal().string();
  return o;
}

void ApplyOverrides(const Options& o, LoadedConfigs* c) {
  if (!o.detector_override.empty()) {
    if (o.detector_override == "onnx") {
      c->runtime.detector.backend = "onnx";
    } else if (o.detector_override.rfind("mock:", 0) == 0) {
      c->runtime.detector.backend = "mock";
      c->runtime.detector.mock_fixture = o.detector_override.substr(5);
    } else if (o.detector_override == "mock") {
      c->runtime.detector.backend = "mock";
    } else if (o.detector_override.rfind("fixed:", 0) == 0) {
      c->runtime.detector.backend = "fixed";
      c->runtime.detector.fixed_boxes = o.detector_override.substr(6);
    } else {
      throw std::runtime_error("--detector must be onnx, mock:<fixture>, or "
                               "fixed:<cam>=<cls>,<x>,<y>,<w>,<h>[;...]");
    }
  }
  if (!o.pose_override.empty()) {
    if (o.pose_override != "foundationpose" && o.pose_override != "mock") {
      throw std::runtime_error("--pose must be foundationpose or mock");
    }
    c->pose_backend.backend = o.pose_override;
    // Mesh files are only needed when the server will load them.
    c->meshes = pose::LoadMeshRegistry(c->config_dir + "/meshes.yaml",
                                       c->pose_backend.backend == "foundationpose", c->repo_root);
  }
}

std::string CamerasJson(const std::vector<std::string>& ids,
                        const std::map<std::string, CameraMeta>& meta) {
  std::ostringstream os;
  os << std::setprecision(17) << "{";
  bool first = true;
  for (const std::string& id : ids) {
    auto it = meta.find(id);
    if (it == meta.end()) continue;
    const CameraMeta& m = it->second;
    os << (first ? "" : ", ") << "\"" << id << "\": {\"serial\": \"" << m.serial
       << "\", \"depth_scale_m\": " << m.depth_scale_m << ", \"K\": [";
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) os << (r + c ? ", " : "") << m.K(r, c);
    os << "], \"sync_mode_requested\": " << m.sync_mode_requested
       << ", \"sync_mode_actual\": " << m.sync_mode_actual
       << ", \"auto_exposure\": " << (m.auto_exposure ? "true" : "false")
       << ", \"exposure\": " << m.exposure << ", \"gain\": " << m.gain
       << ", \"auto_white_balance\": " << (m.auto_white_balance ? "true" : "false")
       << ", \"white_balance_k\": " << m.white_balance_k << ", \"timestamp_domain\": \""
       << m.timestamp_domain << "\"}";
    first = false;
  }
  os << "}";
  return os.str();
}

std::string StatusLine(const PipelineResult& r, double fps, uint64_t processed) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(1) << "pair " << std::setw(6) << r.pair_index << "  n "
     << std::setw(5) << processed << "  skew " << std::setw(6) << r.skew_ms << " ms  drop "
     << std::setw(3) << r.frames_dropped_since_last << "  det";
  for (const auto& [view, ms] : r.timings.detect_ms) os << " " << std::setw(5) << ms;
  os << "  pose";
  for (const auto& [view, ms] : r.timings.pose_ms) os << " " << std::setw(6) << ms;
  os << "  fuse " << std::setw(4) << std::setprecision(2) << r.timings.fusion_ms << "  sg "
     << std::setw(4) << r.timings.scenegraph_ms << "  total " << std::setw(6)
     << std::setprecision(1) << r.timings.total_ms << " ms  " << std::setw(5) << fps
     << " fps  obj " << r.scene.objects.size() << "  facts " << r.pddl.facts.size();
  if (!r.warnings.empty()) os << "  warn " << r.warnings.size();
  return os.str();
}

int RunChecks(const Options& o, LoadedConfigs* c, StartupReport* report, bool devices) {
  CheckConfigCrossRefs(*c, report);
  CheckDetectorBackend(*c, report);
  CheckPoseBackend(*c, report);
  if (devices) CheckDevices(*c, report);
  std::cout << "=== startup checks (" << o.config_dir << ") ===\n" << report->Format();
  return report->ok() ? kExitOk : kExitStartup;
}

std::unique_ptr<Pipeline> BuildPipeline(const LoadedConfigs& c, StartupReport* report) {
  DetectorHandle det = MakeDetector(c.runtime.detector, c.detector, c.repo_root);
#if THE_EYES_WITH_ONNXRUNTIME
  if (det.onnx) {
    const std::string active = det.onnx->active_provider();
    report->Info("detector: active provider " + active);
    if (active != det.requested_provider && active.find("CPU") != std::string::npos) {
      report->Warn("detector running on CPU (FALLBACK from '" + det.requested_provider +
                   "') -- latency numbers from this run are not the deployed configuration");
    }
    for (const std::string& w : det.onnx->warnings()) report->Warn("detector: " + w);
  }
#endif
  PoseHandle pose = MakePoseEstimator(c.pose_backend);
  std::map<std::string, std::unique_ptr<pose::PoseAgent>> agents;
  for (const std::string& v : c.fusion.view_names) {
    agents[v] = std::make_unique<pose::PoseAgent>(c.meshes, c.pose_agent, pose.estimator);
  }
  auto fusion = std::make_unique<fusion::Fusion>(c.fusion, c.extrinsics, ExtentsFrom(c.meshes));
  return std::make_unique<Pipeline>(std::move(det), std::move(pose), std::move(agents),
                                    std::move(fusion), c.predicates, c.scene, c.fusion.view_names);
}

int Run(const Options& o) {
  LoadedConfigs c = LoadAllConfigs(o.config_dir, o.repo_root);
  ApplyOverrides(o, &c);
  const bool live = o.command == "live";
  StartupReport report;
  if (o.command == "check") return RunChecks(o, &c, &report, o.live_checks);
  if (o.record_only && !live) throw std::runtime_error("--record-only applies to live only");

  {
    StartupReport pre;
    if (!o.record_only) {
      CheckConfigCrossRefs(c, &pre);
      CheckDetectorBackend(c, &pre);
      CheckPoseBackend(c, &pre);
    } else {
      CheckConfigCrossRefs(c, &pre);
    }
    if (live) CheckDevices(c, &pre);
    std::cout << "=== startup checks ===\n" << pre.Format();
    if (!pre.ok()) return kExitStartup;
    report = pre;
  }
  const size_t warnings_already_printed = report.warnings.size();

  // --- frame source -------------------------------------------------------
  std::unique_ptr<FrameSource> source;
  int width = 0, height = 0, fps_cfg = 0;
  bool align = true;
  double max_skew_ms = c.cameras.sync.max_skew_ms;
  std::optional<camera::SyncReport> startup_sync;
  if (live) {
#if THE_EYES_WITH_REALSENSE
    PairerConfig pc;
    pc.camera_ids = c.fusion.view_names;
    pc.max_skew_ms = c.cameras.sync.max_skew_ms;
    pc.nominal_period_ms = 1000.0 / c.cameras.streams.fps;
    pc.ring_depth = c.runtime.pairing.ring_depth;
    auto src = std::make_unique<LiveFrameSource>(c.cameras, pc, c.runtime.live, std::cout);
    width = src->streams().color_width;
    height = src->streams().color_height;
    fps_cfg = src->streams().fps;
    align = src->streams().align_depth_to_color;
    startup_sync = src->startup_sync_report();
    report.Info("live: " + startup_sync->verdict);
    source = std::move(src);
#else
    throw std::runtime_error("live capture needs librealsense, which was not found at configure time");
#endif
  } else {
    ReplayConfig rc;
    rc.realtime = o.realtime;
    rc.start_pair = o.start_pair;
    rc.max_pairs = o.max_pairs;
    auto src = std::make_unique<ReplayFrameSource>(o.session_dir, rc);
    const SessionMeta& s = src->session();
    width = s.width;
    height = s.height;
    fps_cfg = s.fps;
    align = s.align_depth_to_color;
    max_skew_ms = s.max_skew_ms;
    startup_sync = s.startup_sync;
    if (s.camera_ids != c.fusion.view_names) {
      throw std::runtime_error("replay: session cameras do not match fusion.yaml views.names");
    }
    for (const std::string& id : s.camera_ids) {
      const std::string& recorded = s.cameras.at(id).serial;
      const std::string& configured = c.extrinsics.cameras.at(id).serial;
      if (recorded != configured) {
        report.Warn("replay: " + id + " was recorded from serial " + recorded +
                    " but extrinsics.yaml describes " + configured);
      }
    }
    source = std::move(src);
  }
  std::cout << "source: " << source->describe() << "\n";

  // --- session id, recorder, metrics log ---------------------------------
  std::string session_id = NowSessionStamp();
  if (!o.record_name.empty()) session_id += "_" + o.record_name;
  std::unique_ptr<Recorder> recorder;
  if (o.record) {
    SessionMeta meta;
    meta.session_id = session_id;
    meta.created_utc = NowUtcString();
    meta.source = live ? "live" : "replay:" + o.session_dir;
    meta.camera_ids = source->camera_ids();
    meta.max_skew_ms = max_skew_ms;
    meta.width = width;
    meta.height = height;
    meta.fps = fps_cfg;
    meta.align_depth_to_color = align;
    meta.cameras = source->meta();
    meta.startup_sync = startup_sync;
    RecorderConfig rc;
    rc.session_dir = fs::path(ResolveAgainst(c.repo_root, c.runtime.output.captures_dir)) / session_id;
    rc.queue_capacity = c.runtime.recorder.queue_capacity;
    rc.png_compression = c.runtime.recorder.png_compression;
    rc.writer_threads = c.runtime.recorder.writer_threads;
    recorder = std::make_unique<Recorder>(rc, meta);
    std::cout << "recording to " << rc.session_dir.string() << "\n";
  }

  std::unique_ptr<Pipeline> pipeline;
  if (!o.record_only) pipeline = BuildPipeline(c, &report);

  std::unique_ptr<MetricsLog> log;
  int64_t run_id = -1;
  if (!o.no_log && !o.record_only) {
    std::string db = o.db_path;
    if (db.empty()) {
      const fs::path dir = fs::path(ResolveAgainst(c.repo_root, c.runtime.output.runs_dir)) / session_id;
      fs::create_directories(dir);
      db = (dir / "metrics.sqlite").string();
    }
    log = std::make_unique<MetricsLog>(db);
    RunInfo info;
    info.source = live ? "live" : "replay";
    info.source_path = live ? source->describe() : o.session_dir;
    info.detector_backend = pipeline->detector().label;
#if THE_EYES_WITH_ONNXRUNTIME
    if (pipeline->detector().onnx) info.detector_provider = pipeline->detector().onnx->active_provider();
#endif
    info.pose_backend = pipeline->pose().label;
    info.config_dir = o.config_dir;
    info.git_rev = GitRevision(c.repo_root);
    info.cameras_json = CamerasJson(source->camera_ids(), source->meta());
    info.notes = report.Format();
    run_id = log->BeginRun(info);
    std::cout << "metrics: " << db << " (run " << run_id << ")\n";
  }
  for (size_t i = warnings_already_printed; i < report.warnings.size(); ++i) {
    std::cout << "  WARNING: " << report.warnings[i] << "\n";
  }

  std::unique_ptr<Viewer> viewer;
  std::unique_ptr<Console> console;
  if (o.show && !o.record_only) {
    if (o.console) {
      console = std::make_unique<Console>(c.fusion.view_names, c.extrinsics);
    } else {
      viewer = std::make_unique<Viewer>(c.fusion.view_names, c.extrinsics);
    }
  }

  // --- main loop ----------------------------------------------------------
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  const int print_every = o.print_every > 0 ? o.print_every : c.runtime.output.print_every;
  const int stall_timeout = live ? c.runtime.live.stall_timeout_ms : 0;
  int consecutive_stalls = 0;
  uint64_t processed = 0, recorded = 0;
  std::vector<std::string> last_facts;
  std::string exit_status = "ok";
  int exit_code = kExitOk;
  const Clock::time_point t_start = Clock::now();
  Clock::time_point t_window = t_start;
  uint64_t window_count = 0;
  double fps = 0.0;

  try {
    for (;;) {
      if (g_stop) {
        exit_status = "interrupted";
        exit_code = kExitInterrupted;
        break;
      }
      if (o.frames > 0 && (o.record_only ? recorded : processed) >= o.frames) break;
      std::optional<FramePair> pair = source->NextPair(stall_timeout);
      if (!pair) {
        if (source->exhausted()) break;
        ++consecutive_stalls;
        std::cout << "stall: no synchronised pair for " << stall_timeout << " ms (" << consecutive_stalls
                  << "/" << c.runtime.live.max_consecutive_stalls << ")\n";
        if (consecutive_stalls >= c.runtime.live.max_consecutive_stalls) {
          throw std::runtime_error("cameras stopped delivering synchronised pairs");
        }
        continue;
      }
      consecutive_stalls = 0;
      if (recorder) {
        if (!recorder->Enqueue(*pair)) {
          std::cout << "recorder: queue full, dropped pair " << pair->pair_index << "\n";
        }
        ++recorded;
        if (o.record_only) {
          if (recorded % static_cast<uint64_t>(print_every) == 0) {
            const double elapsed = ElapsedMs(t_start) / 1000.0;
            std::cout << std::fixed << std::setprecision(1) << "recorded " << recorded << " pairs, "
                      << recorded / elapsed << " pairs/s, skew " << pair->skew_ms << " ms, drop "
                      << pair->frames_dropped_since_last << "\n";
          }
          continue;
        }
      }

      const PipelineResult result = pipeline->Process(*pair);
      ++processed;
      ++window_count;
      if (ElapsedMs(t_window) >= 1000.0) {
        fps = window_count * 1000.0 / ElapsedMs(t_window);
        t_window = Clock::now();
        window_count = 0;
      } else if (fps == 0.0 && ElapsedMs(t_start) > 0.0) {
        fps = processed * 1000.0 / ElapsedMs(t_start);
      }
      if (log) log->WriteFrame(run_id, result);

      for (const auto& [stage, message] : result.warnings) {
        std::cout << "  warning [" << stage << "] pair " << result.pair_index << ": " << message << "\n";
      }
      if (result.pddl.facts != last_facts) {
        std::cout << "--- scene changed at pair " << result.pair_index << " (stamp " << result.stamp_ns
                  << ") ---\n";
        for (const ObjectState& obj : result.scene.objects) {
          std::cout << "  " << std::left << std::setw(20) << obj.id << " views=0b";
          for (int bit = static_cast<int>(c.fusion.view_names.size()) - 1; bit >= 0; --bit) {
            std::cout << ((obj.visible_views >> bit) & 1u);
          }
          std::cout << std::fixed << std::setprecision(3) << "  conf=" << obj.confidence << "  t=("
                    << obj.pose_w.translation().x() << ", " << obj.pose_w.translation().y() << ", "
                    << obj.pose_w.translation().z() << ")";
          if (has_uncertainty(obj)) std::cout << "  sigma=" << obj.position_sigma_m * 1000.0 << " mm";
          std::cout << "\n";
        }
        std::cout << result.pddl.init_block << "\n";
        last_facts = result.pddl.facts;
      }
      if (processed % static_cast<uint64_t>(print_every) == 0) {
        std::cout << StatusLine(result, fps, processed) << "\n";
      }
      if (viewer && !viewer->Show(*pair, result, fps)) {
        exit_status = "quit";
        break;
      }
      if (console && !console->Show(*pair, result, fps)) {
        exit_status = "quit";
        break;
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "the_eyes_run: " << e.what() << "\n";
    exit_status = std::string("error: ") + e.what();
    exit_code = kExitRuntime;
  }

  // --- wrap up ------------------------------------------------------------
  const double elapsed_s = ElapsedMs(t_start) / 1000.0;
  if (log) log->EndRun(run_id, exit_status, processed);
  if (recorder) {
    recorder->Close();
    const Recorder::Stats s = recorder->stats();
    std::cout << "=== recorder ===\n  " << recorder->session_dir().string() << "\n  enqueued " << s.enqueued
              << ", written " << s.written << ", dropped " << s.dropped << ", max queue fill "
              << s.max_queue_fill << "\n";
    for (const std::string& e : s.errors) std::cout << "  ERROR: " << e << "\n";
  }
#if THE_EYES_WITH_REALSENSE
  if (auto* lv = dynamic_cast<LiveFrameSource*>(source.get())) {
    const PairerStats s = lv->pairer_stats();
    std::cout << "=== pairer ===\n  paired " << s.paired << ", dropped (skew) " << s.dropped_skew
              << ", dropped (stale) " << s.dropped_stale << ", source gaps " << s.source_gaps
              << ", timeouts " << s.timeouts << "\n";
    for (const auto& [id, n] : s.pushed) std::cout << "  " << id << " pushed " << n << "\n";
  }
#endif
  if (pipeline) {
    std::cout << "=== latency (" << processed << " pairs in " << std::fixed << std::setprecision(1)
              << elapsed_s << " s, " << (elapsed_s > 0 ? processed / elapsed_s : 0.0)
              << " pairs/s end to end) ===\n"
              << pipeline->Summary();
    if (!last_facts.empty()) {
      std::cout << "=== final PDDL ===\n(:init\n";
      for (const std::string& f : last_facts) std::cout << "  " << f << "\n";
      std::cout << ")\n";
    }
  }
  if (log) std::cout << "metrics: " << log->path() << " run " << run_id << " (" << exit_status << ")\n";
  return exit_code;
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  try {
    o = Parse(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "the_eyes_run: " << e.what() << "\n";
    Usage(std::cerr);
    return kExitStartup;
  }
  try {
    return Run(o);
  } catch (const std::exception& e) {
    std::cerr << "the_eyes_run: " << e.what() << "\n";
    return kExitStartup;
  }
}
