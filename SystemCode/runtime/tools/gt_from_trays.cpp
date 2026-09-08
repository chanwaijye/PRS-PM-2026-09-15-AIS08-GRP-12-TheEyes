// gt_from_trays -- fills a run's `ground_truth` table from the ArUco trays.
//
// This is the tool docs/CAPTURE_PROTOCOL.md 6 step 4 calls for, and the thing
// `ablation_from_log` has had nothing to score against. It closes the chain
//
//     T_world_object = T_world_cam . T_cam_marker . T_marker_tray . T_tray_object
//
// of which only T_cam_marker is measured; the rest is CAD (config/marker_trays.yaml)
// and calibration (config/extrinsics.yaml).
//
// WHY IT AVERAGES, AND WHY IT REPORTS SCATTER
// -------------------------------------------
// The protocol's scored scene is static: objects are seated in trays and never
// touched, so ground truth is a CONSTANT. Solving it once over a window of
// unoccluded opening frames and freezing it is what lets the occlusion
// afterwards be arbitrary -- including both views at once -- because the
// yardstick no longer depends on seeing anything.
//
// The scatter across those solves is ground truth's own noise floor. An ADD-S
// smaller than it is measuring the yardstick rather than the estimator, and the
// report has to be able to say so, so the spread is printed and is not
// optional.
//
// It writes the frozen pose to EVERY pair in the run, because LoadAblationFrames
// matches ground truth to frames by pair_index and a pair without it cannot be
// scored.
//
//   gt_from_trays <session_dir> <metrics.sqlite> [options]
//     --config DIR     config directory (default: ./config)
//     --pairs A:B      solve window, inclusive-exclusive, by pair_index
//                      (default: the first 30 pairs the session YIELDS, which
//                      is not pair_index [0,30) -- a session's indices start
//                      after the discarded warm-up frames, so a fixed range
//                      would silently scan nothing)
//     --run N          run_id to write (default: the newest in the database)
//     --min-obs N      refuse a class with fewer than N accepted observations (default 8)
//     --max-reproj PX  reject a marker whose corners reproject worse (default 1.5)
//     --min-size PX    reject a marker smaller than this on a side (default 40)
//
// RESOLUTION IS NOT A DETAIL HERE. Measured on synthetic renders of the real
// rig geometry with a known object pose: at the pipeline's 848x480 the tray
// markers subtend 23-37 px, cam_over contributes NOTHING, and the recovered
// object pose is 18 mm from truth -- against an ADD-S threshold of about 8.6 mm
// for the cube. At 1920x1080 the same markers subtend 51-84 px, both cameras
// contribute, and the pose lands 2.1 mm from truth.
//
// Worse, the pooled scatter at 848x480 reported 2.1 mm while the pose was 18 mm
// wrong, because a systematic bias does not show up as spread. The
// cross-camera disagreement is the number that did not lie (3.0 mm at 1080p),
// which is why it is printed and why two cameras are worth having.
//
// So ground truth is captured at 1920x1080, exactly as calibration already is
// (`calibrate capture --factory` exists for this reason), and the default
// --min-size refuses the small-marker regime rather than quietly producing
// confident nonsense.
//     --min-ambiguity R  reject a marker whose two planar poses fit almost
//                      equally well (default 2.0; see MarkerObservation::ambiguity)
//     --dry-run        report only, write nothing
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "the_eyes/calibration/marker_detector.hpp"
#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/runtime/marker_trays.hpp"
#include "the_eyes/runtime/metrics_log.hpp"
#include "the_eyes/runtime/replay_frame_source.hpp"

using namespace the_eyes;
using namespace the_eyes::runtime;

namespace {

struct Options {
  std::string session;
  std::string db;
  std::string config = "config";
  uint64_t pair_lo = 0, pair_hi = 0;  // unset => take the first `kDefaultWindow` pairs
  bool window_given = false;
  int64_t run_id = -1;
  int min_obs = 8;
  double max_reproj_px = 1.5;
  double min_size_px = 40.0;
  double min_ambiguity = 2.0;
  bool dry_run = false;
};

int Usage() {
  std::cerr << "usage: gt_from_trays <session_dir> <metrics.sqlite> [--config DIR]\n"
               "         [--pairs A:B] [--run N] [--min-obs N] [--max-reproj PX]\n"
               "         [--min-size PX] [--dry-run]\n";
  return 2;
}

// One accepted observation of one object, already in the world frame.
struct Sample {
  std::string cls;
  std::string camera_id;
  int marker_id;
  double size_px = 0.0;
  Eigen::Isometry3d pose_w;
};

void PrintSpread(const std::string& label, const PoseAverage& a) {
  std::cout << "    " << std::left << std::setw(22) << label << std::right << " n=" << std::setw(4)
            << a.n << "   trans rms " << std::fixed << std::setprecision(2)
            << a.translation_rms_m * 1000.0 << " mm (max " << a.translation_max_m * 1000.0
            << ")   rot rms " << a.rotation_rms_deg << " deg (max " << a.rotation_max_deg << ")\n";
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  std::vector<std::string> pos;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value after " + a);
      return argv[++i];
    };
    try {
      if (a == "--config") o.config = next();
      else if (a == "--run") o.run_id = std::stoll(next());
      else if (a == "--min-obs") o.min_obs = std::stoi(next());
      else if (a == "--max-reproj") o.max_reproj_px = std::stod(next());
      else if (a == "--min-size") o.min_size_px = std::stod(next());
      else if (a == "--min-ambiguity") o.min_ambiguity = std::stod(next());
      else if (a == "--dry-run") o.dry_run = true;
      else if (a == "--pairs") {
        const std::string v = next();
        const auto colon = v.find(':');
        if (colon == std::string::npos) return Usage();
        o.pair_lo = std::stoull(v.substr(0, colon));
        o.pair_hi = std::stoull(v.substr(colon + 1));
        o.window_given = true;
      } else if (a.rfind("--", 0) == 0) return Usage();
      else pos.push_back(a);
    } catch (const std::exception& e) {
      std::cerr << "gt_from_trays: " << e.what() << "\n";
      return Usage();
    }
  }
  if (pos.size() != 2) return Usage();
  o.session = pos[0];
  o.db = pos[1];

  try {
    const TrayConfig trays = LoadTrayConfig(o.config + "/marker_trays.yaml");
    const fusion::ExtrinsicsSet ex = fusion::LoadExtrinsics(o.config + "/extrinsics.yaml");

    constexpr uint64_t kDefaultWindow = 30;
    ReplayFrameSource src(o.session);
    const size_t total = src.total_pairs();

    std::cout << "gt_from_trays\n"
              << "  session   : " << o.session << "  (" << total << " pairs)\n"
              << "  dictionary: " << trays.dictionary << "  marker "
              << trays.marker_length_m * 1000.0 << " mm\n"
              << "  window    : "
              << (o.window_given ? "pair_index [" + std::to_string(o.pair_lo) + ", " +
                                       std::to_string(o.pair_hi) + ")"
                                 : "first " + std::to_string(kDefaultWindow) + " pairs")
              << "\n"
              << "  gates     : reproj <= " << o.max_reproj_px << " px, side >= " << o.min_size_px
              << " px, ambiguity >= " << o.min_ambiguity << ", >= " << o.min_obs
              << " observations per class\n\n";

    calibration::MarkerDetector det(trays.dictionary, trays.marker_length_m);

    std::vector<Sample> samples;
    std::map<std::string, int> rejected_reproj, rejected_size, rejected_ambig, unknown_id;
    // Smallest marker seen at all, accepted or not: it explains an empty result
    // far better than a bare "0 observations".
    std::map<std::string, double> smallest_seen;
    std::vector<uint64_t> all_pairs;
    size_t frames_scanned = 0;

    while (true) {
      auto pair = src.NextPair(0);
      if (!pair) break;
      all_pairs.push_back(pair->pair_index);
      const bool in_window = o.window_given
                                 ? (pair->pair_index >= o.pair_lo && pair->pair_index < o.pair_hi)
                                 : (all_pairs.size() <= kDefaultWindow);
      if (!in_window) continue;

      for (const auto& [camera_id, frame] : pair->frames) {
        if (frame.rgb.empty()) continue;
        auto cam = ex.cameras.find(camera_id);
        if (cam == ex.cameras.end()) {
          throw std::runtime_error("camera '" + camera_id +
                                   "' is in the session but not in extrinsics.yaml");
        }
        cv::Mat bgr;
        cv::cvtColor(frame.rgb, bgr, cv::COLOR_RGB2BGR);
        ++frames_scanned;

        for (const calibration::MarkerObservation& m : det.Detect(bgr, frame.K)) {
          const auto known = trays.cls_of_marker.find(m.id);
          if (known == trays.cls_of_marker.end()) {
            ++unknown_id[std::to_string(m.id)];
            continue;
          }
          auto& sm = smallest_seen[camera_id];
          sm = (sm == 0.0) ? m.apparent_size_px : std::min(sm, m.apparent_size_px);
          if (m.apparent_size_px < o.min_size_px) { ++rejected_size[known->second]; continue; }
          if (m.reprojection_px > o.max_reproj_px) { ++rejected_reproj[known->second]; continue; }
          // A marker whose two planar solutions fit equally well was a coin
          // flip, and the loser is wrong by several degrees while reprojecting
          // perfectly. Reprojection cannot catch this; only the ratio can.
          if (m.ambiguity < o.min_ambiguity) { ++rejected_ambig[known->second]; continue; }

          const Tray& tray = trays.trays.at(known->second);
          const Eigen::Isometry3d T_cam_object = m.T_cam_marker * MarkerToObject(tray, m.id);
          samples.push_back({known->second, camera_id, m.id, m.apparent_size_px,
                             cam->second.T_world_cam * T_cam_object});
        }
      }
    }

    std::cout << "scanned " << frames_scanned << " frames, accepted " << samples.size()
              << " marker observations\n";
    for (const auto& [cam, px] : smallest_seen) {
      std::cout << "  " << cam << ": smallest marker seen " << std::fixed << std::setprecision(0)
                << px << " px a side"
                << (px < o.min_size_px ? "  <-- below --min-size; capture at higher resolution"
                                       : "")
                << "\n";
    }
    for (const auto& [id, n] : unknown_id) {
      std::cout << "  NOTE  marker id " << id << " seen " << n
                << "x but is on no tray -- ignored\n";
    }
    std::cout << "\n";

    // Group by class, and by (class, camera) so the two views can be compared.
    std::map<std::string, std::vector<Eigen::Isometry3d>> by_cls;
    std::map<std::string, std::map<std::string, std::vector<Eigen::Isometry3d>>> by_cls_cam;
    for (const Sample& s : samples) {
      by_cls[s.cls].push_back(s.pose_w);
      by_cls_cam[s.cls][s.camera_id].push_back(s.pose_w);
    }

    std::vector<GroundTruthRow> rows;
    bool ok = true;
    for (const auto& [cls, poses] : by_cls) {
      const PoseAverage avg = AveragePoses(poses);
      const Eigen::Vector3d t = avg.mean.translation();
      double size_sum = 0.0;
      int size_n = 0;
      for (const Sample& sm : samples) {
        if (sm.cls == cls) { size_sum += sm.size_px; ++size_n; }
      }
      std::cout << cls << "\n"
                << "    world pose            (" << std::fixed << std::setprecision(4) << t.x()
                << ", " << t.y() << ", " << t.z() << ") m\n"
                << "    mean marker size      " << std::setprecision(0)
                << (size_n ? size_sum / size_n : 0.0) << " px a side\n";
      PrintSpread("scatter (all views)", avg);
      for (const auto& [cam, p] : by_cls_cam.at(cls)) PrintSpread(cam, AveragePoses(p));

      // The two cameras are independent measurements of the same object, so
      // their disagreement bounds ground-truth accuracy in a way the pooled
      // scatter cannot -- pooling hides a constant per-camera bias.
      if (by_cls_cam.at(cls).size() == 2) {
        auto it = by_cls_cam.at(cls).begin();
        const Eigen::Isometry3d a = AveragePoses(it->second).mean;
        const std::string cam_a = it->first;
        ++it;
        const Eigen::Isometry3d b = AveragePoses(it->second).mean;
        const double dt = (a.translation() - b.translation()).norm();
        const double dr = Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle() * 180.0 /
                          M_PI;
        std::cout << "    cross-camera        " << cam_a << " vs " << it->first << ":  "
                  << std::setprecision(2) << dt * 1000.0 << " mm, " << dr << " deg\n";
      }
      if (avg.n < o.min_obs) {
        std::cout << "    REFUSED: only " << avg.n << " observations (need " << o.min_obs
                  << ")\n";
        ok = false;
        continue;
      }
      for (uint64_t p : all_pairs) {
        GroundTruthRow g;
        g.pair_index = p;
        g.cls = cls;
        g.instance = 0;
        g.pose_w = avg.mean;
        g.source = "aruco_tray";
        rows.push_back(g);
      }
      std::cout << "\n";
    }
    for (const auto& [cls, n] : rejected_reproj)
      std::cout << "  " << cls << ": " << n << " observations rejected on reprojection\n";
    for (const auto& [cls, n] : rejected_size)
      std::cout << "  " << cls << ": " << n << " observations rejected as too small\n";

    if (by_cls.empty()) {
      std::cerr << "\nNo tray markers were found. Ground truth is unchanged.\n";
      return 3;
    }
    if (!ok) {
      std::cerr << "\nAt least one class did not clear its gates. Nothing written -- "
                   "a partially-populated ground_truth table scores some pairs and "
                   "silently skips others.\n";
      return 4;
    }
    if (o.dry_run) {
      std::cout << "\n--dry-run: " << rows.size() << " rows NOT written\n";
      return 0;
    }

    int64_t run = o.run_id;
    if (run < 0) run = LatestRunId(o.db);
    if (run < 0) throw std::runtime_error("no runs in " + o.db);
    MetricsLog log(o.db);
    log.WriteGroundTruth(run, rows);
    std::cout << "\nwrote " << rows.size() << " ground-truth rows for " << by_cls.size()
              << " class(es) across " << all_pairs.size() << " pairs into " << o.db << " run "
              << run << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "gt_from_trays: " << e.what() << "\n";
    return 1;
  }
}
