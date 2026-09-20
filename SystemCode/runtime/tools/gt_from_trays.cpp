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
//   gt_from_trays <input> <metrics.sqlite> [options]
//
// `input` is a recorded session by default, or a STILLS directory with --stills.
//
//     --stills         read <input>/<camera_id>/*.png as independent stills
//                      instead of replaying a session. This is the mode that
//                      matters: see RESOLUTION below. The layout is the one
//                      `calibrate capture` already writes.
//     --intrinsics F   per-camera K for the stills, at the stills' own
//                      resolution. Defaults to <input>/intrinsics.yaml if that
//                      exists, so a --factory run is reproducible afterwards
//                      with no camera attached.
//     --factory        read intrinsics from the attached devices (serials from
//                      cameras.yaml) at the stills' resolution, and WRITE them
//                      to <input>/intrinsics.yaml. Needs the rig; use it once,
//                      on capture day.
//     --config DIR     config directory (default: ./config)
//     --pairs A:B      solve window, inclusive-exclusive, by pair_index
//                      (default: the first 30 pairs the session YIELDS, which
//                      is not pair_index [0,30) -- a session's indices start
//                      after the discarded warm-up frames, so a fixed range
//                      would silently scan nothing). Ignored with --stills,
//                      where every still is used.
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
// confident nonsense. --stills is how those high-resolution frames get in:
// they cannot come from a recorded session, which streams 848x480.
//
// Registering from high-resolution stills and then streaming at 848x480 is
// sound because the camera POSE does not depend on sampling -- the projection
// centre is physically fixed, measured half-FOV 34.93 deg at 848x480 against
// 35.10 deg at 1920x1080 (device_registry.hpp). The INTRINSICS do depend on it,
// which is why the stills carry their own K and why a mismatch between the
// declared resolution and the image is refused rather than scaled.
//     --min-ambiguity R  reject a marker whose two planar poses fit almost
//                      equally well (default 2.0; see MarkerObservation::ambiguity)
//     --joint          solve ONE pose per tray from all its markers' corners at
//                      once, instead of one pose per marker averaged. It widens
//                      the effective target from the 40 mm marker to the marker
//                      spread (114-204 mm here) and is better conditioned in
//                      depth -- measured on synthetic corners, 1.63 mm rms
//                      against 1.73 mm.
//
//                      NOT the default, on evidence. On the first real tray
//                      stills (2026-09-20) it improved the pooled scatter a lot
//                      -- bowl 6.51 -> 2.69 mm, rotation 0.83 -> 0.20 deg -- but
//                      moved the CROSS-CAMERA disagreement, which this file
//                      argues is the figure that does not lie, the wrong way for
//                      two trays of three: bowl 2.71 -> 5.36, mustard 3.78 ->
//                      5.51, red_cube 7.55 -> 6.06 mm. Its joint reprojection
//                      also sits at 0.686 px rms where free per-marker poses
//                      reach ~0.2 px, so the printed trays do not quite match a
//                      rigid coplanar layout. A uniform print-scale error was
//                      tested and ruled out (reprojection is minimised at scale
//                      1.000 and degrades either side), which leaves tray
//                      flatness -- the calliper check PRINT_PLAN gates on, and
//                      which has not been done. Resolve that before making this
//                      the default.
//     --dry-run        report only, write nothing
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <fstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "the_eyes/camera/config.hpp"
#include "the_eyes/camera/device_registry.hpp"

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
  // One pose per tray from all its markers' corners at once, instead of one
  // pose per marker averaged. Better conditioned in principle and measurably
  // better on synthetic corners -- but NOT the default, because on the first
  // real trays it moved the cross-camera disagreement the wrong way for two of
  // three trays. See --joint in the usage above.
  bool joint = false;
  bool dry_run = false;
  bool stills = false;
  bool factory = false;
  std::string intrinsics;
};

int Usage() {
  std::cerr << "usage: gt_from_trays <input> <metrics.sqlite> [--stills] [--factory]\n"
               "         [--intrinsics FILE] [--config DIR] [--pairs A:B] [--run N]\n"
               "         [--min-obs N] [--max-reproj PX] [--min-size PX]\n"
               "         [--min-ambiguity R] [--dry-run]\n\n"
               "  <input> is a recorded session, or a stills directory with --stills.\n"
               "  Ground truth wants 1920x1080 stills, not the pipeline's 848x480.\n";
  return 2;
}

// One accepted observation of one object, already in the world frame.
struct Sample {
  std::string cls;
  std::string camera_id;
  int markers_used = 1;   // 1 on the --per-marker path, else the joint solve's count
  double size_px = 0.0;
  Eigen::Isometry3d pose_w;
};

// Per-camera intrinsics for a set of stills, AT THE STILLS' OWN RESOLUTION.
// The size is carried with K and checked against every image: intrinsics are
// per-resolution, and silently using the pipeline's 848x480 K on a 1920x1080
// still would bias every pose while looking entirely healthy.
struct StillsIntrinsics {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  int width = 0;
  int height = 0;
};

std::map<std::string, StillsIntrinsics> LoadIntrinsics(const std::string& path) {
  std::map<std::string, StillsIntrinsics> out;
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node cams = root["cameras"];
  if (!cams || !cams.IsMap() || cams.size() == 0) {
    throw std::runtime_error(path + ": 'cameras' must be a non-empty map");
  }
  for (const auto& kv : cams) {
    StillsIntrinsics si;
    const YAML::Node& n = kv.second;
    if (!n["K"] || n["K"].size() != 9) {
      throw std::runtime_error(path + ": camera '" + kv.first.as<std::string>() +
                               "' needs a 9-element row-major K");
    }
    for (int i = 0; i < 9; ++i) si.K(i / 3, i % 3) = n["K"][i].as<double>();
    if (!n["width"] || !n["height"]) {
      throw std::runtime_error(path + ": camera '" + kv.first.as<std::string>() +
                               "' needs width and height -- K without the resolution it was "
                               "measured at cannot be checked against an image");
    }
    si.width = n["width"].as<int>();
    si.height = n["height"].as<int>();
    out[kv.first.as<std::string>()] = si;
  }
  return out;
}

void SaveIntrinsics(const std::string& path,
                    const std::map<std::string, StillsIntrinsics>& in) {
  std::ofstream f(path);
  if (!f) throw std::runtime_error("cannot write " + path);
  f << "# Written by gt_from_trays --factory. These are the device's own colour\n"
       "# intrinsics at the resolution of the stills beside this file, captured so\n"
       "# the ground-truth solve can be repeated later with no camera attached.\n"
       "cameras:\n";
  f << std::setprecision(17);
  for (const auto& [id, si] : in) {
    f << "  " << id << ":\n    width: " << si.width << "\n    height: " << si.height
      << "\n    K: [";
    for (int i = 0; i < 9; ++i) f << (i ? ", " : "") << si.K(i / 3, i % 3);
    f << "]\n";
  }
}

// <dir>/<camera_id>/*.png|jpg, the layout `calibrate capture` already writes.
std::map<std::string, std::vector<std::filesystem::path>> FindStills(
    const std::filesystem::path& dir) {
  std::map<std::string, std::vector<std::filesystem::path>> out;
  if (!std::filesystem::is_directory(dir)) {
    throw std::runtime_error("stills: '" + dir.string() + "' is not a directory");
  }
  for (const auto& cam_entry : std::filesystem::directory_iterator(dir)) {
    if (!cam_entry.is_directory()) continue;
    std::vector<std::filesystem::path> imgs;
    for (const auto& f : std::filesystem::directory_iterator(cam_entry.path())) {
      const std::string ext = f.path().extension().string();
      if (ext != ".png" && ext != ".jpg" && ext != ".jpeg") continue;
      // A recorded session's directory has the same shape as a stills
      // directory, and its depth maps would be scanned as if they were colour
      // frames -- finding nothing, but inflating the frame count and hiding
      // that the wrong directory was passed.
      if (f.path().stem().string().size() >= 6 &&
          f.path().stem().string().rfind(".depth") ==
              f.path().stem().string().size() - 6) {
        continue;
      }
      imgs.push_back(f.path());
    }
    // Deterministic order: the same stills must give the same answer twice.
    std::sort(imgs.begin(), imgs.end());
    if (!imgs.empty()) out[cam_entry.path().filename().string()] = std::move(imgs);
  }
  return out;
}

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
      else if (a == "--joint") o.joint = true;
      else if (a == "--dry-run") o.dry_run = true;
      else if (a == "--stills") o.stills = true;
      else if (a == "--factory") o.factory = true;
      else if (a == "--intrinsics") o.intrinsics = next();
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

    std::cout << "gt_from_trays\n"
              << "  input     : " << o.session << (o.stills ? "  (stills)" : "  (session)")
              << "\n"
              << "  dictionary: " << trays.dictionary << "  marker "
              << trays.marker_length_m * 1000.0 << " mm\n"
              << "  solve     : "
              << (o.joint ? "one pose per TRAY from all its corners (joint)"
                          : "one pose per MARKER, averaged")
              << "\n"
              << "  gates     : reproj <= " << o.max_reproj_px << " px, side >= " << o.min_size_px
              << " px, ambiguity >= " << o.min_ambiguity << ", >= " << o.min_obs
              << " observations per class\n";

    calibration::MarkerDetector det(trays.dictionary, trays.marker_length_m);

    std::vector<Sample> samples;
    std::map<std::string, int> rejected_reproj, rejected_size, rejected_ambig, unknown_id;
    // Smallest marker seen at all, accepted or not: it explains an empty result
    // far better than a bare "0 observations".
    std::map<std::string, double> smallest_seen;
    std::vector<uint64_t> all_pairs;
    size_t frames_scanned = 0;

    // One scan path, whichever source the frames came from.
    auto scan = [&](const std::string& camera_id, const cv::Mat& bgr, const Eigen::Matrix3d& K) {
      auto cam = ex.cameras.find(camera_id);
      if (cam == ex.cameras.end()) {
        throw std::runtime_error("camera '" + camera_id +
                                 "' is in the input but not in extrinsics.yaml");
      }
      ++frames_scanned;
      // Markers that passed the per-marker gates, grouped by the tray they are
      // on. The gates stay per-marker either way: a marker that is too small,
      // fits badly or was a coin flip should not enter a joint solve any more
      // than it should enter an average.
      std::map<std::string, std::vector<calibration::MarkerObservation>> accepted;
      for (const calibration::MarkerObservation& m : det.Detect(bgr, K)) {
        const auto known = trays.cls_of_marker.find(m.id);
        if (known == trays.cls_of_marker.end()) {
          ++unknown_id[std::to_string(m.id)];
          continue;
        }
        auto& sm = smallest_seen[camera_id];
        sm = (sm == 0.0) ? m.apparent_size_px : std::min(sm, m.apparent_size_px);
        if (m.apparent_size_px < o.min_size_px) { ++rejected_size[known->second]; continue; }
        if (m.reprojection_px > o.max_reproj_px) { ++rejected_reproj[known->second]; continue; }
        // A marker whose two planar solutions fit equally well was a coin flip,
        // and the loser is wrong by several degrees while reprojecting
        // perfectly. Reprojection cannot catch this; only the ratio can.
        if (m.ambiguity < o.min_ambiguity) { ++rejected_ambig[known->second]; continue; }

        if (!o.joint) {
          const Tray& tray = trays.trays.at(known->second);
          const Eigen::Isometry3d T_cam_object = m.T_cam_marker * MarkerToObject(tray, m.id);
          samples.push_back({known->second, camera_id, 1, m.apparent_size_px,
                             cam->second.T_world_cam * T_cam_object});
        } else {
          accepted[known->second].push_back(m);
        }
      }
      if (!o.joint) return;

      // One pose per tray per frame, from every corner of every accepted marker
      // on it. With one marker this is exactly the old single-marker solve; with
      // more it is better conditioned in depth, which is the whole point.
      for (const auto& [cls, ms] : accepted) {
        const Tray& tray = trays.trays.at(cls);
        std::map<int, Eigen::Vector3d> t_tray_marker;
        for (const TrayMarker& tm : tray.markers) t_tray_marker[tm.id] = tm.t_tray_marker;

        const calibration::MultiMarkerPose jp =
            calibration::SolveMultiMarkerPose(ms, t_tray_marker, trays.marker_length_m, K);
        // The joint fit gets the same two gates as a single marker did. A tray
        // whose markers disagree with their printed layout shows up here as a
        // reprojection that no single marker would have reported.
        if (jp.reprojection_px > o.max_reproj_px) { ++rejected_reproj[cls]; continue; }
        if (jp.ambiguity < o.min_ambiguity) { ++rejected_ambig[cls]; continue; }

        Eigen::Isometry3d T_tray_object = Eigen::Isometry3d::Identity();
        T_tray_object.translation() = tray.t_tray_object;

        double size_sum = 0.0;
        for (const calibration::MarkerObservation& m : ms) size_sum += m.apparent_size_px;
        samples.push_back({cls, camera_id, jp.markers_used,
                           size_sum / static_cast<double>(ms.size()),
                           cam->second.T_world_cam * (jp.T_cam_frame * T_tray_object)});
      }
    };

    if (o.stills) {
      const auto by_cam = FindStills(o.session);
      if (by_cam.empty()) {
        throw std::runtime_error("stills: no <camera_id>/ subdirectory with images under '" +
                                 o.session + "'");
      }
      // Resolution of the stills, needed before intrinsics can be checked or read.
      std::map<std::string, cv::Size> sizes;
      for (const auto& [cam, files] : by_cam) {
        const cv::Mat probe = cv::imread(files.front().string(), cv::IMREAD_COLOR);
        if (probe.empty()) throw std::runtime_error("cannot read " + files.front().string());
        sizes[cam] = probe.size();
      }

      std::map<std::string, StillsIntrinsics> K;
      const std::string sidecar = (std::filesystem::path(o.session) / "intrinsics.yaml").string();
      std::string k_source;
      if (!o.intrinsics.empty()) {
        K = LoadIntrinsics(o.intrinsics);
        k_source = o.intrinsics;
      } else if (std::filesystem::exists(sidecar)) {
        K = LoadIntrinsics(sidecar);
        k_source = sidecar;
      } else if (o.factory) {
        const camera::CameraAgentConfig cams =
            camera::LoadCameraAgentConfig(o.config + "/cameras.yaml");
        for (const auto& [cam, sz] : sizes) {
          std::vector<double> distortion;
          StillsIntrinsics si;
          si.width = sz.width;
          si.height = sz.height;
          si.K = camera::FactoryColorIntrinsics(cams.at(cam).serial, sz.width, sz.height,
                                                &distortion);
          K[cam] = si;
        }
        SaveIntrinsics(sidecar, K);
        k_source = "attached devices (written to " + sidecar + ")";
      } else {
        throw std::runtime_error(
            "stills need intrinsics at their own resolution. Pass --factory to read them "
            "from the attached devices (they will be saved beside the stills), or "
            "--intrinsics FILE. They are NOT taken from the session or scaled from another "
            "resolution, because a wrong K biases every pose while everything still looks "
            "healthy.");
      }

      std::cout << "  intrinsics: " << k_source << "\n";
      for (const auto& [cam, files] : by_cam) {
        const auto it = K.find(cam);
        if (it == K.end()) {
          throw std::runtime_error("no intrinsics for camera '" + cam + "'");
        }
        if (it->second.width != sizes[cam].width || it->second.height != sizes[cam].height) {
          throw std::runtime_error(
              "camera '" + cam + "': stills are " + std::to_string(sizes[cam].width) + "x" +
              std::to_string(sizes[cam].height) + " but the intrinsics are for " +
              std::to_string(it->second.width) + "x" + std::to_string(it->second.height) +
              ". Intrinsics are per-resolution; this is refused rather than rescaled.");
        }
        std::cout << "  " << cam << ": " << files.size() << " stills at " << sizes[cam].width
                  << "x" << sizes[cam].height << "\n";
      }
      std::cout << "\n";

      for (const auto& [cam, files] : by_cam) {
        for (const auto& f : files) {
          const cv::Mat img = cv::imread(f.string(), cv::IMREAD_COLOR);
          if (img.empty()) throw std::runtime_error("cannot read " + f.string());
          if (img.size() != sizes[cam]) {
            throw std::runtime_error(f.string() + " is a different size from the other stills");
          }
          scan(cam, img, K.at(cam).K);
        }
      }

      // No session to enumerate: the pairs to write come from the run itself.
      int64_t run = o.run_id;
      if (run < 0) run = LatestRunId(o.db);
      if (run < 0) throw std::runtime_error("no runs in " + o.db);
      all_pairs = PairIndices(o.db, run);
      if (all_pairs.empty()) {
        throw std::runtime_error("run " + std::to_string(run) + " in " + o.db +
                                 " logged no frames, so there is nothing to attach ground "
                                 "truth to");
      }
      o.run_id = run;
    } else {
      ReplayFrameSource src(o.session);
      std::cout << "  pairs     : " << src.total_pairs() << "\n"
                << "  window    : "
                << (o.window_given ? "pair_index [" + std::to_string(o.pair_lo) + ", " +
                                         std::to_string(o.pair_hi) + ")"
                                   : "first " + std::to_string(kDefaultWindow) + " pairs")
                << "\n\n";
      while (true) {
        auto pair = src.NextPair(0);
        if (!pair) break;
        all_pairs.push_back(pair->pair_index);
        const bool in_window =
            o.window_given ? (pair->pair_index >= o.pair_lo && pair->pair_index < o.pair_hi)
                           : (all_pairs.size() <= kDefaultWindow);
        if (!in_window) continue;
        for (const auto& [camera_id, frame] : pair->frames) {
          if (frame.rgb.empty()) continue;
          cv::Mat bgr;
          cv::cvtColor(frame.rgb, bgr, cv::COLOR_RGB2BGR);
          scan(camera_id, bgr, frame.K);
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
