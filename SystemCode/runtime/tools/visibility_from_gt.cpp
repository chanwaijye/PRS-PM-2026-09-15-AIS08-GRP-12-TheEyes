// visibility_from_gt -- how much of each object each camera actually sees.
//
//   visibility_from_gt <session_dir> <metrics.sqlite> [options]
//
// Renders every ground-truth object's mesh into each view of every recorded
// pair and classifies its silhouette against the measured depth
// (runtime/visibility.hpp). That is the proposal's own definition of the
// occlusion tiers (7.1): T4 means under 25% of the silhouette visible from one
// view, T3 means 25-75%. Until this existed a scene's tier was whatever the
// person who staged it believed -- and a T4 scene that is really 40% visible
// cannot be re-shot once the rig is gone.
//
// Needs ground truth in the run first (gt_from_trays). The session is the
// recording the run was replayed from; ground truth is matched to its pairs by
// pair_index.
//
//     --run N         run_id (default: the newest in the database)
//     --config-dir D  config directory (default: ./config)
//     --tol-mm T      depth agreement tolerance (default 20). Depth noise is
//                     2-4 mm at this standoff and ground truth agrees across
//                     cameras to 3-8 mm; an occluder is centimetres in front.
//     --erode-px E    shrink each silhouette first (default 2), so a pixel or
//                     two of ground-truth error at the rim is not scored
//     --max-pairs N   stop after N pairs that have ground truth
//     --csv FILE      per pair, per view, per object (default:
//                     <db dir>/visibility_run<N>.csv)
//     --overlay DIR   the first scored pair per camera with each silhouette
//                     tinted by class -- green seen, blue behind, red
//                     occluded, yellow no depth (default: <db dir>/visibility_overlay)
//
// On an unoccluded (T1) scene read the `behind` column: it is the share of
// model pixels that landed on something farther away, i.e. how well ground
// truth sits on the object. Large there means check the overlay before
// trusting any number scored against that ground truth.
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/pose/mesh_registry.hpp"
#include "the_eyes/runtime/backends.hpp"
#include "the_eyes/runtime/metrics_log.hpp"
#include "the_eyes/runtime/replay_frame_source.hpp"
#include "the_eyes/runtime/visibility.hpp"

using namespace the_eyes;
using namespace the_eyes::runtime;
namespace fs = std::filesystem;

namespace {

struct Options {
  std::string session;
  std::string db;
  std::string config = "config";
  int64_t run_id = -1;
  double tol_mm = 20.0;
  int erode_px = 2;
  uint64_t max_pairs = 0;
  std::string csv;
  std::string overlay;
};

int Usage() {
  std::cerr << "usage: visibility_from_gt <session_dir> <metrics.sqlite> [--run N]\n"
               "         [--config-dir DIR] [--tol-mm T] [--erode-px E] [--max-pairs N]\n"
               "         [--csv FILE] [--overlay DIR]\n";
  return 2;
}

double Median(std::vector<double> v) {
  v.erase(std::remove_if(v.begin(), v.end(), [](double x) { return std::isnan(x); }), v.end());
  if (v.empty()) return std::nan("");
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// One view of one object across the run.
struct Series {
  std::vector<double> visible, invalid, behind;
  int silhouette_px = 0;  // from the first pair; constant for a static scene
};

// The proposal's tiers (6.4 / 7.1), read off the median over the run.
std::string Band(double v) {
  if (std::isnan(v)) return "no depth on the object";
  if (v < 0.25) return "OCCLUDED  (<25%, T4 band)";
  if (v <= 0.75) return "partial   (25-75%, T3 band)";
  return "clear";
}

cv::Vec3b Tint(VisLabel l) {
  switch (l) {  // BGR
    case VisLabel::kConsistent: return {60, 200, 60};
    case VisLabel::kBehind: return {230, 120, 30};
    case VisLabel::kOccluded: return {40, 40, 230};
    case VisLabel::kInvalid: return {30, 220, 230};
    default: return {0, 0, 0};
  }
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
      if (a == "--run") o.run_id = std::stoll(next());
      else if (a == "--config-dir") o.config = next();
      else if (a == "--tol-mm") o.tol_mm = std::stod(next());
      else if (a == "--erode-px") o.erode_px = std::stoi(next());
      else if (a == "--max-pairs") o.max_pairs = std::stoull(next());
      else if (a == "--csv") o.csv = next();
      else if (a == "--overlay") o.overlay = next();
      else if (a.rfind("--", 0) == 0) return Usage();
      else pos.push_back(a);
    } catch (const std::exception& e) {
      std::cerr << "visibility_from_gt: " << e.what() << "\n";
      return Usage();
    }
  }
  if (pos.size() != 2 || o.tol_mm <= 0.0 || o.erode_px < 0) return Usage();
  o.session = pos[0];
  o.db = pos[1];

  try {
    const std::string repo_root = fs::absolute(o.config).parent_path().string();
    const fusion::ExtrinsicsSet ex = fusion::LoadExtrinsics(o.config + "/extrinsics.yaml");
    const pose::MeshRegistry meshes =
        pose::LoadMeshRegistry(o.config + "/meshes.yaml", true, repo_root);

    if (o.run_id < 0) o.run_id = LatestRunId(o.db);
    if (o.run_id < 0) throw std::runtime_error("no runs in " + o.db);
    const std::vector<GroundTruthRow> gt_rows = ReadGroundTruth(o.db, o.run_id);
    if (gt_rows.empty()) {
      std::cerr << "run " << o.run_id << " in " << o.db
                << " has no ground truth. Fill it with gt_from_trays first -- visibility is "
                   "measured against the ground-truth silhouette and there is nothing else to "
                   "measure it against.\n";
      return 3;
    }
    std::map<uint64_t, std::vector<const GroundTruthRow*>> gt_of_pair;
    for (const GroundTruthRow& g : gt_rows) gt_of_pair[g.pair_index].push_back(&g);

    const fs::path db_dir = fs::absolute(o.db).parent_path();
    if (o.csv.empty()) o.csv = (db_dir / ("visibility_run" + std::to_string(o.run_id) + ".csv")).string();
    if (o.overlay.empty()) o.overlay = (db_dir / "visibility_overlay").string();

    std::cout << "visibility_from_gt\n"
              << "  session   : " << o.session << "\n"
              << "  run       : " << o.run_id << " in " << o.db << ", " << gt_rows.size()
              << " ground-truth rows over " << gt_of_pair.size() << " pairs (source "
              << gt_rows.front().source << ")\n"
              << "  tolerance : " << o.tol_mm << " mm, silhouette eroded " << o.erode_px
              << " px\n\n";

    std::map<std::string, TriMesh> mesh_of;
    auto mesh_for = [&](const std::string& cls) -> const TriMesh& {
      auto it = mesh_of.find(cls);
      if (it != mesh_of.end()) return it->second;
      const MeshEntry* e = meshes.Find(cls);
      if (!e) throw std::runtime_error("ground truth names class '" + cls + "', not in meshes.yaml");
      return mesh_of.emplace(cls, LoadObjMesh(ResolveAgainst(repo_root, e->mesh_path))).first->second;
    };

    // Ground truth for a static scene is one pose written to every pair, so a
    // render is reused until the pose it was made from changes.
    struct Cached {
      Eigen::Isometry3d pose_w;
      cv::Mat rendered;
    };
    std::map<std::string, Cached> cache;

    std::ofstream csv(o.csv);
    if (!csv) throw std::runtime_error("cannot write " + o.csv);
    csv << "pair_index,camera_id,cls,instance,silhouette_px,invalid_px,occluded_px,"
           "consistent_px,behind_px,visible_fraction\n";

    std::map<std::string, Series> series;  // "<camera>  <cls>#<instance>"
    ReplayFrameSource src(o.session);
    uint64_t scored = 0, no_gt = 0;
    bool overlay_written = false;
    while (true) {
      auto pair = src.NextPair(0);
      if (!pair) break;
      const auto g = gt_of_pair.find(pair->pair_index);
      if (g == gt_of_pair.end()) {
        ++no_gt;
        continue;
      }
      for (const auto& [cam_id, frame] : pair->frames) {
        if (frame.depth.empty()) continue;
        const auto cam = ex.cameras.find(cam_id);
        if (cam == ex.cameras.end()) {
          throw std::runtime_error("camera '" + cam_id + "' is in the session but not in extrinsics.yaml");
        }
        const Eigen::Isometry3d T_cam_world = cam->second.T_world_cam.inverse();
        cv::Mat canvas;
        if (!overlay_written) cv::cvtColor(frame.rgb, canvas, cv::COLOR_RGB2BGR);

        for (const GroundTruthRow* row : g->second) {
          const std::string key = cam_id + "  " + row->cls + "#" + std::to_string(row->instance);
          Cached& c = cache[key];
          if (c.rendered.empty() || !c.pose_w.isApprox(row->pose_w, 1e-12) ||
              c.rendered.size() != frame.depth.size()) {
            c.pose_w = row->pose_w;
            c.rendered = RenderDepth(mesh_for(row->cls), T_cam_world * row->pose_w, frame.K,
                                     frame.depth.cols, frame.depth.rows);
          }
          cv::Mat labels;
          const VisibilityCounts vc =
              MeasureVisibility(c.rendered, frame.depth, frame.depth_scale_m, o.tol_mm / 1000.0,
                                o.erode_px, overlay_written ? nullptr : &labels);
          Series& s = series[key];
          if (s.visible.empty()) s.silhouette_px = vc.silhouette;
          s.visible.push_back(vc.visible_fraction());
          s.invalid.push_back(vc.invalid_fraction());
          s.behind.push_back(vc.behind_fraction());
          csv << pair->pair_index << "," << cam_id << "," << row->cls << "," << row->instance << ","
              << vc.silhouette << "," << vc.invalid << "," << vc.occluded << "," << vc.consistent
              << "," << vc.behind << "," << std::setprecision(6) << vc.visible_fraction() << "\n";

          if (!overlay_written) {
            for (int y = 0; y < labels.rows; ++y) {
              for (int x = 0; x < labels.cols; ++x) {
                const auto l = static_cast<VisLabel>(labels.at<uint8_t>(y, x));
                if (l == VisLabel::kNone) continue;
                cv::Vec3b& px = canvas.at<cv::Vec3b>(y, x);
                const cv::Vec3b t = Tint(l);
                for (int k = 0; k < 3; ++k) px[k] = static_cast<uint8_t>((px[k] + t[k]) / 2);
              }
            }
          }
        }
        if (!overlay_written) {
          fs::create_directories(o.overlay);
          const fs::path out =
              fs::path(o.overlay) / (cam_id + "_pair" + std::to_string(pair->pair_index) + ".png");
          cv::imwrite(out.string(), canvas);
        }
      }
      overlay_written = true;
      ++scored;
      if (o.max_pairs && scored >= o.max_pairs) break;
    }

    if (scored == 0) {
      std::cerr << "no pair of " << o.session << " has ground truth in run " << o.run_id
                << " -- is this the session the run was replayed from?\n";
      return 3;
    }

    std::cout << std::left << std::setw(30) << "view / object" << std::right << std::setw(7)
              << "pairs" << std::setw(8) << "sil px" << std::setw(9) << "visible" << std::setw(7)
              << "min" << std::setw(7) << "max" << std::setw(9) << "no depth" << std::setw(8)
              << "behind" << "   band (median visible)\n";
    bool behind_warning = false;
    for (const auto& [key, s] : series) {
      const double med = Median(s.visible);
      double lo = 1.0, hi = 0.0;
      for (double v : s.visible) {
        if (std::isnan(v)) continue;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
      }
      const double behind = Median(s.behind);
      std::cout << std::left << std::setw(30) << key << std::right << std::setw(7)
                << s.visible.size() << std::setw(8) << s.silhouette_px << std::fixed
                << std::setprecision(2) << std::setw(9) << med << std::setw(7) << lo
                << std::setw(7) << hi << std::setw(8) << std::setprecision(1)
                << 100.0 * Median(s.invalid) << "%" << std::setw(7) << 100.0 * behind << "%"
                << "   " << Band(med) << "\n";
      if (behind > 0.25) behind_warning = true;
    }
    std::cout << "\n" << scored << " pairs scored";
    if (no_gt) std::cout << ", " << no_gt << " session pairs had no ground truth in the run";
    std::cout << "\n  csv     : " << o.csv << "\n  overlay : " << o.overlay << "\n";
    if (behind_warning) {
      std::cout << "\nWARNING: in at least one view over a quarter of the model's pixels landed on "
                   "something farther away. That is ground truth (or depth) not sitting on the "
                   "object, not occlusion -- check the overlay before scoring against it.\n";
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "visibility_from_gt: " << e.what() << "\n";
    return 1;
  }
}
