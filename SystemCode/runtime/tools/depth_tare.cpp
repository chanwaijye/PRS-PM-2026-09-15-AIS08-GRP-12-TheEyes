// depth_tare -- fits each camera's depth scale against the bare table.
//
//   depth_tare <empty_table_session> [--config-dir DIR] [--half-x M] [--half-y M] [--write]
//
// The session is a recording of the EMPTY desk (`capture_day.sh begin` makes
// one). Every depth pixel inside the shared region is compared with where its
// ray meets world z = 0 through the calibrated extrinsics; see
// runtime/depth_correction.hpp for why this exists and what it found.
//
// Fits on the even pairs and judges on the odd ones. A scale is APPLIED only
// when it leaves a quarter or less of the held-out error across range; otherwise the camera
// is written as 1.0 and the reason is printed. With --write the result goes to
// <config-dir>/depth_correction.yaml, which the runner and the tools pick up.
//
//     --half-x, --half-y   half-extent of the table region used, metres
//                          (default 0.230 x 0.185, the 2026-10-04 shared view)
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/runtime/depth_correction.hpp"
#include "the_eyes/runtime/replay_frame_source.hpp"

using namespace the_eyes;
using namespace the_eyes::runtime;

namespace {

int Usage() {
  std::cerr << "usage: depth_tare <empty_table_session> [--config-dir DIR] [--half-x M]\n"
               "                  [--half-y M] [--write]\n";
  return 2;
}

double Median(std::vector<double> v) {
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

// Median height of the measured table above world z = 0, mm, after `scale`.
// A point at camera-z `zm` on a ray that meets the table at `zp` sits at
// t_z * (1 - zm / zp).
double TableHeightMm(const PlaneSamples& s, double scale, double cam_height_m) {
  std::vector<double> h;
  h.reserve(s.z_measured.size());
  for (size_t i = 0; i < s.z_measured.size(); ++i) {
    h.push_back(cam_height_m * (1.0 - scale * s.z_measured[i] / s.z_plane[i]) * 1000.0);
  }
  return Median(std::move(h));
}

std::string Bins(const std::vector<double>& e) {
  std::ostringstream os;
  os << std::showpos << std::fixed << std::setprecision(1);
  for (double x : e) os << std::setw(6) << x * 1000.0;
  return os.str();
}

}  // namespace

int main(int argc, char** argv) {
  std::string session, config = "config";
  double half_x = 0.230, half_y = 0.185;
  bool write = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value after " + a);
      return argv[++i];
    };
    try {
      if (a == "--config-dir") config = next();
      else if (a == "--half-x") half_x = std::stod(next());
      else if (a == "--half-y") half_y = std::stod(next());
      else if (a == "--write") write = true;
      else if (a.rfind("--", 0) == 0) return Usage();
      else if (session.empty()) session = a;
      else return Usage();
    } catch (const std::exception& e) {
      std::cerr << "depth_tare: " << e.what() << "\n";
      return Usage();
    }
  }
  if (session.empty()) return Usage();

  try {
    const fusion::ExtrinsicsSet ex = fusion::LoadExtrinsics(config + "/extrinsics.yaml");
    std::map<std::string, PlaneSamples> fit, check;
    ReplayFrameSource src(session);
    size_t n = 0;
    while (true) {
      auto pair = src.NextPair(0);
      if (!pair) break;
      for (const auto& [cam, frame] : pair->frames) {
        if (frame.depth.empty()) continue;
        const auto it = ex.cameras.find(cam);
        if (it == ex.cameras.end()) {
          throw std::runtime_error("camera '" + cam + "' is in the session but not in extrinsics.yaml");
        }
        CollectTablePlaneSamples(frame, it->second.T_world_cam, half_x, half_y, 0.05, 3,
                                 (n % 2 == 0) ? &fit[cam] : &check[cam]);
      }
      ++n;
    }
    if (n < 4) throw std::runtime_error("session has fewer than 4 pairs");

    std::cout << "depth_tare\n  session : " << session << " (" << n << " pairs)\n"
              << "  region  : +-" << half_x * 1000 << " x +-" << half_y * 1000
              << " mm of the table about the world origin\n"
              << "  fit on even pairs, judged on odd pairs; errors are depth minus table, mm,\n"
              << "  in five equal-count range bins from near to far\n\n";

    std::map<std::string, double> applied;
    std::ostringstream yaml;
    for (const auto& [cam, f] : fit) {
      const PlaneSamples& c = check.at(cam);
      if (f.z_measured.size() < 10000 || c.z_measured.size() < 10000) {
        throw std::runtime_error(cam + ": too few table pixels in the region (" +
                                 std::to_string(f.z_measured.size()) +
                                 ") -- is the desk empty and the region inside the view?");
      }
      const double s = FitDepthScale(f);
      const bool apply = FitIsWorthApplying(c, s);
      const double h_cam = ex.cameras.at(cam).T_world_cam.translation().z();
      const double before = TableHeightMm(c, 1.0, h_cam), after = TableHeightMm(c, s, h_cam);
      double lo = c.z_plane[0], hi = c.z_plane[0];
      for (double z : c.z_plane) { lo = std::min(lo, z); hi = std::max(hi, z); }
      std::cout << cam << "   " << c.z_measured.size() << " held-out pixels, range " << std::fixed
                << std::setprecision(2) << lo << "-" << hi << " m\n"
                << "    raw            table at " << std::showpos << std::setprecision(2) << before
                << std::noshowpos << " mm   by range:" << Bins(BinnedDepthError(c, 1.0)) << "\n"
                << "    x" << std::setprecision(4) << s << "        table at " << std::showpos
                << std::setprecision(2) << after << std::noshowpos
                << " mm   by range:" << Bins(BinnedDepthError(c, s)) << "\n"
                << "    " << (apply ? "APPLIED: the scale removes at least three quarters of the error across range"
                                    : "NOT APPLIED: more than a quarter of the error survives the best scale, "
                                      "so this is not a scale error; left at 1.0")
                << "\n\n";
      applied[cam] = apply ? s : 1.0;
      yaml << "  " << cam << ":\n    scale: " << std::setprecision(4) << std::fixed << applied[cam]
           << "\n    fitted_scale: " << s << "\n    table_height_before_mm: "
           << std::setprecision(2) << before << "\n    table_height_after_mm: "
           << (apply ? after : before) << "\n    held_out_pixels: " << c.z_measured.size() << "\n";
    }

    if (!write) {
      std::cout << "not written (pass --write to save " << config << "/depth_correction.yaml)\n";
      return 0;
    }
    const std::string path = config + "/depth_correction.yaml";
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write " + path);
    out << "# Per-camera depth scale -- GENERATED by build/runtime/depth_tare. Do not hand-edit;\n"
           "# re-run it on an empty-table recording. See runtime/depth_correction.hpp.\n"
           "#\n"
           "# `scale` multiplies metric depth for that camera wherever the pipeline or a\n"
           "# tool reads it; recordings keep the raw sensor values. A camera whose error\n"
           "# is not a scale is written as 1.0, with what was fitted kept beside it.\n"
           "fitted_from: \"" << std::filesystem::absolute(session).lexically_normal().string() << "\"\n"
           "region_half_extent_m: [" << half_x << ", " << half_y << "]\n"
           "cameras:\n" << yaml.str();
    std::cout << "wrote " << path << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "depth_tare: " << e.what() << "\n";
    return 1;
  }
}
