#include "the_eyes/runtime/depth_correction.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::runtime {

double DepthCorrection::For(const std::string& camera_id) const {
  const auto it = scale.find(camera_id);
  return it == scale.end() ? 1.0 : it->second;
}

std::string DepthCorrection::Describe() const {
  if (scale.empty()) return "none";
  std::ostringstream os;
  os << std::fixed << std::setprecision(4);
  bool first = true;
  for (const auto& [cam, s] : scale) {
    os << (first ? "" : ", ") << cam << " x" << s;
    first = false;
  }
  return os.str();
}

DepthCorrection LoadDepthCorrection(const std::string& yaml_path) {
  DepthCorrection out;
  if (!std::filesystem::exists(yaml_path)) return out;
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception& e) {
    throw std::runtime_error("depth_correction.yaml: cannot read '" + yaml_path + "': " + e.what());
  }
  const YAML::Node cams = root["cameras"];
  if (!cams || !cams.IsMap()) {
    throw std::runtime_error("depth_correction.yaml: 'cameras' must be a map");
  }
  for (const auto& kv : cams) {
    const std::string cam = kv.first.as<std::string>();
    if (!kv.second["scale"]) {
      throw std::runtime_error("depth_correction.yaml: camera '" + cam + "' has no scale");
    }
    const double s = kv.second["scale"].as<double>();
    if (!(s > 0.9 && s < 1.1)) {
      throw std::runtime_error("depth_correction.yaml: camera '" + cam + "' scale " +
                               std::to_string(s) + " is outside (0.9, 1.1)");
    }
    out.scale[cam] = s;
  }
  return out;
}

void CollectTablePlaneSamples(const camera::CameraFrame& frame,
                              const Eigen::Isometry3d& T_world_cam, double half_x_m,
                              double half_y_m, double max_dev_m, int stride, PlaneSamples* out) {
  if (frame.depth.empty() || frame.depth.type() != CV_16UC1) {
    throw std::invalid_argument("CollectTablePlaneSamples: frame has no CV_16UC1 depth");
  }
  if (stride < 1) throw std::invalid_argument("CollectTablePlaneSamples: stride must be >= 1");
  const Eigen::Matrix3d Kinv = frame.K.inverse();
  const Eigen::Matrix3d R = T_world_cam.linear();
  const Eigen::Vector3d t = T_world_cam.translation();
  for (int v = 0; v < frame.depth.rows; v += stride) {
    const uint16_t* row = frame.depth.ptr<uint16_t>(v);
    for (int u = 0; u < frame.depth.cols; u += stride) {
      if (row[u] == 0) continue;
      const double z_meas = row[u] * frame.depth_scale_m;
      // The ray per unit of camera-z, in the world frame.
      const Eigen::Vector3d dir = R * (Kinv * Eigen::Vector3d(u, v, 1.0));
      if (dir.z() >= -1e-9) continue;  // not looking down at the table
      const double z_plane = -t.z() / dir.z();
      if (z_plane <= 0.0 || std::abs(z_meas - z_plane) > max_dev_m) continue;
      const Eigen::Vector3d p = t + dir * z_plane;
      if (std::abs(p.x()) > half_x_m || std::abs(p.y()) > half_y_m) continue;
      out->z_measured.push_back(z_meas);
      out->z_plane.push_back(z_plane);
    }
  }
}

double FitDepthScale(const PlaneSamples& s) {
  if (s.z_measured.empty() || s.z_measured.size() != s.z_plane.size()) {
    throw std::invalid_argument("FitDepthScale: need matched, non-empty samples");
  }
  double num = 0.0, den = 0.0;
  for (size_t i = 0; i < s.z_measured.size(); ++i) {
    num += s.z_plane[i] * s.z_measured[i];
    den += s.z_measured[i] * s.z_measured[i];
  }
  return num / den;
}

std::vector<double> BinnedDepthError(const PlaneSamples& s, double scale, int bins) {
  if (s.z_measured.empty() || s.z_measured.size() != s.z_plane.size() || bins < 1) {
    throw std::invalid_argument("BinnedDepthError: need matched, non-empty samples and bins >= 1");
  }
  std::vector<size_t> order(s.z_plane.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](size_t a, size_t b) { return s.z_plane[a] < s.z_plane[b]; });
  std::vector<double> out;
  const size_t n = order.size();
  for (int b = 0; b < bins; ++b) {
    const size_t lo = n * static_cast<size_t>(b) / static_cast<size_t>(bins);
    const size_t hi = n * static_cast<size_t>(b + 1) / static_cast<size_t>(bins);
    if (hi <= lo) continue;
    std::vector<double> e;
    e.reserve(hi - lo);
    for (size_t i = lo; i < hi; ++i) {
      e.push_back(scale * s.z_measured[order[i]] - s.z_plane[order[i]]);
    }
    std::nth_element(e.begin(), e.begin() + e.size() / 2, e.end());
    out.push_back(e[e.size() / 2]);
  }
  return out;
}

bool FitIsWorthApplying(const PlaneSamples& held_out, double scale) {
  auto rms = [](const std::vector<double>& v) {
    double sq = 0.0;
    for (double x : v) sq += x * x;
    return std::sqrt(sq / static_cast<double>(v.size()));
  };
  return rms(BinnedDepthError(held_out, scale)) <= 0.25 * rms(BinnedDepthError(held_out, 1.0));
}

}  // namespace the_eyes::runtime
