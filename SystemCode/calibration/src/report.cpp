#include "the_eyes/calibration/report.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

namespace the_eyes::calibration {
namespace {

Eigen::Isometry3d ParseRowMajor4x4(const YAML::Node& node, const std::string& where) {
  if (!node || !node.IsSequence() || node.size() != 16) {
    throw std::runtime_error("calibration.yaml: '" + where + "' must be 16 doubles, row-major");
  }
  Eigen::Matrix4d m;
  for (int i = 0; i < 16; ++i) m(i / 4, i % 4) = node[i].as<double>();

  const Eigen::Matrix3d r = m.topLeftCorner<3, 3>();
  if ((r * r.transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-6 || r.determinant() < 0.0) {
    throw std::runtime_error("calibration.yaml: '" + where +
                             "' rotation block is not a proper rotation");
  }
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = r;
  out.translation() = m.topRightCorner<3, 1>();
  return out;
}

}  // namespace

QualityThresholds LoadQualityThresholds(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("calibration.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }
  const YAML::Node node = root["quality"];
  if (!node || !node.IsMap()) {
    throw std::runtime_error("calibration.yaml: missing section 'quality'");
  }

  QualityThresholds thresholds;
  const auto require = [&node](const char* key) {
    if (!node[key]) {
      throw std::runtime_error(std::string("calibration.yaml: missing 'quality.") + key + "'");
    }
    return node[key];
  };
  thresholds.max_reprojection_rmse_px = require("max_reprojection_rmse_px").as<double>();
  thresholds.max_round_trip_error_m = require("max_round_trip_error_m").as<double>();
  thresholds.min_views_for_intrinsics = require("min_views_for_intrinsics").as<int>();
  thresholds.min_shared_corners = require("min_shared_corners").as<int>();

  if (!(thresholds.max_reprojection_rmse_px > 0.0) ||
      !(thresholds.max_round_trip_error_m > 0.0)) {
    throw std::runtime_error("calibration.yaml: quality thresholds must be positive");
  }
  return thresholds;
}

WorldConfig LoadWorldConfig(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("calibration.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }
  const YAML::Node node = root["world"];
  if (!node || !node.IsMap()) {
    throw std::runtime_error("calibration.yaml: missing section 'world'");
  }

  WorldConfig config;
  if (!node["frame"]) throw std::runtime_error("calibration.yaml: missing 'world.frame'");
  config.frame = node["frame"].as<std::string>();
  config.T_world_board = ParseRowMajor4x4(node["T_world_board"], "world.T_world_board");
  return config;
}

RoundTripResult ComputeRoundTrip(const BoardObservation& observation_a,
                                 const CameraCalibration& camera_a,
                                 const BoardObservation& observation_b,
                                 const CameraCalibration& camera_b) {
  RoundTripResult result;

  const std::vector<int> shared = SharedCorners(observation_a, observation_b);
  result.shared_corners = static_cast<int>(shared.size());
  if (shared.empty()) return result;

  // Rays from OBSERVED pixels, not from the board model.
  //
  // The earlier implementation pushed each camera's PnP of the board through
  // that camera's own extrinsics. Since T_world_cam is DERIVED from that same
  // PnP as T_world_board * T_cam_board^-1, both sides collapsed to
  // T_world_board * board_point and the error was identically zero however bad
  // the calibration was -- a gate that could not fire, on a real capture whose
  // registration RMSE was 3.12 px. The unit test only caught a perturbation
  // injected into T_world_cam AFTER registration, which the real path never
  // does.
  //
  // What the criterion actually asks is whether the two cameras' sightlines to
  // one physical corner meet. That question is answered by the detections, so
  // it cannot collapse: the rays miss by however much the two views disagree.
  // It is also what per-camera reprojection RMSE is structurally blind to --
  // notably a planar-target pose flip, where one camera's PnP lands on the
  // mirror solution, reprojects well, and points its rays somewhere else
  // entirely.
  const auto world_rays = [](const BoardObservation& observation,
                             const CameraCalibration& camera, const std::vector<int>& ids) {
    std::map<int, Eigen::Vector3d> by_id;
    std::vector<cv::Point2f> pixels;
    std::vector<int> order;
    for (size_t i = 0; i < observation.corner_ids.size(); ++i) {
      if (!std::binary_search(ids.begin(), ids.end(), observation.corner_ids[i])) continue;
      pixels.push_back(observation.image_points[i]);
      order.push_back(observation.corner_ids[i]);
    }
    if (pixels.empty()) return by_id;

    // Normalised (undistorted) image coordinates: the ray through a pixel in
    // camera coordinates, before the lens model. Passing no P returns exactly
    // that, which is why the distortion coefficients have to come along.
    std::vector<cv::Point2f> normalised;
    cv::undistortPoints(pixels, normalised, camera.intrinsics.K, camera.intrinsics.distortion);
    for (size_t i = 0; i < normalised.size(); ++i) {
      const Eigen::Vector3d ray_cam(normalised[i].x, normalised[i].y, 1.0);
      by_id[order[i]] = (camera.T_world_cam.linear() * ray_cam).normalized();
    }
    return by_id;
  };

  const std::map<int, Eigen::Vector3d> rays_a = world_rays(observation_a, camera_a, shared);
  const std::map<int, Eigen::Vector3d> rays_b = world_rays(observation_b, camera_b, shared);

  const Eigen::Vector3d origin_a = camera_a.T_world_cam.translation();
  const Eigen::Vector3d origin_b = camera_b.T_world_cam.translation();
  const Eigen::Vector3d between = origin_a - origin_b;

  std::vector<double> errors;
  errors.reserve(shared.size());
  for (const int id : shared) {
    const auto it_a = rays_a.find(id);
    const auto it_b = rays_b.find(id);
    if (it_a == rays_a.end() || it_b == rays_b.end()) continue;
    const Eigen::Vector3d& u = it_a->second;
    const Eigen::Vector3d& v = it_b->second;

    // Closest approach between two skew lines. Both directions are unit, so the
    // usual a and c terms are 1.
    const double b = u.dot(v);
    const double denominator = 1.0 - b * b;
    if (denominator < 1e-9) {
      // Near-parallel sightlines carry no depth information and the closest
      // point is undefined. Fall back to the perpendicular offset between the
      // two lines, which is still the right quantity.
      errors.push_back((between - between.dot(u) * u).norm());
      continue;
    }
    const double d = u.dot(between);
    const double e = v.dot(between);
    const double t_a = (b * e - d) / denominator;
    const double t_b = (e - b * d) / denominator;
    errors.push_back(((origin_a + t_a * u) - (origin_b + t_b * v)).norm());
  }
  if (errors.empty()) return result;

  std::sort(errors.begin(), errors.end());
  result.mean_error_m =
      std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
  result.max_error_m = errors.back();
  const size_t p95 = static_cast<size_t>(0.95 * static_cast<double>(errors.size() - 1));
  result.p95_error_m = errors[p95];
  return result;
}

void Evaluate(CalibrationReport* report) {
  if (report == nullptr) return;
  report->failures.clear();

  for (const CameraCalibration& camera : report->cameras) {
    if (camera.intrinsics.source == IntrinsicSource::kFactory) {
      // Factory intrinsics have no calibration views, so neither the view-count
      // gate nor an intrinsic RMSE applies. The registration view is the real
      // check: it exercises these intrinsics against actual board corners, and
      // if the factory values were wrong the board would not reproject.
      if (camera.registration_rmse_px > report->thresholds.max_reprojection_rmse_px) {
        std::ostringstream os;
        os << camera.camera_id << ": registration reprojection RMSE "
           << camera.registration_rmse_px << " px exceeds "
           << report->thresholds.max_reprojection_rmse_px
           << " px using FACTORY intrinsics -- either the board pose is bad or the "
              "factory calibration is not good enough for this rig; run a full "
              "intrinsic sweep to find out which";
        report->failures.push_back(os.str());
      }
    } else {
      if (camera.intrinsics.reprojection_rmse_px > report->thresholds.max_reprojection_rmse_px) {
        std::ostringstream os;
        os << camera.camera_id << ": intrinsic reprojection RMSE "
           << camera.intrinsics.reprojection_rmse_px << " px exceeds "
           << report->thresholds.max_reprojection_rmse_px << " px";
        report->failures.push_back(os.str());
      }
      if (camera.intrinsics.views_used < report->thresholds.min_views_for_intrinsics) {
        std::ostringstream os;
        os << camera.camera_id << ": only " << camera.intrinsics.views_used
           << " calibration views, need " << report->thresholds.min_views_for_intrinsics;
        report->failures.push_back(os.str());
      }
    }
    if (camera.serial.empty()) {
      // CLAUDE.md constraint 4: cameras are addressed by serial, never by
      // enumeration order. An entry without one is unusable at runtime.
      report->failures.push_back(camera.camera_id + ": no serial number recorded");
    }
  }

  if (report->round_trip.shared_corners < report->thresholds.min_shared_corners) {
    std::ostringstream os;
    os << "round-trip: only " << report->round_trip.shared_corners
       << " corners visible to both cameras, need " << report->thresholds.min_shared_corners
       << " -- move the board so both views see it well";
    report->failures.push_back(os.str());
  } else if (report->round_trip.max_error_m > report->thresholds.max_round_trip_error_m) {
    std::ostringstream os;
    os << "round-trip: max disagreement " << report->round_trip.max_error_m * 1000.0
       << " mm exceeds " << report->thresholds.max_round_trip_error_m * 1000.0 << " mm";
    report->failures.push_back(os.str());
  }
}

std::string CalibrationReport::Format() const {
  std::ostringstream os;
  os << std::fixed;
  os << "Calibration report -- world frame: " << world_frame << "\n";
  os << std::string(72, '-') << "\n";

  for (const CameraCalibration& camera : cameras) {
    const Eigen::Matrix3d K = camera.intrinsics.KEigen();
    os << camera.camera_id << "  (serial " << (camera.serial.empty() ? "MISSING" : camera.serial)
       << ")\n";
    os << "  intrinsics           : "
       << (camera.intrinsics.source == IntrinsicSource::kFactory
               ? "FACTORY (from device)"
               : "calibrated here")
       << "\n";
    if (camera.intrinsics.source != IntrinsicSource::kFactory) {
      os << "  views                : " << camera.intrinsics.views_used << "\n";
    }
    os << std::setprecision(2);
    os << "  fx, fy               : " << K(0, 0) << ", " << K(1, 1) << " px\n";
    os << "  cx, cy               : " << K(0, 2) << ", " << K(1, 2) << " px\n";
    os << std::setprecision(4);
    if (camera.intrinsics.source != IntrinsicSource::kFactory) {
      os << "  intrinsic RMSE       : " << camera.intrinsics.reprojection_rmse_px << " px\n";
    }
    os << "  registration RMSE    : " << camera.registration_rmse_px << " px\n";
    os << std::setprecision(1);
    os << "  world position       : [" << camera.T_world_cam.translation().x() * 1000.0 << ", "
       << camera.T_world_cam.translation().y() * 1000.0 << ", "
       << camera.T_world_cam.translation().z() * 1000.0 << "] mm\n";
    os << std::setprecision(4);
  }

  os << "\nround trip (a point seen by both -> same world coordinate)\n";
  os << "  shared corners       : " << round_trip.shared_corners << "\n";
  os << std::setprecision(2);
  os << "  mean / p95 / max     : " << round_trip.mean_error_m * 1000.0 << " / "
     << round_trip.p95_error_m * 1000.0 << " / " << round_trip.max_error_m * 1000.0 << " mm\n";

  os << "\nthresholds: reprojection < " << thresholds.max_reprojection_rmse_px
     << " px, round trip < " << thresholds.max_round_trip_error_m * 1000.0 << " mm\n";

  if (passed()) {
    os << "\nRESULT: PASS\n";
  } else {
    os << "\nRESULT: FAIL\n";
    for (const std::string& failure : failures) os << "  - " << failure << "\n";
  }
  return os.str();
}

void WriteExtrinsicsYaml(const CalibrationReport& report, const std::string& out_path,
                         bool force) {
  if (!report.passed() && !force) {
    std::ostringstream os;
    os << "refusing to write extrinsics from a failing calibration (" << report.failures.size()
       << " failure(s)). Fusion's gate would catch the RMSE, but a file that exists is "
          "assumed good by everything else. Pass force to override.";
    throw std::runtime_error(os.str());
  }

  std::ofstream file(out_path);
  if (!file) throw std::runtime_error("cannot write '" + out_path + "'");

  file << std::setprecision(17);
  file << "# Generated by calib_agent. Consumed by fusion (INTERFACES.md §4).\n";
  file << "#\n";
  file << "# Re-run calibration after ANY mechanical change to the rig. These numbers are\n";
  file << "# only valid for the mounting they were measured on, and nothing downstream can\n";
  file << "# detect that the cameras moved -- every fused pose simply becomes wrong by a\n";
  file << "# fixed offset while staying perfectly self-consistent.\n";
  if (!report.passed()) {
    file << "#\n# WARNING: written from a FAILING calibration (force). Failures:\n";
    for (const std::string& failure : report.failures) file << "#   - " << failure << "\n";
  }
  file << "world_frame: " << report.world_frame << "\n";
  file << "cameras:\n";

  for (const CameraCalibration& camera : report.cameras) {
    file << "  " << camera.camera_id << ":\n";
    file << "    serial: \"" << camera.serial << "\"\n";
    file << "    T_world_cam: [";
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        if (r != 0 || c != 0) file << ", ";
        const double v = (r < 3) ? (c < 3 ? camera.T_world_cam.linear()(r, c)
                                          : camera.T_world_cam.translation()(r))
                                 : (c == 3 ? 1.0 : 0.0);
        file << v;
      }
    }
    file << "]\n";
    // Fusion gates on this number. For factory intrinsics the intrinsic RMSE
    // does not exist, so the registration RMSE is what is written -- it is the
    // figure that actually measured these intrinsics against real corners.
    file << "    reprojection_rmse_px: "
         << (camera.intrinsics.source == IntrinsicSource::kFactory
                 ? camera.registration_rmse_px
                 : camera.intrinsics.reprojection_rmse_px)
         << "\n";
  }
}

}  // namespace the_eyes::calibration
