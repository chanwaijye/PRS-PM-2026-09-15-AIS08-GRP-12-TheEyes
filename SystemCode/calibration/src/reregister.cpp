#include "the_eyes/calibration/reregister.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::calibration {

PoseDelta ComputePoseDelta(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  const Eigen::Isometry3d d = a.inverse() * b;
  PoseDelta delta;
  delta.translation_m = d.translation().norm();
  // AngleAxis reports [0, pi], so the magnitude is sign- and order-independent.
  delta.rotation_deg = Eigen::AngleAxisd(d.linear()).angle() * 180.0 / M_PI;
  return delta;
}

RigidityThresholds LoadRigidityThresholds(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  RigidityThresholds thresholds;
  const YAML::Node node = root["rigidity"];
  if (!node) return thresholds;  // absent means "use the derived defaults"

  if (node["max_baseline_shift_m"]) {
    thresholds.max_baseline_shift_m = node["max_baseline_shift_m"].as<double>();
  }
  if (node["max_baseline_rotation_deg"]) {
    thresholds.max_baseline_rotation_deg = node["max_baseline_rotation_deg"].as<double>();
  }
  if (node["warn_world_shift_m"]) {
    thresholds.warn_world_shift_m = node["warn_world_shift_m"].as<double>();
  }

  if (thresholds.max_baseline_shift_m <= 0.0 || thresholds.max_baseline_rotation_deg <= 0.0) {
    throw std::runtime_error(
        "calibration.yaml: rigidity thresholds must be positive. A zero or negative "
        "threshold can never pass, which would make re-registration permanently "
        "unavailable rather than obviously misconfigured.");
  }
  // A baseline gate at or above the round-trip gate cannot do its job: the rig
  // would be declared intact right up to the point where the round trip fails
  // anyway, and the operator would be told to re-register when the real problem
  // is a bumped tripod.
  const YAML::Node quality = root["quality"];
  if (quality && quality["max_round_trip_error_m"]) {
    const double round_trip = quality["max_round_trip_error_m"].as<double>();
    if (thresholds.max_baseline_shift_m >= round_trip) {
      std::ostringstream os;
      os << "calibration.yaml: rigidity.max_baseline_shift_m ("
         << thresholds.max_baseline_shift_m * 1000.0
         << " mm) must be below quality.max_round_trip_error_m (" << round_trip * 1000.0
         << " mm), or the rigidity check can never fire before the round trip does.";
      throw std::runtime_error(os.str());
    }
  }
  return thresholds;
}

ReRegistrationReport CheckReRegistration(const std::vector<std::string>& camera_ids,
                                         const std::vector<Eigen::Isometry3d>& previous,
                                         const std::vector<Eigen::Isometry3d>& current,
                                         const RigidityThresholds& thresholds) {
  if (camera_ids.size() != previous.size() || camera_ids.size() != current.size()) {
    throw std::runtime_error(
        "CheckReRegistration: camera_ids, previous and current must be the same length");
  }

  ReRegistrationReport report;
  report.thresholds = thresholds;

  if (camera_ids.empty()) throw std::runtime_error("CheckReRegistration: no cameras");

  for (size_t i = 0; i < camera_ids.size(); ++i) {
    CameraReRegistration camera;
    camera.camera_id = camera_ids[i];
    camera.previous_T_world_cam = previous[i];
    camera.current_T_world_cam = current[i];
    camera.world_delta = ComputePoseDelta(previous[i], current[i]);
    report.cameras.push_back(std::move(camera));
  }

  if (camera_ids.size() < 2) {
    report.failures.push_back(
        camera_ids[0] +
        ": a single camera has no baseline, so the rigid-body assumption cannot be "
        "tested. Re-registration is indistinguishable from a bumped tripod here -- "
        "run a full calibration.");
    return report;
  }

  // Every pair. A chain (0-1, 1-2) would let two link errors cancel and pass a
  // rig whose middle camera has rotated.
  for (size_t i = 0; i < camera_ids.size(); ++i) {
    for (size_t j = i + 1; j < camera_ids.size(); ++j) {
      BaselineCheck check;
      check.camera_a = camera_ids[i];
      check.camera_b = camera_ids[j];

      // Invariant to the world frame moving: any common transform applied to
      // both poses cancels in the inverse-times product.
      const Eigen::Isometry3d before = previous[i].inverse() * previous[j];
      const Eigen::Isometry3d after = current[i].inverse() * current[j];
      check.delta = ComputePoseDelta(before, after);
      check.intact = check.delta.translation_m <= thresholds.max_baseline_shift_m &&
                     check.delta.rotation_deg <= thresholds.max_baseline_rotation_deg;

      if (!check.intact) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(2);
        os << check.camera_a << " <-> " << check.camera_b << ": baseline moved by "
           << check.delta.translation_m * 1000.0 << " mm / " << check.delta.rotation_deg
           << " deg (limits " << thresholds.max_baseline_shift_m * 1000.0 << " mm / "
           << thresholds.max_baseline_rotation_deg
           << " deg) -- these cameras moved relative to each other, so "
              "re-registration cannot recover this. Run a full calibration.";
        report.failures.push_back(os.str());
      }
      report.baselines.push_back(std::move(check));
    }
  }

  for (const CameraReRegistration& camera : report.cameras) {
    if (camera.world_delta.translation_m > thresholds.warn_world_shift_m) {
      std::ostringstream os;
      os << std::fixed << std::setprecision(1);
      os << camera.camera_id << ": world frame moved "
         << camera.world_delta.translation_m * 1000.0 << " mm since the last calibration, "
         << "above the " << thresholds.warn_world_shift_m * 1000.0
         << " mm expected for desk movement. Confirm the board is back on its marked "
            "position -- a misplaced board redefines the world frame, and NOTHING "
            "downstream can detect it.";
      report.warnings.push_back(os.str());
    }
  }

  return report;
}

std::string ReRegistrationReport::Format() const {
  std::ostringstream os;
  os << std::fixed;
  os << "Re-registration report\n";
  os << std::string(72, '-') << "\n";

  os << "world frame motion (expected: the table moved)\n";
  os << std::setprecision(2);
  for (const CameraReRegistration& camera : cameras) {
    const Eigen::Vector3d before = camera.previous_T_world_cam.translation();
    const Eigen::Vector3d after = camera.current_T_world_cam.translation();
    os << "  " << camera.camera_id << std::setprecision(1) << "  [" << before.x() * 1000.0 << ", "
       << before.y() * 1000.0 << ", " << before.z() * 1000.0 << "] -> [" << after.x() * 1000.0
       << ", " << after.y() * 1000.0 << ", " << after.z() * 1000.0 << "] mm"
       << std::setprecision(2) << "   delta " << camera.world_delta.translation_m * 1000.0
       << " mm / " << camera.world_delta.rotation_deg << " deg\n";
  }

  os << "\nbaseline rigidity (must NOT have moved)\n";
  if (baselines.empty()) {
    os << "  none -- fewer than two cameras\n";
  }
  for (const BaselineCheck& check : baselines) {
    os << "  " << check.camera_a << " <-> " << check.camera_b << "   "
       << check.delta.translation_m * 1000.0 << " mm / " << check.delta.rotation_deg << " deg   "
       << (check.intact ? "INTACT" : "MOVED") << "\n";
  }

  os << "\nlimits: baseline < " << thresholds.max_baseline_shift_m * 1000.0 << " mm / "
     << thresholds.max_baseline_rotation_deg << " deg\n";

  for (const std::string& warning : warnings) os << "\nWARNING: " << warning << "\n";

  if (passed()) {
    os << "\nRESULT: RIG INTACT -- re-registration is valid\n";
  } else {
    os << "\nRESULT: FAIL\n";
    for (const std::string& failure : failures) os << "  - " << failure << "\n";
  }
  return os.str();
}

}  // namespace the_eyes::calibration
