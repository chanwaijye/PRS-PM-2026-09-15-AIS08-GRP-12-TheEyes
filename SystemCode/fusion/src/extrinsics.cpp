#include "the_eyes/fusion/extrinsics.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace the_eyes::fusion {
namespace {

Eigen::Isometry3d ParseRowMajor4x4(const YAML::Node& node, const std::string& camera_id) {
  if (!node || !node.IsSequence() || node.size() != 16) {
    throw std::runtime_error("extrinsics.yaml: '" + camera_id +
                             ".T_world_cam' must be 16 doubles, row-major");
  }
  Eigen::Matrix4d m;
  for (int i = 0; i < 16; ++i) m(i / 4, i % 4) = node[i].as<double>();

  // A transposed or hand-edited matrix produces a pose that is wrong in a way
  // no downstream test would attribute to calibration, so validate here.
  const Eigen::Matrix3d r = m.topLeftCorner<3, 3>();
  const double orthonormality_err = (r * r.transpose() - Eigen::Matrix3d::Identity()).norm();
  if (orthonormality_err > 1e-6) {
    std::ostringstream os;
    os << "extrinsics.yaml: '" << camera_id << ".T_world_cam' rotation block is not "
       << "orthonormal (||RR^T - I|| = " << orthonormality_err << ")";
    throw std::runtime_error(os.str());
  }
  if (r.determinant() < 0.0) {
    throw std::runtime_error("extrinsics.yaml: '" + camera_id +
                             ".T_world_cam' rotation block is a reflection (det < 0)");
  }
  if ((m.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-9) {
    throw std::runtime_error("extrinsics.yaml: '" + camera_id +
                             ".T_world_cam' bottom row must be [0 0 0 1]");
  }

  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = r;
  t.translation() = m.topRightCorner<3, 1>();
  return t;
}

}  // namespace

const CameraExtrinsics& ExtrinsicsSet::at(const std::string& camera_id) const {
  const auto it = cameras.find(camera_id);
  if (it == cameras.end()) {
    throw std::out_of_range("no extrinsics for camera '" + camera_id + "'");
  }
  return it->second;
}

uint32_t ExtrinsicsSet::ViewBitFor(const std::string& camera_id,
                                   const std::vector<std::string>& view_names) {
  const auto it = std::find(view_names.begin(), view_names.end(), camera_id);
  if (it == view_names.end()) return 0u;
  return 1u << static_cast<uint32_t>(std::distance(view_names.begin(), it));
}

ExtrinsicsSet LoadExtrinsics(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("extrinsics.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }

  ExtrinsicsSet out;
  if (!root["world_frame"] || !root["world_frame"].IsScalar()) {
    throw std::runtime_error("extrinsics.yaml: missing 'world_frame'");
  }
  out.world_frame = root["world_frame"].as<std::string>();

  const YAML::Node cameras = root["cameras"];
  if (!cameras || !cameras.IsMap() || cameras.size() == 0) {
    throw std::runtime_error("extrinsics.yaml: 'cameras' must be a non-empty map");
  }

  for (const auto& kv : cameras) {
    const std::string id = kv.first.as<std::string>();
    const YAML::Node& node = kv.second;

    CameraExtrinsics cam;
    cam.camera_id = id;
    if (!node["serial"] || !node["serial"].IsScalar()) {
      // CLAUDE.md constraint 4: cameras are addressed by serial, never by
      // enumeration order, so a missing serial is a hard error.
      throw std::runtime_error("extrinsics.yaml: '" + id + ".serial' is required");
    }
    cam.serial = node["serial"].as<std::string>();
    cam.T_world_cam = ParseRowMajor4x4(node["T_world_cam"], id);

    if (!node["reprojection_rmse_px"] || !node["reprojection_rmse_px"].IsScalar()) {
      throw std::runtime_error("extrinsics.yaml: '" + id +
                               ".reprojection_rmse_px' is required (it is the quality gate)");
    }
    cam.reprojection_rmse_px = node["reprojection_rmse_px"].as<double>();
    if (cam.reprojection_rmse_px < 0.0) {
      throw std::runtime_error("extrinsics.yaml: '" + id +
                               ".reprojection_rmse_px' must be non-negative");
    }
    out.cameras.emplace(id, cam);
  }

  // Two cameras sharing a serial means the calibration was run against one
  // physical unit twice -- every fused pose would then be a self-comparison.
  for (auto a = out.cameras.begin(); a != out.cameras.end(); ++a) {
    for (auto b = std::next(a); b != out.cameras.end(); ++b) {
      if (a->second.serial == b->second.serial) {
        throw std::runtime_error("extrinsics.yaml: cameras '" + a->first + "' and '" + b->first +
                                 "' share serial '" + a->second.serial + "'");
      }
    }
  }
  return out;
}

void AssertCalibrationQuality(const ExtrinsicsSet& ex, double max_reprojection_rmse_px) {
  std::vector<std::string> failures;
  for (const auto& [id, cam] : ex.cameras) {
    if (cam.reprojection_rmse_px > max_reprojection_rmse_px) {
      std::ostringstream os;
      os << id << " (" << cam.reprojection_rmse_px << " px)";
      failures.push_back(os.str());
    }
  }
  if (failures.empty()) return;

  std::ostringstream os;
  os << "refusing to start: calibration quality gate failed at "
     << max_reprojection_rmse_px << " px for: ";
  for (size_t i = 0; i < failures.size(); ++i) {
    if (i > 0) os << ", ";
    os << failures[i];
  }
  os << ". Re-run calib_agent; do not fuse against these extrinsics.";
  throw std::runtime_error(os.str());
}

Eigen::Isometry3d ToWorld(const PoseEstimate& est, const CameraExtrinsics& cam) {
  return cam.T_world_cam * est.pose_cam;
}

}  // namespace the_eyes::fusion
