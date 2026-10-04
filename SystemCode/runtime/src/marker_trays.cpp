#include "the_eyes/runtime/marker_trays.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <yaml-cpp/yaml.h>

namespace the_eyes::runtime {
namespace {

Eigen::Vector3d Vec3(const YAML::Node& n, const std::string& what) {
  if (!n || !n.IsSequence() || n.size() != 3) {
    throw std::runtime_error("marker_trays.yaml: " + what + " must be a 3-element sequence");
  }
  return Eigen::Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

}  // namespace

TrayConfig LoadTrayConfig(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception& e) {
    throw std::runtime_error("marker_trays.yaml: cannot read '" + yaml_path + "': " + e.what());
  }

  TrayConfig cfg;
  if (!root["dictionary"] || root["dictionary"].as<std::string>().empty()) {
    throw std::runtime_error("marker_trays.yaml: 'dictionary' is required -- it must match the "
                             "dictionary the markers were printed from or nothing decodes");
  }
  cfg.dictionary = root["dictionary"].as<std::string>();

  if (!root["marker_length_m"]) {
    throw std::runtime_error("marker_trays.yaml: 'marker_length_m' is required");
  }
  cfg.marker_length_m = root["marker_length_m"].as<double>();
  if (!(cfg.marker_length_m > 0.0)) {
    // Marker size scales every translation ArUco recovers, so a wrong value
    // produces poses that are wrong by a constant factor while every
    // reprojection stays small -- the same trap as the board's square length.
    throw std::runtime_error("marker_trays.yaml: marker_length_m must be > 0");
  }

  const YAML::Node trays = root["trays"];
  if (!trays || !trays.IsMap() || trays.size() == 0) {
    throw std::runtime_error("marker_trays.yaml: 'trays' must be a non-empty map");
  }

  for (const auto& kv : trays) {
    Tray t;
    t.cls = kv.first.as<std::string>();
    const YAML::Node& n = kv.second;
    t.t_tray_object = Vec3(n["t_tray_object"], t.cls + ".t_tray_object");
    if (n["plate_m"]) {
      t.plate_thickness_m = Vec3(n["plate_m"], t.cls + ".plate_m").z();
      if (!(t.plate_thickness_m > 0.0)) {
        throw std::runtime_error("marker_trays.yaml: " + t.cls + ".plate_m thickness must be > 0");
      }
    }

    const YAML::Node markers = n["markers"];
    if (!markers || !markers.IsSequence() || markers.size() == 0) {
      throw std::runtime_error("marker_trays.yaml: tray '" + t.cls + "' has no markers");
    }
    for (const YAML::Node& m : markers) {
      TrayMarker mk;
      if (!m["id"]) throw std::runtime_error("marker_trays.yaml: a marker of '" + t.cls +
                                             "' has no id");
      mk.id = m["id"].as<int>();
      mk.t_tray_marker = Vec3(m["t_tray_marker"], t.cls + " marker " + std::to_string(mk.id));

      auto seen = cfg.cls_of_marker.find(mk.id);
      if (seen != cfg.cls_of_marker.end()) {
        throw std::runtime_error("marker_trays.yaml: marker id " + std::to_string(mk.id) +
                                 " appears on both '" + seen->second + "' and '" + t.cls +
                                 "'. An observation of it could not be attributed to one object, "
                                 "and both would receive plausible-looking ground truth.");
      }
      cfg.cls_of_marker[mk.id] = t.cls;
      t.markers.push_back(mk);
    }
    cfg.trays[t.cls] = std::move(t);
  }
  return cfg;
}

Eigen::Isometry3d MarkerToObject(const Tray& tray, int marker_id) {
  for (const TrayMarker& m : tray.markers) {
    if (m.id != marker_id) continue;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = tray.t_tray_object - m.t_tray_marker;
    return T;
  }
  throw std::runtime_error("marker " + std::to_string(marker_id) + " is not on tray '" + tray.cls +
                           "'");
}

PoseAverage AveragePoses(const std::vector<Eigen::Isometry3d>& poses) {
  PoseAverage out;
  out.n = static_cast<int>(poses.size());
  if (poses.empty()) return out;

  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  for (const Eigen::Isometry3d& p : poses) {
    t += p.translation();
    Eigen::Quaterniond q(p.linear());
    q.normalize();
    const Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
    M += v * v.transpose();
  }
  t /= static_cast<double>(poses.size());

  // Markley: the average rotation is the eigenvector of sum(q q^T) with the
  // largest eigenvalue. q and -q are the same rotation and contribute the same
  // outer product, so this needs no hemisphere fix-up.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(M);
  const Eigen::Vector4d v = es.eigenvectors().col(3);
  Eigen::Quaterniond qm(v(0), v(1), v(2), v(3));
  qm.normalize();

  out.mean = Eigen::Isometry3d::Identity();
  out.mean.linear() = qm.toRotationMatrix();
  out.mean.translation() = t;

  double t_sq = 0.0, r_sq = 0.0;
  for (const Eigen::Isometry3d& p : poses) {
    const double dt = (p.translation() - t).norm();
    t_sq += dt * dt;
    out.translation_max_m = std::max(out.translation_max_m, dt);

    Eigen::Quaterniond q(p.linear());
    q.normalize();
    // Shortest-arc angle between the sample and the mean.
    double dot = std::abs(q.dot(qm));
    dot = std::min(1.0, std::max(-1.0, dot));
    const double deg = 2.0 * std::acos(dot) * 180.0 / M_PI;
    r_sq += deg * deg;
    out.rotation_max_deg = std::max(out.rotation_max_deg, deg);
  }
  const double n = static_cast<double>(poses.size());
  out.translation_rms_m = std::sqrt(t_sq / n);
  out.rotation_rms_deg = std::sqrt(r_sq / n);
  return out;
}

double RotationAngleDeg(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  Eigen::Quaterniond qa(a), qb(b);
  qa.normalize();
  qb.normalize();
  const double dot = std::min(1.0, std::abs(qa.dot(qb)));
  return 2.0 * std::acos(dot) * 180.0 / M_PI;
}

Eigen::Vector3d PixelOnWorldPlane(const Eigen::Vector2d& px, const Eigen::Matrix3d& K,
                                  const Eigen::Isometry3d& T_world_cam, double plane_z) {
  const Eigen::Vector3d dir = T_world_cam.linear() * (K.inverse() * Eigen::Vector3d(px.x(), px.y(), 1.0));
  const Eigen::Vector3d origin = T_world_cam.translation();
  if (std::abs(dir.z()) < 1e-12) {
    throw std::runtime_error("PixelOnWorldPlane: ray is parallel to the plane");
  }
  const double s = (plane_z - origin.z()) / dir.z();
  if (!(s > 0.0)) {
    throw std::runtime_error("PixelOnWorldPlane: the plane is behind the camera along this ray");
  }
  return origin + s * dir;
}

PlanarFit FitPlanarRigid(const std::vector<Eigen::Vector2d>& model_xy,
                         const std::vector<Eigen::Vector2d>& world_xy) {
  if (model_xy.size() != world_xy.size() || model_xy.size() < 2) {
    throw std::invalid_argument("FitPlanarRigid: need >= 2 points and equal counts");
  }
  const double n = static_cast<double>(model_xy.size());
  Eigen::Vector2d cm = Eigen::Vector2d::Zero(), cw = Eigen::Vector2d::Zero();
  for (size_t i = 0; i < model_xy.size(); ++i) {
    cm += model_xy[i];
    cw += world_xy[i];
  }
  cm /= n;
  cw /= n;

  // Procrustes in 2-D without scale: the rotation that best carries the
  // centred model onto the centred points is atan2(sum of cross, sum of dot).
  double dot = 0.0, cross = 0.0;
  for (size_t i = 0; i < model_xy.size(); ++i) {
    const Eigen::Vector2d m = model_xy[i] - cm, w = world_xy[i] - cw;
    dot += m.x() * w.x() + m.y() * w.y();
    cross += m.x() * w.y() - m.y() * w.x();
  }
  PlanarFit out;
  out.n = static_cast<int>(model_xy.size());
  out.yaw_rad = std::atan2(cross, dot);
  const Eigen::Rotation2Dd R(out.yaw_rad);
  out.t = cw - R * cm;
  double sq = 0.0;
  for (size_t i = 0; i < model_xy.size(); ++i) {
    sq += (R * model_xy[i] + out.t - world_xy[i]).squaredNorm();
  }
  out.rms_m = std::sqrt(sq / n);
  return out;
}

std::array<Eigen::Vector2d, 4> MarkerCornersXY(double length_m) {
  const double h = length_m / 2.0;
  return {Eigen::Vector2d(-h, h), Eigen::Vector2d(h, h), Eigen::Vector2d(h, -h),
          Eigen::Vector2d(-h, -h)};
}

Eigen::Isometry3d PoseOnPlane(const PlanarFit& fit, double z) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(fit.yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(fit.t.x(), fit.t.y(), z);
  return T;
}

OutlierScreen ScreenPoseOutliers(const std::vector<Eigen::Isometry3d>& poses,
                                 double max_rotation_deg, double max_translation_m) {
  if (max_rotation_deg <= 0.0 || max_translation_m <= 0.0) {
    throw std::invalid_argument("ScreenPoseOutliers: limits must be positive");
  }
  OutlierScreen out;
  const size_t n = poses.size();
  out.keep.assign(n, true);
  if (n == 0) return out;

  // Pairwise distances once; n is a few hundred at most (stills x markers).
  std::vector<double> rot(n * n, 0.0), trans(n * n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      const double r = RotationAngleDeg(poses[i].linear(), poses[j].linear());
      const double t = (poses[i].translation() - poses[j].translation()).norm();
      rot[i * n + j] = rot[j * n + i] = r;
      trans[i * n + j] = trans[j * n + i] = t;
    }
  }

  double best = 0.0;
  for (size_t i = 0; i < n; ++i) {
    double sum = 0.0;
    for (size_t j = 0; j < n; ++j) {
      sum += rot[i * n + j] / max_rotation_deg + trans[i * n + j] / max_translation_m;
    }
    // Strict < keeps the first of equal candidates, so the answer is
    // deterministic for a given input order.
    if (out.medoid < 0 || sum < best) {
      best = sum;
      out.medoid = static_cast<int>(i);
    }
  }

  const size_t m = static_cast<size_t>(out.medoid);
  for (size_t i = 0; i < n; ++i) {
    if (rot[i * n + m] > max_rotation_deg || trans[i * n + m] > max_translation_m) {
      out.keep[i] = false;
      ++out.rejected;
    }
  }
  return out;
}

}  // namespace the_eyes::runtime
