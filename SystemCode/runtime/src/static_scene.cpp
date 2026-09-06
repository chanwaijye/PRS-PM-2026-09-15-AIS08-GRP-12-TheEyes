#include "the_eyes/runtime/static_scene.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::runtime {
namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

Eigen::Vector3d Vec3(const YAML::Node& node, const std::string& what) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error("scene.yaml: " + what + " must be a sequence of 3 numbers");
  }
  Eigen::Vector3d v;
  for (int i = 0; i < 3; ++i) {
    v[i] = node[static_cast<size_t>(i)].as<double>();
    if (!std::isfinite(v[i])) throw std::runtime_error("scene.yaml: " + what + " is not finite");
  }
  return v;
}

}  // namespace

StaticScene LoadStaticScene(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("scene.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }
  StaticScene scene;
  if (!root["world_frame"] || !root["world_frame"].IsScalar()) {
    throw std::runtime_error("scene.yaml: missing 'world_frame'");
  }
  scene.world_frame = root["world_frame"].as<std::string>();
  if (scene.world_frame.empty()) throw std::runtime_error("scene.yaml: 'world_frame' is empty");

  const YAML::Node objects = root["objects"];
  if (!objects || !objects.IsSequence()) {
    throw std::runtime_error("scene.yaml: 'objects' must be a sequence");
  }
  std::set<std::string> ids;
  for (const YAML::Node& node : objects) {
    StaticObject o;
    if (!node["id"] || !node["cls"]) {
      throw std::runtime_error("scene.yaml: every object needs id and cls");
    }
    o.id = node["id"].as<std::string>();
    o.cls = node["cls"].as<std::string>();
    if (o.id.empty() || o.cls.empty()) throw std::runtime_error("scene.yaml: empty id or cls");
    if (!ids.insert(o.id).second) throw std::runtime_error("scene.yaml: duplicate id '" + o.id + "'");

    const YAML::Node pose = node["pose_w"];
    if (!pose || !pose.IsMap()) {
      throw std::runtime_error("scene.yaml: object '" + o.id + "' needs pose_w {t, rpy_deg}");
    }
    const Eigen::Vector3d t = Vec3(pose["t"], "'" + o.id + "' pose_w.t");
    const Eigen::Vector3d rpy = pose["rpy_deg"]
                                    ? Vec3(pose["rpy_deg"], "'" + o.id + "' pose_w.rpy_deg")
                                    : Eigen::Vector3d::Zero();
    o.pose_w = Eigen::Isometry3d::Identity();
    o.pose_w.linear() = (Eigen::AngleAxisd(rpy[2] * kDegToRad, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(rpy[1] * kDegToRad, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rpy[0] * kDegToRad, Eigen::Vector3d::UnitX()))
                            .toRotationMatrix();
    o.pose_w.translation() = t;

    o.extent = Vec3(node["extent"], "'" + o.id + "' extent");
    if (!(o.extent.minCoeff() > 0.0)) {
      throw std::runtime_error("scene.yaml: object '" + o.id + "' extent must be positive");
    }
    scene.objects.push_back(std::move(o));
  }
  return scene;
}

void StaticScene::Inject(SceneState& state, const std::vector<std::string>& view_names) const {
  uint32_t all_views = 0;
  for (size_t i = 0; i < view_names.size(); ++i) all_views |= (1u << i);
  for (const StaticObject& s : objects) {
    for (const ObjectState& existing : state.objects) {
      if (existing.id == s.id) {
        throw std::runtime_error("static scene: id '" + s.id +
                                 "' collides with a fused object of the same id");
      }
    }
    ObjectState o;
    o.id = s.id;
    o.cls = s.cls;
    o.pose_w = s.pose_w;
    o.confidence = 1.0;
    o.visible_views = all_views;
    o.extent = s.extent;
    o.stamp_ns = state.stamp_ns;
    // position_sigma_m / orientation_sigma_rad stay at their -1 default.
    o.velocity_w = Eigen::Vector3d::Zero();
    state.objects.push_back(std::move(o));
  }
}

}  // namespace the_eyes::runtime
