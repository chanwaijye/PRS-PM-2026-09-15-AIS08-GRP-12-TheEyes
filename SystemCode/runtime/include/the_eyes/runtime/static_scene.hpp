// Objects that exist in the world frame without being perceived: the table.
// Nothing upstream detects it, so without this injection no object can ever
// be `on` anything (INTERFACES.md §6). Loaded from config/scene.yaml.
#ifndef THE_EYES_RUNTIME_STATIC_SCENE_HPP
#define THE_EYES_RUNTIME_STATIC_SCENE_HPP

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "the_eyes/types.hpp"

namespace the_eyes::runtime {

struct StaticObject {
  std::string id;
  std::string cls;
  Eigen::Isometry3d pose_w = Eigen::Isometry3d::Identity();
  Eigen::Vector3d extent = Eigen::Vector3d::Zero();   // half-extents, world-axis-aligned
};

struct StaticScene {
  std::string world_frame;
  std::vector<StaticObject> objects;

  // Appends every static object to `state` with visible_views = all bits of
  // view_names (never occluded), confidence 1.0, stamp = state.stamp_ns,
  // velocity 0 and NO uncertainty (sigmas stay -1: no estimator produced
  // them, and INTERFACES.md §5 reserves 0 for a measured perfect certainty).
  // Throws std::runtime_error if an id collides with an object already there.
  void Inject(SceneState& state, const std::vector<std::string>& view_names) const;
};

// Throws std::runtime_error on a missing world_frame, duplicate ids, a
// non-positive extent, or a malformed pose.
StaticScene LoadStaticScene(const std::string& yaml_path);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_STATIC_SCENE_HPP
