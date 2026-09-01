// CAD mesh registry — INTERFACES.md §7.
//
// The class name is the join key between the detector and the pose estimator.
// A detection whose class has no entry here is dropped: the estimator cannot
// run without a mesh, and letting it through would fail deeper in the stack
// where the cause is far less obvious.
#ifndef THE_EYES_POSE_MESH_REGISTRY_HPP
#define THE_EYES_POSE_MESH_REGISTRY_HPP

#include <map>
#include <string>
#include <vector>

#include "the_eyes/types.hpp"

namespace the_eyes::pose {

class MeshRegistry {
 public:
  MeshRegistry() = default;
  explicit MeshRegistry(std::map<std::string, MeshEntry> entries)
      : entries_(std::move(entries)) {}

  // Returns nullptr when the class is not registered. Callers must check --
  // that null is the signal to drop the detection.
  const MeshEntry* Find(const std::string& cls) const;

  bool Contains(const std::string& cls) const { return Find(cls) != nullptr; }
  size_t size() const { return entries_.size(); }
  std::vector<std::string> classes() const;

 private:
  std::map<std::string, MeshEntry> entries_;
};

// Throws std::runtime_error on a missing file, a missing required field, or a
// non-positive diameter. Validation is strict because a silently defaulted
// diameter_m makes every ADD-S number wrong without making anything fail.
//
// When `require_mesh_files` is true, each `mesh:` path must exist on disk,
// resolved relative to `asset_root` when the path is relative. Tests leave it
// false so the registry is exercisable before any CAD asset exists.
MeshRegistry LoadMeshRegistry(const std::string& yaml_path,
                              bool require_mesh_files = true,
                              const std::string& asset_root = ".");

}  // namespace the_eyes::pose

#endif  // THE_EYES_POSE_MESH_REGISTRY_HPP
