#include "the_eyes/pose/mesh_registry.hpp"

#include <filesystem>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::pose {
namespace {

Eigen::Vector3d ParseExtent(const YAML::Node& node, const std::string& cls) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error("meshes.yaml: '" + cls +
                             ".extent' must be a sequence of 3 half-extents in metres");
  }
  Eigen::Vector3d e;
  for (int i = 0; i < 3; ++i) e[i] = node[i].as<double>();
  if ((e.array() <= 0.0).any()) {
    throw std::runtime_error("meshes.yaml: '" + cls + ".extent' must be strictly positive");
  }
  return e;
}

}  // namespace

const MeshEntry* MeshRegistry::Find(const std::string& cls) const {
  const auto it = entries_.find(cls);
  return (it == entries_.end()) ? nullptr : &it->second;
}

std::vector<std::string> MeshRegistry::classes() const {
  std::vector<std::string> out;
  out.reserve(entries_.size());
  for (const auto& [cls, _] : entries_) out.push_back(cls);
  return out;  // std::map iterates sorted, so this is deterministic
}

MeshRegistry LoadMeshRegistry(const std::string& yaml_path, bool require_mesh_files,
                              const std::string& asset_root) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("meshes.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }

  const YAML::Node objects = root["objects"];
  if (!objects || !objects.IsMap() || objects.size() == 0) {
    throw std::runtime_error("meshes.yaml: 'objects' must be a non-empty map");
  }

  std::map<std::string, MeshEntry> entries;
  for (const auto& kv : objects) {
    const std::string cls = kv.first.as<std::string>();
    const YAML::Node& node = kv.second;

    MeshEntry entry;
    entry.cls = cls;

    if (!node["mesh"] || !node["mesh"].IsScalar()) {
      throw std::runtime_error("meshes.yaml: '" + cls + ".mesh' is required");
    }
    entry.mesh_path = node["mesh"].as<std::string>();

    // symmetric decides whether fusion may average this object's orientation at
    // all, so an absent flag is a real ambiguity rather than a default.
    if (!node["symmetric"] || !node["symmetric"].IsScalar()) {
      throw std::runtime_error("meshes.yaml: '" + cls +
                               ".symmetric' is required (fusion must know whether "
                               "orientation may be averaged)");
    }
    entry.symmetric = node["symmetric"].as<bool>();

    // Optional. Naming the group is what lets fusion treat two equivalent
    // orientations as the same pose instead of merely tolerating that they
    // differ.
    if (node["symmetry_group"]) {
      const std::string g = node["symmetry_group"].as<std::string>();
      if (g == "none") entry.symmetry_group = SymmetryGroup::kNone;
      else if (g == "cubic") entry.symmetry_group = SymmetryGroup::kCubic;
      else if (g == "axial_z") entry.symmetry_group = SymmetryGroup::kAxialZ;
      else {
        throw std::runtime_error("meshes.yaml: '" + cls + ".symmetry_group' is '" + g +
                                 "'; expected none, cubic or axial_z");
      }
      if (entry.symmetry_group != SymmetryGroup::kNone && !entry.symmetric) {
        throw std::runtime_error("meshes.yaml: '" + cls + "' declares symmetry_group '" + g +
                                 "' but symmetric: false -- the two contradict each other");
      }
    }

    if (!node["diameter_m"] || !node["diameter_m"].IsScalar()) {
      throw std::runtime_error("meshes.yaml: '" + cls +
                               ".diameter_m' is required (ADD-S normalises by it)");
    }
    entry.diameter_m = node["diameter_m"].as<double>();
    if (!(entry.diameter_m > 0.0)) {
      std::ostringstream os;
      os << "meshes.yaml: '" << cls << ".diameter_m' = " << entry.diameter_m
         << " must be strictly positive";
      throw std::runtime_error(os.str());
    }

    entry.extent = ParseExtent(node["extent"], cls);

    if (require_mesh_files) {
      std::filesystem::path path(entry.mesh_path);
      if (path.is_relative()) path = std::filesystem::path(asset_root) / path;
      if (!std::filesystem::exists(path)) {
        throw std::runtime_error("meshes.yaml: '" + cls + ".mesh' not found at '" +
                                 path.string() + "'");
      }
    }
    entries.emplace(cls, std::move(entry));
  }
  return MeshRegistry(std::move(entries));
}

}  // namespace the_eyes::pose
