#include "the_eyes/scenegraph/predicates.hpp"

#include <algorithm>
#include <set>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::scenegraph {
namespace {

// Reads a required positive-or-zero scalar, failing loudly. A missing tolerance
// falling back to a default would produce a wrong scene graph that still looks
// reasonable, which is the worst failure mode for this module.
double RequireDouble(const YAML::Node& parent, const std::string& section,
                     const std::string& key, double lo, double hi) {
  const YAML::Node node = parent[key];
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("predicates.yaml: missing '" + section + "." + key + "'");
  }
  const double v = node.as<double>();
  if (!(v >= lo && v <= hi)) {
    std::ostringstream os;
    os << "predicates.yaml: '" << section << "." << key << "' = " << v
       << " outside [" << lo << ", " << hi << "]";
    throw std::runtime_error(os.str());
  }
  return v;
}

const YAML::Node RequireSection(const YAML::Node& root, const std::string& key) {
  const YAML::Node node = root[key];
  if (!node || !node.IsMap()) {
    throw std::runtime_error("predicates.yaml: missing section '" + key + "'");
  }
  return node;
}

// Sorted view of the scene: extraction must not depend on the order fusion
// happened to emit objects in.
std::vector<const ObjectState*> SortedValidObjects(const SceneState& scene,
                                                   std::vector<std::string>* warnings) {
  std::vector<const ObjectState*> out;
  out.reserve(scene.objects.size());
  for (const ObjectState& o : scene.objects) {
    if (is_invalid(o)) {
      warnings->push_back("dropped object '" + o.id +
                          "': visible_views == 0 (not seen by any camera)");
      continue;
    }
    out.push_back(&o);
  }
  std::sort(out.begin(), out.end(),
            [](const ObjectState* a, const ObjectState* b) { return a->id < b->id; });

  // Duplicate ids would make `clear` and `on` ambiguous. Keep the first.
  std::vector<const ObjectState*> deduped;
  deduped.reserve(out.size());
  for (const ObjectState* o : out) {
    if (!deduped.empty() && deduped.back()->id == o->id) {
      warnings->push_back("dropped object '" + o->id + "': duplicate id");
      continue;
    }
    deduped.push_back(o);
  }
  return deduped;
}

}  // namespace

double Aabb::volume() const {
  const Eigen::Vector3d d = (max - min).cwiseMax(0.0);
  return d.x() * d.y() * d.z();
}

double Aabb::xy_footprint_area() const {
  const Eigen::Vector3d d = (max - min).cwiseMax(0.0);
  return d.x() * d.y();
}

double Aabb::xy_overlap_area(const Aabb& other) const {
  const double dx = std::min(max.x(), other.max.x()) - std::max(min.x(), other.min.x());
  const double dy = std::min(max.y(), other.max.y()) - std::max(min.y(), other.min.y());
  if (dx <= 0.0 || dy <= 0.0) return 0.0;
  return dx * dy;
}

Aabb AabbOf(const ObjectState& o) {
  // extent is world-axis-aligned half-extents, so pose_w's rotation is
  // deliberately unused. This is what lets unknown_object — whose rotation is
  // meaningless by contract — participate in these rules safely.
  //
  // This ALSO assumes pose_w.translation() is the object's centre, not merely
  // some point rigidly attached to it. That holds because the object frame's
  // origin is defined as the mesh AABB centre (config/meshes.yaml), which
  // training/prepare_meshes.py enforces and
  // MeshRegistry.MeshesAreCentredOnTheirAabb re-checks against the OBJ files.
  // A raw scanned mesh breaks it silently: the YCB bowl's own origin is 43.7 mm
  // off centre, so every box built here would be displaced by 27% of the
  // object's diameter while every number downstream stayed self-consistent.
  const Eigen::Vector3d c = o.pose_w.translation();
  const Eigen::Vector3d e = o.extent.cwiseAbs();
  return Aabb{c - e, c + e};
}

PredicateConfig LoadPredicateConfig(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("predicates.yaml: cannot parse '" + yaml_path +
                             "': " + e.what());
  }

  PredicateConfig cfg;
  const YAML::Node on = RequireSection(root, "on");
  cfg.on_surface_tol_m = RequireDouble(on, "on", "surface_tol_m", 0.0, 1.0);
  cfg.on_min_overlap_ratio = RequireDouble(on, "on", "min_overlap_ratio", 0.0, 1.0);

  const YAML::Node in = RequireSection(root, "in");
  cfg.in_containment_margin_m = RequireDouble(in, "in", "containment_margin_m", 0.0, 1.0);
  cfg.in_max_volume_ratio = RequireDouble(in, "in", "max_volume_ratio", 0.0, 1.0);

  const YAML::Node views = RequireSection(root, "views");
  const YAML::Node names = views["names"];
  if (!names || !names.IsSequence() || names.size() == 0) {
    throw std::runtime_error("predicates.yaml: 'views.names' must be a non-empty sequence");
  }
  if (names.size() > 32) {
    throw std::runtime_error("predicates.yaml: 'views.names' exceeds the 32 bits "
                             "available in ObjectState::visible_views");
  }
  cfg.view_names.clear();
  for (const YAML::Node& n : names) cfg.view_names.push_back(n.as<std::string>());
  return cfg;
}

bool IsIn(const ObjectState& a, const ObjectState& b, const PredicateConfig& cfg) {
  if (a.id == b.id) return false;
  const Aabb ba = AabbOf(a);
  const Aabb bb = AabbOf(b);

  // A container must be able to hold it. Without this, two coincident objects
  // each read as inside the other.
  const double vol_b = bb.volume();
  if (vol_b <= 0.0) return false;
  if (ba.volume() > cfg.in_max_volume_ratio * vol_b) return false;

  const Eigen::Vector3d m = Eigen::Vector3d::Constant(cfg.in_containment_margin_m);
  const Eigen::Vector3d lo = bb.min + m;
  const Eigen::Vector3d hi = bb.max - m;
  if ((lo.array() > hi.array()).any()) return false;  // container thinner than margin

  const Eigen::Vector3d c = a.pose_w.translation();
  return (c.array() >= lo.array()).all() && (c.array() <= hi.array()).all();
}

bool IsOn(const ObjectState& a, const ObjectState& b, const PredicateConfig& cfg) {
  if (a.id == b.id) return false;

  // Containment wins over support: an object inside a box is `in` it, not `on`
  // it, even when it rests on the box's inner floor. PM1's domain treats these
  // as distinct, so emitting both would be a contradiction, not extra detail.
  if (IsIn(a, b, cfg)) return false;

  const Aabb ba = AabbOf(a);
  const Aabb bb = AabbOf(b);

  // a's base sits within tolerance of b's top surface.
  if (std::abs(ba.min.z() - bb.max.z()) > cfg.on_surface_tol_m) return false;

  // ...and enough of a's footprint is actually over b.
  const double footprint = ba.xy_footprint_area();
  if (footprint <= 0.0) return false;
  return ba.xy_overlap_area(bb) / footprint >= cfg.on_min_overlap_ratio;
}

PredicateResult Extract(const SceneState& scene, const PredicateConfig& cfg) {
  PredicateResult result;
  const std::vector<const ObjectState*> objs = SortedValidObjects(scene, &result.warnings);

  // Containment is resolved first because it is exclusive: an object inside a
  // container is not on ANYTHING else, not merely not on the container. A ball
  // in a shallow bin has its base within the `on` tolerance of the table top
  // underneath, and emitting (on ball table) would tell PM1 it can pick the
  // ball straight off the table.
  std::set<std::string> contained;
  for (const ObjectState* a : objs) {
    for (const ObjectState* b : objs) {
      if (a == b) continue;
      if (IsIn(*a, *b, cfg)) {
        result.facts.push_back("(in " + a->id + " " + b->id + ")");
        contained.insert(a->id);
      }
    }
  }

  // Objects that something rests on, so `clear` can be the complement.
  std::set<std::string> supporting;

  for (const ObjectState* a : objs) {
    if (contained.count(a->id) != 0) continue;
    for (const ObjectState* b : objs) {
      if (a == b) continue;
      if (IsOn(*a, *b, cfg)) {
        result.facts.push_back("(on " + a->id + " " + b->id + ")");
        supporting.insert(b->id);
      }
    }
  }

  for (const ObjectState* a : objs) {
    if (supporting.count(a->id) == 0) {
      result.facts.push_back("(clear " + a->id + ")");
    }
    // occluded_from is read straight off the fusion visibility bitmask rather
    // than re-derived geometrically — fusion already knows which views
    // contributed, and re-deriving it here would be a second, disagreeing
    // source of truth.
    for (size_t i = 0; i < cfg.view_names.size(); ++i) {
      if (!visible_from(*a, 1u << i)) {
        result.facts.push_back("(occluded_from " + a->id + " " + cfg.view_names[i] + ")");
      }
    }
  }

  std::sort(result.facts.begin(), result.facts.end());
  std::sort(result.warnings.begin(), result.warnings.end());
  return result;
}

void Annotate(SceneState& scene, const PredicateConfig& cfg) {
  const PredicateResult result = Extract(scene, cfg);
  for (ObjectState& o : scene.objects) {
    o.relations.clear();
    // A fact belongs to an object when the object is its first argument.
    const std::string prefix_end = " " + o.id + " ";
    for (const std::string& fact : result.facts) {
      const size_t sp = fact.find(' ');
      if (sp == std::string::npos) continue;
      const size_t arg_end = fact.find_first_of(" )", sp + 1);
      if (arg_end == std::string::npos) continue;
      if (fact.compare(sp + 1, arg_end - sp - 1, o.id) == 0) {
        o.relations.push_back(fact);
      }
    }
  }
}

}  // namespace the_eyes::scenegraph
