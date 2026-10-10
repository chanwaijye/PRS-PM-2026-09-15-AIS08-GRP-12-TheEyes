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

bool IsGroundClass(const ObjectState& o, const PredicateConfig& cfg) {
  return std::find(cfg.ground_classes.begin(), cfg.ground_classes.end(), o.cls) !=
         cfg.ground_classes.end();
}

// The support check: the subset of `objs` that is grounded, in the same
// (id-sorted) order. The rest go to result->unsupported.
//
// Grounded is the least fixed point of: a ground class is grounded; an object
// `on` or `in` a grounded object is grounded. It is computed on the PAIRWISE
// rules, before Extract makes containment exclusive. Doing it afterwards would
// let a phantom container take a real object with it: the object would have
// lost its `on` facts to the container, and then its only support to the
// check.
std::vector<const ObjectState*> KeepGrounded(const std::vector<const ObjectState*>& objs,
                                             const PredicateConfig& cfg,
                                             PredicateResult* result) {
  const size_t n = objs.size();
  std::vector<bool> grounded(n, false);
  bool any_ground = false;
  for (size_t i = 0; i < n; ++i) {
    grounded[i] = IsGroundClass(*objs[i], cfg);
    any_ground = any_ground || grounded[i];
  }
  if (!any_ground && n > 0) {
    // Not a property of the scene but of the configuration: with nothing to
    // stand on, every object is about to be dropped, and that should not look
    // like an empty table.
    result->warnings.push_back(
        "support check: no object of a ground class in the scene, so nothing in it is "
        "supported (scene.yaml and predicates.yaml support.ground_classes disagree?)");
  }

  // rests[i][j]: object i rests on, or is inside, object j.
  std::vector<std::vector<bool>> rests(n, std::vector<bool>(n, false));
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      if (i == j) continue;
      rests[i][j] = IsIn(*objs[i], *objs[j], cfg) || IsOn(*objs[i], *objs[j], cfg);
    }
  }
  for (bool changed = true; changed;) {
    changed = false;
    for (size_t i = 0; i < n; ++i) {
      if (grounded[i]) continue;
      for (size_t j = 0; j < n; ++j) {
        if (grounded[j] && rests[i][j]) {
          grounded[i] = true;
          changed = true;
          break;
        }
      }
    }
  }

  std::vector<const ObjectState*> kept;
  kept.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    if (grounded[i]) {
      kept.push_back(objs[i]);
    } else {
      result->unsupported.push_back(objs[i]->id);
    }
  }
  return kept;
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

  // Required, but legitimately empty: a scene with no containers is a real
  // configuration. Absent is a different thing from empty and is rejected --
  // silently defaulting would either invent containers or suppress the whole
  // predicate, and both look like a working scene graph.
  const YAML::Node containers = in["container_classes"];
  if (!containers || !containers.IsSequence()) {
    throw std::runtime_error(
        "predicates.yaml: missing 'in.container_classes' (a sequence; use [] "
        "for a scene with no containers)");
  }
  cfg.in_container_classes.clear();
  for (const YAML::Node& n : containers) {
    const std::string cls = n.as<std::string>();
    if (cls.empty()) {
      throw std::runtime_error("predicates.yaml: empty class name in 'in.container_classes'");
    }
    cfg.in_container_classes.push_back(cls);
  }

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

  // Optional, so a predicates.yaml saved with a run from before the check
  // existed still loads and still means what it meant: no check. Strict when
  // present -- half a section is not a way to ask for the default.
  if (root["support"]) {
    const YAML::Node support = RequireSection(root, "support");
    const YAML::Node require = support["require_support"];
    if (!require || !require.IsScalar()) {
      throw std::runtime_error("predicates.yaml: missing 'support.require_support'");
    }
    try {
      cfg.require_support = require.as<bool>();
    } catch (const YAML::Exception&) {
      throw std::runtime_error("predicates.yaml: 'support.require_support' must be true or false");
    }
    const YAML::Node ground = support["ground_classes"];
    if (!ground || !ground.IsSequence()) {
      throw std::runtime_error("predicates.yaml: missing 'support.ground_classes' (a sequence)");
    }
    cfg.ground_classes.clear();
    for (const YAML::Node& n : ground) {
      const std::string cls = n.as<std::string>();
      if (cls.empty()) {
        throw std::runtime_error("predicates.yaml: empty class name in 'support.ground_classes'");
      }
      cfg.ground_classes.push_back(cls);
    }
    if (cfg.require_support && cfg.ground_classes.empty()) {
      throw std::runtime_error(
          "predicates.yaml: 'support.require_support' is true but 'support.ground_classes' is "
          "empty -- with nothing to stand on, every object would be dropped");
    }
  }
  return cfg;
}

bool IsIn(const ObjectState& a, const ObjectState& b, const PredicateConfig& cfg) {
  if (a.id == b.id) return false;

  // b must be a DECLARED container. Geometry cannot establish this and the
  // rest of this function should not pretend otherwise: an axis-aligned box is
  // all the shape information available here, and a table is exactly that --
  // wide, thin, and hollow-looking to a centroid test. On the 2026-09-06 live
  // run `(in red_cube_01 table_01)` was emitted 26 times, because a pose low by
  // more than half the 20 mm table thickness puts the cube's centroid between
  // the slab's top and bottom faces while the volume ratio stays tiny. Every
  // geometric test below passed; the claim was still false.
  //
  // Default-deny is the deliberate direction. `in` is a strong assertion:
  // Extract below makes containment exclusive, so a false positive silently
  // deletes every true `on` fact for that object, whereas a false negative
  // forgoes only the `in` fact itself. This also settles `unknown_object` with
  // no special case: no CAD model, no class semantics, never a container.
  if (std::find(cfg.in_container_classes.begin(), cfg.in_container_classes.end(),
                b.cls) == cfg.in_container_classes.end()) {
    return false;
  }

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
  // it, even when it rests on the box's inner floor. Both answer "what holds
  // this object", so emitting both would be a contradiction, not extra detail.
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
  std::vector<const ObjectState*> objs = SortedValidObjects(scene, &result.warnings);
  // From here on an unsupported object does not exist: it can be neither the
  // subject of a fact nor the thing another object is on.
  if (cfg.require_support) objs = KeepGrounded(objs, cfg, &result);

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
    // occluded_from is read straight off fusion's bitmask of the views that
    // had an estimate of the object, rather than re-derived geometrically —
    // fusion already knows which views saw it, and re-deriving it here would
    // be a second, disagreeing source of truth. NOT visible_views: a view the
    // disagreement gate dropped saw the object, and reading its missing bit as
    // "hidden" reported unoccluded objects as occluded (ObjectState).
    for (size_t i = 0; i < cfg.view_names.size(); ++i) {
      if (!observed_from(*a, 1u << i)) {
        result.facts.push_back("(occluded_from " + a->id + " " + cfg.view_names[i] + ")");
      }
    }
  }

  std::sort(result.facts.begin(), result.facts.end());
  std::sort(result.unsupported.begin(), result.unsupported.end());
  std::sort(result.warnings.begin(), result.warnings.end());
  return result;
}

PredicateResult Annotate(SceneState& scene, const PredicateConfig& cfg) {
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
  return result;
}

}  // namespace the_eyes::scenegraph
