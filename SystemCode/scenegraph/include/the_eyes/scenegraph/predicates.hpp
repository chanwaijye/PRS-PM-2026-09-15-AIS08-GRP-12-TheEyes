// Relational predicate extraction over fused object poses.
//
// Written from first principles (CLAUDE.md constraint 2): no scene-graph
// library. The rules are the ones tabulated in INTERFACES.md §6.
//
// Determinism is a contract requirement — identical SceneState in, identical
// fact set out — so every traversal here runs over an id-sorted copy and every
// output vector is sorted before it is returned.
#ifndef THE_EYES_SCENEGRAPH_PREDICATES_HPP
#define THE_EYES_SCENEGRAPH_PREDICATES_HPP

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "the_eyes/types.hpp"

namespace the_eyes::scenegraph {

// Tolerances, loaded from config/predicates.yaml. Defaults here mirror that
// file so unit tests can construct one without touching the filesystem; the
// runtime path always loads from disk.
struct PredicateConfig {
  double on_surface_tol_m = 0.015;
  double on_min_overlap_ratio = 0.25;
  double in_containment_margin_m = 0.005;
  double in_max_volume_ratio = 0.5;
  // Index i in this vector corresponds to bit i of ObjectState::visible_views.
  std::vector<std::string> view_names = {"cam_front", "cam_over"};
};

// Throws std::runtime_error on a missing file or a malformed/out-of-range value.
// Bad tolerances silently produce a plausible-looking wrong scene graph, so
// this validates rather than falling back to defaults.
PredicateConfig LoadPredicateConfig(const std::string& yaml_path);

// World-axis-aligned bounding box derived from translation +/- extent.
struct Aabb {
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();

  double volume() const;
  // Footprint overlap area with `other` in the world XY plane, m^2.
  double xy_overlap_area(const Aabb& other) const;
  double xy_footprint_area() const;
};

Aabb AabbOf(const ObjectState& o);

struct PredicateResult {
  // Fully-formed PDDL facts, e.g. "(on red_cube_01 table_01)". Sorted.
  std::vector<std::string> facts;
  // Objects dropped or ids rewritten, with the reason. Never silent.
  std::vector<std::string> warnings;
};

// Individual rules, exposed so fixtures can assert one rule at a time.
//
// NOTE: IsOn is pairwise and cannot see third objects. Containment is exclusive
// -- an object that is `in` anything is `on` nothing -- and that global rule is
// applied by Extract, not here. IsOn(a, b) alone can therefore return true for a
// pair that Extract correctly emits no `on` fact for.
bool IsOn(const ObjectState& a, const ObjectState& b, const PredicateConfig& cfg);
bool IsIn(const ObjectState& a, const ObjectState& b, const PredicateConfig& cfg);

// Extracts the full fact set. Objects with visible_views == 0 are dropped with
// a warning (INTERFACES.md §5); duplicate ids are dropped after the first.
PredicateResult Extract(const SceneState& scene, const PredicateConfig& cfg);

// Same rules, but writes the facts back into each object's `relations` field,
// which is the field fusion leaves empty for this module to fill.
void Annotate(SceneState& scene, const PredicateConfig& cfg);

}  // namespace the_eyes::scenegraph

#endif  // THE_EYES_SCENEGRAPH_PREDICATES_HPP
