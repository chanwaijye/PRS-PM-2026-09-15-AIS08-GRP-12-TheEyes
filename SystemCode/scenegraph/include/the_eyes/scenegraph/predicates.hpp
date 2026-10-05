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
  // Classes that may be the CONTAINER argument of `in`. Default-deny: a class
  // absent from this list can never be `b` in (in ?a ?b). Geometry alone cannot
  // tell a container from a slab -- a table is a wide, thin box, and a pose
  // estimate low by more than half the table's thickness puts the object's
  // centroid inside it, satisfying every geometric test. See IsIn.
  std::vector<std::string> in_container_classes = {"bowl"};
  // Index i in this vector corresponds to bit i of ObjectState::visible_views.
  std::vector<std::string> view_names = {"cam_front", "cam_over"};

  // The support check (see Extract): an object that rests on nothing is left
  // out of the fact set. OFF in a default-constructed config, unlike the
  // tolerances above -- a bare config is the pairwise vocabulary and nothing
  // else, which is what a one-rule fixture wants. The shipped predicates.yaml
  // turns it on, and a test pins that.
  bool require_support = false;
  // Classes that need nothing under them: where every chain of support has to
  // end. Declared for the reason containers are -- a box cannot say whether it
  // is the table or something on it.
  std::vector<std::string> ground_classes = {"table"};
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
  // Ids left out of `facts` by the support check: they rest on nothing. Sorted.
  // A list of its own, not a warning per object, because this is the steady
  // state of a real scene rather than an event -- a tray the detector takes
  // for a bowl is here on every frame, and the runner prints every warning.
  std::vector<std::string> unsupported;
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
//
// With cfg.require_support, an object is reported only if it is GROUNDED: it is
// of a ground class, or it is `on` or `in` an object that is grounded. Anything
// else -- floating, sunk through the table, or resting only on something that
// itself rests on nothing -- is left out of every fact, as subject and as
// object, and named in `unsupported`. The frontend cannot make this call: on
// the 2026-10-05 recordings a pose at the occluder's range, a cube fitted to
// the occluder's print and a tray taken for a bowl all carried pose scores of
// 0.86-0.97, against 0.64 for the one correct estimate beside them. What they
// had in common is that none of them rested on anything.
//
// Grounding is decided on the pairwise rules alone, before containment is made
// exclusive, so an ungrounded container cannot take a grounded object down
// with it: a cube on the table that also falls inside a phantom bowl is kept,
// the bowl is not, and the cube then reads as on the table.
//
// It asserts a static scene. An object in a gripper rests on nothing by this
// definition; a consumer with a hand in the scene declares it a ground class
// or turns the check off.
PredicateResult Extract(const SceneState& scene, const PredicateConfig& cfg);

// Same rules, but writes the facts back into each object's `relations` field,
// which is the field fusion leaves empty for this module to fill. An
// unsupported object stays in the scene with no relations: the scene is
// fusion's account of what it tracked, and the returned result says which of
// them the scene graph did not report.
PredicateResult Annotate(SceneState& scene, const PredicateConfig& cfg);

}  // namespace the_eyes::scenegraph

#endif  // THE_EYES_SCENEGRAPH_PREDICATES_HPP
