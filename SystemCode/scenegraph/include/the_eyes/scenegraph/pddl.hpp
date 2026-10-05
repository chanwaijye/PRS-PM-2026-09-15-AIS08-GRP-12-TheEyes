// Emission of the PDDL (:init ...) block consumed by PM1's planner.
//
// INTERFACES.md §6: the predicate vocabulary must match PM1's domain exactly.
// Adding a predicate here without adding it there produces a plan-time parse
// error in a different repo, so the vocabulary lives in predicates.hpp and this
// file only formats it.
#ifndef THE_EYES_SCENEGRAPH_PDDL_HPP
#define THE_EYES_SCENEGRAPH_PDDL_HPP

#include <string>
#include <vector>

#include "the_eyes/scenegraph/predicates.hpp"
#include "the_eyes/types.hpp"

namespace the_eyes::scenegraph {

struct PddlResult {
  std::string init_block;              // "(:init\n  (on a b)\n  ...\n)"
  std::vector<std::string> facts;      // the same facts, unwrapped, sorted
  std::vector<std::string> unsupported;  // ids the support check left out, sorted
  std::vector<std::string> warnings;   // dropped objects, renamed ids
};

// PDDL names admit letters, digits, '-' and '_', and must start with a letter.
// Anything else is rewritten; the rewrite is reported as a warning rather than
// applied silently, because ids are the join key back to fusion's object
// instances and a silent rename breaks that correspondence.
std::string SanitizePddlName(const std::string& id, bool* changed);

// Full path: sanitize ids -> extract predicates -> format. Deterministic.
PddlResult EmitInit(const SceneState& scene, const PredicateConfig& cfg);

}  // namespace the_eyes::scenegraph

#endif  // THE_EYES_SCENEGRAPH_PDDL_HPP
