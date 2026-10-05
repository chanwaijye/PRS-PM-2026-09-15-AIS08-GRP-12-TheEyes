#include "the_eyes/scenegraph/pddl.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <unordered_map>

namespace the_eyes::scenegraph {

std::string SanitizePddlName(const std::string& id, bool* changed) {
  std::string out;
  out.reserve(id.size());
  bool dirty = false;
  for (const char c : id) {
    const unsigned char u = static_cast<unsigned char>(c);
    if (std::isalnum(u) || c == '-' || c == '_') {
      out.push_back(c);
    } else {
      out.push_back('_');
      dirty = true;
    }
  }
  if (out.empty()) {
    out = "obj";
    dirty = true;
  } else if (!std::isalpha(static_cast<unsigned char>(out.front()))) {
    // A leading digit is legal in some PDDL parsers and not others. Prefixing
    // is cheap; discovering the difference during a live demo is not.
    out.insert(out.begin(), 'o');
    dirty = true;
  }
  if (changed != nullptr) *changed = dirty;
  return out;
}

PddlResult EmitInit(const SceneState& scene, const PredicateConfig& cfg) {
  PddlResult result;

  // Sanitize up front so the fact strings never carry an illegal name.
  SceneState renamed = scene;
  std::unordered_map<std::string, int> seen;
  for (ObjectState& o : renamed.objects) {
    bool changed = false;
    const std::string safe = SanitizePddlName(o.id, &changed);
    if (changed) {
      result.warnings.push_back("renamed object id '" + o.id + "' -> '" + safe +
                                "' to satisfy PDDL name rules");
    }
    // Sanitizing can collide two distinct ids onto one name, which would fuse
    // two objects into one PDDL constant. Disambiguate rather than collide.
    if (++seen[safe] > 1) {
      const std::string uniq = safe + "_" + std::to_string(seen[safe]);
      result.warnings.push_back("id '" + o.id + "' collided with '" + safe +
                                "' after sanitizing; using '" + uniq + "'");
      o.id = uniq;
    } else {
      o.id = safe;
    }
  }

  const PredicateResult pred = Extract(renamed, cfg);
  result.facts = pred.facts;
  result.unsupported = pred.unsupported;
  result.warnings.insert(result.warnings.end(), pred.warnings.begin(),
                         pred.warnings.end());
  std::sort(result.warnings.begin(), result.warnings.end());

  std::ostringstream os;
  os << "(:init\n";
  for (const std::string& fact : result.facts) os << "  " << fact << "\n";
  os << ")";
  result.init_block = os.str();
  return result;
}

}  // namespace the_eyes::scenegraph
