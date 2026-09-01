#include "the_eyes/calibration/observation.hpp"

#include <algorithm>

namespace the_eyes::calibration {

std::vector<int> SharedCorners(const BoardObservation& a, const BoardObservation& b) {
  std::vector<int> sorted_a = a.corner_ids;
  std::vector<int> sorted_b = b.corner_ids;
  std::sort(sorted_a.begin(), sorted_a.end());
  std::sort(sorted_b.begin(), sorted_b.end());

  std::vector<int> shared;
  std::set_intersection(sorted_a.begin(), sorted_a.end(), sorted_b.begin(), sorted_b.end(),
                        std::back_inserter(shared));
  return shared;
}

}  // namespace the_eyes::calibration
