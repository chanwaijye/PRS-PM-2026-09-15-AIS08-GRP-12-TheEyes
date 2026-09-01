#include "the_eyes/detector/depth_encoding.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace the_eyes::detector {

void DepthEncoding::Validate() const {
  if (!(max_m > min_m) || !(min_m > 0.0)) {
    std::ostringstream os;
    os << "DepthEncoding: need 0 < min_m (" << min_m << ") < max_m (" << max_m << ")";
    throw std::invalid_argument(os.str());
  }
  if (!(sentinel < valid_floor)) {
    std::ostringstream os;
    os << "DepthEncoding: sentinel (" << sentinel << ") must be strictly below valid_floor ("
       << valid_floor << ") — otherwise 'no data' and 'closest possible object' encode to the "
       << "same value, which is precisely what this encoding exists to prevent";
    throw std::invalid_argument(os.str());
  }
  if (!(valid_floor > 0.0f) || !(valid_floor < 1.0f)) {
    throw std::invalid_argument("DepthEncoding: valid_floor must lie in (0, 1)");
  }
}

float DepthEncoding::Encode(uint16_t raw, double depth_scale_m) const {
  // Raw 0 is the sensor saying "I have no measurement here", not "zero metres".
  if (raw == 0) return sentinel;
  if (!(depth_scale_m > 0.0)) return sentinel;

  const double metres = static_cast<double>(raw) * depth_scale_m;
  // Clip rather than drop: a reading outside the working range is a real
  // observation that saturates, and turning it into a hole would fabricate
  // missing data the sensor never reported.
  const double clipped = std::clamp(metres, min_m, max_m);
  const double t = (clipped - min_m) / (max_m - min_m);
  return valid_floor + static_cast<float>(t) * (1.0f - valid_floor);
}

double DepthEncoding::Decode(float encoded) const {
  if (IsInvalid(encoded)) return -1.0;
  const double t = (encoded - valid_floor) / (1.0f - valid_floor);
  return min_m + t * (max_m - min_m);
}

}  // namespace the_eyes::detector
