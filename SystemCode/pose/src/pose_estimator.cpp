#include "the_eyes/pose/pose_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>

namespace the_eyes::pose {

bool Frame::valid() const {
  if (rgb.empty() || depth.empty()) return false;
  if (rgb.type() != CV_8UC3) return false;
  if (depth.type() != CV_16UC1) return false;
  // Depth must be aligned to colour (INTERFACES.md §1). Identical dimensions
  // are the cheapest check that alignment was actually applied -- an unaligned
  // depth stream typically differs in resolution, and if it does not, the
  // startup timestamp assertion is the other line of defence.
  if (rgb.size() != depth.size()) return false;
  return depth_scale_m > 0.0;
}

void LatencyStats::Record(double milliseconds) {
  if (std::isfinite(milliseconds) && milliseconds >= 0.0) samples_.push_back(milliseconds);
}

void LatencyStats::Reset() { samples_.clear(); }

double LatencyStats::mean_ms() const {
  if (samples_.empty()) return 0.0;
  return std::accumulate(samples_.begin(), samples_.end(), 0.0) /
         static_cast<double>(samples_.size());
}

double LatencyStats::percentile_ms(double p) const {
  if (samples_.empty()) return 0.0;
  std::vector<double> sorted = samples_;
  std::sort(sorted.begin(), sorted.end());
  const double clamped = std::min(1.0, std::max(0.0, p));
  const size_t index = static_cast<size_t>(
      std::llround(clamped * static_cast<double>(sorted.size() - 1)));
  return sorted[index];
}

double LatencyStats::max_ms() const {
  if (samples_.empty()) return 0.0;
  return *std::max_element(samples_.begin(), samples_.end());
}

std::string LatencyStats::Format(const std::string& label) const {
  std::ostringstream os;
  os << std::fixed << std::setprecision(2);
  os << label << ": n=" << samples_.size();
  if (samples_.empty()) {
    os << " (no samples)";
    return os.str();
  }
  os << " mean=" << mean_ms() << "ms"
     << " p50=" << percentile_ms(0.5) << "ms"
     << " p95=" << percentile_ms(0.95) << "ms"
     << " max=" << max_ms() << "ms";
  return os.str();
}

}  // namespace the_eyes::pose
