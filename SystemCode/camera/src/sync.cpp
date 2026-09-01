#include "the_eyes/camera/sync.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace the_eyes::camera {

double SkewSample::skew_ms() const {
  // Signed, in the sense "a minus b". Kept signed so a persistent one-sided
  // offset stays distinguishable from symmetric jitter.
  const double a = static_cast<double>(stamp_a_ns);
  const double b = static_cast<double>(stamp_b_ns);
  return (a - b) * 1e-6;
}

std::vector<SkewSample> PairNearest(const std::vector<uint64_t>& stamps_a,
                                    const std::vector<uint64_t>& stamps_b) {
  std::vector<SkewSample> samples;
  if (stamps_a.empty() || stamps_b.empty()) return samples;

  std::vector<uint64_t> sorted_b = stamps_b;
  std::sort(sorted_b.begin(), sorted_b.end());

  for (const uint64_t a : stamps_a) {
    // Nearest in time, not nearest in index: real streams are not lock-stepped,
    // and index pairing would measure buffering order rather than capture skew.
    const auto upper = std::lower_bound(sorted_b.begin(), sorted_b.end(), a);
    uint64_t best = 0;
    double best_distance = -1.0;

    const auto consider = [&](uint64_t candidate) {
      const double distance = std::abs(static_cast<double>(candidate) - static_cast<double>(a));
      if (best_distance < 0.0 || distance < best_distance) {
        best_distance = distance;
        best = candidate;
      }
    };
    if (upper != sorted_b.end()) consider(*upper);
    if (upper != sorted_b.begin()) consider(*std::prev(upper));

    samples.push_back(SkewSample{a, best});
  }
  return samples;
}

SyncReport AnalyseSkew(const std::vector<SkewSample>& samples, const std::string& camera_a,
                       const std::string& camera_b, double max_skew_ms) {
  SyncReport report;
  report.camera_a = camera_a;
  report.camera_b = camera_b;
  report.samples = static_cast<int>(samples.size());

  if (samples.empty()) {
    report.verdict = "no frames captured -- cannot assess synchronisation";
    return report;
  }

  std::vector<double> magnitudes;
  magnitudes.reserve(samples.size());
  double signed_sum = 0.0;
  for (const SkewSample& sample : samples) {
    const double skew = sample.skew_ms();
    signed_sum += skew;
    magnitudes.push_back(std::abs(skew));
  }

  std::sort(magnitudes.begin(), magnitudes.end());
  report.mean_signed_skew_ms = signed_sum / static_cast<double>(samples.size());
  report.mean_skew_ms =
      std::accumulate(magnitudes.begin(), magnitudes.end(), 0.0) /
      static_cast<double>(magnitudes.size());
  report.max_skew_ms = magnitudes.back();
  report.p95_skew_ms =
      magnitudes[static_cast<size_t>(0.95 * static_cast<double>(magnitudes.size() - 1))];
  double variance = 0.0;
  for (const SkewSample& sample : samples) {
    const double d = sample.skew_ms() - report.mean_signed_skew_ms;
    variance += d * d;
  }
  report.jitter_ms = std::sqrt(variance / static_cast<double>(samples.size()));

  for (const double magnitude : magnitudes) {
    if (magnitude > max_skew_ms) ++report.outliers;
  }

  // Gate on JITTER, not on the raw maximum. A single late frame in sixty makes
  // max exceed a frame period while the streams are in fact locked; conversely
  // a large fixed offset with near-zero jitter is synchronised hardware plus a
  // constant pipeline delay. Judging on max conflates the two.
  const double outlier_fraction =
      static_cast<double>(report.outliers) / static_cast<double>(samples.size());
  report.within_threshold = report.jitter_ms <= max_skew_ms && outlier_fraction <= 0.05;

  std::ostringstream os;
  if (report.within_threshold) {
    os << "jitter " << report.jitter_ms << " ms (" << report.outliers << "/" << report.samples
       << " outliers)";
    if (std::abs(report.mean_signed_skew_ms) > 3.0 * report.jitter_ms &&
        std::abs(report.mean_signed_skew_ms) > 1.0) {
      os << "; the " << report.mean_signed_skew_ms
         << " ms mean is a FIXED OFFSET, not desynchronisation -- it is constant within a run "
            "and can be calibrated out";
    }
  } else if (outlier_fraction > 0.05) {
    os << report.outliers << " of " << report.samples << " samples exceed " << max_skew_ms
       << " ms -- frames are being dropped, or the streams are not locked";
  } else {
    os << "jitter " << report.jitter_ms << " ms exceeds " << max_skew_ms
       << " ms. Check the hardware sync cable between the two units: "
       << "inter_cam_sync_mode is not a software-only setting.";
  }
  report.verdict = os.str();
  return report;
}

}  // namespace the_eyes::camera
