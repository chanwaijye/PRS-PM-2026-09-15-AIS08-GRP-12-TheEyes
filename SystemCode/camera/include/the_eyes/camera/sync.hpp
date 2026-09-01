// Cross-camera timestamp agreement.
//
// INTERFACES.md §1: frame timestamps across the two namespaces must agree
// within one frame period, asserted at startup and logged.
//
// Pure logic over timestamps, with no librealsense dependency, so the check
// itself is unit-testable without hardware.
#ifndef THE_EYES_CAMERA_SYNC_HPP
#define THE_EYES_CAMERA_SYNC_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace the_eyes::camera {

struct SkewSample {
  uint64_t stamp_a_ns = 0;
  uint64_t stamp_b_ns = 0;
  double skew_ms() const;
};

struct SyncReport {
  std::string camera_a;
  std::string camera_b;
  int samples = 0;
  double mean_skew_ms = 0.0;
  double max_skew_ms = 0.0;
  double p95_skew_ms = 0.0;
  // Signed mean: a persistent one-sided offset is a fixed pipeline delay, which
  // is correctable. Symmetric jitter about zero is not, and the two call for
  // completely different fixes -- so the sign is kept rather than absolute-valued
  // away.
  double mean_signed_skew_ms = 0.0;
  // Standard deviation of the SIGNED skew. This -- not the mean -- is what says
  // whether the cameras are synchronised. A constant offset is a fixed pipeline
  // delay and can be calibrated out; jitter cannot, and it is what actually
  // smears a moving object between the two views.
  double jitter_ms = 0.0;
  // Samples beyond the threshold. A handful among many is a dropped frame or a
  // pairing artefact; a large fraction is a genuine sync failure, and the two
  // call for completely different responses.
  int outliers = 0;
  bool within_threshold = false;
  std::string verdict;
};

SyncReport AnalyseSkew(const std::vector<SkewSample>& samples, const std::string& camera_a,
                       const std::string& camera_b, double max_skew_ms);

// Pairs each frame from A with the nearest-in-time frame from B. Real streams
// are not lock-stepped, so pairing by index would measure buffering order
// rather than capture skew.
std::vector<SkewSample> PairNearest(const std::vector<uint64_t>& stamps_a,
                                    const std::vector<uint64_t>& stamps_b);

}  // namespace the_eyes::camera

#endif  // THE_EYES_CAMERA_SYNC_HPP
