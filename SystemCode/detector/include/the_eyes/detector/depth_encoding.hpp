// Depth → network input channel, for the 4-channel RGB-D early-fusion stem.
//
// THE PROBLEM. RealSense returns raw 0 for stereo holes — dark surfaces,
// specular highlights, depth discontinuities, anything out of range. Fed
// straight in, the network reads 0 as "object at zero range", which is the most
// confident possible statement about a pixel that carries no information at all.
//
// THE CHOICE (AGENTS.md: "sentinel or a validity mask channel — decide once and
// document it"). This is the SENTINEL encoding:
//
//   valid   d ∈ [min_m, max_m]  ->  [valid_floor, 1.0]   (linear)
//   invalid raw 0               ->  sentinel  (0.0, strictly below valid_floor)
//
// Invalid is therefore unreachable by any valid measurement, so the network can
// learn it as a distinct state, and the input stays 4 channels as specified.
//
// The alternative — a 5th validity-mask channel — is semantically cleaner but
// contradicts the 4-channel spec, costs another channel of compute, and hands
// the network a signal it can already infer from the sentinel.
//
// Depth is normalised in METRIC METRES, never raw uint16: the raw scale depends
// on the device's depth_scale, so a model trained on raw units silently breaks
// on a camera with a different scale.
#ifndef THE_EYES_DETECTOR_DEPTH_ENCODING_HPP
#define THE_EYES_DETECTOR_DEPTH_ENCODING_HPP

#include <cstdint>

#include <opencv2/core.hpp>

namespace the_eyes::detector {

struct DepthEncoding {
  // Working range for tabletop manipulation. Measurements outside it are
  // CLIPPED rather than discarded — they are real observations, just saturated,
  // and discarding them would manufacture holes the sensor did not report.
  double min_m = 0.20;
  double max_m = 1.50;
  // Lowest value a valid measurement can take. The gap below it is what makes
  // the sentinel distinguishable.
  float valid_floor = 0.1f;
  float sentinel = 0.0f;

  // Value used to pad the depth channel when letterboxing, as a raw uint8.
  //
  // Semantically this SHOULD be the sentinel -- padding is not a measurement.
  // But ultralytics pads every channel with 114 (verified: a 4-channel array
  // letterboxes to [114 114 114 114]), so a model trained through that pipeline
  // has learned "border == 114", which decodes to a surface at ~0.70 m. With a
  // 640x480 source that border is a quarter of the input.
  //
  // Inference must match training, so this defaults to the training value
  // rather than to the semantically correct one. Set it to 0 only alongside a
  // training pipeline that pads depth with the sentinel.
  int letterbox_pad_u8 = 114;

  // Throws std::invalid_argument if the range is degenerate or the sentinel is
  // not strictly below valid_floor — without that gap, "no data" and "closest
  // possible object" become the same input, which is the exact failure this
  // encoding exists to prevent.
  void Validate() const;

  float Encode(uint16_t raw, double depth_scale_m) const;
  bool IsInvalid(float encoded) const { return encoded < valid_floor; }

  // Inverse, for tests and for visualising what the network actually sees.
  // Returns a negative value for the sentinel.
  double Decode(float encoded) const;
};

}  // namespace the_eyes::detector

#endif  // THE_EYES_DETECTOR_DEPTH_ENCODING_HPP
