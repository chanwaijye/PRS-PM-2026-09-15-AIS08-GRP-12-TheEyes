// ROI derivation from a detector bounding box.
#ifndef THE_EYES_POSE_ROI_HPP
#define THE_EYES_POSE_ROI_HPP

#include <opencv2/core.hpp>

namespace the_eyes::pose {

struct RoiConfig {
  double margin_ratio = 0.15;
  int min_size_px = 24;
};

// Expands `bbox` by margin_ratio on each side and clamps it to an
// `image_width` x `image_height` frame.
//
// Returns an empty Rect when the result is smaller than min_size_px in either
// dimension, or when the bbox misses the image entirely. An empty return means
// "drop this detection" -- a crop too small to constrain 6-DoF produces a
// confident-looking wrong pose, which is worse for fusion than no pose at all,
// because fusion weights by score and cannot tell the difference.
cv::Rect2i ComputeRoi(const cv::Rect2i& bbox, int image_width, int image_height,
                      const RoiConfig& cfg);

}  // namespace the_eyes::pose

#endif  // THE_EYES_POSE_ROI_HPP
