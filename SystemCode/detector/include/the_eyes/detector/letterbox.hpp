// Aspect-preserving resize with padding, and the inverse mapping.
//
// YOLOv8 takes a square input. Squashing a 16:9 frame into it distorts every
// object and costs accuracy, so the image is scaled by one factor and padded.
// The inverse must then be applied to every box -- getting that wrong produces
// detections that are subtly offset in a way that looks like a calibration
// error rather than a preprocessing bug.
#ifndef THE_EYES_DETECTOR_LETTERBOX_HPP
#define THE_EYES_DETECTOR_LETTERBOX_HPP

#include <opencv2/core.hpp>

namespace the_eyes::detector {

struct LetterboxTransform {
  double scale = 1.0;   // source pixels -> network pixels
  int pad_x = 0;        // padding added on the LEFT, network pixels
  int pad_y = 0;        // padding added on the TOP, network pixels
  cv::Size source_size;
  cv::Size network_size;

  // Maps a box from network pixel coordinates back to the source image, and
  // clamps it to the source bounds.
  cv::Rect2i ToSource(const cv::Rect2f& network_box) const;
};

// Writes the letterboxed image into `dst` (network_size square) and returns the
// transform needed to invert it.
//
// Type-agnostic: call it once on the CV_8UC3 colour image and again on the
// CV_16UC1 aligned depth with the SAME source size, and both land on the same
// grid. Pad depth with 0 — raw 0 is what the sensor reports for "no
// measurement", so the padding encodes as the invalid sentinel for free.
LetterboxTransform Letterbox(const cv::Mat& src, cv::Mat* dst, int network_size,
                             const cv::Scalar& pad_value = cv::Scalar(114, 114, 114));

}  // namespace the_eyes::detector

#endif  // THE_EYES_DETECTOR_LETTERBOX_HPP
