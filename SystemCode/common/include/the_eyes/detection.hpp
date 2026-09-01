// INTERFACES.md §2 — Detector → Pose Estimator.
//
// Split out of types.hpp because it needs OpenCV: fusion and scenegraph must be
// buildable and testable without linking cv.
#ifndef THE_EYES_DETECTION_HPP
#define THE_EYES_DETECTION_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/types.hpp>

namespace the_eyes {

struct Detection {
  std::string cls;        // join key to the mesh registry; a detection whose
                          // class has no registered mesh is dropped with a
                          // warning and must not reach the pose estimator
  float score = 0.0f;     // detector confidence [0,1]
  cv::Rect2i bbox;        // pixel coords in the color frame
  uint32_t track_id = 0;  // stable within one camera; 0 = untracked
};

struct DetectionSet {
  std::string camera_id;  // "cam_front" | "cam_over"
  uint64_t stamp_ns = 0;  // source frame timestamp, NOT receive time
  std::vector<Detection> detections;
};

}  // namespace the_eyes

#endif  // THE_EYES_DETECTION_HPP
