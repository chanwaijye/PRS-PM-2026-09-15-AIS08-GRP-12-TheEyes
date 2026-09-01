// One camera's view of the board in one frame.
#ifndef THE_EYES_CALIBRATION_OBSERVATION_HPP
#define THE_EYES_CALIBRATION_OBSERVATION_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace the_eyes::calibration {

// ChArUco reports a SUBSET of corners -- whichever markers were resolvable --
// so corner ids travel alongside the points. Assuming a full grid is the
// classic ChArUco bug: it silently pairs image points with the wrong 3D points.
struct BoardObservation {
  std::string camera_id;
  uint64_t stamp_ns = 0;
  std::vector<int> corner_ids;
  std::vector<cv::Point2f> image_points;
  cv::Size image_size;

  bool valid() const {
    return corner_ids.size() == image_points.size() && !corner_ids.empty() &&
           image_size.width > 0 && image_size.height > 0;
  }
  size_t size() const { return corner_ids.size(); }
};

// Corner ids present in both observations, sorted. The basis of the round-trip
// check and of any stereo constraint.
std::vector<int> SharedCorners(const BoardObservation& a, const BoardObservation& b);

}  // namespace the_eyes::calibration

#endif  // THE_EYES_CALIBRATION_OBSERVATION_HPP
