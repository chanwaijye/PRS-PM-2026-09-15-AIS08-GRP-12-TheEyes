// ChArUco corner detection from images.
//
// Isolated behind this one header because it is the only part of calib_agent
// that needs opencv_contrib's aruco module. Everything else works on
// BoardObservations, which synthetic generation can produce -- so the solver
// stays fully testable without contrib, and without images.
#ifndef THE_EYES_CALIBRATION_CHARUCO_DETECTOR_HPP
#define THE_EYES_CALIBRATION_CHARUCO_DETECTOR_HPP

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "the_eyes/calibration/board.hpp"
#include "the_eyes/calibration/observation.hpp"

namespace the_eyes::calibration {

class CharucoDetector {
 public:
  // Throws std::runtime_error on an unknown dictionary name.
  explicit CharucoDetector(const BoardSpec& board);
  ~CharucoDetector();

  CharucoDetector(const CharucoDetector&) = delete;
  CharucoDetector& operator=(const CharucoDetector&) = delete;

  // Returns an observation with however many corners were resolvable -- often
  // far fewer than the board has. An empty result is normal (board not in
  // frame), not an error.
  BoardObservation Detect(const cv::Mat& image, const std::string& camera_id,
                          uint64_t stamp_ns) const;

  // Renders the board to a PNG for printing.
  //
  // Print at 100% scale and MEASURE a square with callipers afterwards: printer
  // scaling is the single most common source of a calibration that converges
  // beautifully to the wrong metric scale, because every reprojection error
  // stays small while every distance is wrong by a constant factor.
  void WriteBoardImage(const std::string& out_path, int pixels_per_square = 120) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace the_eyes::calibration

#endif  // THE_EYES_CALIBRATION_CHARUCO_DETECTOR_HPP
