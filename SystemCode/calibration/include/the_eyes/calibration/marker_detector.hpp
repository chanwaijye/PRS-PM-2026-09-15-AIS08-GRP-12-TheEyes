// Standalone ArUco marker detection: id + T_cam_marker per marker in an image.
//
// Separate from CharucoDetector because the ground-truth trays carry loose
// markers, not a board -- there is no shared plane to interpolate chessboard
// corners on, and each marker is solved on its own four corners.
//
// Isolated behind this header for the same reason CharucoDetector is: it is the
// only other part of the project that needs opencv_contrib's aruco, and
// everything downstream of it works on plain Eigen poses that a test can
// synthesise without images.
#ifndef THE_EYES_CALIBRATION_MARKER_DETECTOR_HPP
#define THE_EYES_CALIBRATION_MARKER_DETECTOR_HPP

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace the_eyes::calibration {

struct MarkerObservation {
  int id = -1;
  Eigen::Isometry3d T_cam_marker = Eigen::Isometry3d::Identity();
  // Mean corner reprojection error, px. A marker resolved at a glancing angle
  // or half in shadow still returns a pose; this is what says not to trust it.
  double reprojection_px = 0.0;
  // Apparent side length in pixels. A small marker gives a noisier pose, and
  // its size is the cheapest warning of that.
  double apparent_size_px = 0.0;

  // Planar-pose ambiguity: error(second solution) / error(best solution), >= 1.
  //
  // A square marker admits TWO poses that reproject almost identically, and the
  // degeneracy is worst when the marker faces the camera head-on. Both fit the
  // corners, so `reprojection_px` cannot tell them apart -- measured on
  // synthetic renders, a fronto-parallel 40 mm marker at 0.7 m returns the
  // WRONG one with 9 degrees of orientation error while reprojecting perfectly.
  // A ratio near 1.0 means the solver essentially guessed; a large ratio means
  // the chosen pose is well separated from its twin.
  double ambiguity = 0.0;
};

class MarkerDetector {
 public:
  // marker_length_m scales every recovered translation, so it is the one input
  // whose error is invisible: poses stay self-consistent and come out wrong by
  // a constant factor. Throws on an unknown dictionary or a non-positive length.
  MarkerDetector(const std::string& dictionary, double marker_length_m);
  ~MarkerDetector();

  MarkerDetector(const MarkerDetector&) = delete;
  MarkerDetector& operator=(const MarkerDetector&) = delete;

  // `image` may be colour or grey. `K` is the pinhole matrix for THIS image's
  // resolution; passing one from a different resolution silently biases every
  // pose. Distortion is assumed already removed (the RealSense colour stream is
  // rectified), so no coefficients are taken -- if that ever stops being true
  // this signature has to change rather than quietly accept zeros.
  //
  // An empty result is normal (no tray in frame), not an error.
  std::vector<MarkerObservation> Detect(const cv::Mat& image, const Eigen::Matrix3d& K) const;

  // True when the build has opencv_aruco. Detect() throws when this is false,
  // rather than returning an empty vector that would read as "no markers".
  static bool available();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace the_eyes::calibration

#endif  // THE_EYES_CALIBRATION_MARKER_DETECTOR_HPP
