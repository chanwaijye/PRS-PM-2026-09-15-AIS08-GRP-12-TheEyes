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

#include <array>
#include <map>
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

  // The four detected corners in image pixels, in the detector's own order.
  //
  // Kept because a pose from ONE marker is only as well conditioned as that
  // marker is wide: a 40 mm square at 0.7 m has poorly constrained depth, and
  // no amount of averaging independent single-marker poses fixes the
  // conditioning. Several markers on one rigid tray can instead be solved in a
  // SINGLE PnP over all their corners, which widens the effective target to the
  // marker spread -- 114 mm on the narrowest tray here, 204 mm on the widest.
  // That needs the corners, not just the per-marker pose. See
  // SolveMultiMarkerPose.
  std::array<Eigen::Vector2d, 4> corners{};
};

// One pose from several coplanar markers whose positions on a rigid body are
// known -- a ground-truth tray, in this project.
struct MultiMarkerPose {
  Eigen::Isometry3d T_cam_frame = Eigen::Isometry3d::Identity();
  double reprojection_px = 0.0;   // RMS over every corner used
  double ambiguity = 0.0;         // as MarkerObservation::ambiguity, for the joint fit
  int markers_used = 0;
};

// Solves ONE pose for the frame the markers are mounted on, from all their
// corners at once.
//
// MEASURED on this rig, 2026-09-20, against the per-marker-average path it
// replaces (cross-camera disagreement, the figure that does not lie):
//
//     tray             markers   spread     per-marker avg   joint
//     bowl                4    144-204 mm       2.71 mm      see docs
//     mustard_bottle      2       160 mm        3.78 mm
//     red_cube            2       114 mm        7.55 mm
//
// The ordering is monotonic in marker spread, which is the whole argument for
// this function: averaging independent poses suppresses random error but not
// the depth conditioning of a small planar target, and red_cube's 7.55 mm was
// consuming nearly all of the ~8.6 mm ADD-S threshold it exists to judge.
//
// `t_frame_marker` gives each marker id's centre in the frame's coordinates;
// markers not present in it are ignored, and markers in it but not detected are
// simply absent. Every marker is assumed coplanar with the frame's z = 0 plane
// and identically oriented, which is what config/marker_trays.yaml declares.
//
// Throws if no detected marker has a known position, or if marker_length_m is
// not positive.
MultiMarkerPose SolveMultiMarkerPose(const std::vector<MarkerObservation>& markers,
                                     const std::map<int, Eigen::Vector3d>& t_frame_marker,
                                     double marker_length_m, const Eigen::Matrix3d& K);

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
