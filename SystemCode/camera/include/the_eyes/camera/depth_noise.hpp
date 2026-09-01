// Depth-noise characterisation: plane fit and RMS error against a flat wall.
//
// AGENTS.md §1 lists this as a DELIVERABLE, not a debugging step: the
// per-distance RMS is the measured input to the evaluation error budget, and it
// is what says whether a fused-pose error is sensor noise or an algorithm
// problem.
//
// The fit is pure geometry and is unit-tested against synthetic point clouds
// with known noise, so the metric is trustworthy before any wall is measured.
#ifndef THE_EYES_CAMERA_DEPTH_NOISE_HPP
#define THE_EYES_CAMERA_DEPTH_NOISE_HPP

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace the_eyes::camera {

struct PlaneFit {
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();  // unit length
  double offset_m = 0.0;                              // plane: normal . p + offset = 0
  double rms_error_m = 0.0;
  int points_used = 0;
  bool valid = false;

  double SignedDistance(const Eigen::Vector3d& point) const;
};

// Total-least-squares plane fit: minimises perpendicular distance, unlike a
// z = ax + by + c least-squares fit which minimises vertical distance and is
// biased when the plane is tilted relative to the camera. A wall is never
// perfectly fronto-parallel.
PlaneFit FitPlane(const std::vector<Eigen::Vector3d>& points);

// Back-projects a depth image's central ROI into camera-frame points.
// Zero-depth pixels are invalid and are skipped rather than treated as points
// at the origin.
std::vector<Eigen::Vector3d> DepthToPoints(const cv::Mat& depth, const Eigen::Matrix3d& K,
                                           double depth_scale_m, double roi_fraction);

struct DepthNoiseMeasurement {
  double nominal_distance_m = 0.0;
  double measured_distance_m = 0.0;   // plane's perpendicular offset from the origin
  // Median depth of the ROI points. Reported alongside the plane offset because
  // the two AGREE only when the scene really is a plane facing the camera. A fit
  // forced through a non-planar scene can put the plane nearly edge-on, giving a
  // tiny perpendicular offset while every point is metres away -- which reads as
  // a broken sensor when nothing is wrong.
  double median_depth_m = 0.0;
  double rms_error_m = 0.0;
  double valid_pixel_fraction = 0.0;
  int frames = 0;
  int points_per_frame = 0;

  // True when the plane fit is not credible as a wall measurement: the residual
  // is a large fraction of the distance, or the plane offset disagrees with the
  // median depth. Either means "this was not a flat wall".
  bool planar() const;
};

struct DepthNoiseReport {
  std::string camera_id;
  std::string serial;
  std::vector<DepthNoiseMeasurement> measurements;
  std::string Format() const;
};

}  // namespace the_eyes::camera

#endif  // THE_EYES_CAMERA_DEPTH_NOISE_HPP
