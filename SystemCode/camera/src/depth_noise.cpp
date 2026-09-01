#include "the_eyes/camera/depth_noise.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <Eigen/Eigenvalues>

namespace the_eyes::camera {

double PlaneFit::SignedDistance(const Eigen::Vector3d& point) const {
  return normal.dot(point) + offset_m;
}

PlaneFit FitPlane(const std::vector<Eigen::Vector3d>& points) {
  PlaneFit fit;
  // Three points define a plane, but a fit from three has zero residual by
  // construction and its RMS would be a meaningless zero.
  if (points.size() < 10) return fit;

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& p : points) centroid += p;
  centroid /= static_cast<double>(points.size());

  // Total least squares: the plane normal is the eigenvector of the scatter
  // matrix with the smallest eigenvalue. This minimises PERPENDICULAR distance.
  // A z = ax + by + c least-squares fit minimises vertical distance instead and
  // is biased whenever the plane is tilted relative to the camera -- and a wall
  // is never perfectly fronto-parallel.
  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (const Eigen::Vector3d& p : points) {
    const Eigen::Vector3d d = p - centroid;
    scatter += d * d.transpose();
  }

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  if (solver.info() != Eigen::Success) return fit;

  fit.normal = solver.eigenvectors().col(0).normalized();  // smallest eigenvalue
  // Orient the normal toward the camera so `measured_distance` reads positive.
  if (fit.normal.z() > 0.0) fit.normal = -fit.normal;
  fit.offset_m = -fit.normal.dot(centroid);

  double squared_sum = 0.0;
  for (const Eigen::Vector3d& p : points) {
    const double d = fit.SignedDistance(p);
    squared_sum += d * d;
  }
  fit.rms_error_m = std::sqrt(squared_sum / static_cast<double>(points.size()));
  fit.points_used = static_cast<int>(points.size());
  fit.valid = true;
  return fit;
}

std::vector<Eigen::Vector3d> DepthToPoints(const cv::Mat& depth, const Eigen::Matrix3d& K,
                                           double depth_scale_m, double roi_fraction) {
  std::vector<Eigen::Vector3d> points;
  if (depth.empty() || depth.type() != CV_16UC1) return points;
  if (!(depth_scale_m > 0.0) || !(roi_fraction > 0.0) || roi_fraction > 1.0) return points;

  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);
  if (!(fx > 0.0) || !(fy > 0.0)) return points;

  // Central ROI only: the frame edges see the wall most obliquely and carry the
  // worst depth, which would dominate an RMS taken over the whole image.
  const int half_w = static_cast<int>(depth.cols * roi_fraction / 2.0);
  const int half_h = static_cast<int>(depth.rows * roi_fraction / 2.0);
  const int x0 = std::max(0, depth.cols / 2 - half_w);
  const int x1 = std::min(depth.cols, depth.cols / 2 + half_w);
  const int y0 = std::max(0, depth.rows / 2 - half_h);
  const int y1 = std::min(depth.rows, depth.rows / 2 + half_h);

  points.reserve(static_cast<size_t>((x1 - x0) * (y1 - y0)));
  for (int y = y0; y < y1; ++y) {
    const uint16_t* row = depth.ptr<uint16_t>(y);
    for (int x = x0; x < x1; ++x) {
      const uint16_t raw = row[x];
      // Zero means "no depth here", not "a point at the camera origin".
      // Including those would drag the plane fit toward the sensor.
      if (raw == 0) continue;
      const double z = raw * depth_scale_m;
      points.emplace_back((x - cx) * z / fx, (y - cy) * z / fy, z);
    }
  }
  return points;
}

bool DepthNoiseMeasurement::planar() const {
  if (!(median_depth_m > 0.0)) return false;
  // A real wall fits to a small fraction of its distance, and its plane offset
  // matches the median point depth. 5% and 20% are deliberately loose -- this is
  // distinguishing "wall" from "room", not grading the measurement.
  const double relative_rms = rms_error_m / median_depth_m;
  const double offset_disagreement =
      std::abs(measured_distance_m - median_depth_m) / median_depth_m;
  return relative_rms < 0.05 && offset_disagreement < 0.20;
}

std::string DepthNoiseReport::Format() const {
  std::ostringstream os;
  os << "Depth-noise characterisation -- " << camera_id << " (serial " << serial << ")\n";
  os << std::string(78, '-') << "\n";
  os << std::left << std::setw(10) << "nominal" << std::setw(14) << "median depth"
     << std::setw(12) << "plane off." << std::setw(12) << "RMS (mm)" << std::setw(11)
     << "RMS/dist" << std::setw(10) << "valid px" << std::setw(8) << "frames"
     << "planar?" << "\n";
  for (const DepthNoiseMeasurement& m : measurements) {
    os << std::left << std::fixed << std::setprecision(3) << std::setw(10)
       << m.nominal_distance_m << std::setw(14) << m.median_depth_m << std::setw(12)
       << m.measured_distance_m << std::setprecision(2) << std::setw(12)
       << (m.rms_error_m * 1000.0) << std::setprecision(4) << std::setw(11)
       << (m.median_depth_m > 0.0 ? m.rms_error_m / m.median_depth_m : 0.0)
       << std::setprecision(3) << std::setw(10) << m.valid_pixel_fraction << std::setw(8)
       << m.frames << (m.planar() ? "yes" : "NO -- not a flat wall") << "\n";
  }
  os << "\nRMS is the perpendicular residual of a total-least-squares plane fit over the\n"
     << "central ROI. This feeds the evaluation error budget: a fused-pose error at or\n"
     << "below these figures is sensor noise, not an algorithm problem.\n";
  return os.str();
}

}  // namespace the_eyes::camera
