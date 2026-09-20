#include "the_eyes/calibration/solver.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

namespace the_eyes::calibration {
namespace {

// PnP needs at least 4 correspondences for a unique solution; ChArUco corners
// are coplanar, so 4 is the true minimum and anything less is degenerate.
constexpr size_t kMinCornersForPnp = 6;

void ObjectAndImagePoints(const BoardObservation& observation, const BoardSpec& board,
                          std::vector<cv::Point3f>* object_points,
                          std::vector<cv::Point2f>* image_points) {
  object_points->clear();
  image_points->clear();
  for (size_t i = 0; i < observation.corner_ids.size(); ++i) {
    // Pair by corner ID, never by index: ChArUco returns a subset, and pairing
    // positionally silently associates image points with the wrong 3D points.
    const Eigen::Vector3d p = board.CornerPosition(observation.corner_ids[i]);
    object_points->emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
                                static_cast<float>(p.z()));
    image_points->push_back(observation.image_points[i]);
  }
}

Eigen::Isometry3d ToIsometry(const cv::Mat& rvec, const cv::Mat& tvec) {
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) out.linear()(r, c) = rotation.at<double>(r, c);
    out.translation()(r) = tvec.at<double>(r);
  }
  return out;
}

void ToRvecTvec(const Eigen::Isometry3d& pose, cv::Mat* rvec, cv::Mat* tvec) {
  cv::Mat rotation(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) rotation.at<double>(r, c) = pose.linear()(r, c);
  }
  cv::Rodrigues(rotation, *rvec);
  *tvec = (cv::Mat_<double>(3, 1) << pose.translation().x(), pose.translation().y(),
           pose.translation().z());
}

// Largest single-corner reprojection error in the same fit. Reported beside the
// RMSE so that one mislocalised corner is distinguishable from a bad pose.
double MaxResidual(const std::vector<cv::Point2f>& a, const std::vector<cv::Point2f>& b) {
  if (a.empty() || a.size() != b.size()) return 0.0;
  double worst = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    const double dx = a[i].x - b[i].x;
    const double dy = a[i].y - b[i].y;
    worst = std::max(worst, std::sqrt(dx * dx + dy * dy));
  }
  return worst;
}

double Rmse(const std::vector<cv::Point2f>& a, const std::vector<cv::Point2f>& b) {
  if (a.empty() || a.size() != b.size()) return 0.0;
  double sum = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    const double dx = a[i].x - b[i].x;
    const double dy = a[i].y - b[i].y;
    sum += dx * dx + dy * dy;
  }
  return std::sqrt(sum / static_cast<double>(a.size()));
}

}  // namespace

Eigen::Matrix3d IntrinsicResult::KEigen() const {
  Eigen::Matrix3d out = Eigen::Matrix3d::Identity();
  if (K.empty()) return out;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) out(r, c) = K.at<double>(r, c);
  }
  return out;
}

IntrinsicResult FactoryIntrinsics(const Eigen::Matrix3d& K, const cv::Size& image_size,
                                  const std::vector<double>& distortion) {
  IntrinsicResult result;
  result.source = IntrinsicSource::kFactory;
  result.image_size = image_size;
  result.views_used = 0;

  result.K = cv::Mat::eye(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) result.K.at<double>(r, c) = K(r, c);
  }
  result.distortion = cv::Mat::zeros(1, static_cast<int>(distortion.size()), CV_64F);
  for (size_t i = 0; i < distortion.size(); ++i) {
    result.distortion.at<double>(0, static_cast<int>(i)) = distortion[i];
  }
  return result;
}

IntrinsicResult CalibrateIntrinsics(const std::vector<BoardObservation>& observations,
                                    const BoardSpec& board, int min_views) {
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;
  cv::Size image_size;

  for (const BoardObservation& observation : observations) {
    if (!observation.valid() || observation.size() < kMinCornersForPnp) continue;
    std::vector<cv::Point3f> object;
    std::vector<cv::Point2f> image;
    ObjectAndImagePoints(observation, board, &object, &image);
    object_points.push_back(std::move(object));
    image_points.push_back(std::move(image));
    if (image_size.width == 0) {
      image_size = observation.image_size;
    } else if (image_size != observation.image_size) {
      throw std::runtime_error("CalibrateIntrinsics: observations have differing image sizes; "
                               "intrinsics are only meaningful for one resolution");
    }
  }

  if (static_cast<int>(object_points.size()) < min_views) {
    std::ostringstream os;
    os << "CalibrateIntrinsics: only " << object_points.size() << " usable views, need "
       << min_views << ". Too few views fit noise -- the reported RMSE would come out low "
       << "precisely because the solution is under-constrained.";
    throw std::runtime_error(os.str());
  }

  IntrinsicResult result;
  result.source = IntrinsicSource::kCalibrated;
  result.image_size = image_size;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  result.reprojection_rmse_px = cv::calibrateCamera(object_points, image_points, image_size,
                                                    result.K, result.distortion, rvecs, tvecs);
  result.views_used = static_cast<int>(object_points.size());

  // cv::calibrateCamera returns the overall RMS; per-view values localise a bad
  // capture, which is what actually tells you which frame to reshoot.
  for (size_t i = 0; i < object_points.size(); ++i) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points[i], rvecs[i], tvecs[i], result.K, result.distortion,
                      projected);
    result.per_view_rmse_px.push_back(Rmse(image_points[i], projected));
  }
  return result;
}

PoseResult EstimateBoardPose(const BoardObservation& observation,
                             const IntrinsicResult& intrinsics, const BoardSpec& board) {
  if (!observation.valid()) {
    throw std::invalid_argument("EstimateBoardPose: malformed observation");
  }
  if (observation.size() < kMinCornersForPnp) {
    std::ostringstream os;
    os << "EstimateBoardPose: " << observation.size() << " corners is too few to constrain "
       << "6-DoF (need " << kMinCornersForPnp << ")";
    throw std::runtime_error(os.str());
  }

  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  ObjectAndImagePoints(observation, board, &object_points, &image_points);

  cv::Mat rvec;
  cv::Mat tvec;
  // IPPE_SQUARE is for 4-point square targets; the corners here are coplanar
  // but many, so ITERATIVE with a planar-friendly init is the right choice.
  if (!cv::solvePnP(object_points, image_points, intrinsics.K, intrinsics.distortion, rvec, tvec,
                    false, cv::SOLVEPNP_ITERATIVE)) {
    throw std::runtime_error("EstimateBoardPose: solvePnP failed to converge");
  }

  PoseResult result;
  result.T_cam_board = ToIsometry(rvec, tvec);
  result.corners_used = static_cast<int>(observation.size());

  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, intrinsics.K, intrinsics.distortion, projected);
  result.reprojection_rmse_px = Rmse(image_points, projected);
  result.max_residual_px = MaxResidual(image_points, projected);
  return result;
}

Eigen::Isometry3d RegisterToWorld(const Eigen::Isometry3d& T_cam_board,
                                  const Eigen::Isometry3d& T_world_board) {
  return T_world_board * T_cam_board.inverse();
}

double ReprojectionRmse(const BoardObservation& observation, const IntrinsicResult& intrinsics,
                        const BoardSpec& board, const Eigen::Isometry3d& T_cam_board) {
  if (!observation.valid()) return 0.0;
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  ObjectAndImagePoints(observation, board, &object_points, &image_points);

  cv::Mat rvec;
  cv::Mat tvec;
  ToRvecTvec(T_cam_board, &rvec, &tvec);

  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, intrinsics.K, intrinsics.distortion, projected);
  return Rmse(image_points, projected);
}

}  // namespace the_eyes::calibration
