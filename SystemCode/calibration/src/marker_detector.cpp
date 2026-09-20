#include "the_eyes/calibration/marker_detector.hpp"

#include "aruco_dictionary.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace the_eyes::calibration {

struct MarkerDetector::Impl {
  double length_m = 0.0;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::DetectorParameters> parameters;
};

bool MarkerDetector::available() { return true; }

MarkerDetector::MarkerDetector(const std::string& dictionary, double marker_length_m)
    : impl_(std::make_unique<Impl>()) {
  if (!(marker_length_m > 0.0)) {
    throw std::runtime_error("marker detector: marker_length_m must be > 0");
  }
  impl_->length_m = marker_length_m;
  impl_->dictionary = cv::aruco::getPredefinedDictionary(DictionaryId(dictionary));
  impl_->parameters = cv::aruco::DetectorParameters::create();
  // Subpixel corner refinement. Without it the corners land on integer pixels
  // and a 40 mm marker at 0.7 m carries several degrees of orientation noise --
  // which propagates straight into ground truth.
  impl_->parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

MarkerDetector::~MarkerDetector() = default;

std::vector<MarkerObservation> MarkerDetector::Detect(const cv::Mat& image,
                                                      const Eigen::Matrix3d& K) const {
  std::vector<MarkerObservation> out;
  if (image.empty()) return out;

  cv::Mat grey;
  if (image.channels() == 3) {
    cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
  } else {
    grey = image;
  }

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(grey, impl_->dictionary, corners, ids, impl_->parameters);
  if (ids.empty()) return out;

  cv::Mat cam = (cv::Mat_<double>(3, 3) << K(0, 0), K(0, 1), K(0, 2), K(1, 0), K(1, 1), K(1, 2),
                 K(2, 0), K(2, 1), K(2, 2));
  const cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);

  // The marker's own four corners in its own frame: +x right, +y up, +z out of
  // the face, origin at the centre -- the convention config/marker_trays.yaml
  // is written against.
  const double h = impl_->length_m / 2.0;
  const std::vector<cv::Point3f> object_pts = {{-static_cast<float>(h), static_cast<float>(h), 0},
                                               {static_cast<float>(h), static_cast<float>(h), 0},
                                               {static_cast<float>(h), -static_cast<float>(h), 0},
                                               {-static_cast<float>(h), -static_cast<float>(h), 0}};

  for (size_t i = 0; i < ids.size(); ++i) {
    MarkerObservation m;
    m.id = ids[i];

    // solvePnPGeneric with IPPE_SQUARE returns BOTH planar solutions.
    // estimatePoseSingleMarkers returns only the winner and hides the twin.
    //
    // The reprojection errors are recomputed here rather than read from the
    // solver's optional output: that Mat's element type is not guaranteed and
    // reading it as double produced denormal garbage, which would have made the
    // ambiguity gate pass or fail at random.
    std::vector<cv::Mat> rvs, tvs;
    const int n = cv::solvePnPGeneric(object_pts, corners[i], cam, dist, rvs, tvs, false,
                                      cv::SOLVEPNP_IPPE_SQUARE);
    if (n < 1 || rvs.empty()) continue;

    auto mean_reproj = [&](const cv::Mat& rv, const cv::Mat& tv) {
      std::vector<cv::Point2f> proj;
      cv::projectPoints(object_pts, rv, tv, cam, dist, proj);
      double e = 0.0;
      for (size_t c = 0; c < proj.size() && c < corners[i].size(); ++c) {
        e += cv::norm(proj[c] - corners[i][c]);
      }
      return e / static_cast<double>(proj.size());
    };

    size_t best = 0;
    double best_err = mean_reproj(rvs[0], tvs[0]);
    double second_err = std::numeric_limits<double>::infinity();
    for (size_t j = 1; j < rvs.size(); ++j) {
      const double e = mean_reproj(rvs[j], tvs[j]);
      if (e < best_err) {
        second_err = best_err;
        best_err = e;
        best = j;
      } else if (e < second_err) {
        second_err = e;
      }
    }
    m.reprojection_px = best_err;
    m.ambiguity = best_err > 1e-9 ? second_err / best_err
                                  : std::numeric_limits<double>::infinity();

    const cv::Vec3d rvec(rvs[best].at<double>(0), rvs[best].at<double>(1), rvs[best].at<double>(2));
    const cv::Vec3d tvec(tvs[best].at<double>(0), tvs[best].at<double>(1), tvs[best].at<double>(2));

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d Re;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) Re(r, c) = R.at<double>(r, c);
    m.T_cam_marker = Eigen::Isometry3d::Identity();
    m.T_cam_marker.linear() = Re;
    m.T_cam_marker.translation() = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);

    double perim = 0.0;
    for (size_t c = 0; c < corners[i].size(); ++c) {
      perim += cv::norm(corners[i][(c + 1) % corners[i].size()] - corners[i][c]);
    }
    m.apparent_size_px = perim / 4.0;

    for (size_t c = 0; c < 4 && c < corners[i].size(); ++c) {
      m.corners[c] = Eigen::Vector2d(corners[i][c].x, corners[i][c].y);
    }

    out.push_back(m);
  }
  return out;
}

MultiMarkerPose SolveMultiMarkerPose(const std::vector<MarkerObservation>& markers,
                                     const std::map<int, Eigen::Vector3d>& t_frame_marker,
                                     double marker_length_m, const Eigen::Matrix3d& K) {
  if (!(marker_length_m > 0.0)) {
    throw std::runtime_error("SolveMultiMarkerPose: marker_length_m must be > 0");
  }

  // Marker corners in the marker's own frame, in the detector's order:
  // +x right, +y up, +z out of the face, origin at the centre. The markers
  // share the frame's orientation (config/marker_trays.yaml declares
  // translation only), so a corner in frame coordinates is just the marker's
  // centre plus this offset.
  const double h = marker_length_m / 2.0;
  const Eigen::Vector3d offset[4] = {{-h, h, 0.0}, {h, h, 0.0}, {h, -h, 0.0}, {-h, -h, 0.0}};

  std::vector<cv::Point3f> object_pts;
  std::vector<cv::Point2f> image_pts;
  int used = 0;
  for (const MarkerObservation& m : markers) {
    const auto placed = t_frame_marker.find(m.id);
    if (placed == t_frame_marker.end()) continue;  // not on this frame: ignore
    for (int c = 0; c < 4; ++c) {
      const Eigen::Vector3d p = placed->second + offset[c];
      object_pts.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
                              static_cast<float>(p.z()));
      image_pts.emplace_back(static_cast<float>(m.corners[c].x()),
                             static_cast<float>(m.corners[c].y()));
    }
    ++used;
  }
  if (used == 0) {
    throw std::runtime_error(
        "SolveMultiMarkerPose: none of the detected markers has a known position on this frame");
  }

  cv::Mat cam = (cv::Mat_<double>(3, 3) << K(0, 0), K(0, 1), K(0, 2), K(1, 0), K(1, 1), K(1, 2),
                 K(2, 0), K(2, 1), K(2, 2));
  const cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);

  // Every point is coplanar, so IPPE applies and -- as with a single marker --
  // returns BOTH planar solutions rather than hiding the twin. The ambiguity
  // ratio stays meaningful at the frame level, which matters because the point
  // of solving jointly is that a wider spread separates the two solutions.
  // IPPE_SQUARE is only valid for exactly one square's four corners.
  const cv::SolvePnPMethod flag = (used == 1) ? cv::SOLVEPNP_IPPE_SQUARE : cv::SOLVEPNP_IPPE;
  std::vector<cv::Mat> rvs, tvs;
  const int n = cv::solvePnPGeneric(object_pts, image_pts, cam, dist, rvs, tvs, false, flag);
  if (n < 1 || rvs.empty()) {
    throw std::runtime_error("SolveMultiMarkerPose: solvePnP found no solution");
  }

  auto rms_reproj = [&](const cv::Mat& rv, const cv::Mat& tv) {
    std::vector<cv::Point2f> proj;
    cv::projectPoints(object_pts, rv, tv, cam, dist, proj);
    double sum = 0.0;
    for (size_t i = 0; i < proj.size() && i < image_pts.size(); ++i) {
      const double d = cv::norm(proj[i] - image_pts[i]);
      sum += d * d;
    }
    return std::sqrt(sum / static_cast<double>(proj.size()));
  };

  size_t best = 0;
  double best_err = rms_reproj(rvs[0], tvs[0]);
  double second_err = std::numeric_limits<double>::infinity();
  for (size_t j = 1; j < rvs.size(); ++j) {
    const double e = rms_reproj(rvs[j], tvs[j]);
    if (e < best_err) {
      second_err = best_err;
      best_err = e;
      best = j;
    } else if (e < second_err) {
      second_err = e;
    }
  }

  MultiMarkerPose out;
  out.markers_used = used;
  out.reprojection_px = best_err;
  out.ambiguity =
      best_err > 1e-9 ? second_err / best_err : std::numeric_limits<double>::infinity();

  const cv::Vec3d rvec(rvs[best].at<double>(0), rvs[best].at<double>(1), rvs[best].at<double>(2));
  const cv::Vec3d tvec(tvs[best].at<double>(0), tvs[best].at<double>(1), tvs[best].at<double>(2));
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d Re;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) Re(r, c) = R.at<double>(r, c);
  out.T_cam_frame = Eigen::Isometry3d::Identity();
  out.T_cam_frame.linear() = Re;
  out.T_cam_frame.translation() = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);
  return out;
}

}  // namespace the_eyes::calibration
