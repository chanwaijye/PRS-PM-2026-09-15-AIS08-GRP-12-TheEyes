#include "the_eyes/calibration/synthetic.hpp"

#include <cmath>
#include <random>

#include <opencv2/calib3d.hpp>

namespace the_eyes::calibration {

BoardObservation ProjectBoard(const SyntheticCamera& camera, const BoardSpec& board,
                              const Eigen::Isometry3d& T_cam_board, uint64_t stamp_ns,
                              const SyntheticNoise& noise) {
  BoardObservation observation;
  observation.camera_id = camera.camera_id;
  observation.stamp_ns = stamp_ns;
  observation.image_size = camera.image_size;

  std::mt19937 rng(noise.seed ^ static_cast<uint32_t>(stamp_ns));
  std::normal_distribution<double> pixel_noise(0.0, noise.pixel_sigma);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  cv::Mat K = (cv::Mat_<double>(3, 3) << camera.fx, 0, camera.cx, 0, camera.fy, camera.cy, 0, 0, 1);
  cv::Mat distortion(1, static_cast<int>(camera.distortion.size()), CV_64F);
  for (size_t i = 0; i < camera.distortion.size(); ++i) {
    distortion.at<double>(0, static_cast<int>(i)) = camera.distortion[i];
  }

  cv::Mat rotation(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) rotation.at<double>(r, c) = T_cam_board.linear()(r, c);
  }
  cv::Mat rvec;
  cv::Rodrigues(rotation, rvec);
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << T_cam_board.translation().x(),
                        T_cam_board.translation().y(), T_cam_board.translation().z());

  const std::vector<cv::Point3f> object_points = board.AllCornerPositions();
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, K, distortion, projected);

  for (int id = 0; id < board.corner_count(); ++id) {
    // A corner behind the camera projects to a finite point that is pure
    // nonsense, so depth must be checked rather than relying on image bounds.
    const Eigen::Vector3d in_cam = T_cam_board * board.CornerPosition(id);
    if (in_cam.z() <= 1e-6) continue;

    cv::Point2f p = projected[static_cast<size_t>(id)];
    if (noise.pixel_sigma > 0.0) {
      p.x += static_cast<float>(pixel_noise(rng));
      p.y += static_cast<float>(pixel_noise(rng));
    }
    if (p.x < 0.0f || p.y < 0.0f || p.x >= static_cast<float>(camera.image_size.width) ||
        p.y >= static_cast<float>(camera.image_size.height)) {
      continue;  // outside the sensor, exactly as a real detection would be
    }
    // Detector dropout: real ChArUco loses markers to blur and glare.
    if (noise.dropout_probability > 0.0 && uniform(rng) < noise.dropout_probability) continue;

    observation.corner_ids.push_back(id);
    observation.image_points.push_back(p);
  }
  return observation;
}

std::vector<Eigen::Isometry3d> SampleBoardPoses(const BoardSpec& board, int count,
                                                uint32_t seed) {
  // Centre the board in its own frame so tilting rotates about the middle
  // rather than swinging a corner out of frame.
  const double width = board.corners_x() * board.square_length_m;
  const double height = board.corners_y() * board.square_length_m;

  std::mt19937 rng(seed);
  // Matched to the physical board's usable window: a 720x540 mm board fits the
  // 848x480 frame only beyond ~0.68 m, and its markers stop decoding past
  // ~1.27 m. Sampling outside that would validate the solver over distances the
  // real board cannot be used at.
  std::uniform_real_distribution<double> distance(0.70, 1.25);
  std::uniform_real_distribution<double> lateral(-0.10, 0.10);
  // Varied tilt is what separates focal length from board distance. A stack of
  // fronto-parallel views leaves them nearly degenerate, and the calibration
  // converges to a confident wrong answer.
  std::uniform_real_distribution<double> tilt(-0.55, 0.55);
  std::uniform_real_distribution<double> spin(-3.14159, 3.14159);

  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = (Eigen::AngleAxisd(tilt(rng), Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(tilt(rng), Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(spin(rng), Eigen::Vector3d::UnitZ()))
                        .toRotationMatrix();
    pose.translation() = Eigen::Vector3d(lateral(rng), lateral(rng), distance(rng)) -
                         pose.linear() * Eigen::Vector3d(width / 2.0, height / 2.0, 0.0);
    poses.push_back(pose);
  }
  return poses;
}

}  // namespace the_eyes::calibration
