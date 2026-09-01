// Synthetic board observations from known ground truth.
//
// The primary development mode: generate observations by projecting the board's
// known corners through a known camera pose and intrinsics, then hand them to
// the solver and check it recovers what went in. That validates the solver
// independently of detector quality, lens distortion, and lighting -- none of
// which exist yet.
#ifndef THE_EYES_CALIBRATION_SYNTHETIC_HPP
#define THE_EYES_CALIBRATION_SYNTHETIC_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "the_eyes/calibration/board.hpp"
#include "the_eyes/calibration/observation.hpp"

namespace the_eyes::calibration {

struct SyntheticCamera {
  std::string camera_id;
  cv::Size image_size{1280, 720};
  double fx = 900.0;
  double fy = 900.0;
  double cx = 640.0;
  double cy = 360.0;
  // Plumb-bob k1, k2, p1, p2, k3. Zero by default: a distortion-free synthetic
  // camera isolates solver error from distortion-model error.
  std::vector<double> distortion{0.0, 0.0, 0.0, 0.0, 0.0};
};

struct SyntheticNoise {
  double pixel_sigma = 0.0;       // corner localisation noise, pixels
  double dropout_probability = 0.0;  // corners the detector fails to resolve
  uint32_t seed = 20260820;
};

// Projects every board corner through `T_cam_board`, keeping only those landing
// inside the image. Returns an observation with fewer corners than the board
// has -- exactly as a real ChArUco detection does.
BoardObservation ProjectBoard(const SyntheticCamera& camera, const BoardSpec& board,
                              const Eigen::Isometry3d& T_cam_board, uint64_t stamp_ns = 0,
                              const SyntheticNoise& noise = {});

// A spread of board poses in front of the camera: varied distance, tilt, and
// in-plane position. Intrinsic calibration needs this variety -- views that are
// all fronto-parallel at one distance leave focal length and distortion
// poorly separated.
std::vector<Eigen::Isometry3d> SampleBoardPoses(const BoardSpec& board, int count,
                                                uint32_t seed = 20260820);

}  // namespace the_eyes::calibration

#endif  // THE_EYES_CALIBRATION_SYNTHETIC_HPP
