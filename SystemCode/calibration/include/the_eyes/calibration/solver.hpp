// Intrinsic calibration, per-view pose recovery, and world registration.
//
// Everything here operates on BoardObservations, never on images, so the whole
// solver is exercisable against synthetic observations generated from known
// ground truth -- which is what validates it independently of image quality
// (AGENTS.md §2).
#ifndef THE_EYES_CALIBRATION_SOLVER_HPP
#define THE_EYES_CALIBRATION_SOLVER_HPP

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "the_eyes/calibration/board.hpp"
#include "the_eyes/calibration/observation.hpp"

namespace the_eyes::calibration {

// Where a camera's intrinsics came from. This is not bookkeeping: a factory
// result has no calibration views, so the "enough views" quality gate does not
// apply to it and must not be run against views_used == 0.
enum class IntrinsicSource {
  kCalibrated,  // solved here from board observations
  kFactory,     // read from the device's own calibration
};

struct IntrinsicResult {
  cv::Mat K;                       // 3x3 CV_64F
  cv::Mat distortion;              // 1x5 CV_64F, plumb-bob
  double reprojection_rmse_px = 0.0;
  std::vector<double> per_view_rmse_px;
  int views_used = 0;
  cv::Size image_size;
  IntrinsicSource source = IntrinsicSource::kCalibrated;

  Eigen::Matrix3d KEigen() const;
};

// Wraps a device's own factory calibration as an IntrinsicResult.
//
// Worth using rather than solving: the D400 reports genuine per-unit values
// (measured 603.43 vs 607.01 px across the two units here) and reports ZERO
// distortion on the colour stream, because it comes out rectified. The main
// prize in a 40-view intrinsic sweep is the distortion model, and there is
// none to recover.
//
// The trade is that reprojection_rmse_px cannot be filled in -- there are no
// views to reproject. Quality is then judged on the registration view instead,
// which does exercise these intrinsics against real corners.
IntrinsicResult FactoryIntrinsics(const Eigen::Matrix3d& K, const cv::Size& image_size,
                                  const std::vector<double>& distortion = {0, 0, 0, 0, 0});

// Throws std::runtime_error if fewer than `min_views` usable observations are
// supplied. Intrinsics from too few views fit noise: the reported RMSE comes
// out low precisely because the solution is under-constrained, which is the
// most misleading possible failure.
IntrinsicResult CalibrateIntrinsics(const std::vector<BoardObservation>& observations,
                                    const BoardSpec& board, int min_views = 12);

struct PoseResult {
  Eigen::Isometry3d T_cam_board = Eigen::Isometry3d::Identity();
  double reprojection_rmse_px = 0.0;
  // Worst single corner in the same fit. An RMSE is an average, so one badly
  // localised corner out of 49 can push it over the acceptance gate while the
  // recovered pose is fine -- that failure cost a session's debugging on
  // 2026-09-20 before it was traced to marker corner refinement. Carrying the
  // maximum alongside the mean makes the difference between "this pose is bad"
  // and "one corner is bad" visible without re-running anything.
  double max_residual_px = 0.0;
  int corners_used = 0;
};

// Recovers the board's pose in the camera frame by PnP. Throws if the
// observation has too few corners to constrain 6-DoF.
PoseResult EstimateBoardPose(const BoardObservation& observation, const IntrinsicResult& intrinsics,
                             const BoardSpec& board);

// T_world_cam = T_world_board * (T_cam_board)^-1.
//
// The board's measured placement is what DEFINES the world frame, so an error
// in T_world_board is a rigid offset on every camera and every fused pose --
// and it is invisible to every downstream check, because everything downstream
// stays self-consistent. This is why config/calibration.yaml carries a [VERIFY]
// on that matrix.
Eigen::Isometry3d RegisterToWorld(const Eigen::Isometry3d& T_cam_board,
                                  const Eigen::Isometry3d& T_world_board);

// Reprojection RMSE of one observation under a known pose and intrinsics.
double ReprojectionRmse(const BoardObservation& observation, const IntrinsicResult& intrinsics,
                        const BoardSpec& board, const Eigen::Isometry3d& T_cam_board);

}  // namespace the_eyes::calibration

#endif  // THE_EYES_CALIBRATION_SOLVER_HPP
