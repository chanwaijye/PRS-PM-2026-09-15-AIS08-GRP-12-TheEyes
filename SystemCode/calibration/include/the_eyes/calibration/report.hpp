// Calibration quality report and the extrinsics.yaml writer.
//
// AGENTS.md §2 makes the report a deliverable: the reprojection RMSE it
// carries becomes the gate fusion refuses to start above (INTERFACES.md §4).
#ifndef THE_EYES_CALIBRATION_REPORT_HPP
#define THE_EYES_CALIBRATION_REPORT_HPP

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "the_eyes/calibration/board.hpp"
#include "the_eyes/calibration/observation.hpp"
#include "the_eyes/calibration/solver.hpp"

namespace the_eyes::calibration {

struct QualityThresholds {
  double max_reprojection_rmse_px = 0.5;
  double max_round_trip_error_m = 0.005;
  int min_views_for_intrinsics = 12;
  int min_shared_corners = 12;
};

QualityThresholds LoadQualityThresholds(const std::string& yaml_path);

// Loaded from calibration.yaml. The board's measured placement is what defines
// the world frame.
struct WorldConfig {
  std::string frame = "table_origin";
  Eigen::Isometry3d T_world_board = Eigen::Isometry3d::Identity();
};

WorldConfig LoadWorldConfig(const std::string& yaml_path);

// One camera's calibration outcome.
struct CameraCalibration {
  std::string camera_id;
  std::string serial;              // cameras are addressed by serial, never index
  IntrinsicResult intrinsics;
  Eigen::Isometry3d T_world_cam = Eigen::Isometry3d::Identity();
  double registration_rmse_px = 0.0;  // RMSE of the world-registration view
};

// The acceptance criterion that actually says whether the two views agree: a
// point seen by both must land in the same world position from either.
//
// Measured as the closest approach between the two cameras' world-frame
// sightlines to the same corner -- how far apart the rays pass. Zero means they
// intersect, i.e. the two views agree exactly about where that corner is.
struct RoundTripResult {
  int shared_corners = 0;
  double mean_error_m = 0.0;
  double max_error_m = 0.0;
  double p95_error_m = 0.0;
};

// Takes the DETECTED image points, not the board model.
//
// That is the fix for a defect found 2026-08-28: comparing model points pushed
// through each camera's own extrinsics is degenerate, because T_world_cam is
// itself derived from that camera's PnP of the same observation, so both sides
// reduce to T_world_board * board_point and the error is identically zero. The
// board spec is not a parameter any more precisely because using it was the
// bug.
RoundTripResult ComputeRoundTrip(const BoardObservation& observation_a,
                                 const CameraCalibration& camera_a,
                                 const BoardObservation& observation_b,
                                 const CameraCalibration& camera_b);

struct CalibrationReport {
  std::string world_frame;
  std::vector<CameraCalibration> cameras;
  RoundTripResult round_trip;
  QualityThresholds thresholds;

  std::vector<std::string> failures;  // empty means the calibration passed
  bool passed() const { return failures.empty(); }

  std::string Format() const;
};

// Fills `failures` with every criterion that did not hold. Every one is listed
// rather than stopping at the first: knowing both cameras failed, versus one, is
// the difference between remounting the rig and reshooting one capture.
void Evaluate(CalibrationReport* report);

// Writes config/extrinsics.yaml in the INTERFACES.md §4 format.
//
// Refuses to write a failing calibration unless `force` is set. Fusion's gate
// reads reprojection_rmse_px from this file and would catch it -- but a file
// that exists is assumed good by everything else, and a bad one silently
// poisons every fused pose.
void WriteExtrinsicsYaml(const CalibrationReport& report, const std::string& out_path,
                         bool force = false);

}  // namespace the_eyes::calibration

#endif  // THE_EYES_CALIBRATION_REPORT_HPP
