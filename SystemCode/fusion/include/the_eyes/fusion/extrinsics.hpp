// Extrinsics loading and the calibration quality gate.
//
// INTERFACES.md §4. The gate is the reason this is its own translation unit:
// fusion must refuse to start on bad extrinsics rather than silently emitting
// poses that are wrong by a fixed offset in every frame.
#ifndef THE_EYES_FUSION_EXTRINSICS_HPP
#define THE_EYES_FUSION_EXTRINSICS_HPP

#include <map>
#include <string>

#include "the_eyes/types.hpp"

namespace the_eyes::fusion {

struct ExtrinsicsSet {
  std::string world_frame;
  // Keyed by camera_id, matching PoseEstimate::camera_id.
  std::map<std::string, CameraExtrinsics> cameras;

  // Throws std::out_of_range if the camera is unknown -- a pose from an
  // uncalibrated camera must never be silently dropped or silently trusted.
  const CameraExtrinsics& at(const std::string& camera_id) const;

  // Maps camera_id to its ObjectState::visible_views bit, using the order in
  // view_names. Returns 0 for an unknown camera.
  static uint32_t ViewBitFor(const std::string& camera_id,
                             const std::vector<std::string>& view_names);
};

// Throws std::runtime_error on a missing file, a malformed matrix, or a
// T_world_cam whose rotation block is not orthonormal.
ExtrinsicsSet LoadExtrinsics(const std::string& yaml_path);

// Throws std::runtime_error naming every camera that fails the gate.
void AssertCalibrationQuality(const ExtrinsicsSet& ex, double max_reprojection_rmse_px);

// Transform a per-view pose into the world frame.
Eigen::Isometry3d ToWorld(const PoseEstimate& est, const CameraExtrinsics& cam);

}  // namespace the_eyes::fusion

#endif  // THE_EYES_FUSION_EXTRINSICS_HPP
