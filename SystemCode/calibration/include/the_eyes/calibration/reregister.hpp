// Session re-registration: recover T_world_cam without a full recalibration.
//
// The rig this project runs on mounts the cameras on floor tripods and uses a
// shared computer desk as the table (docs/RIG_BOM.md §A). The desk gets typed
// on and leaned on, so the tabletop moves relative to the tripods between
// sessions. Because the world frame is glued to the tabletop, that invalidates
// every T_world_cam -- silently, because nothing downstream can tell a moved
// table from a correct one. Every fused pose simply becomes wrong by a fixed
// offset while staying perfectly self-consistent.
//
// A full recalibration recovers it but costs a 40-view intrinsic sweep. The
// observation this module rests on: only T_world_cam is affected.
//
//   intrinsics        rigid to the camera body   -- unchanged
//   T_camA_camB       rigid to the tripods       -- unchanged
//   T_world_cam       glued to the tabletop      -- CHANGED
//
// So one board view per camera, PnP, and re-derive. ~30 s instead of ~20 min.
//
// The load-bearing part is not that arithmetic -- `RegisterToWorld` already
// does it -- but the check below it. Re-registration is only valid if the
// cameras did not move RELATIVE TO EACH OTHER. If a tripod was bumped, the
// re-derived poses are garbage and would be written out looking healthy. That
// is a worse failure than the one this module exists to fix, because it
// converts an invisible error into an invisible error with a fresh timestamp
// on it.
//
// The test that separates the two cases:
//
//   T_camA_camB = T_world_camA^-1 * T_world_camB
//
// is INVARIANT to the world frame moving. Recompute it from the new poses and
// compare against the stored calibration:
//
//   unchanged  -> the table moved, the rig is intact, re-registration is valid
//   changed    -> a camera moved, re-registration is INVALID, recalibrate
//
// Everything here is pure geometry on Isometry3d, so it is exercisable with
// zero hardware and no images (AGENTS.md §2).
#ifndef THE_EYES_CALIBRATION_REREGISTER_HPP
#define THE_EYES_CALIBRATION_REREGISTER_HPP

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace the_eyes::calibration {

// A rigid-body difference, split into the two numbers a human can act on.
// Reporting a 4x4 residual to someone standing at a tripod helps nobody.
struct PoseDelta {
  double translation_m = 0.0;
  double rotation_deg = 0.0;
};

// The rigid-body difference between two poses: translation and rotation
// magnitude of a^-1 * b. Order-independent in magnitude.
PoseDelta ComputePoseDelta(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b);

struct RigidityThresholds {
  // How far the camera-to-camera transform may drift before the rig is
  // declared broken. Below the 5 mm round-trip gate with margin: a baseline
  // error of X mm surfaces as roughly X mm of round-trip disagreement, so a
  // baseline shift this test passed must not be able to fail that one.
  double max_baseline_shift_m = 0.003;
  // Chosen so it induces no more lateral error at the working distance than
  // max_baseline_shift_m does: atan(0.003 / 0.70) = 0.25 deg.
  double max_baseline_rotation_deg = 0.25;
  // Not a failure -- the table is EXPECTED to move, that is the whole point.
  // But a desk does not slide 20 mm on its own. A shift this large more likely
  // means the board was put back in the wrong place, which silently redefines
  // the world frame and is invisible to every check downstream.
  double warn_world_shift_m = 0.020;
};

RigidityThresholds LoadRigidityThresholds(const std::string& yaml_path);

// One camera's before/after.
struct CameraReRegistration {
  std::string camera_id;
  Eigen::Isometry3d previous_T_world_cam = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d current_T_world_cam = Eigen::Isometry3d::Identity();
  // How far this camera's pose moved in the world frame. With an intact rig
  // this is the tabletop's motion, and every camera reports the same story.
  PoseDelta world_delta;
};

// One camera pair's rigidity result. This is the check that matters.
struct BaselineCheck {
  std::string camera_a;
  std::string camera_b;
  PoseDelta delta;  // change in T_camA_camB, invariant to world-frame motion
  bool intact = false;
};

struct ReRegistrationReport {
  std::vector<CameraReRegistration> cameras;
  std::vector<BaselineCheck> baselines;  // every pair, not just adjacent ones
  RigidityThresholds thresholds;

  std::vector<std::string> failures;
  std::vector<std::string> warnings;

  bool passed() const { return failures.empty(); }
  std::string Format() const;
};

// Compares a fresh set of world registrations against the stored calibration.
//
// `camera_ids`, `previous` and `current` must describe the same cameras in the
// same order; throws std::runtime_error otherwise.
//
// Checks EVERY pair rather than a chain. With three cameras a chain would pass
// while a middle camera has rotated, because the two link errors can cancel.
//
// Fails on a single camera by design: with one camera there is no baseline, so
// the rigid-body assumption cannot be tested at all and a re-registration
// cannot be distinguished from a bumped tripod. Silently accepting that is the
// exact failure this module exists to prevent.
ReRegistrationReport CheckReRegistration(const std::vector<std::string>& camera_ids,
                                         const std::vector<Eigen::Isometry3d>& previous,
                                         const std::vector<Eigen::Isometry3d>& current,
                                         const RigidityThresholds& thresholds);

}  // namespace the_eyes::calibration

#endif  // THE_EYES_CALIBRATION_REREGISTER_HPP
