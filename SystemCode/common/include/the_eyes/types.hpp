// Shared data contracts. See INTERFACES.md — this header IS that contract
// in code form. Fields may be added, never removed or re-typed: ObjectState has
// consumers outside this repo (PM1 planner, PM4 world model).
//
// Deliberately Eigen-only. The detector-side contract (§2) needs OpenCV and
// lives in detection.hpp so that fusion and scenegraph never link cv.
#ifndef THE_EYES_TYPES_HPP
#define THE_EYES_TYPES_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace the_eyes {

// Bit positions within ObjectState::visible_views. The bit order is part of the
// contract; a camera's namespace maps to a bit here, and to a serial number in
// the launch file.
enum ViewBit : uint32_t {
  kViewCamFront = 1u << 0,
  kViewCamOver  = 1u << 1,
};

// INTERFACES.md §3 — Pose Estimator → Fusion.
// Pose is in THIS camera's optical frame; the estimator knows no world frame.
struct PoseEstimate {
  std::string camera_id;              // "cam_front" | "cam_over"
  std::string cls;
  uint32_t track_id = 0;
  uint64_t stamp_ns = 0;
  Eigen::Isometry3d pose_cam = Eigen::Isometry3d::Identity();
  double score = 0.0;                 // FoundationPose hypothesis score [0,1]
  bool symmetric = false;             // fusion must not average orientation
                                      // naively when this is set
  // World-axis-aligned half-extents of the object's bounding volume. ADDITIVE
  // extension to INTERFACES.md §3: fusion must fill ObjectState::extent and
  // nothing upstream supplied it. For a known class this is the CAD mesh AABB;
  // for the DBSCAN path it is the cluster bounding volume, which exists only
  // per-estimate and so cannot come from a class-keyed registry.
  // Zero means "not supplied" -- fusion then falls back to its extent registry.
  Eigen::Vector3d extent = Eigen::Vector3d::Zero();
};

// INTERFACES.md §4 — one entry per camera, loaded from config/extrinsics.yaml.
struct CameraExtrinsics {
  std::string camera_id;
  std::string serial;                 // cameras are selected by serial, never
                                      // by /dev/video* or enumeration order
  Eigen::Isometry3d T_world_cam = Eigen::Isometry3d::Identity();
  double reprojection_rmse_px = 0.0;  // calibration quality gate; fusion refuses
                                      // to start if this exceeds threshold
};

// INTERFACES.md §5 — THE stable external contract. PM1 and PM4 depend on it.
struct ObjectState {
  std::string id;                     // stable instance id, e.g. "red_cube_01"
  std::string cls;                    // YOLO class, or "unknown_object"
  Eigen::Isometry3d pose_w = Eigen::Isometry3d::Identity();
  double confidence = 0.0;
  std::vector<std::string> relations; // filled by scenegraph; empty from fusion
  uint32_t visible_views = 0;         // ViewBit mask; 0 is invalid, drop it
  Eigen::Vector3d extent = Eigen::Vector3d::Zero();  // half-extents, WORLD-AXIS-
                                      // ALIGNED, so pose_w rotation does not
                                      // enter bounding-volume geometry
  uint64_t stamp_ns = 0;

  // --- Belief, not just an estimate. ADDITIVE extension to INTERFACES.md §5 --
  //
  // `confidence` above is a detector/estimator score. It says how sure the
  // FRONTEND was that it saw the thing; it says nothing about how well the
  // pose is pinned down in metres. Those are different questions and only the
  // second one tells a consumer whether a geometric test over this pose is
  // safe to trust. The per-object EKF already computes the second one every
  // frame -- these fields stop it being discarded.
  //
  // NEGATIVE means "not supplied", because zero is a legitimate-looking value
  // that would read as perfect certainty. A consumer must check before use;
  // has_uncertainty() below is the check.
  double position_sigma_m = -1.0;       // 1-sigma RMS position uncertainty, m
  double orientation_sigma_rad = -1.0;  // 1-sigma RMS orientation uncertainty

  // World-frame linear velocity from the filter's constant-velocity state.
  // Zero for a static object, which is the common tabletop case -- so unlike
  // the sigmas, zero is a meaningful value here and needs no sentinel.
  Eigen::Vector3d velocity_w = Eigen::Vector3d::Zero();
};

struct SceneState {
  uint64_t stamp_ns = 0;
  std::vector<ObjectState> objects;
};

// INTERFACES.md §7 — mesh registry entry. diameter_m is required: the ADD-S
// threshold is a fraction of object diameter, so evaluation cannot run without.
struct MeshEntry {
  std::string cls;
  std::string mesh_path;              // must be watertight
  bool symmetric = false;
  double diameter_m = 0.0;
  // World-axis-aligned half-extents of the mesh AABB. ADDITIVE extension to
  // INTERFACES.md §7, for the same reason PoseEstimate gained one: fusion must
  // fill ObjectState::extent and this is where a known class's value comes
  // from. Derived from the mesh; carried in config so no runtime OBJ parsing
  // is needed on the critical path.
  Eigen::Vector3d extent = Eigen::Vector3d::Zero();
};

// True when the object carries no view at all. Such objects must be dropped
// rather than emitted (INTERFACES.md §5).
inline bool is_invalid(const ObjectState& o) { return o.visible_views == 0; }

inline bool visible_from(const ObjectState& o, uint32_t view_bit) {
  return (o.visible_views & view_bit) != 0u;
}

// True when the producer filled in the uncertainty fields. A consumer that
// reasons about pose uncertainty MUST gate on this: a producer predating the
// field, or one running without a temporal filter, supplies neither sigma, and
// treating the sentinel as a real number would read as perfect certainty.
inline bool has_uncertainty(const ObjectState& o) {
  return o.position_sigma_m >= 0.0 && o.orientation_sigma_rad >= 0.0;
}

}  // namespace the_eyes

#endif  // THE_EYES_TYPES_HPP
