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

  // --- What the depth images say about this pose. ADDITIVE (2026-10-07) ------
  //
  // `score` is the estimator's own opinion, and on real data it does not rank
  // the views: on the half-hidden bottle of 2026-10-06 the view that put the
  // object 125 mm from the truth scored 0.94 and the view that had it to 3 mm
  // scored 0.84. These two are measurements instead -- the object rendered at
  // the estimated pose, compared with what a depth camera saw there.
  //
  // depth_support  share of the pose's silhouette in ITS OWN view where the
  //                measured depth agrees with the pose.
  // seen_through   the largest share, over the OTHER views, of the pose's
  //                silhouette where that view measures a surface BEYOND the
  //                object: another camera looking through the place the object
  //                is claimed to occupy. A surface in front of it is occlusion
  //                and counts for nothing either way.
  //
  // Both in [0, 1]. NEGATIVE means "not supplied", as with ObjectState's
  // sigmas, and fusion then falls back to `score`. See has_view_evidence().
  double depth_support = -1.0;
  double seen_through = -1.0;
};

// Whether both depth-evidence fields were supplied. A producer that has no
// second view to consult leaves seen_through negative, and an estimate with
// only half the evidence is treated as having none.
inline bool has_view_evidence(const PoseEstimate& e) {
  return e.depth_support >= 0.0 && e.seen_through >= 0.0;
}

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

  // --- Which views saw it. ADDITIVE extension to INTERFACES.md §5, 2026-10-11 -
  //
  // `visible_views` above is the views whose estimate went INTO pose_w. A view
  // the disagreement gate dropped is not in it, though that view saw the
  // object and estimated it: it was outvoted about where the object is, not
  // blind to it. Reading a missing `visible_views` bit as "hidden from that
  // camera" reported an unoccluded bottle as occluded on 300 of 300 pairs in
  // four scored scenes (docs/OCTOBER_PLAN.md, 10 Oct).
  //
  // This is every view with an estimate associated to this object in this
  // frame, fused or gated out: a superset of `visible_views`. An estimate
  // refused by the depth evidence is NOT in it -- another camera sees through
  // the place it claims, so it is not an estimate of this object.
  //
  // ZERO means "not supplied" (a valid object has at least its fused view
  // here). Use observed_from() below, which then falls back to
  // `visible_views`, the only answer there was before this field.
  uint32_t observed_views = 0;
};

struct SceneState {
  uint64_t stamp_ns = 0;
  std::vector<ObjectState> objects;
};

// INTERFACES.md §7 — mesh registry entry. diameter_m is required: the ADD-S
// threshold is a fraction of object diameter, so evaluation cannot run without.
// Which rotations of an object, about its own origin, leave it looking the
// same. `symmetric` alone says only THAT two views may legitimately report
// different orientations; fusion needs to know WHICH orientations are the same
// before it can average them, gate on them, or filter them over time.
// ADDITIVE extension to INTERFACES.md §7, 2026-10-04.
enum class SymmetryGroup {
  kNone,    // every orientation is distinct, or the group is not declared
  kCubic,   // the 24 rotations of a cube about its centre
  kAxialZ,  // any rotation about the object's own z axis
};

struct MeshEntry {
  std::string cls;
  std::string mesh_path;              // must be watertight
  bool symmetric = false;
  // kNone on a `symmetric: true` entry means "symmetric in some undeclared
  // way", which is what every entry was before this field existed; fusion then
  // falls back to taking one view's orientation rather than combining them.
  SymmetryGroup symmetry_group = SymmetryGroup::kNone;
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

// True when `view_bit`'s camera had an estimate of this object in this frame,
// whether or not fusion used it. This, not visible_from(), is what "the camera
// can see it" means. Falls back to visible_views when the producer predates
// `observed_views`.
inline bool observed_from(const ObjectState& o, uint32_t view_bit) {
  const uint32_t views = (o.observed_views != 0u) ? o.observed_views : o.visible_views;
  return (views & view_bit) != 0u;
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
