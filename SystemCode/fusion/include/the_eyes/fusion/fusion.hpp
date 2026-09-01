// Multi-view object-level pose fusion.
//
// PRIMARY TECHNICAL CONTRIBUTION. Written from first principles per CLAUDE.md
// constraint 2: no packaged fusion, no packaged pose averaging, no packaged EKF.
//
// Pipeline per frame:
//   per-view poses -> world frame -> cross-view association -> disagreement
//   gate -> confidence-weighted fuse -> track association -> per-object EKF
//   -> SceneState
//
// Consumes PoseEstimate (INTERFACES.md §3) and extrinsics (§4); produces
// SceneState (§5).
#ifndef THE_EYES_FUSION_FUSION_HPP
#define THE_EYES_FUSION_FUSION_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "the_eyes/fusion/config.hpp"
#include "the_eyes/fusion/ekf.hpp"
#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/types.hpp"

namespace the_eyes::fusion {

// Fallback world-axis-aligned half-extents by class, from the CAD mesh AABB.
// Consulted only when a PoseEstimate carries no extent of its own.
using ExtentRegistry = std::map<std::string, Eigen::Vector3d>;

// One object instance as seen from one or more views, after association and
// gating but before temporal filtering. Exposed so the ablation harness can run
// the same input through a subset of views.
struct FusedObservation {
  std::string cls;
  Eigen::Isometry3d pose_w = Eigen::Isometry3d::Identity();
  double confidence = 0.0;
  uint32_t visible_views = 0;
  Eigen::Vector3d extent = Eigen::Vector3d::Zero();
  bool symmetric = false;
  // Views dropped by the disagreement gate, as visible_views-style bits. Kept
  // separate: a rejected view saw the object, it just disagreed about where.
  uint32_t rejected_views = 0;
  // True when exactly two views disagreed past the gate and there was no
  // majority to break the tie, so the higher-scoring view was taken on faith.
  bool two_view_conflict = false;
};

struct FusionDiagnostics {
  int views_rejected = 0;
  int two_view_conflicts = 0;
  int tracks_created = 0;
  int tracks_retired = 0;
  std::vector<std::string> warnings;
};

// Association + gating + fusing, with no temporal state. Free function so the
// ablation harness can call it on a view subset without a Fusion instance.
std::vector<FusedObservation> FuseFrame(const std::vector<PoseEstimate>& estimates,
                                        const ExtrinsicsSet& extrinsics,
                                        const FusionConfig& cfg,
                                        const ExtentRegistry& extents,
                                        FusionDiagnostics* diagnostics);

class Fusion {
 public:
  // Throws std::runtime_error if any camera fails the calibration quality gate
  // (INTERFACES.md §4) -- bad extrinsics poison every fused pose, so fusion
  // refuses to start rather than emitting confidently wrong output.
  Fusion(FusionConfig cfg, ExtrinsicsSet extrinsics, ExtentRegistry extents = {});

  // Fuse one frame's worth of per-view estimates and advance the filters.
  // Objects not observed this frame are NOT emitted -- visible_views would be
  // 0, which INTERFACES.md §5 defines as invalid -- but their tracks are kept
  // for ekf.max_coast_s so a reappearing object keeps its id.
  SceneState Process(const std::vector<PoseEstimate>& estimates, uint64_t stamp_ns);

  const FusionDiagnostics& diagnostics() const { return diagnostics_; }
  size_t active_track_count() const { return tracks_.size(); }

 private:
  struct Track {
    std::string id;
    std::string cls;
    std::unique_ptr<ObjectEkf> ekf;
    uint64_t last_seen_ns = 0;
    Eigen::Vector3d extent = Eigen::Vector3d::Zero();
  };

  std::string AllocateId(const std::string& cls);

  FusionConfig cfg_;
  ExtrinsicsSet extrinsics_;
  ExtentRegistry extents_;
  std::vector<Track> tracks_;
  std::map<std::string, int> class_counters_;
  uint64_t last_stamp_ns_ = 0;
  FusionDiagnostics diagnostics_;
};

}  // namespace the_eyes::fusion

#endif  // THE_EYES_FUSION_FUSION_HPP
