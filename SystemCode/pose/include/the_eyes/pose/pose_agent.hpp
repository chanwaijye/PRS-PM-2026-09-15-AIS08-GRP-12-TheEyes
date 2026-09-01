// pose_agent orchestration: DetectionSet + Frame -> PoseEstimates.
//
// Consumes INTERFACES.md §2, produces §3. Owns the mesh-registry join, ROI
// derivation, the backend call, and latency accounting. Deliberately knows
// nothing about world frames or extrinsics -- see pose_estimator.hpp.
#ifndef THE_EYES_POSE_POSE_AGENT_HPP
#define THE_EYES_POSE_POSE_AGENT_HPP

#include <memory>
#include <string>
#include <vector>

#include "the_eyes/pose/mesh_registry.hpp"
#include "the_eyes/pose/pose_estimator.hpp"
#include "the_eyes/pose/roi.hpp"

namespace the_eyes::pose {

struct PoseAgentConfig {
  RoiConfig roi;
  // Estimates below this score are discarded rather than passed on. Fusion
  // weights by score but does not threshold, so a near-zero-confidence pose
  // still drags the fused estimate toward itself.
  double min_score = 0.10;
};

// Loads the `roi` and `estimator.min_score` sections of config/pose.yaml.
// Throws std::runtime_error on a missing file or an out-of-range value.
PoseAgentConfig LoadPoseAgentConfig(const std::string& yaml_path);

struct PoseAgentDiagnostics {
  int dropped_no_mesh = 0;
  int dropped_roi_too_small = 0;
  int dropped_low_score = 0;
  int declined_by_estimator = 0;
  std::vector<std::string> warnings;
};

class PoseAgent {
 public:
  PoseAgent(MeshRegistry registry, PoseAgentConfig config,
            std::shared_ptr<PoseEstimator> estimator);

  // Throws std::invalid_argument when `detections.camera_id` does not match
  // `frame.camera_id`, or when the frame fails its own validity check. Both are
  // wiring errors, and proceeding would attribute poses to the wrong camera --
  // which fusion cannot detect and which poisons every world-frame transform
  // downstream.
  std::vector<PoseEstimate> Process(const DetectionSet& detections, const Frame& frame);

  const PoseAgentDiagnostics& diagnostics() const { return diagnostics_; }
  const LatencyStats& latency() const { return latency_; }
  void ResetLatency() { latency_.Reset(); }
  const MeshRegistry& registry() const { return registry_; }
  std::string estimator_name() const { return estimator_->name(); }

 private:
  MeshRegistry registry_;
  PoseAgentConfig config_;
  std::shared_ptr<PoseEstimator> estimator_;
  PoseAgentDiagnostics diagnostics_;
  LatencyStats latency_;
};

}  // namespace the_eyes::pose

#endif  // THE_EYES_POSE_POSE_AGENT_HPP
