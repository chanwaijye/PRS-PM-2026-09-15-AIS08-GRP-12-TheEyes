// Everything one processed FramePair produced, as a value. The pipeline fills
// it, the metrics log persists it, the console and viewer render it. Keeping
// it a plain struct is what lets logging move off the main thread later
// without touching the pipeline.
#ifndef THE_EYES_RUNTIME_PIPELINE_RESULT_HPP
#define THE_EYES_RUNTIME_PIPELINE_RESULT_HPP

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "the_eyes/detection.hpp"
#include "the_eyes/fusion/fusion.hpp"
#include "the_eyes/pose/pose_agent.hpp"
#include "the_eyes/scenegraph/pddl.hpp"
#include "the_eyes/types.hpp"

namespace the_eyes::runtime {

struct StageTimings {
  std::map<std::string, double> detect_ms;   // per camera
  std::map<std::string, double> pose_ms;     // per camera
  double fusion_ms = 0.0;
  double scenegraph_ms = 0.0;
  double total_ms = 0.0;
};

struct PipelineResult {
  uint64_t pair_index = 0;
  uint64_t stamp_ns = 0;
  double skew_ms = 0.0;
  uint64_t frames_dropped_since_last = 0;
  // camera_id -> (stamp_ns, frame_number) of the frame actually processed
  std::map<std::string, std::pair<uint64_t, uint64_t>> view_stamps;
  // camera_id -> gain the brightness normalisation applied to this frame
  // (1.0 when it is off, or when the frame was already bright enough)
  std::map<std::string, double> brightness_gain;
  std::map<std::string, DetectionSet> detections;
  std::map<std::string, std::vector<PoseEstimate>> estimates;
  SceneState scene;                 // after static injection and Annotate
  std::vector<std::string> static_ids;   // ids in `scene` that came from scene.yaml
  // Whether the scene graph's support check ran on this pair, and the ids in
  // `scene` it left out of the facts because they rest on nothing. They stay in
  // `scene`: that is what fusion tracked, and the difference is the finding.
  bool support_checked = false;
  std::vector<std::string> unsupported_ids;
  scenegraph::PddlResult pddl;
  StageTimings timings;
  fusion::FusionDiagnostics fusion_diag;
  std::map<std::string, pose::PoseAgentDiagnostics> pose_diag;
  std::vector<std::pair<std::string, std::string>> warnings;   // (stage, message)
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_PIPELINE_RESULT_HPP
