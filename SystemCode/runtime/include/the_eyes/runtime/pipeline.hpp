// The chain, one FramePair at a time:
//
//   FramePair -> detector (per view) -> pose_agent (per view) -> fusion
//     -> static scene injection -> scenegraph -> PDDL
//
// Same modules as integration/pipeline_demo, but fed by a FrameSource and
// producing a PipelineResult per pair with every stage timed.
#ifndef THE_EYES_RUNTIME_PIPELINE_HPP
#define THE_EYES_RUNTIME_PIPELINE_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "the_eyes/fusion/fusion.hpp"
#include "the_eyes/pose/pose_agent.hpp"
#include "the_eyes/runtime/backends.hpp"
#include "the_eyes/runtime/brightness.hpp"
#include "the_eyes/runtime/depth_correction.hpp"
#include "the_eyes/runtime/frame_pair.hpp"
#include "the_eyes/runtime/pipeline_result.hpp"
#include "the_eyes/runtime/static_scene.hpp"
#include "the_eyes/runtime/view_evidence.hpp"
#include "the_eyes/scenegraph/predicates.hpp"

namespace the_eyes::runtime {

// Fallback extents by class, from the mesh registry.
fusion::ExtentRegistry ExtentsFrom(const pose::MeshRegistry& registry);

// Declared symmetry group by class, from the mesh registry, in the form
// FusionConfig::symmetry takes. Classes with no declared group are left out.
std::map<std::string, SymmetryGroup> SymmetriesFrom(const pose::MeshRegistry& registry);

class Pipeline {
 public:
  Pipeline(DetectorHandle detector, PoseHandle pose,
           std::map<std::string, std::unique_ptr<pose::PoseAgent>> agents,
           std::unique_ptr<fusion::Fusion> fusion, scenegraph::PredicateConfig predicates,
           StaticScene statics, std::vector<std::string> view_names);

  PipelineResult Process(const FramePair& pair);

  // Per-camera depth scale, applied to the depth unit of every frame handed to
  // the detector and the pose estimator. Identity until set.
  void set_depth_correction(DepthCorrection c) { depth_correction_ = std::move(c); }
  const DepthCorrection& depth_correction() const { return depth_correction_; }

  // Brightness normalisation of the colour image handed to the detector, and
  // to the pose estimator when apply_to_pose is set. Off until set.
  void set_brightness(BrightnessConfig c) { brightness_ = c; }
  const BrightnessConfig& brightness() const { return brightness_; }

  // Depth evidence for every per-view estimate, measured after the pose stage
  // and before fusion (view_evidence.hpp). Off until set: estimates then carry
  // "not supplied" and fusion's gate falls back to the estimator's score.
  void set_view_evidence(ViewEvidence e) { view_evidence_ = std::move(e); }
  const ViewEvidence& view_evidence() const { return view_evidence_; }

  // Cumulative per-stage latency: "detect:<cam>", "pose:<cam>", "evidence", "fusion",
  // "scenegraph", "total".
  const std::map<std::string, pose::LatencyStats>& latency() const { return latency_; }
  const DetectorHandle& detector() const { return detector_; }
  const PoseHandle& pose() const { return pose_; }
  const std::vector<std::string>& view_names() const { return view_names_; }
  std::string Summary() const;

 private:
  DetectorHandle detector_;
  PoseHandle pose_;
  std::map<std::string, std::unique_ptr<pose::PoseAgent>> agents_;
  std::unique_ptr<fusion::Fusion> fusion_;
  scenegraph::PredicateConfig predicates_;
  StaticScene statics_;
  std::vector<std::string> view_names_;
  std::map<std::string, pose::LatencyStats> latency_;
  size_t detector_warnings_seen_ = 0;
  DepthCorrection depth_correction_;
  BrightnessConfig brightness_;
  ViewEvidence view_evidence_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_PIPELINE_HPP
