// Pose-error metrics and the occlusion ablation harness.
//
// CLAUDE.md: "The fusion occlusion ablation is never cut. It is the entire
// technical claim." This harness is therefore a deliverable, not a debugging
// tool -- it runs one identical input four ways and emits comparable ADD-S,
// which is the project's headline result.
//
// Three things vary between arms, and the harness varies them one at a time:
//
//   views    one camera or both        -> what a second viewpoint buys
//   policy   naive mean or gated fuse  -> what THIS MODULE's per-frame policy buys
//   filter   raw or EKF-smoothed       -> what temporal averaging buys
//
// The eight arms, in table order:
//
//   cam_front       one view,   raw          |  the per-view floor
//   cam_over        one view,   raw          |
//   naive           both views, naive, raw   |  BASELINE: association +
//                                            |  unweighted mean, no gate
//   gated           both views, gated, raw   |  the policy, no filter
//   cam_front+ekf   one view,   EKF          |
//   cam_over+ekf    one view,   EKF          |
//   naive+ekf       both views, naive, EKF   |  baseline with the filter
//   fused           both views, gated, EKF   |  the deployed system
//
// With depth evidence in use (FusionConfig::view_evidence, 2026-10-07) each
// tier has a fifth row between `naive` and the policy:
//
//   gate-score      both views, gated, raw   |  the gate WITHOUT the evidence:
//   gate-score+ekf  both views, gated, EKF   |  a conflict settled by score
//
// so `gated - gate-score` is what the evidence buys, on its own.
//
// Read it as pairs that differ in exactly one thing. `gated - naive` and
// `fused - naive+ekf` are the policy's contribution, at each filter setting.
// `naive - cam_front` is the second camera's. `cam_front+ekf - cam_front` and
// `naive+ekf - naive` are the filter's.
//
// The naive arm is what makes the claim falsifiable. Beating a single view is
// not evidence that this module works -- two cameras see more than one, and
// averaging two noisy measurements reduces error however the average is
// written. Both of those hold for code nobody would call a contribution.
//
// The filter axis exists because of a confound found in the 2026-09-05 review:
// the naive arm was defined as "no filter" while the single-view and fused arms
// ran the EKF, so the reported fused-minus-naive gap was entirely the EKF's
// temporal averaging and the policy's true contribution on that input was zero
// (naive 9.00 mm vs fused 9.01 mm with the filter made transparent). A
// comparison across arms that differ in two things attributes the sum of both
// to whichever one the author had in mind. If the policy delta comes out at
// zero on real captures, that is a real finding about the operating regime and
// is reported as one.
#ifndef THE_EYES_FUSION_METRICS_HPP
#define THE_EYES_FUSION_METRICS_HPP

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "the_eyes/fusion/fusion.hpp"
#include "the_eyes/types.hpp"

namespace the_eyes::fusion {

// Points sampled from the object's CAD mesh, in the object's own frame.
using ModelPoints = std::vector<Eigen::Vector3d>;

struct ModelEntry {
  ModelPoints points;
  double diameter_m = 0.0;  // INTERFACES.md §7; ADD-S is normalised by this
};
using ModelRegistry = std::map<std::string, ModelEntry>;

// ADD: mean distance between corresponding model points under the two poses.
double Add(const Eigen::Isometry3d& estimated, const Eigen::Isometry3d& ground_truth,
           const ModelPoints& model);

// ADD-S: mean distance to the CLOSEST model point rather than the corresponding
// one. This is the symmetry-aware variant -- for an object with a symmetry, a
// pose off by a symmetry element is visually identical and ADD would punish it
// while ADD-S correctly does not. O(m^2) in the model point count.
double AddS(const Eigen::Isometry3d& estimated, const Eigen::Isometry3d& ground_truth,
            const ModelPoints& model);

// Corner + face-centre + edge-midpoint sampling of a box. A stand-in for CAD
// mesh sampling so the harness is runnable on synthetic scenes today; real
// evaluation samples the registered mesh instead.
ModelPoints BoxModelPoints(const Eigen::Vector3d& half_extents, int samples_per_axis = 5);

double BoxDiameter(const Eigen::Vector3d& half_extents);

struct GroundTruthObject {
  std::string cls;
  Eigen::Isometry3d pose_w = Eigen::Isometry3d::Identity();
};

struct AblationFrame {
  uint64_t stamp_ns = 0;
  std::vector<PoseEstimate> estimates;      // all views; the harness filters
  std::vector<GroundTruthObject> ground_truth;
};

// One arm's numbers for one class. The pooled row hides which object a
// multi-object scene fails on: on t2_clutter_c (2026-10-07) "66.8% inside" was
// two objects at 100% and the cube at 0%.
struct AblationClassStats {
  int ground_truth_objects = 0;
  int matched = 0;
  int missed = 0;
  double mean_add_s_m = 0.0;            // over matched instances only
  double recall_at_0p1_diameter = 0.0;  // over ALL instances, misses included
};

struct AblationRow {
  std::string label;                  // see the arm table at the top of this file
  std::vector<std::string> views;     // which cameras' estimates this arm saw
  FusionMode mode = FusionMode::kGated;
  bool temporal_filter = false;
  // True for the `gate-score` rows: the gated policy with the depth evidence
  // withheld, in a table whose configuration otherwise uses it.
  bool by_score = false;
  int ground_truth_objects = 0;       // total GT instances across all frames
  int matched = 0;                    // GT instances an estimate was found for
  int missed = 0;                     // GT instances with no estimate
  double mean_add_s_m = 0.0;          // over matched instances only
  double mean_add_s_fraction = 0.0;   // ADD-S / object diameter
  // Fraction of ALL GT instances (not just matched) with ADD-S below 10% of
  // diameter. Counting misses in the denominator keeps a configuration that
  // simply fails to see the object from scoring well on the few it does see.
  double recall_at_0p1_diameter = 0.0;
  // The same, per class. One entry for a single-object scene.
  std::map<std::string, AblationClassStats> by_class;
};

struct AblationResult {
  std::vector<AblationRow> rows;
  // Human-readable table for the report. Numbers come from the run, never from
  // a default (CLAUDE.md constraint 5).
  std::string Format() const;
};

// Runs `frames` once per configuration with a fresh Fusion instance each time,
// so no temporal state leaks between configurations. Eight rows, or ten when
// the configuration uses depth evidence: see above.
AblationResult RunOcclusionAblation(const std::vector<AblationFrame>& frames,
                                    const ExtrinsicsSet& extrinsics, const FusionConfig& cfg,
                                    const ExtentRegistry& extents, const ModelRegistry& models,
                                    double match_max_distance_m = 0.15);

}  // namespace the_eyes::fusion

#endif  // THE_EYES_FUSION_METRICS_HPP
