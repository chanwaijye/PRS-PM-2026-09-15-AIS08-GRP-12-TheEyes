// Fusion tuning, loaded from config/fusion.yaml.
//
// Every constant that shapes an estimate lives here rather than in the source.
// Loading is strict: a missing or out-of-range value throws, because a silently
// defaulted gate threshold produces plausible-looking wrong poses.
#ifndef THE_EYES_FUSION_CONFIG_HPP
#define THE_EYES_FUSION_CONFIG_HPP

#include <map>
#include <string>
#include <vector>

#include "the_eyes/types.hpp"

namespace the_eyes::fusion {

struct CalibrationGateConfig {
  double max_reprojection_rmse_px = 0.5;
};

struct AssociationConfig {
  // Must exceed DisagreementGateConfig::max_translation_m; LoadFusionConfig
  // enforces it. See config/fusion.yaml for why.
  double max_distance_m = 0.15;
};

struct DisagreementGateConfig {
  double max_translation_m = 0.05;
  double max_rotation_rad = 30.0 * 3.14159265358979323846 / 180.0;
  double two_view_conflict_confidence_penalty = 0.5;
};

struct EkfConfig {
  double process_noise_position_m = 0.01;
  double process_noise_velocity_mps = 0.10;
  double process_noise_orientation_rad = 5.0 * 3.14159265358979323846 / 180.0;
  double measurement_noise_position_m = 0.01;
  double measurement_noise_orientation_rad = 3.0 * 3.14159265358979323846 / 180.0;
  double initial_position_sigma_m = 0.10;
  double initial_velocity_sigma_mps = 0.50;
  double initial_orientation_sigma_rad = 45.0 * 3.14159265358979323846 / 180.0;
  double max_coast_s = 1.0;
};

struct TrackingConfig {
  double track_match_max_distance_m = 0.10;
};

// Which SPATIAL fusion policy to run -- how one frame's per-view estimates of
// the same object are combined. This is one of two independent switches; the
// temporal filter is the other (FusionConfig::temporal_filter), and the two
// must stay separate for the ablation to mean anything. See below.
//
// kNaive exists for one reason: the occlusion ablation's headline claim is
// about THIS module, and comparing "fused" only against single views cannot
// support that claim. Two cameras beat one camera by construction -- more
// viewpoints see more objects, and averaging two noisy measurements of the
// same quantity reduces error whoever writes the average. Neither result
// depends on the disagreement gate, the confidence weighting or the symmetry
// handling, which is what the policy actually contributes.
//
// kNaive is therefore the baseline a reader would reach for unprompted:
// associate across views, take the unweighted mean, believe every view. The
// gated-minus-naive delta, AT THE SAME temporal_filter SETTING, is the part
// attributable to the policy rather than to owning a second camera.
enum class FusionMode {
  kGated,  // the system: confidence-weighted, disagreement-gated, symmetry-aware
  kNaive,  // ablation baseline: unweighted mean, no gate, no symmetry case
};

struct FusionConfig {
  // Deliberately NOT read from fusion.yaml. The deployed system is always
  // kGated; kNaive is an evaluation harness setting, and a YAML key for it
  // would be a way to ship the baseline to the robot by accident.
  FusionMode mode = FusionMode::kGated;

  // Whether the per-object EKF's output is what gets emitted (true, the
  // deployed system) or the raw per-frame fuse is (false). Also not read from
  // fusion.yaml, for the same reason as `mode`.
  //
  // This is a separate switch from `mode` because of a confound found in the
  // 2026-09-05 review. The naive arm used to be defined as "unweighted mean AND
  // no filter", while the single-view and fused arms both ran the EKF. On a
  // static object with independent per-frame noise the EKF is a running
  // average over time, and it cuts error by far more than any one-frame policy
  // can -- so the whole fused-minus-naive gap, and the "a second camera makes
  // matters worse" reading, was the filter, not the policy. Making the filter
  // transparent gave naive 9.00 mm vs fused 9.01 mm. With the two switches
  // independent the harness can hold the filter fixed and vary the policy,
  // which is the only comparison that isolates the policy.
  bool temporal_filter = true;

  CalibrationGateConfig calibration;
  AssociationConfig association;
  DisagreementGateConfig disagreement_gate;
  EkfConfig ekf;
  TrackingConfig tracking;
  // Index i is bit i of ObjectState::visible_views. Must match predicates.yaml.
  std::vector<std::string> view_names = {"cam_front", "cam_over"};

  // Symmetry group by class (see symmetry.hpp). NOT read from fusion.yaml: it
  // is a property of the objects, declared once in meshes.yaml, and whoever
  // builds the config copies it in. A class that is absent has no declared
  // group, and an estimate flagged `symmetric` then gets the older, weaker
  // handling -- one view's orientation taken, never combined, gated or made
  // continuous for the filter.
  std::map<std::string, SymmetryGroup> symmetry;
};

FusionConfig LoadFusionConfig(const std::string& yaml_path);

}  // namespace the_eyes::fusion

#endif  // THE_EYES_FUSION_CONFIG_HPP
