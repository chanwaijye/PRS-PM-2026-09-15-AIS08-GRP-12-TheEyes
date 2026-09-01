// Fusion tuning, loaded from config/fusion.yaml.
//
// Every constant that shapes an estimate lives here rather than in the source.
// Loading is strict: a missing or out-of-range value throws, because a silently
// defaulted gate threshold produces plausible-looking wrong poses.
#ifndef THE_EYES_FUSION_CONFIG_HPP
#define THE_EYES_FUSION_CONFIG_HPP

#include <string>
#include <vector>

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

struct FusionConfig {
  CalibrationGateConfig calibration;
  AssociationConfig association;
  DisagreementGateConfig disagreement_gate;
  EkfConfig ekf;
  TrackingConfig tracking;
  // Index i is bit i of ObjectState::visible_views. Must match predicates.yaml.
  std::vector<std::string> view_names = {"cam_front", "cam_over"};
};

FusionConfig LoadFusionConfig(const std::string& yaml_path);

}  // namespace the_eyes::fusion

#endif  // THE_EYES_FUSION_CONFIG_HPP
