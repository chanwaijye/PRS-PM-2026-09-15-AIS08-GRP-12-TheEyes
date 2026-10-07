#include "the_eyes/fusion/config.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::fusion {
namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

const YAML::Node RequireSection(const YAML::Node& root, const std::string& key) {
  const YAML::Node node = root[key];
  if (!node || !node.IsMap()) {
    throw std::runtime_error("fusion.yaml: missing section '" + key + "'");
  }
  return node;
}

double RequireDouble(const YAML::Node& parent, const std::string& section,
                     const std::string& key, double lo, double hi) {
  const YAML::Node node = parent[key];
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("fusion.yaml: missing '" + section + "." + key + "'");
  }
  const double v = node.as<double>();
  if (!std::isfinite(v) || v < lo || v > hi) {
    std::ostringstream os;
    os << "fusion.yaml: '" << section << "." << key << "' = " << v << " outside [" << lo
       << ", " << hi << "]";
    throw std::runtime_error(os.str());
  }
  return v;
}

}  // namespace

FusionConfig LoadFusionConfig(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("fusion.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }

  FusionConfig cfg;

  const YAML::Node calib = RequireSection(root, "calibration");
  cfg.calibration.max_reprojection_rmse_px =
      RequireDouble(calib, "calibration", "max_reprojection_rmse_px", 0.0, 100.0);

  const YAML::Node assoc = RequireSection(root, "association");
  cfg.association.max_distance_m = RequireDouble(assoc, "association", "max_distance_m", 0.0, 10.0);

  const YAML::Node gate = RequireSection(root, "disagreement_gate");
  cfg.disagreement_gate.max_translation_m =
      RequireDouble(gate, "disagreement_gate", "max_translation_m", 0.0, 10.0);
  cfg.disagreement_gate.max_rotation_rad =
      RequireDouble(gate, "disagreement_gate", "max_rotation_deg", 0.0, 180.0) * kDegToRad;
  cfg.disagreement_gate.two_view_conflict_confidence_penalty = RequireDouble(
      gate, "disagreement_gate", "two_view_conflict_confidence_penalty", 0.0, 1.0);

  // Optional: a fusion.yaml saved with a run from before the rule existed still
  // loads and still means what it meant. Strict when present.
  if (root["view_evidence"]) {
    const YAML::Node ev = RequireSection(root, "view_evidence");
    const YAML::Node enabled = ev["enabled"];
    if (!enabled || !enabled.IsScalar()) {
      throw std::runtime_error("fusion.yaml: missing 'view_evidence.enabled'");
    }
    try {
      cfg.view_evidence.enabled = enabled.as<bool>();
    } catch (const YAML::Exception&) {
      throw std::runtime_error("fusion.yaml: 'view_evidence.enabled' must be true or false");
    }
    cfg.view_evidence.max_seen_through =
        RequireDouble(ev, "view_evidence", "max_seen_through", 0.01, 1.0);
    cfg.view_evidence.min_gap = RequireDouble(ev, "view_evidence", "min_gap", 0.0, 1.0);
    cfg.view_evidence.agree_tol_m = RequireDouble(ev, "view_evidence", "agree_tol_m", 1e-4, 1.0);
    cfg.view_evidence.through_tol_m =
        RequireDouble(ev, "view_evidence", "through_tol_m", 1e-4, 1.0);
  }

  const YAML::Node ekf = RequireSection(root, "ekf");
  cfg.ekf.process_noise_position_m =
      RequireDouble(ekf, "ekf", "process_noise_position_m", 0.0, 10.0);
  cfg.ekf.process_noise_velocity_mps =
      RequireDouble(ekf, "ekf", "process_noise_velocity_mps", 0.0, 100.0);
  cfg.ekf.process_noise_orientation_rad =
      RequireDouble(ekf, "ekf", "process_noise_orientation_deg", 0.0, 180.0) * kDegToRad;
  cfg.ekf.measurement_noise_position_m =
      RequireDouble(ekf, "ekf", "measurement_noise_position_m", 1e-6, 10.0);
  cfg.ekf.measurement_noise_orientation_rad =
      RequireDouble(ekf, "ekf", "measurement_noise_orientation_deg", 1e-4, 180.0) * kDegToRad;
  cfg.ekf.initial_position_sigma_m =
      RequireDouble(ekf, "ekf", "initial_position_sigma_m", 1e-6, 100.0);
  cfg.ekf.initial_velocity_sigma_mps =
      RequireDouble(ekf, "ekf", "initial_velocity_sigma_mps", 1e-6, 100.0);
  cfg.ekf.initial_orientation_sigma_rad =
      RequireDouble(ekf, "ekf", "initial_orientation_sigma_deg", 1e-4, 180.0) * kDegToRad;
  cfg.ekf.max_coast_s = RequireDouble(ekf, "ekf", "max_coast_s", 0.0, 3600.0);

  const YAML::Node tracking = RequireSection(root, "tracking");
  cfg.tracking.track_match_max_distance_m =
      RequireDouble(tracking, "tracking", "track_match_max_distance_m", 0.0, 10.0);

  // Cross-field invariant. A gate wider than the association radius is silently
  // inert: the disagreeing views it exists to judge have already been split into
  // separate object instances by association, so the gate never sees a cluster
  // to reject anything from. Fail loudly rather than ship a dead safety check.
  if (cfg.disagreement_gate.max_translation_m >= cfg.association.max_distance_m) {
    std::ostringstream os;
    os << "fusion.yaml: disagreement_gate.max_translation_m ("
       << cfg.disagreement_gate.max_translation_m
       << " m) must be strictly less than association.max_distance_m ("
       << cfg.association.max_distance_m
       << " m), otherwise the gate can never fire -- outlier views become "
          "phantom single-view objects instead of being rejected.";
    throw std::runtime_error(os.str());
  }

  const YAML::Node views = RequireSection(root, "views");
  const YAML::Node names = views["names"];
  if (!names || !names.IsSequence() || names.size() == 0) {
    throw std::runtime_error("fusion.yaml: 'views.names' must be a non-empty sequence");
  }
  if (names.size() > 32) {
    throw std::runtime_error("fusion.yaml: 'views.names' exceeds the 32 bits available in "
                             "ObjectState::visible_views");
  }
  cfg.view_names.clear();
  for (const YAML::Node& n : names) cfg.view_names.push_back(n.as<std::string>());
  return cfg;
}

}  // namespace the_eyes::fusion
