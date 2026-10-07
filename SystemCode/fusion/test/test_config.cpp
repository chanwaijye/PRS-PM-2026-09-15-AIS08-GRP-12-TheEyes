#include "the_eyes/fusion/config.hpp"

#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "the_eyes/types.hpp"

namespace the_eyes::fusion {
namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

std::string ConfigPath(const std::string& name) {
  return std::string(THE_EYES_CONFIG_DIR) + "/" + name;
}

TEST(FusionConfig, LoadsTheShippedFile) {
  const FusionConfig cfg = LoadFusionConfig(ConfigPath("fusion.yaml"));
  EXPECT_DOUBLE_EQ(cfg.calibration.max_reprojection_rmse_px, 0.5);
  EXPECT_DOUBLE_EQ(cfg.association.max_distance_m, 0.15);
  EXPECT_DOUBLE_EQ(cfg.disagreement_gate.max_translation_m, 0.05);
  EXPECT_NEAR(cfg.disagreement_gate.max_rotation_rad, 30.0 * kDegToRad, 1e-12);
  EXPECT_DOUBLE_EQ(cfg.disagreement_gate.two_view_conflict_confidence_penalty, 0.5);
  EXPECT_DOUBLE_EQ(cfg.ekf.max_coast_s, 1.0);
  EXPECT_DOUBLE_EQ(cfg.tracking.track_match_max_distance_m, 0.10);
}

TEST(FusionConfig, DefaultsMatchTheShippedFile) {
  // The in-code defaults exist so unit tests can run without the filesystem.
  // If they drift from the YAML, a test passing in isolation would mean nothing
  // about the deployed configuration.
  const FusionConfig loaded = LoadFusionConfig(ConfigPath("fusion.yaml"));
  const FusionConfig defaults;
  EXPECT_DOUBLE_EQ(loaded.calibration.max_reprojection_rmse_px,
                   defaults.calibration.max_reprojection_rmse_px);
  EXPECT_DOUBLE_EQ(loaded.association.max_distance_m, defaults.association.max_distance_m);
  EXPECT_DOUBLE_EQ(loaded.disagreement_gate.max_translation_m,
                   defaults.disagreement_gate.max_translation_m);
  EXPECT_NEAR(loaded.disagreement_gate.max_rotation_rad,
              defaults.disagreement_gate.max_rotation_rad, 1e-12);
  EXPECT_DOUBLE_EQ(loaded.ekf.max_coast_s, defaults.ekf.max_coast_s);
  EXPECT_EQ(loaded.view_names, defaults.view_names);
}

TEST(FusionConfig, ViewOrderMatchesPredicatesConfig) {
  // fusion.yaml and predicates.yaml each define the view order that fixes the
  // ObjectState::visible_views bit positions. If they disagree, fusion sets one
  // bit and the scene graph reads another, and every occluded_from fact silently
  // names the wrong camera.
  const FusionConfig cfg = LoadFusionConfig(ConfigPath("fusion.yaml"));

  const YAML::Node predicates = YAML::LoadFile(ConfigPath("predicates.yaml"));
  ASSERT_TRUE(predicates["views"] && predicates["views"]["names"]);
  std::vector<std::string> predicate_views;
  for (const YAML::Node& n : predicates["views"]["names"]) {
    predicate_views.push_back(n.as<std::string>());
  }
  EXPECT_EQ(cfg.view_names, predicate_views);
}

TEST(FusionConfig, ViewOrderMatchesTheViewBitEnum) {
  const FusionConfig cfg = LoadFusionConfig(ConfigPath("fusion.yaml"));
  ASSERT_GE(cfg.view_names.size(), 2u);
  EXPECT_EQ(1u << 0, static_cast<uint32_t>(kViewCamFront));
  EXPECT_EQ(1u << 1, static_cast<uint32_t>(kViewCamOver));
  EXPECT_EQ(cfg.view_names[0], "cam_front");
  EXPECT_EQ(cfg.view_names[1], "cam_over");
}

// The shipped file with its view_evidence section replaced by `section`.
std::string ShippedWithEvidenceSection(const std::string& section) {
  std::ifstream in(ConfigPath("fusion.yaml"));
  std::string body((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const size_t from = body.find("\nview_evidence:");
  const size_t to = body.find("\nekf:");
  EXPECT_NE(from, std::string::npos);
  EXPECT_NE(to, std::string::npos);
  body = body.substr(0, from + 1) + section + body.substr(to);
  char tmpl[] = "/tmp/the_eyes_fusion_cfg_XXXXXX";
  const int fd = ::mkstemp(tmpl);
  if (fd >= 0) ::close(fd);
  std::ofstream(tmpl) << body;
  return tmpl;
}

TEST(FusionConfig, TheShippedFileUsesTheDepthEvidence) {
  const FusionConfig cfg = LoadFusionConfig(ConfigPath("fusion.yaml"));
  EXPECT_TRUE(cfg.view_evidence.enabled);
  EXPECT_NEAR(cfg.view_evidence.max_seen_through, 1.0 / 3.0, 1e-3);
  EXPECT_DOUBLE_EQ(cfg.view_evidence.min_gap, 0.15);
  EXPECT_DOUBLE_EQ(cfg.view_evidence.agree_tol_m, 0.020);
  EXPECT_DOUBLE_EQ(cfg.view_evidence.through_tol_m, 0.040);
  // The in-code threshold and tolerances are the shipped ones; only `enabled`
  // differs, so that a bare FusionConfig is the gate without the evidence.
  const FusionConfig defaults;
  EXPECT_FALSE(defaults.view_evidence.enabled);
  EXPECT_NEAR(defaults.view_evidence.max_seen_through, cfg.view_evidence.max_seen_through, 1e-3);
  EXPECT_DOUBLE_EQ(defaults.view_evidence.min_gap, cfg.view_evidence.min_gap);
  EXPECT_DOUBLE_EQ(defaults.view_evidence.agree_tol_m, cfg.view_evidence.agree_tol_m);
  EXPECT_DOUBLE_EQ(defaults.view_evidence.through_tol_m, cfg.view_evidence.through_tol_m);
}

TEST(FusionConfig, TheEvidenceSectionIsOptionalButStrictWhenPresent) {
  // A fusion.yaml saved with a run from before 2026-10-07 has no such section,
  // still loads, and still means the gate it meant.
  const std::string none = ShippedWithEvidenceSection("");
  EXPECT_FALSE(LoadFusionConfig(none).view_evidence.enabled);
  std::remove(none.c_str());

  const std::string off = ShippedWithEvidenceSection(
      "view_evidence:\n  enabled: false\n  max_seen_through: 0.5\n  min_gap: 0.2\n"
      "  agree_tol_m: 0.01\n  through_tol_m: 0.03\n");
  const FusionConfig cfg = LoadFusionConfig(off);
  EXPECT_FALSE(cfg.view_evidence.enabled);
  EXPECT_DOUBLE_EQ(cfg.view_evidence.max_seen_through, 0.5);
  EXPECT_DOUBLE_EQ(cfg.view_evidence.min_gap, 0.2);
  EXPECT_DOUBLE_EQ(cfg.view_evidence.through_tol_m, 0.03);
  std::remove(off.c_str());

  const std::string rest = "  agree_tol_m: 0.02\n  through_tol_m: 0.04\n";
  for (const std::string& bad : {
           "view_evidence:\n  max_seen_through: 0.33\n  min_gap: 0.1\n" + rest,   // no switch
           "view_evidence:\n  enabled: true\n  min_gap: 0.1\n" + rest,            // no threshold
           "view_evidence:\n  enabled: true\n  max_seen_through: 0.33\n" + rest,  // no margin
           // a zero threshold would veto every estimate
           "view_evidence:\n  enabled: true\n  max_seen_through: 0.0\n  min_gap: 0.1\n" + rest,
           "view_evidence:\n  enabled: perhaps\n  max_seen_through: 0.33\n  min_gap: 0.1\n" + rest,
       }) {
    const std::string path = ShippedWithEvidenceSection(bad);
    EXPECT_THROW(LoadFusionConfig(path), std::runtime_error) << bad;
    std::remove(path.c_str());
  }
}

TEST(FusionConfig, MissingSectionThrows) {
  char tmpl[] = "/tmp/the_eyes_fusion_cfg_XXXXXX";
  const int fd = ::mkstemp(tmpl);
  if (fd >= 0) ::close(fd);
  std::ofstream(tmpl) << "calibration:\n  max_reprojection_rmse_px: 0.5\n";
  EXPECT_THROW(LoadFusionConfig(tmpl), std::runtime_error);
  std::remove(tmpl);
}

TEST(FusionConfig, RejectsAGateWiderThanTheAssociationRadius) {
  // Regression: with the gate wider than the association radius, a disagreeing
  // view is split off as a separate object before the gate can judge it, so the
  // gate silently never fires. The loader must refuse that configuration.
  const std::string body =
      "calibration:\n  max_reprojection_rmse_px: 0.5\n"
      "association:\n  max_distance_m: 0.05\n"
      "disagreement_gate:\n  max_translation_m: 0.08\n  max_rotation_deg: 30.0\n"
      "  two_view_conflict_confidence_penalty: 0.5\n"
      "ekf:\n  process_noise_position_m: 0.01\n  process_noise_velocity_mps: 0.1\n"
      "  process_noise_orientation_deg: 5.0\n  measurement_noise_position_m: 0.01\n"
      "  measurement_noise_orientation_deg: 3.0\n  initial_position_sigma_m: 0.1\n"
      "  initial_velocity_sigma_mps: 0.5\n  initial_orientation_sigma_deg: 45.0\n"
      "  max_coast_s: 1.0\n"
      "tracking:\n  track_match_max_distance_m: 0.1\n"
      "views:\n  names: [\"cam_front\", \"cam_over\"]\n";

  char tmpl[] = "/tmp/the_eyes_fusion_inv_XXXXXX";
  const int fd = ::mkstemp(tmpl);
  if (fd >= 0) ::close(fd);
  std::ofstream(tmpl) << body;
  try {
    LoadFusionConfig(tmpl);
    std::remove(tmpl);
    FAIL() << "expected the invariant check to reject this configuration";
  } catch (const std::runtime_error& e) {
    std::remove(tmpl);
    const std::string msg = e.what();
    EXPECT_NE(msg.find("can never fire"), std::string::npos) << msg;
  }
}

TEST(FusionConfig, ShippedFileSatisfiesTheGateInvariant) {
  const FusionConfig cfg = LoadFusionConfig(ConfigPath("fusion.yaml"));
  EXPECT_LT(cfg.disagreement_gate.max_translation_m, cfg.association.max_distance_m);
}

TEST(FusionConfig, MissingFileThrows) {
  EXPECT_THROW(LoadFusionConfig("/nonexistent/fusion.yaml"), std::runtime_error);
}

}  // namespace
}  // namespace the_eyes::fusion
