// Session re-registration.
//
// The whole module is pure geometry on Isometry3d, so every case below is
// exercised with no cameras, no images, and no aruco.
#include <gtest/gtest.h>

#include <cmath>

#include "the_eyes/calibration/reregister.hpp"

namespace the_eyes::calibration {
namespace {

// The rig from docs/RIG_BOM.md: d = 0.70 m, elevation 35 deg, azimuth +/-40 deg.
Eigen::Isometry3d RigCamera(double azimuth_deg) {
  const double d = 0.70, elevation = 35.0 * M_PI / 180.0;
  const double a = azimuth_deg * M_PI / 180.0;
  const double r = d * std::cos(elevation);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(r * std::sin(a), -r * std::cos(a), d * std::sin(elevation));
  // Aim at the workspace origin.
  const Eigen::Vector3d forward = -T.translation().normalized();
  Eigen::Vector3d right = forward.cross(Eigen::Vector3d::UnitZ()).normalized();
  Eigen::Matrix3d R;
  R.col(0) = right;
  R.col(1) = forward.cross(right);
  R.col(2) = forward;
  T.linear() = R;
  return T;
}

std::vector<Eigen::Isometry3d> Rig() { return {RigCamera(-40.0), RigCamera(40.0)}; }
const std::vector<std::string> kIds{"cam_left", "cam_right"};

// What a table shift actually does to T_world_cam. The world frame is glued to
// the tabletop, so the tabletop moving by X is every camera's pose moving by
// X^-1 in world coordinates -- applied identically to both.
std::vector<Eigen::Isometry3d> MoveTable(const std::vector<Eigen::Isometry3d>& rig,
                                         const Eigen::Isometry3d& table_motion) {
  std::vector<Eigen::Isometry3d> moved;
  for (const Eigen::Isometry3d& T : rig) moved.push_back(table_motion.inverse() * T);
  return moved;
}

TEST(PoseDelta, IdenticalPosesHaveZeroDelta) {
  const Eigen::Isometry3d T = RigCamera(-40.0);
  const PoseDelta d = ComputePoseDelta(T, T);
  EXPECT_NEAR(d.translation_m, 0.0, 1e-12);
  EXPECT_NEAR(d.rotation_deg, 0.0, 1e-9);
}

TEST(PoseDelta, MagnitudeIsOrderIndependent) {
  const Eigen::Isometry3d a = RigCamera(-40.0);
  Eigen::Isometry3d b = a;
  b.translation() += Eigen::Vector3d(0.01, -0.02, 0.005);
  b.linear() = Eigen::AngleAxisd(0.03, Eigen::Vector3d(1, 2, 3).normalized()) * b.linear();

  const PoseDelta forward = ComputePoseDelta(a, b);
  const PoseDelta backward = ComputePoseDelta(b, a);
  EXPECT_NEAR(forward.translation_m, backward.translation_m, 1e-12);
  EXPECT_NEAR(forward.rotation_deg, backward.rotation_deg, 1e-9);
}

TEST(PoseDelta, PureRotationReportsNoTranslation) {
  const Eigen::Isometry3d a = RigCamera(0.0);
  Eigen::Isometry3d b = a;
  b.linear() = a.linear() * Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const PoseDelta d = ComputePoseDelta(a, b);
  // Rotating about the camera's own centre moves no point of the pose origin.
  EXPECT_NEAR(d.translation_m, 0.0, 1e-12);
  EXPECT_NEAR(d.rotation_deg, 0.05 * 180.0 / M_PI, 1e-9);
}

// ---------------------------------------------------------------------------
// The case the module exists for: the table moved, the rig did not.
// ---------------------------------------------------------------------------

TEST(ReRegistration, TableTranslationLeavesTheBaselineIntact) {
  const std::vector<Eigen::Isometry3d> before = Rig();
  Eigen::Isometry3d shift = Eigen::Isometry3d::Identity();
  shift.translation() = Eigen::Vector3d(0.012, -0.008, 0.0);  // desk nudged 14 mm

  const ReRegistrationReport report =
      CheckReRegistration(kIds, before, MoveTable(before, shift), RigidityThresholds{});

  EXPECT_TRUE(report.passed()) << report.Format();
  ASSERT_EQ(report.baselines.size(), 1u);
  EXPECT_TRUE(report.baselines[0].intact);
  EXPECT_NEAR(report.baselines[0].delta.translation_m, 0.0, 1e-12);
  EXPECT_NEAR(report.baselines[0].delta.rotation_deg, 0.0, 1e-9);

  // Both cameras report the same world motion, because it is one table.
  for (const CameraReRegistration& camera : report.cameras) {
    EXPECT_NEAR(camera.world_delta.translation_m, shift.translation().norm(), 1e-9);
  }
}

TEST(ReRegistration, TableRotationLeavesTheBaselineIntact) {
  const std::vector<Eigen::Isometry3d> before = Rig();
  Eigen::Isometry3d twist = Eigen::Isometry3d::Identity();
  twist.linear() = Eigen::AngleAxisd(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).matrix();

  const ReRegistrationReport report =
      CheckReRegistration(kIds, before, MoveTable(before, twist), RigidityThresholds{});

  EXPECT_TRUE(report.passed()) << report.Format();
  EXPECT_NEAR(report.baselines[0].delta.rotation_deg, 0.0, 1e-9);
  // A 2 deg twist is a real world-frame change, and it must be reported as one.
  EXPECT_NEAR(report.cameras[0].world_delta.rotation_deg, 2.0, 1e-6);
}

TEST(ReRegistration, LargeTableMotionIsAWarningNotAFailure) {
  const std::vector<Eigen::Isometry3d> before = Rig();
  Eigen::Isometry3d shift = Eigen::Isometry3d::Identity();
  shift.translation() = Eigen::Vector3d(0.05, 0.0, 0.0);  // 50 mm

  const ReRegistrationReport report =
      CheckReRegistration(kIds, before, MoveTable(before, shift), RigidityThresholds{});

  // The rig is intact, so this is recoverable -- but 50 mm is more likely a
  // misplaced board than a moved desk, and that would silently redefine world.
  EXPECT_TRUE(report.passed()) << report.Format();
  EXPECT_EQ(report.warnings.size(), 2u);  // one per camera
}

// ---------------------------------------------------------------------------
// The case the module exists to REFUSE: a camera moved.
// ---------------------------------------------------------------------------

TEST(ReRegistration, BumpedTripodFailsTheBaselineCheck) {
  const std::vector<Eigen::Isometry3d> before = Rig();
  std::vector<Eigen::Isometry3d> after = before;
  after[1].translation() += Eigen::Vector3d(0.0, 0.0, 0.004);  // 4 mm, one camera

  const ReRegistrationReport report =
      CheckReRegistration(kIds, before, after, RigidityThresholds{});

  EXPECT_FALSE(report.passed());
  EXPECT_FALSE(report.baselines[0].intact);
  EXPECT_NEAR(report.baselines[0].delta.translation_m, 0.004, 1e-9);
  EXPECT_NE(report.failures[0].find("Run a full calibration"), std::string::npos);
}

TEST(ReRegistration, BumpedTripodIsNotMaskedByASimultaneousTableShift) {
  // The nasty case: the desk was knocked AND a tripod was nudged. The world
  // motion looks ordinary, so only the baseline check separates them.
  std::vector<Eigen::Isometry3d> before = Rig();
  Eigen::Isometry3d shift = Eigen::Isometry3d::Identity();
  shift.translation() = Eigen::Vector3d(0.010, 0.006, 0.0);
  std::vector<Eigen::Isometry3d> after = MoveTable(before, shift);
  after[0].linear() =
      Eigen::AngleAxisd(0.5 * M_PI / 180.0, Eigen::Vector3d::UnitX()).matrix() * after[0].linear();

  const ReRegistrationReport report =
      CheckReRegistration(kIds, before, after, RigidityThresholds{});

  EXPECT_FALSE(report.passed()) << report.Format();
  EXPECT_NEAR(report.baselines[0].delta.rotation_deg, 0.5, 1e-6);
  EXPECT_TRUE(report.warnings.empty());  // world motion alone looked innocent
}

TEST(ReRegistration, SmallCameraDriftUnderThresholdStillPasses) {
  const std::vector<Eigen::Isometry3d> before = Rig();
  std::vector<Eigen::Isometry3d> after = before;
  after[1].translation() += Eigen::Vector3d(0.0, 0.0, 0.001);  // 1 mm < 3 mm gate

  const ReRegistrationReport report =
      CheckReRegistration(kIds, before, after, RigidityThresholds{});
  EXPECT_TRUE(report.passed()) << report.Format();
}

// ---------------------------------------------------------------------------
// Structure
// ---------------------------------------------------------------------------

TEST(ReRegistration, SingleCameraIsRefusedBecauseThereIsNothingToCheckAgainst) {
  const ReRegistrationReport report =
      CheckReRegistration({"cam_left"}, {RigCamera(-40.0)}, {RigCamera(-40.0)},
                          RigidityThresholds{});

  EXPECT_FALSE(report.passed());
  EXPECT_TRUE(report.baselines.empty());
  EXPECT_NE(report.failures[0].find("no baseline"), std::string::npos);
}

TEST(ReRegistration, ThreeCamerasCheckAllPairsNotAChain) {
  const std::vector<std::string> ids{"a", "b", "c"};
  const std::vector<Eigen::Isometry3d> before{RigCamera(-40.0), RigCamera(0.0), RigCamera(40.0)};
  EXPECT_EQ(CheckReRegistration(ids, before, before, RigidityThresholds{}).baselines.size(), 3u);

  // Rotate the MIDDLE camera. A chain check (a-b, b-c) can let the two link
  // errors cancel; all-pairs cannot, because a-c stays clean while both links
  // that touch b move.
  std::vector<Eigen::Isometry3d> after = before;
  after[1].linear() =
      Eigen::AngleAxisd(0.6 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).matrix() * after[1].linear();

  const ReRegistrationReport report = CheckReRegistration(ids, before, after,
                                                          RigidityThresholds{});
  EXPECT_FALSE(report.passed());
  EXPECT_EQ(report.failures.size(), 2u);  // a-b and b-c, not a-c
  for (const BaselineCheck& check : report.baselines) {
    const bool touches_b = check.camera_a == "b" || check.camera_b == "b";
    EXPECT_EQ(check.intact, !touches_b) << check.camera_a << "<->" << check.camera_b;
  }
}

TEST(ReRegistration, MismatchedInputLengthsThrow) {
  EXPECT_THROW(CheckReRegistration(kIds, Rig(), {RigCamera(0.0)}, RigidityThresholds{}),
               std::runtime_error);
  EXPECT_THROW(CheckReRegistration({}, {}, {}, RigidityThresholds{}), std::runtime_error);
}

TEST(ReRegistration, FormatNamesTheOutcome) {
  const std::vector<Eigen::Isometry3d> before = Rig();
  EXPECT_NE(CheckReRegistration(kIds, before, before, RigidityThresholds{}).Format().find(
                "RIG INTACT"),
            std::string::npos);

  std::vector<Eigen::Isometry3d> after = before;
  after[0].translation() += Eigen::Vector3d(0.02, 0, 0);
  const std::string text = CheckReRegistration(kIds, before, after, RigidityThresholds{}).Format();
  EXPECT_NE(text.find("MOVED"), std::string::npos);
  EXPECT_NE(text.find("RESULT: FAIL"), std::string::npos);
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

TEST(RigidityThresholds, DefaultsAreConsistentWithTheRoundTripGate) {
  const RigidityThresholds t;
  // A baseline gate at or above the 5 mm round-trip gate could never fire first.
  EXPECT_LT(t.max_baseline_shift_m, 0.005);
  // The rotation gate should cost about the same lateral error at 0.70 m as
  // the translation gate does, or one of the two dominates for no reason.
  const double lateral_from_rotation = 0.70 * t.max_baseline_rotation_deg * M_PI / 180.0;
  EXPECT_NEAR(lateral_from_rotation, t.max_baseline_shift_m, 0.0005);
}

TEST(RigidityThresholds, LoadsFromTheRealConfig) {
  const RigidityThresholds t =
      LoadRigidityThresholds(std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml");
  EXPECT_GT(t.max_baseline_shift_m, 0.0);
  EXPECT_LT(t.max_baseline_shift_m, 0.005);
  EXPECT_GT(t.max_baseline_rotation_deg, 0.0);
}

}  // namespace
}  // namespace the_eyes::calibration
