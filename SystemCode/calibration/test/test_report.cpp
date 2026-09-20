#include "the_eyes/calibration/report.hpp"

#include <cstdio>
#include <fstream>
#include <unistd.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "the_eyes/calibration/synthetic.hpp"

namespace the_eyes::calibration {
namespace {

std::string TempPath() {
  char tmpl[] = "/tmp/the_eyes_extrinsics_XXXXXX";
  const int fd = ::mkstemp(tmpl);
  if (fd >= 0) ::close(fd);
  return tmpl;
}

BoardSpec Board() {
  BoardSpec s;
  s.squares_x = 8;
  s.squares_y = 6;
  s.square_length_m = 0.030;
  s.marker_length_m = 0.022;
  return s;
}

SyntheticCamera Camera(const std::string& id) {
  SyntheticCamera c;
  c.camera_id = id;
  c.image_size = cv::Size(1280, 720);
  c.fx = 900.0;
  c.fy = 900.0;
  c.cx = 640.0;
  c.cy = 360.0;
  return c;
}

std::vector<BoardObservation> Views(const SyntheticCamera& camera, const BoardSpec& board,
                                    int count) {
  std::vector<BoardObservation> out;
  const auto poses = SampleBoardPoses(board, count);
  for (size_t i = 0; i < poses.size(); ++i) {
    BoardObservation o = ProjectBoard(camera, board, poses[i], static_cast<uint64_t>(i));
    if (o.size() >= 12) out.push_back(std::move(o));
  }
  return out;
}

// A full two-camera calibration from synthetic data, with both cameras viewing
// the board at the world origin from known poses.
struct Rig {
  BoardSpec board = Board();
  Eigen::Isometry3d T_world_cam_front_truth;
  Eigen::Isometry3d T_world_cam_over_truth;
  BoardObservation registration_front;
  BoardObservation registration_over;
  CalibrationReport report;
};

Rig BuildRig(double round_trip_perturbation_m = 0.0, const SyntheticNoise& noise = {},
             const Eigen::Isometry3d& T_world_board = Eigen::Isometry3d::Identity()) {
  Rig rig;
  rig.report.world_frame = "table_origin";
  rig.report.thresholds = QualityThresholds{};

  // Two viewpoints on the board lying at the world origin.
  // The board lies face-up on the table, so a camera looking at its front face
  // must be rotated ~180 deg about x relative to the board frame. Without that
  // flip the maths stays self-consistent but places both cameras BELOW the
  // table -- a good reminder that "the numbers are consistent" and "the setup
  // is physical" are different claims.
  const Eigen::Matrix3d face_up =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

  Eigen::Isometry3d T_front_board = Eigen::Isometry3d::Identity();
  T_front_board.linear() =
      Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitX()).toRotationMatrix() * face_up;
  T_front_board.translation() = Eigen::Vector3d(-0.10, -0.08, 0.60);

  Eigen::Isometry3d T_over_board = Eigen::Isometry3d::Identity();
  T_over_board.linear() =
      Eigen::AngleAxisd(-0.20, Eigen::Vector3d::UnitY()).toRotationMatrix() * face_up;
  T_over_board.translation() = Eigen::Vector3d(-0.09, -0.07, 0.75);

  const struct { const char* id; const char* serial; Eigen::Isometry3d T_cam_board; } specs[] = {
      {"cam_front", "261722073945", T_front_board},
      {"cam_over", "000000000001", T_over_board},
  };

  for (const auto& spec : specs) {
    const SyntheticCamera camera = Camera(spec.id);
    CameraCalibration calibration;
    calibration.camera_id = spec.id;
    calibration.serial = spec.serial;
    calibration.intrinsics = CalibrateIntrinsics(Views(camera, rig.board, 40), rig.board);

    BoardObservation registration =
        ProjectBoard(camera, rig.board, spec.T_cam_board, /*stamp_ns=*/0, noise);
    const PoseResult pose = EstimateBoardPose(registration, calibration.intrinsics, rig.board);
    calibration.registration_rmse_px = pose.reprojection_rmse_px;
    calibration.registration_max_residual_px = pose.max_residual_px;
    calibration.T_world_cam = RegisterToWorld(pose.T_cam_board, T_world_board);

    if (std::string(spec.id) == "cam_over" && round_trip_perturbation_m != 0.0) {
      // Nudge one camera's extrinsics: the two views now disagree about where a
      // shared point is, which is exactly what the round-trip check exists for.
      calibration.T_world_cam.translation().x() += round_trip_perturbation_m;
      rig.registration_over = registration;
    } else if (std::string(spec.id) == "cam_over") {
      rig.registration_over = registration;
    } else {
      rig.registration_front = registration;
    }
    rig.report.cameras.push_back(std::move(calibration));
  }

  rig.report.round_trip =
      ComputeRoundTrip(rig.registration_front, rig.report.cameras[0], rig.registration_over,
                       rig.report.cameras[1]);
  Evaluate(&rig.report);
  return rig;
}

TEST(Report, SyntheticRigPassesEveryCriterion) {
  const Rig rig = BuildRig();
  EXPECT_TRUE(rig.report.passed()) << rig.report.Format();
  for (const CameraCalibration& camera : rig.report.cameras) {
    EXPECT_LT(camera.intrinsics.reprojection_rmse_px, 0.5);
    EXPECT_LT(camera.registration_rmse_px, 0.5);
  }
}

TEST(Report, CamerasEndUpAboveTheTableNotBelowIt) {
  // Self-consistent maths can still describe an impossible rig. With the board
  // face-up at the world origin, both cameras must sit at positive world z.
  const Rig rig = BuildRig();
  for (const CameraCalibration& camera : rig.report.cameras) {
    EXPECT_GT(camera.T_world_cam.translation().z(), 0.0)
        << camera.camera_id << " is below the table";
  }
}

TEST(Report, RoundTripAgreesWhenExtrinsicsAreCorrect) {
  // The acceptance criterion: a point seen by both cameras maps to the same
  // world coordinate from either.
  const Rig rig = BuildRig();
  EXPECT_GE(rig.report.round_trip.shared_corners, 12);
  EXPECT_LT(rig.report.round_trip.max_error_m, 0.005);
  EXPECT_LT(rig.report.round_trip.mean_error_m, 0.001);
}

TEST(Report, RoundTripCatchesAMisregisteredCamera) {
  // 2 cm of extrinsic error. Every per-camera reprojection RMSE stays perfect --
  // each camera is internally consistent -- so ONLY the round trip can see this.
  const Rig rig = BuildRig(0.02);
  EXPECT_GT(rig.report.round_trip.max_error_m, 0.005);
  EXPECT_FALSE(rig.report.passed());

  bool named = false;
  for (const std::string& failure : rig.report.failures) {
    if (failure.find("round-trip") != std::string::npos) named = true;
  }
  EXPECT_TRUE(named) << rig.report.Format();

  for (const CameraCalibration& camera : rig.report.cameras) {
    EXPECT_LT(camera.intrinsics.reprojection_rmse_px, 0.5)
        << "per-camera RMSE is blind to this failure, which is the point";
  }
}

TEST(Report, RoundTripIsNotZeroByConstruction) {
  // The regression test for the 2026-08-28 defect.
  //
  // The old implementation pushed the board MODEL point through each camera's
  // extrinsics. Because T_world_cam is derived from that camera's PnP of the
  // same observation, both sides reduced to T_world_board * board_point and the
  // error came out identically zero -- on a real capture whose registration
  // RMSE was 3.12 px, the report printed 0.00 / 0.00 / 0.00 mm.
  //
  // Nothing here is perturbed after registration. The rig is built exactly the
  // way RunReal builds it, with realistic corner noise, so any disagreement is
  // the genuine article. Under the old implementation this asserts against an
  // exact zero and fails.
  SyntheticNoise noise;
  noise.pixel_sigma = 0.3;
  const Rig rig = BuildRig(0.0, noise);

  EXPECT_GT(rig.report.round_trip.max_error_m, 1e-6)
      << "the round trip collapsed to zero -- it is measuring the board model "
         "rather than the detections again";
  // And it must still be a sane number, not noise amplified into nonsense: sub-
  // millimetre disagreement from 0.3 px of corner noise at this geometry.
  EXPECT_LT(rig.report.round_trip.max_error_m, 0.005) << rig.report.Format();
  EXPECT_TRUE(rig.report.passed()) << rig.report.Format();
}

TEST(Report, RoundTripMeasuresTheRigNotTheWorldPlacement) {
  // The check asks whether the two cameras agree with EACH OTHER. Where the
  // board was declared to sit is a rigid transform applied to both, so it
  // cannot change the answer -- and if it could, the number would be partly
  // reporting T_world_board, which no calibration can validate.
  SyntheticNoise noise;
  noise.pixel_sigma = 0.3;

  Eigen::Isometry3d elsewhere = Eigen::Isometry3d::Identity();
  elsewhere.linear() =
      (Eigen::AngleAxisd(0.52, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
  elsewhere.translation() = Eigen::Vector3d(0.30, -0.20, 0.05);

  const Rig at_origin = BuildRig(0.0, noise);
  const Rig moved = BuildRig(0.0, noise, elsewhere);

  EXPECT_NEAR(at_origin.report.round_trip.max_error_m, moved.report.round_trip.max_error_m, 1e-9);
  EXPECT_NEAR(at_origin.report.round_trip.mean_error_m, moved.report.round_trip.mean_error_m,
              1e-9);
}

TEST(Report, EveryFailureIsListedNotJustTheFirst) {
  // Knowing both cameras failed, versus one, is the difference between
  // remounting the rig and reshooting a single capture.
  CalibrationReport report;
  report.world_frame = "table_origin";
  report.thresholds = QualityThresholds{};
  for (const char* id : {"cam_front", "cam_over"}) {
    CameraCalibration camera;
    camera.camera_id = id;
    camera.serial = "SERIAL";
    camera.intrinsics.reprojection_rmse_px = 3.0;   // fails
    camera.intrinsics.views_used = 4;               // also fails
    report.cameras.push_back(camera);
  }
  Evaluate(&report);
  EXPECT_GE(report.failures.size(), 4u);
}

TEST(FactoryIntrinsics, WrapsDeviceValuesWithoutViews) {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K << 607.01, 0.0, 428.59, 0.0, 606.79, 242.68, 0.0, 0.0, 1.0;  // measured, cam_front
  const IntrinsicResult result = FactoryIntrinsics(K, cv::Size(848, 480));

  EXPECT_EQ(result.source, IntrinsicSource::kFactory);
  EXPECT_EQ(result.views_used, 0);
  EXPECT_TRUE(result.KEigen().isApprox(K, 1e-12));
  // The D400 colour stream reports zero distortion -- it comes out rectified.
  for (int i = 0; i < result.distortion.cols; ++i) {
    EXPECT_DOUBLE_EQ(result.distortion.at<double>(0, i), 0.0);
  }
}

TEST(FactoryIntrinsics, SkipsTheViewCountGateButStillGatesOnRegistration) {
  // A factory result has no calibration views. Running the "enough views" check
  // against views_used == 0 would fail every factory calibration outright.
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K << 607.01, 0.0, 428.59, 0.0, 606.79, 242.68, 0.0, 0.0, 1.0;

  CalibrationReport report;
  report.world_frame = "table_origin";
  report.thresholds = QualityThresholds{};
  CameraCalibration camera;
  camera.camera_id = "cam_front";
  camera.serial = "261722073945";
  camera.intrinsics = FactoryIntrinsics(K, cv::Size(848, 480));
  camera.registration_rmse_px = 0.21;   // inside the 0.5 px gate
  report.cameras.push_back(camera);
  report.round_trip.shared_corners = 30;

  Evaluate(&report);
  EXPECT_TRUE(report.passed()) << report.Format();

  // ...and a bad registration still fails, naming the factory intrinsics as a
  // possible cause rather than silently accepting them.
  report.cameras[0].registration_rmse_px = 1.8;
  Evaluate(&report);
  ASSERT_FALSE(report.passed());
  EXPECT_NE(report.failures[0].find("FACTORY"), std::string::npos) << report.failures[0];
}

TEST(FactoryIntrinsics, WriterEmitsTheRegistrationRmseAsFusionsGate) {
  // Fusion gates on reprojection_rmse_px. With factory intrinsics there is no
  // intrinsic RMSE, so writing 0.0 would hand fusion a meaningless pass.
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K << 607.01, 0.0, 428.59, 0.0, 606.79, 242.68, 0.0, 0.0, 1.0;

  CalibrationReport report;
  report.world_frame = "table_origin";
  report.thresholds = QualityThresholds{};
  CameraCalibration camera;
  camera.camera_id = "cam_front";
  camera.serial = "261722073945";
  camera.intrinsics = FactoryIntrinsics(K, cv::Size(848, 480));
  camera.registration_rmse_px = 0.33;
  report.cameras.push_back(camera);
  report.round_trip.shared_corners = 30;
  Evaluate(&report);
  ASSERT_TRUE(report.passed());

  const std::string path = TempPath();
  WriteExtrinsicsYaml(report, path);
  const YAML::Node root = YAML::LoadFile(path);
  EXPECT_NEAR(root["cameras"]["cam_front"]["reprojection_rmse_px"].as<double>(), 0.33, 1e-9);
  std::remove(path.c_str());
}

TEST(Report, MissingSerialIsAFailure) {
  // CLAUDE.md constraint 4: cameras are addressed by serial, never by
  // enumeration order. An entry without one is unusable at runtime.
  CalibrationReport report;
  report.thresholds = QualityThresholds{};
  CameraCalibration camera;
  camera.camera_id = "cam_front";
  camera.intrinsics.reprojection_rmse_px = 0.1;
  camera.intrinsics.views_used = 40;
  report.cameras.push_back(camera);
  report.round_trip.shared_corners = 30;
  Evaluate(&report);

  ASSERT_FALSE(report.passed());
  EXPECT_NE(report.failures[0].find("serial"), std::string::npos);
}

TEST(Report, TooFewSharedCornersIsAFailure) {
  CalibrationReport report;
  report.thresholds = QualityThresholds{};
  CameraCalibration camera;
  camera.camera_id = "cam_front";
  camera.serial = "S";
  camera.intrinsics.reprojection_rmse_px = 0.1;
  camera.intrinsics.views_used = 40;
  report.cameras.push_back(camera);
  report.round_trip.shared_corners = 3;
  Evaluate(&report);
  EXPECT_FALSE(report.passed());
}

TEST(Writer, EmitsTheInterfacesSection4Format) {
  const Rig rig = BuildRig();
  const std::string path = TempPath();
  ASSERT_NO_THROW(WriteExtrinsicsYaml(rig.report, path));

  const YAML::Node root = YAML::LoadFile(path);
  EXPECT_EQ(root["world_frame"].as<std::string>(), "table_origin");

  const YAML::Node cameras = root["cameras"];
  ASSERT_TRUE(cameras && cameras.IsMap());
  ASSERT_EQ(cameras.size(), 2u);

  for (const char* id : {"cam_front", "cam_over"}) {
    const YAML::Node camera = cameras[id];
    ASSERT_TRUE(camera) << id;
    EXPECT_FALSE(camera["serial"].as<std::string>().empty());
    EXPECT_EQ(camera["T_world_cam"].size(), 16u);
    EXPECT_GE(camera["reprojection_rmse_px"].as<double>(), 0.0);
    // Bottom row must be [0 0 0 1] -- fusion's loader rejects anything else.
    EXPECT_DOUBLE_EQ(camera["T_world_cam"][12].as<double>(), 0.0);
    EXPECT_DOUBLE_EQ(camera["T_world_cam"][15].as<double>(), 1.0);
  }
  std::remove(path.c_str());
}

TEST(Writer, RefusesToWriteAFailingCalibrationUnlessForced) {
  // Fusion's gate reads reprojection_rmse_px and would catch it, but a file
  // that exists is assumed good by everything else.
  Rig rig = BuildRig();
  rig.report.failures.push_back("synthetic failure for the test");

  const std::string path = TempPath();
  EXPECT_THROW(WriteExtrinsicsYaml(rig.report, path), std::runtime_error);
  EXPECT_NO_THROW(WriteExtrinsicsYaml(rig.report, path, /*force=*/true));

  // A forced write must say so in the file itself.
  std::ifstream file(path);
  const std::string contents((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
  EXPECT_NE(contents.find("WARNING"), std::string::npos);
  std::remove(path.c_str());
}

TEST(Report, FormatMentionsTheOutcomeAndThresholds) {
  const Rig rig = BuildRig();
  const std::string text = rig.report.Format();
  EXPECT_NE(text.find("cam_front"), std::string::npos);
  EXPECT_NE(text.find("round trip"), std::string::npos);
  EXPECT_NE(text.find("RESULT: PASS"), std::string::npos);
}

TEST(Config, LoadsQualityAndWorldFromTheShippedFile) {
  const std::string path = std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml";
  const QualityThresholds thresholds = LoadQualityThresholds(path);
  EXPECT_DOUBLE_EQ(thresholds.max_reprojection_rmse_px, 0.5);
  EXPECT_DOUBLE_EQ(thresholds.max_round_trip_error_m, 0.005);

  const WorldConfig world = LoadWorldConfig(path);
  EXPECT_EQ(world.frame, "table_origin");
}

TEST(Config, ShippedWorldPlacementHasTheBoardPatternSideUp) {
  // The identity that used to ship here mirrored the rig: on a real capture it
  // placed both cameras at negative world z while they were plainly above the
  // board. Nothing else catches that -- reprojection RMSE and the round trip
  // are both invariant to this matrix -- so the property is pinned here.
  //
  // The ChArUco frame has +z INTO the panel. A board lying pattern-up therefore
  // has its +z pointing DOWN in world terms.
  const WorldConfig world = LoadWorldConfig(std::string(THE_EYES_CONFIG_DIR) +
                                            "/calibration.yaml");
  const Eigen::Vector3d board_z = world.T_world_board.linear() * Eigen::Vector3d::UnitZ();
  EXPECT_LT(board_z.z(), -0.99) << "board +z must point into the tabletop; identity (or any "
                                   "rotation leaving +z up) describes a board face-down and "
                                   "mirrors every camera pose";

  // Still a proper rotation, not a reflection that happens to flip z.
  EXPECT_NEAR(world.T_world_board.linear().determinant(), 1.0, 1e-12);
}

TEST(Config, ShippedWorldPlacementCentresThePatternOnTheOrigin) {
  // The tape marks go on the pattern corners and the geometry in RIG_BOM.md is
  // measured from the workspace centre, so the pattern centre must land on the
  // world origin. If it does not, every camera position comes out offset from
  // the numbers a tape measure can check -- which is the only check there is.
  const std::string path = std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml";
  const WorldConfig world = LoadWorldConfig(path);
  const BoardSpec board = LoadBoardSpec(path);

  const Eigen::Vector3d pattern_centre(0.5 * board.squares_x * board.square_length_m,
                                       0.5 * board.squares_y * board.square_length_m, 0.0);
  const Eigen::Vector3d in_world = world.T_world_board * pattern_centre;
  EXPECT_NEAR(in_world.x(), 0.0, 1e-9);
  EXPECT_NEAR(in_world.y(), 0.0, 1e-9);
  // z is the board thickness, whatever it has been measured as. Not asserted --
  // it is a [VERIFY] in the config, not a derived quantity.
}

TEST(Config, ThresholdMatchesFusionsGate) {
  // calib_agent's target and the gate fusion refuses to start above must be the
  // same number, or a calibration this module calls acceptable would be
  // rejected at runtime -- or worse, the reverse.
  const QualityThresholds thresholds =
      LoadQualityThresholds(std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml");
  const YAML::Node fusion =
      YAML::LoadFile(std::string(THE_EYES_CONFIG_DIR) + "/fusion.yaml")["calibration"];
  ASSERT_TRUE(fusion && fusion["max_reprojection_rmse_px"]);
  EXPECT_DOUBLE_EQ(thresholds.max_reprojection_rmse_px,
                   fusion["max_reprojection_rmse_px"].as<double>());
}

}  // namespace
}  // namespace the_eyes::calibration
