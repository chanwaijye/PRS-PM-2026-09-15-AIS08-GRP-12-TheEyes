// The calib_agent -> fusion contract, exercised end to end.
//
// calib_agent WRITES config/extrinsics.yaml and fusion READS it
// (INTERFACES.md §4). Each module tests its own half, which leaves the
// seam untested -- and the seam is a hand-written YAML emitter feeding a
// hand-written YAML parser, which is exactly where a format drift would hide.
#include <cstdio>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

#include "the_eyes/calibration/report.hpp"
#include "the_eyes/calibration/solver.hpp"
#include "the_eyes/calibration/synthetic.hpp"
#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/fusion/fusion.hpp"

namespace the_eyes {
namespace {

std::string TempPath() {
  char tmpl[] = "/tmp/the_eyes_contract_XXXXXX";
  const int fd = ::mkstemp(tmpl);
  if (fd >= 0) ::close(fd);
  return tmpl;
}

calibration::BoardSpec Board() {
  calibration::BoardSpec s;
  s.squares_x = 8;
  s.squares_y = 6;
  s.square_length_m = 0.030;
  s.marker_length_m = 0.022;
  return s;
}

// A complete synthetic calibration of the two-camera rig.
calibration::CalibrationReport Calibrate() {
  const calibration::BoardSpec board = Board();
  calibration::CalibrationReport report;
  report.world_frame = "table_origin";
  report.thresholds = calibration::QualityThresholds{};

  const Eigen::Matrix3d face_up =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

  struct Spec { const char* id; const char* serial; double tilt; double height; };
  const Spec specs[] = {{"cam_front", "261722073945", 0.35, 0.60},
                        {"cam_over", "000000000001", -0.20, 0.75}};

  std::vector<calibration::BoardObservation> registrations;
  for (const Spec& spec : specs) {
    calibration::SyntheticCamera camera;
    camera.camera_id = spec.id;

    std::vector<calibration::BoardObservation> views;
    for (const Eigen::Isometry3d& pose : calibration::SampleBoardPoses(board, 40)) {
      calibration::BoardObservation observation =
          calibration::ProjectBoard(camera, board, pose);
      if (observation.size() >= 12) views.push_back(std::move(observation));
    }

    Eigen::Isometry3d T_cam_board = Eigen::Isometry3d::Identity();
    T_cam_board.linear() =
        Eigen::AngleAxisd(spec.tilt, Eigen::Vector3d::UnitX()).toRotationMatrix() * face_up;
    T_cam_board.translation() = Eigen::Vector3d(-0.10, -0.08, spec.height);

    calibration::CameraCalibration calib;
    calib.camera_id = spec.id;
    calib.serial = spec.serial;
    calib.intrinsics = calibration::CalibrateIntrinsics(views, board);

    calibration::BoardObservation registration =
        calibration::ProjectBoard(camera, board, T_cam_board);
    const calibration::PoseResult pose =
        calibration::EstimateBoardPose(registration, calib.intrinsics, board);
    calib.registration_rmse_px = pose.reprojection_rmse_px;
    calib.registration_max_residual_px = pose.max_residual_px;
    calib.T_world_cam =
        calibration::RegisterToWorld(pose.T_cam_board, Eigen::Isometry3d::Identity());

    registrations.push_back(std::move(registration));
    report.cameras.push_back(std::move(calib));
  }

  report.round_trip = calibration::ComputeRoundTrip(registrations[0], report.cameras[0],
                                                    registrations[1], report.cameras[1]);
  calibration::Evaluate(&report);
  return report;
}

TEST(CalibrationContract, FusionLoadsWhatCalibAgentWrites) {
  const calibration::CalibrationReport report = Calibrate();
  ASSERT_TRUE(report.passed()) << report.Format();

  const std::string path = TempPath();
  ASSERT_NO_THROW(calibration::WriteExtrinsicsYaml(report, path));

  // fusion's loader is strict: it validates orthonormality, rejects
  // reflections, requires the bottom row, and refuses duplicate serials.
  fusion::ExtrinsicsSet loaded;
  ASSERT_NO_THROW(loaded = fusion::LoadExtrinsics(path)) << "fusion rejected calib's output";

  EXPECT_EQ(loaded.world_frame, "table_origin");
  ASSERT_EQ(loaded.cameras.size(), 2u);

  for (const calibration::CameraCalibration& written : report.cameras) {
    const CameraExtrinsics& read = loaded.at(written.camera_id);
    EXPECT_EQ(read.serial, written.serial);
    EXPECT_NEAR(read.reprojection_rmse_px, written.intrinsics.reprojection_rmse_px, 1e-9);
    // The pose must survive the round trip through text, not merely parse.
    EXPECT_TRUE(read.T_world_cam.isApprox(written.T_world_cam, 1e-9))
        << written.camera_id << " pose changed passing through YAML";
  }
  std::remove(path.c_str());
}

TEST(CalibrationContract, FusionStartsOnCalibAgentsOutput) {
  // The end of the chain: a calibration this module calls acceptable must be
  // one fusion agrees to run on. If the two thresholds ever drift apart, this
  // is where it shows.
  const calibration::CalibrationReport report = Calibrate();
  const std::string path = TempPath();
  calibration::WriteExtrinsicsYaml(report, path);

  const fusion::ExtrinsicsSet extrinsics = fusion::LoadExtrinsics(path);
  EXPECT_NO_THROW(fusion::Fusion(fusion::FusionConfig{}, extrinsics));
  std::remove(path.c_str());
}

TEST(CalibrationContract, FusionRefusesAForcedFailingCalibration) {
  // calib_agent can be forced to emit a failing calibration. Fusion's gate is
  // the second line of defence and must still catch it.
  calibration::CalibrationReport report = Calibrate();
  report.cameras[1].intrinsics.reprojection_rmse_px = 2.5;  // past the 0.5 px gate
  calibration::Evaluate(&report);
  ASSERT_FALSE(report.passed());

  const std::string path = TempPath();
  ASSERT_THROW(calibration::WriteExtrinsicsYaml(report, path), std::runtime_error);
  ASSERT_NO_THROW(calibration::WriteExtrinsicsYaml(report, path, /*force=*/true));

  const fusion::ExtrinsicsSet extrinsics = fusion::LoadExtrinsics(path);
  EXPECT_THROW(fusion::Fusion(fusion::FusionConfig{}, extrinsics), std::runtime_error);
  std::remove(path.c_str());
}

TEST(CalibrationContract, CameraIdsMatchTheConfiguredViewOrder) {
  // The camera_id strings calib writes are what fusion maps to
  // ObjectState::visible_views bits. A mismatch would silently mislabel which
  // camera saw what.
  const calibration::CalibrationReport report = Calibrate();
  const fusion::FusionConfig config;
  for (const calibration::CameraCalibration& camera : report.cameras) {
    EXPECT_NE(fusion::ExtrinsicsSet::ViewBitFor(camera.camera_id, config.view_names), 0u)
        << camera.camera_id << " has no visible_views bit in fusion.yaml views.names";
  }
}

}  // namespace
}  // namespace the_eyes
