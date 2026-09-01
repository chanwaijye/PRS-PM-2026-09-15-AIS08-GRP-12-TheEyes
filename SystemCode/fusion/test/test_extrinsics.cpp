#include "the_eyes/fusion/extrinsics.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

namespace the_eyes::fusion {
namespace {

// Writes a temporary YAML file and removes it on scope exit.
class TempYaml {
 public:
  explicit TempYaml(const std::string& body) {
    char tmpl[] = "/tmp/the_eyes_extrinsics_XXXXXX";
    const int fd = ::mkstemp(tmpl);
    if (fd >= 0) ::close(fd);
    path_ = tmpl;
    std::ofstream(path_) << body;
  }
  ~TempYaml() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

std::string Identity4x4() {
  return "[1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]";
}

std::string TwoCameras(double rmse_front, double rmse_over) {
  return "world_frame: table_origin\ncameras:\n"
         "  cam_front:\n    serial: \"AAA\"\n    T_world_cam: " + Identity4x4() +
         "\n    reprojection_rmse_px: " + std::to_string(rmse_front) +
         "\n  cam_over:\n    serial: \"BBB\"\n    T_world_cam: " + Identity4x4() +
         "\n    reprojection_rmse_px: " + std::to_string(rmse_over) + "\n";
}

TEST(Extrinsics, LoadsTheShippedConfigFile) {
  const ExtrinsicsSet ex =
      LoadExtrinsics(std::string(THE_EYES_CONFIG_DIR) + "/extrinsics.yaml");
  EXPECT_EQ(ex.world_frame, "table_origin");
  ASSERT_EQ(ex.cameras.size(), 2u);
  EXPECT_EQ(ex.at("cam_front").serial, "261722073945");
  EXPECT_TRUE(ex.cameras.count("cam_over"));
}

TEST(Extrinsics, QualityGateRefusesToStartOnBadCalibration) {
  const TempYaml f(TwoCameras(0.2, 1.7));
  const ExtrinsicsSet ex = LoadExtrinsics(f.path());

  EXPECT_NO_THROW(AssertCalibrationQuality(ex, 2.0));
  try {
    AssertCalibrationQuality(ex, 0.5);
    FAIL() << "expected the gate to reject cam_over";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("cam_over"), std::string::npos) << msg;
    EXPECT_EQ(msg.find("cam_front"), std::string::npos)
        << "the passing camera should not be named: " << msg;
  }
}

TEST(Extrinsics, UnknownCameraThrowsRatherThanReturningIdentity) {
  const TempYaml f(TwoCameras(0.1, 0.1));
  const ExtrinsicsSet ex = LoadExtrinsics(f.path());
  EXPECT_THROW(ex.at("cam_left"), std::out_of_range);
}

TEST(Extrinsics, RejectsNonOrthonormalRotation) {
  const TempYaml f("world_frame: w\ncameras:\n  cam_front:\n    serial: \"A\"\n"
                   "    T_world_cam: [2,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n"
                   "    reprojection_rmse_px: 0.1\n");
  EXPECT_THROW(LoadExtrinsics(f.path()), std::runtime_error);
}

TEST(Extrinsics, RejectsReflection) {
  const TempYaml f("world_frame: w\ncameras:\n  cam_front:\n    serial: \"A\"\n"
                   "    T_world_cam: [-1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n"
                   "    reprojection_rmse_px: 0.1\n");
  EXPECT_THROW(LoadExtrinsics(f.path()), std::runtime_error);
}

TEST(Extrinsics, RejectsSharedSerialAcrossCameras) {
  // Both entries pointing at one physical unit would make every "fused" pose a
  // comparison of a camera with itself.
  const TempYaml f("world_frame: w\ncameras:\n"
                   "  cam_front:\n    serial: \"SAME\"\n    T_world_cam: " + Identity4x4() +
                   "\n    reprojection_rmse_px: 0.1\n"
                   "  cam_over:\n    serial: \"SAME\"\n    T_world_cam: " + Identity4x4() +
                   "\n    reprojection_rmse_px: 0.1\n");
  EXPECT_THROW(LoadExtrinsics(f.path()), std::runtime_error);
}

TEST(Extrinsics, RejectsMissingSerialAndMissingRmse) {
  const TempYaml no_serial("world_frame: w\ncameras:\n  cam_front:\n    T_world_cam: " +
                           Identity4x4() + "\n    reprojection_rmse_px: 0.1\n");
  EXPECT_THROW(LoadExtrinsics(no_serial.path()), std::runtime_error);

  const TempYaml no_rmse("world_frame: w\ncameras:\n  cam_front:\n    serial: \"A\"\n"
                         "    T_world_cam: " + Identity4x4() + "\n");
  EXPECT_THROW(LoadExtrinsics(no_rmse.path()), std::runtime_error);
}

TEST(Extrinsics, MissingFileThrows) {
  EXPECT_THROW(LoadExtrinsics("/nonexistent/extrinsics.yaml"), std::runtime_error);
}

TEST(Extrinsics, ToWorldComposesCameraPoseWithExtrinsics) {
  CameraExtrinsics cam;
  cam.camera_id = "cam_over";
  // Camera 0.9 m above the origin, optical z pointing down at the table.
  cam.T_world_cam = Eigen::Isometry3d::Identity();
  cam.T_world_cam.linear() =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  cam.T_world_cam.translation() = Eigen::Vector3d(0.0, 0.0, 0.9);

  PoseEstimate est;
  est.camera_id = "cam_over";
  // Object 0.85 m along the camera's optical axis, i.e. just above the table.
  est.pose_cam.translation() = Eigen::Vector3d(0.0, 0.0, 0.85);

  const Eigen::Isometry3d world = ToWorld(est, cam);
  EXPECT_NEAR(world.translation().x(), 0.0, 1e-12);
  EXPECT_NEAR(world.translation().y(), 0.0, 1e-12);
  EXPECT_NEAR(world.translation().z(), 0.05, 1e-12);
}

TEST(Extrinsics, ViewBitFollowsConfiguredViewOrder) {
  const std::vector<std::string> views = {"cam_front", "cam_over"};
  EXPECT_EQ(ExtrinsicsSet::ViewBitFor("cam_front", views), kViewCamFront);
  EXPECT_EQ(ExtrinsicsSet::ViewBitFor("cam_over", views), kViewCamOver);
  EXPECT_EQ(ExtrinsicsSet::ViewBitFor("cam_left", views), 0u);
}

}  // namespace
}  // namespace the_eyes::fusion
