#include "the_eyes/camera/config.hpp"

#include <cstdio>
#include <fstream>
#include <unistd.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace the_eyes::camera {
namespace {

class TempYaml {
 public:
  explicit TempYaml(const std::string& body) {
    char tmpl[] = "/tmp/the_eyes_cameras_XXXXXX";
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

std::string ConfigPath() { return std::string(THE_EYES_CONFIG_DIR) + "/cameras.yaml"; }

TEST(CameraConfig, LoadsTheShippedFile) {
  const CameraAgentConfig config = LoadCameraAgentConfig(ConfigPath());
  ASSERT_EQ(config.cameras.size(), 2u);
  EXPECT_EQ(config.at("cam_front").serial, "261722073945");
  EXPECT_EQ(config.at("cam_over").serial, "261822074567");
  // Master/slave, enabled 2026-08-25 once the physical sync cable arrived.
  // Before the cable, a slave opened fine and then never delivered a frame --
  // a slave captures only when triggered.
  EXPECT_EQ(config.at("cam_front").inter_cam_sync_mode, 1);
  EXPECT_EQ(config.at("cam_over").inter_cam_sync_mode, 2);
  EXPECT_EQ(config.streams.depth_width, 848);
  EXPECT_EQ(config.streams.fps, 30);
  EXPECT_TRUE(config.streams.align_depth_to_color);
}

TEST(CameraConfig, UnknownCameraThrows) {
  const CameraAgentConfig config = LoadCameraAgentConfig(ConfigPath());
  EXPECT_THROW(config.at("cam_left"), std::out_of_range);
}

TEST(CameraConfig, MissingSerialIsAHardError) {
  // CLAUDE.md constraint 4: without a serial there is no stable way to address
  // the device, and falling back to enumeration order is the exact failure the
  // rule exists to prevent.
  const TempYaml f("cameras:\n  cam_front:\n    inter_cam_sync_mode: 1\n");
  EXPECT_THROW(LoadCameraAgentConfig(f.path()), std::runtime_error);

  const TempYaml empty("cameras:\n  cam_front:\n    serial: \"\"\n");
  EXPECT_THROW(LoadCameraAgentConfig(empty.path()), std::runtime_error);
}

TEST(CameraConfig, DuplicateSerialsAreRejected) {
  // Two entries on one physical unit would make every dual-view result a
  // comparison of a camera with itself.
  const TempYaml f("cameras:\n  cam_front:\n    serial: \"SAME\"\n"
                   "  cam_over:\n    serial: \"SAME\"\n");
  EXPECT_THROW(LoadCameraAgentConfig(f.path()), std::runtime_error);
}

TEST(CameraConfig, RejectsDisablingDepthAlignment) {
  // INTERFACES.md §1 requires depth aligned to colour, and FoundationPose
  // assumes registered RGB-D. Allowing it off would silently break pose.
  const TempYaml f("cameras:\n  cam_front:\n    serial: \"A\"\n"
                   "streams:\n  align_depth_to_color: false\n");
  EXPECT_THROW(LoadCameraAgentConfig(f.path()), std::runtime_error);
}

TEST(CameraConfig, SyncModeZeroIsAValidExplicitRequest) {
  // Regression: mode 0 must survive loading as a real value, because the driver
  // writes it unconditionally -- the setting persists in camera NVRAM, so 0 has
  // to be actively written to undo a previous master/slave configuration.
  const TempYaml f("cameras:\n  cam_front:\n    serial: \"A\"\n"
                   "    inter_cam_sync_mode: 0\n");
  const CameraAgentConfig config = LoadCameraAgentConfig(f.path());
  EXPECT_EQ(config.at("cam_front").inter_cam_sync_mode, 0);
}

TEST(CameraConfig, RejectsAnInvalidSyncMode) {
  const TempYaml f("cameras:\n  cam_front:\n    serial: \"A\"\n    inter_cam_sync_mode: 7\n");
  EXPECT_THROW(LoadCameraAgentConfig(f.path()), std::runtime_error);
}

TEST(CameraConfig, RejectsEmptyCameraListAndMissingFile) {
  const TempYaml f("cameras: {}\n");
  EXPECT_THROW(LoadCameraAgentConfig(f.path()), std::runtime_error);
  EXPECT_THROW(LoadCameraAgentConfig("/nonexistent/cameras.yaml"), std::runtime_error);
}

TEST(CameraConfig, CameraIdsMatchFusionsViewOrder) {
  // These ids are what fusion maps to ObjectState::visible_views bits. A name
  // here that fusion does not know would silently drop that camera's estimates.
  const CameraAgentConfig config = LoadCameraAgentConfig(ConfigPath());
  const YAML::Node views =
      YAML::LoadFile(std::string(THE_EYES_CONFIG_DIR) + "/fusion.yaml")["views"]["names"];
  ASSERT_TRUE(views && views.IsSequence());

  std::vector<std::string> names;
  for (const YAML::Node& n : views) names.push_back(n.as<std::string>());
  for (const auto& [camera_id, entry] : config.cameras) {
    EXPECT_NE(std::find(names.begin(), names.end(), camera_id), names.end())
        << camera_id << " has no visible_views bit in fusion.yaml";
  }
}

TEST(CameraConfig, SerialsMatchThoseRecordedInExtrinsics) {
  // extrinsics.yaml also carries serials. If the two files disagree, fusion
  // would transform one camera's poses with the other's extrinsics.
  const CameraAgentConfig config = LoadCameraAgentConfig(ConfigPath());
  const YAML::Node cameras =
      YAML::LoadFile(std::string(THE_EYES_CONFIG_DIR) + "/extrinsics.yaml")["cameras"];
  ASSERT_TRUE(cameras && cameras.IsMap());

  for (const auto& [camera_id, entry] : config.cameras) {
    const YAML::Node node = cameras[camera_id];
    if (!node) continue;  // extrinsics may not be calibrated yet
    const std::string recorded = node["serial"].as<std::string>();
    // A placeholder is fine; a WRONG serial is not.
    if (recorded == "000000000000") continue;
    EXPECT_EQ(recorded, entry.serial)
        << camera_id << ": cameras.yaml and extrinsics.yaml disagree about the serial";
  }
}

}  // namespace
}  // namespace the_eyes::camera
