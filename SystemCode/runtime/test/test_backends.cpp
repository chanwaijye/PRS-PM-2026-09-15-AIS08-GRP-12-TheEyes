#include <cstdlib>
#include <fstream>

#include <gtest/gtest.h>

#include "the_eyes/runtime/backends.hpp"
#include "the_eyes/runtime/startup_checks.hpp"

namespace {

using namespace the_eyes;
using namespace the_eyes::runtime;

std::string TempDirPath() {
  char tmpl[] = "/tmp/the_eyes_backends_XXXXXX";
  const char* d = mkdtemp(tmpl);
  if (d == nullptr) throw std::runtime_error("mkdtemp failed");
  return d;
}

}  // namespace

TEST(Backends, ResolvesRelativePathsAgainstTheRepoRoot) {
  EXPECT_EQ(ResolveAgainst("/repo", "assets/m.onnx"), "/repo/assets/m.onnx");
  EXPECT_EQ(ResolveAgainst("/repo", "/abs/m.onnx"), "/abs/m.onnx");
  EXPECT_EQ(ResolveAgainst("", "rel"), "rel");
}

TEST(Backends, MockDetectorFromAFixture) {
  const std::string dir = TempDirPath();
  std::ofstream(dir + "/fixture.yaml") << "frames:\n  - {camera_id: cam_front, stamp_ns: 5, "
                                          "detections: [{cls: bowl, bbox: [1, 2, 30, 40]}]}\n";
  DetectorBackendConfig cfg;
  cfg.backend = "mock";
  cfg.mock_fixture = "fixture.yaml";
  detector::DetectorConfig dcfg;
  DetectorHandle h = MakeDetector(cfg, dcfg, dir);
  ASSERT_TRUE(h.detector);
  EXPECT_EQ(h.onnx, nullptr);
  EXPECT_EQ(h.label, "mock:fixture.yaml");
  EXPECT_EQ(h.detector->name(), "mock:fixture");

  detector::DetectorInput in;
  in.camera_id = "cam_front";
  in.stamp_ns = 5;
  in.rgb = cv::Mat(50, 50, CV_8UC3, cv::Scalar(0, 0, 0));
  in.depth = cv::Mat(50, 50, CV_16UC1, cv::Scalar(700));
  const DetectionSet out = h.detector->Detect(in);
  ASSERT_EQ(out.detections.size(), 1u);
  EXPECT_EQ(out.detections[0].cls, "bowl");
  EXPECT_NE(out.detections[0].track_id, 0u);

  cfg.mock_fixture = "missing.yaml";
  EXPECT_THROW(MakeDetector(cfg, dcfg, dir), std::runtime_error);
  cfg.mock_fixture = "";
  EXPECT_THROW(MakeDetector(cfg, dcfg, dir), std::runtime_error);
  cfg.backend = "nope";
  EXPECT_THROW(MakeDetector(cfg, dcfg, dir), std::runtime_error);
}

TEST(Backends, OnnxWithoutAModelFileFailsLoudly) {
  DetectorBackendConfig cfg;
  cfg.backend = "onnx";
  detector::DetectorConfig dcfg;
  dcfg.model.onnx_path = "assets/models/does_not_exist.onnx";
  try {
    MakeDetector(cfg, dcfg, TempDirPath());
    FAIL() << "expected a throw";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("export_onnx.py"), std::string::npos) << e.what();
  }
}

TEST(Backends, PoseFactories) {
  PoseBackendConfig cfg;
  cfg.backend = "mock";
  PoseHandle mock = MakePoseEstimator(cfg);
  ASSERT_TRUE(mock.estimator);
  EXPECT_EQ(mock.client, nullptr);
  EXPECT_EQ(mock.estimator->name(), "mock:identity");

  cfg.backend = "foundationpose";
  cfg.socket_path = "/tmp/the_eyes_definitely_no_server.sock";
  PoseHandle fp = MakePoseEstimator(cfg);
  ASSERT_TRUE(fp.estimator);
  EXPECT_NE(fp.client, nullptr);
  EXPECT_EQ(fp.label, "foundationpose");

  cfg.backend = "nope";
  EXPECT_THROW(MakePoseEstimator(cfg), std::runtime_error);
}

TEST(StartupChecks, LoadsTheShippedConfigsAndTheyCrossReference) {
  const LoadedConfigs c = LoadAllConfigs(THE_EYES_CONFIG_DIR, THE_EYES_REPO_ROOT);
  StartupReport r;
  CheckConfigCrossRefs(c, &r);
  EXPECT_TRUE(r.ok()) << r.Format();
  EXPECT_FALSE(r.info.empty());
}

TEST(StartupChecks, DetectsAMismatchedSerialAndViewOrder) {
  LoadedConfigs c = LoadAllConfigs(THE_EYES_CONFIG_DIR, THE_EYES_REPO_ROOT);
  c.extrinsics.cameras.at("cam_front").serial = "000000000000";
  c.fusion.view_names = {"cam_over", "cam_front"};
  c.scene.world_frame = "elsewhere";
  StartupReport r;
  CheckConfigCrossRefs(c, &r);
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.failures.size(), 3u) << r.Format();
}

TEST(StartupChecks, TheSupportCheckNeedsSomethingInTheSceneToStandOn) {
  // Shipped: on, and scene.yaml's table is the ground it names.
  LoadedConfigs c = LoadAllConfigs(THE_EYES_CONFIG_DIR, THE_EYES_REPO_ROOT);
  ASSERT_TRUE(c.predicates.require_support);
  StartupReport shipped;
  CheckConfigCrossRefs(c, &shipped);
  EXPECT_TRUE(shipped.ok()) << shipped.Format();
  EXPECT_NE(shipped.Format().find("support check: on"), std::string::npos) << shipped.Format();

  // A ground class nothing in the scene has: every object would be dropped on
  // every pair, and the run would look like an empty table.
  c.predicates.ground_classes = {"shelf"};
  StartupReport r;
  CheckConfigCrossRefs(c, &r);
  ASSERT_EQ(r.failures.size(), 1u) << r.Format();
  EXPECT_NE(r.failures[0].find("every object would be dropped"), std::string::npos) << r.failures[0];

  // Off, it has nothing to check.
  c.predicates.require_support = false;
  StartupReport off;
  CheckConfigCrossRefs(c, &off);
  EXPECT_TRUE(off.ok()) << off.Format();
  EXPECT_NE(off.Format().find("support check: off"), std::string::npos);
}

TEST(StartupChecks, PoseSocketFailureNamesTheServerCommand) {
  LoadedConfigs c = LoadAllConfigs(THE_EYES_CONFIG_DIR, THE_EYES_REPO_ROOT);
  c.pose_backend.backend = "foundationpose";
  c.pose_backend.socket_path = "/tmp/the_eyes_definitely_no_server.sock";
  StartupReport r;
  CheckPoseBackend(c, &r);
  ASSERT_EQ(r.failures.size(), 1u);
  EXPECT_NE(r.failures[0].find("foundationpose_server.py --socket "
                               "/tmp/the_eyes_definitely_no_server.sock"),
            std::string::npos)
      << r.failures[0];

  c.pose_backend.backend = "mock";
  StartupReport ok;
  CheckPoseBackend(c, &ok);
  EXPECT_TRUE(ok.ok());
}

TEST(StartupChecks, DetectorCheckReportsAMissingModel) {
  LoadedConfigs c = LoadAllConfigs(THE_EYES_CONFIG_DIR, THE_EYES_REPO_ROOT);
  c.runtime.detector.backend = "onnx";
  c.detector.model.onnx_path = "assets/models/nope.onnx";
  StartupReport r;
  CheckDetectorBackend(c, &r);
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.Format().find("export_onnx.py"), std::string::npos);

  c.runtime.detector.backend = "mock";
  c.runtime.detector.mock_fixture = "";
  StartupReport m;
  CheckDetectorBackend(c, &m);
  EXPECT_FALSE(m.ok());
}
