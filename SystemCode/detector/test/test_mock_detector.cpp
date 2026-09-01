#include "the_eyes/detector/mock_detector.hpp"

#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace the_eyes::detector {
namespace {

class TempYaml {
 public:
  explicit TempYaml(const std::string& body) {
    char tmpl[] = "/tmp/the_eyes_fixture_XXXXXX";
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

const char* kFixture = R"(
frames:
  - camera_id: cam_front
    stamp_ns: 1000000000
    detections:
      - cls: mustard_bottle
        score: 0.93
        bbox: [100, 80, 60, 120]
      - cls: bowl
        score: 0.81
        bbox: [300, 200, 90, 50]
  - camera_id: cam_front
    stamp_ns: 1033000000
    detections:
      - cls: mustard_bottle
        score: 0.91
        bbox: [104, 82, 60, 120]
  - camera_id: cam_over
    stamp_ns: 1000000000
    detections:
      - cls: bowl
        score: 0.88
        bbox: [50, 50, 80, 80]
)";

DetectorInput MakeInput(const std::string& camera_id, uint64_t stamp_ns) {
  DetectorInput input;
  input.camera_id = camera_id;
  input.stamp_ns = stamp_ns;
  input.rgb = cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(0));
  return input;
}

TEST(MockDetector, ReturnsTheFixtureDetectionsForAMatchingFrame) {
  const TempYaml f(kFixture);
  MockDetector detector(LoadDetectionFixture(f.path()), TrackingConfig{});

  const DetectionSet set = detector.Detect(MakeInput("cam_front", 1'000'000'000ULL));
  EXPECT_EQ(set.camera_id, "cam_front");
  EXPECT_EQ(set.stamp_ns, 1'000'000'000ULL);
  ASSERT_EQ(set.detections.size(), 2u);
  EXPECT_EQ(set.detections[0].cls, "mustard_bottle");
  EXPECT_NEAR(set.detections[0].score, 0.93f, 1e-6);
  EXPECT_EQ(set.detections[0].bbox, cv::Rect2i(100, 80, 60, 120));
}

TEST(MockDetector, AssignsTrackIdsAndKeepsThemStableAcrossFrames) {
  const TempYaml f(kFixture);
  MockDetector detector(LoadDetectionFixture(f.path()), TrackingConfig{});

  const DetectionSet first = detector.Detect(MakeInput("cam_front", 1'000'000'000ULL));
  const DetectionSet second = detector.Detect(MakeInput("cam_front", 1'033'000'000ULL));

  ASSERT_EQ(second.detections.size(), 1u);
  EXPECT_NE(first.detections[0].track_id, 0u);
  EXPECT_EQ(second.detections[0].track_id, first.detections[0].track_id)
      << "the bottle barely moved between frames";
}

TEST(MockDetector, TrackIdsAreScopedPerCamera) {
  // INTERFACES.md §2: stable across frames WITHIN one camera. Sharing a tracker
  // between cameras would make an id mean two different things.
  const TempYaml f(kFixture);
  MockDetector detector(LoadDetectionFixture(f.path()), TrackingConfig{});

  const DetectionSet front = detector.Detect(MakeInput("cam_front", 1'000'000'000ULL));
  const DetectionSet over = detector.Detect(MakeInput("cam_over", 1'000'000'000ULL));

  ASSERT_FALSE(front.detections.empty());
  ASSERT_EQ(over.detections.size(), 1u);
  EXPECT_EQ(over.detections[0].cls, "bowl");
  // Both trackers start at 1 -- the ids are only meaningful alongside camera_id.
  EXPECT_EQ(over.detections[0].track_id, 1u);
}

TEST(MockDetector, UnmatchedFrameYieldsNothingRatherThanFabricatedDetections) {
  const TempYaml f(kFixture);
  MockDetector detector(LoadDetectionFixture(f.path()), TrackingConfig{});

  const DetectionSet set = detector.Detect(MakeInput("cam_front", 9'999'999'999ULL));
  EXPECT_TRUE(set.detections.empty());
  EXPECT_EQ(set.camera_id, "cam_front");
  EXPECT_EQ(set.stamp_ns, 9'999'999'999ULL);
}

TEST(MockDetector, FixtureLoaderRejectsMalformedEntries) {
  const TempYaml no_frames("something: else\n");
  EXPECT_THROW(LoadDetectionFixture(no_frames.path()), std::runtime_error);

  const TempYaml no_stamp("frames:\n  - camera_id: cam_front\n");
  EXPECT_THROW(LoadDetectionFixture(no_stamp.path()), std::runtime_error);

  const TempYaml bad_bbox(
      "frames:\n  - camera_id: cam_front\n    stamp_ns: 1\n    detections:\n"
      "      - cls: cube\n        bbox: [1, 2, 3]\n");
  EXPECT_THROW(LoadDetectionFixture(bad_bbox.path()), std::runtime_error);

  EXPECT_THROW(LoadDetectionFixture("/nonexistent/fixture.yaml"), std::runtime_error);
}

TEST(DetectorConfig, LoadsTheShippedFile) {
  const DetectorConfig cfg =
      LoadDetectorConfig(std::string(THE_EYES_CONFIG_DIR) + "/detector.yaml");
  EXPECT_EQ(cfg.model.input_size, 640);
  EXPECT_FALSE(cfg.model.class_names.empty());
  EXPECT_NEAR(cfg.postprocess.score_threshold, 0.25f, 1e-6);
  EXPECT_NEAR(cfg.postprocess.iou_threshold, 0.45f, 1e-6);
  EXPECT_EQ(cfg.postprocess.max_detections, 64);
  EXPECT_NEAR(cfg.tracking.iou_match_threshold, 0.30f, 1e-6);
}

TEST(DetectorConfig, LoadsTheRgbdDepthEncoding) {
  const DetectorConfig cfg =
      LoadDetectorConfig(std::string(THE_EYES_CONFIG_DIR) + "/detector.yaml");
  EXPECT_EQ(cfg.model.input_channels, 4) << "the deployed network is RGB-D";
  EXPECT_NEAR(cfg.model.depth.min_m, 0.20, 1e-9);
  EXPECT_NEAR(cfg.model.depth.max_m, 1.50, 1e-9);
  EXPECT_FLOAT_EQ(cfg.model.depth.valid_floor, 0.1f);
  EXPECT_FLOAT_EQ(cfg.model.depth.sentinel, 0.0f);
  EXPECT_NO_THROW(cfg.model.depth.Validate());
}

TEST(DetectorConfig, RejectsADepthEncodingThatDefeatsTheSentinel) {
  // Caught at load rather than at first inference: a sentinel inside the valid
  // band silently erases the distinction between "no data" and "very close".
  const TempYaml f(
      "model:\n  onnx_path: a.onnx\n  input_size: 640\n  input_channels: 4\n"
      "  class_names: [cube]\n"
      "depth:\n  min_m: 0.2\n  max_m: 1.5\n  valid_floor: 0.1\n  sentinel: 0.4\n"
      "inference:\n  provider: cpu\n"
      "postprocess:\n  score_threshold: 0.25\n  iou_threshold: 0.45\n  max_detections: 64\n"
      "tracking:\n  iou_match_threshold: 0.3\n  max_age_frames: 15\n");
  EXPECT_THROW(LoadDetectorConfig(f.path()), std::runtime_error);
}

TEST(DetectorConfig, ClassNamesMatchTheMeshRegistryKeys) {
  // The class string is the join key from the detector all the way to the pose
  // estimator. A name here with no mesh entry means every detection of it is
  // dropped at pose_agent -- silently, from the detector's point of view.
  const DetectorConfig cfg =
      LoadDetectorConfig(std::string(THE_EYES_CONFIG_DIR) + "/detector.yaml");
  const YAML::Node meshes =
      YAML::LoadFile(std::string(THE_EYES_CONFIG_DIR) + "/meshes.yaml")["objects"];
  ASSERT_TRUE(meshes && meshes.IsMap());

  for (const std::string& cls : cfg.model.class_names) {
    EXPECT_TRUE(meshes[cls].IsDefined())
        << "detector class '" << cls << "' has no entry in meshes.yaml";
  }
}

TEST(DetectorConfig, RejectsDuplicateClassNames) {
  const TempYaml f(
      "model:\n  onnx_path: a.onnx\n  input_size: 640\n  class_names: [cube, bowl, cube]\n"
      "inference:\n  provider: cpu\n"
      "postprocess:\n  score_threshold: 0.25\n  iou_threshold: 0.45\n  max_detections: 64\n"
      "tracking:\n  iou_match_threshold: 0.3\n  max_age_frames: 15\n");
  EXPECT_THROW(LoadDetectorConfig(f.path()), std::runtime_error);
}

TEST(DetectorConfig, RejectsAnUnknownProvider) {
  const TempYaml f(
      "model:\n  onnx_path: a.onnx\n  input_size: 640\n  class_names: [cube]\n"
      "inference:\n  provider: vulkan\n"
      "postprocess:\n  score_threshold: 0.25\n  iou_threshold: 0.45\n  max_detections: 64\n"
      "tracking:\n  iou_match_threshold: 0.3\n  max_age_frames: 15\n");
  EXPECT_THROW(LoadDetectorConfig(f.path()), std::runtime_error);
}

}  // namespace
}  // namespace the_eyes::detector
