// pose_agent orchestration tests. AGENTS.md §4 acceptance criteria that
// need no hardware and no model: the mesh-registry join, ROI gating, latency
// accounting, and the camera-optical-frame contract.
#include "the_eyes/pose/pose_agent.hpp"

#include <memory>

#include <gtest/gtest.h>

#include "the_eyes/pose/mock_estimator.hpp"

namespace the_eyes::pose {
namespace {

constexpr int kWidth = 640;
constexpr int kHeight = 480;

Frame MakeFrame(const std::string& camera_id = "cam_front", uint64_t stamp_ns = 1'000'000'000ULL) {
  Frame frame;
  frame.camera_id = camera_id;
  frame.stamp_ns = stamp_ns;
  frame.rgb = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(20, 30, 40));
  frame.depth = cv::Mat(kHeight, kWidth, CV_16UC1, cv::Scalar(700));
  frame.depth_scale_m = 0.001;
  frame.K << 600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0;
  return frame;
}

Detection MakeDetection(const std::string& cls, uint32_t track_id = 1,
                        cv::Rect2i bbox = cv::Rect2i(200, 150, 120, 160)) {
  Detection d;
  d.cls = cls;
  d.score = 0.85f;
  d.bbox = bbox;
  d.track_id = track_id;
  return d;
}

MeshRegistry TestRegistry() {
  std::map<std::string, MeshEntry> entries;

  MeshEntry mustard;
  mustard.cls = "mustard_bottle";
  mustard.mesh_path = "assets/meshes/mustard_bottle.obj";
  mustard.symmetric = false;
  mustard.diameter_m = 0.196;
  mustard.extent = Eigen::Vector3d(0.048, 0.043, 0.093);
  entries.emplace(mustard.cls, mustard);

  MeshEntry bowl;
  bowl.cls = "bowl";
  bowl.mesh_path = "assets/meshes/bowl.obj";
  bowl.symmetric = true;
  bowl.diameter_m = 0.160;
  bowl.extent = Eigen::Vector3d(0.075, 0.075, 0.035);
  entries.emplace(bowl.cls, bowl);

  return MeshRegistry(std::move(entries));
}

PoseAgent MakeAgent(std::shared_ptr<PoseEstimator> estimator = nullptr) {
  if (estimator == nullptr) estimator = MockPoseEstimator::Identity(0.5, 0.9);
  return PoseAgent(TestRegistry(), PoseAgentConfig{}, std::move(estimator));
}

// ---------------------------------------------------------------------------
// The mesh-registry join -- INTERFACES.md §2
// ---------------------------------------------------------------------------

TEST(PoseAgent, DetectionWithNoRegisteredMeshIsDroppedWithAWarning) {
  PoseAgent agent = MakeAgent();
  const Frame frame = MakeFrame();

  DetectionSet set;
  set.camera_id = "cam_front";
  set.stamp_ns = frame.stamp_ns;
  set.detections = {MakeDetection("mustard_bottle"), MakeDetection("banana", 2)};

  const std::vector<PoseEstimate> out = agent.Process(set, frame);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].cls, "mustard_bottle");
  EXPECT_EQ(agent.diagnostics().dropped_no_mesh, 1);
  ASSERT_FALSE(agent.diagnostics().warnings.empty());
  EXPECT_NE(agent.diagnostics().warnings[0].find("banana"), std::string::npos);
}

TEST(PoseAgent, UnregisteredClassNeverReachesTheEstimator) {
  // The contract says it "must not reach the pose estimator", which is stronger
  // than "is absent from the output". This asserts the stronger thing.
  struct CountingEstimator : PoseEstimator {
    int calls = 0;
    std::optional<PoseEstimate> Estimate(const PoseRequest&) override {
      ++calls;
      return std::nullopt;
    }
    std::string name() const override { return "counting"; }
  };
  auto counting = std::make_shared<CountingEstimator>();
  PoseAgent agent = MakeAgent(counting);

  DetectionSet set;
  set.camera_id = "cam_front";
  set.detections = {MakeDetection("banana"), MakeDetection("kiwi", 2)};
  agent.Process(set, MakeFrame());

  EXPECT_EQ(counting->calls, 0);
}

// ---------------------------------------------------------------------------
// The §3 contract on what comes out
// ---------------------------------------------------------------------------

TEST(PoseAgent, EstimateCarriesMeshRegistrySymmetryAndExtent) {
  // Fusion must not average a symmetric object's orientation, and it fills
  // ObjectState::extent from here -- neither may come from the estimator's own
  // opinion.
  PoseAgent agent = MakeAgent();
  DetectionSet set;
  set.camera_id = "cam_front";
  set.detections = {MakeDetection("bowl")};

  const std::vector<PoseEstimate> out = agent.Process(set, MakeFrame());
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(out[0].symmetric);
  EXPECT_LT((out[0].extent - Eigen::Vector3d(0.075, 0.075, 0.035)).norm(), 1e-12);
}

TEST(PoseAgent, StampComesFromTheSourceFrameNotReceiveTime) {
  PoseAgent agent = MakeAgent();
  const Frame frame = MakeFrame("cam_front", 4'242'000'000ULL);
  DetectionSet set;
  set.camera_id = "cam_front";
  set.stamp_ns = 4'242'000'000ULL;
  set.detections = {MakeDetection("mustard_bottle")};

  const std::vector<PoseEstimate> out = agent.Process(set, frame);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].stamp_ns, 4'242'000'000ULL);
}

TEST(PoseAgent, PoseIsInTheCameraOpticalFrame) {
  // The identity mock places the object 0.5 m along +z, the optical axis. If
  // anything here applied a world transform, this would not hold.
  PoseAgent agent = MakeAgent(MockPoseEstimator::Identity(0.5, 0.9));
  DetectionSet set;
  set.camera_id = "cam_front";
  set.detections = {MakeDetection("mustard_bottle")};

  const std::vector<PoseEstimate> out = agent.Process(set, MakeFrame());
  ASSERT_EQ(out.size(), 1u);
  EXPECT_LT((out[0].pose_cam.translation() - Eigen::Vector3d(0, 0, 0.5)).norm(), 1e-12);
  EXPECT_TRUE(out[0].pose_cam.linear().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(PoseAgent, CameraIdIsCopiedFromTheFrame) {
  PoseAgent agent = MakeAgent();
  DetectionSet set;
  set.camera_id = "cam_over";
  set.detections = {MakeDetection("mustard_bottle")};

  const std::vector<PoseEstimate> out = agent.Process(set, MakeFrame("cam_over"));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].camera_id, "cam_over");
}

// ---------------------------------------------------------------------------
// Wiring errors
// ---------------------------------------------------------------------------

TEST(PoseAgent, MismatchedCameraIdThrows) {
  // Proceeding would attribute poses to the wrong camera, which fusion cannot
  // detect and which poisons every world-frame transform downstream.
  PoseAgent agent = MakeAgent();
  DetectionSet set;
  set.camera_id = "cam_over";
  set.detections = {MakeDetection("mustard_bottle")};
  EXPECT_THROW(agent.Process(set, MakeFrame("cam_front")), std::invalid_argument);
}

TEST(PoseAgent, InvalidFrameThrows) {
  PoseAgent agent = MakeAgent();
  DetectionSet set;
  set.camera_id = "cam_front";
  set.detections = {MakeDetection("mustard_bottle")};

  Frame empty;
  empty.camera_id = "cam_front";
  EXPECT_THROW(agent.Process(set, empty), std::invalid_argument);

  // Depth not aligned to colour: different sizes.
  Frame unaligned = MakeFrame();
  unaligned.depth = cv::Mat(240, 320, CV_16UC1, cv::Scalar(700));
  EXPECT_THROW(agent.Process(set, unaligned), std::invalid_argument);
}

TEST(PoseAgent, StampSkewBeyondOneFramePeriodWarnsButProceeds) {
  // Sync jitter is real; dropping everything would be worse than flagging it.
  PoseAgent agent = MakeAgent();
  DetectionSet set;
  set.camera_id = "cam_front";
  set.stamp_ns = 1'000'000'000ULL;
  set.detections = {MakeDetection("mustard_bottle")};

  const std::vector<PoseEstimate> out = agent.Process(set, MakeFrame("cam_front", 1'200'000'000ULL));
  EXPECT_EQ(out.size(), 1u);
  ASSERT_FALSE(agent.diagnostics().warnings.empty());
  EXPECT_NE(agent.diagnostics().warnings[0].find("ms apart"), std::string::npos);
}

// ---------------------------------------------------------------------------
// Gating
// ---------------------------------------------------------------------------

TEST(PoseAgent, TinyDetectionIsDroppedBeforeEstimation) {
  PoseAgent agent = MakeAgent();
  DetectionSet set;
  set.camera_id = "cam_front";
  set.detections = {MakeDetection("mustard_bottle", 1, cv::Rect2i(300, 300, 6, 6))};

  const std::vector<PoseEstimate> out = agent.Process(set, MakeFrame());
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(agent.diagnostics().dropped_roi_too_small, 1);
}

TEST(PoseAgent, LowScoreEstimateIsDropped) {
  // Fusion weights by score but does not threshold, so a near-zero-confidence
  // pose still drags the fused estimate toward itself.
  PoseAgent agent = MakeAgent(MockPoseEstimator::Identity(0.5, /*score=*/0.02));
  DetectionSet set;
  set.camera_id = "cam_front";
  set.detections = {MakeDetection("mustard_bottle")};

  const std::vector<PoseEstimate> out = agent.Process(set, MakeFrame());
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(agent.diagnostics().dropped_low_score, 1);
}

TEST(PoseAgent, DeclinedHypothesisIsCountedNotAnError) {
  struct Declining : PoseEstimator {
    std::optional<PoseEstimate> Estimate(const PoseRequest&) override { return std::nullopt; }
    std::string name() const override { return "declining"; }
  };
  PoseAgent agent = MakeAgent(std::make_shared<Declining>());
  DetectionSet set;
  set.camera_id = "cam_front";
  set.detections = {MakeDetection("mustard_bottle")};

  EXPECT_NO_THROW(agent.Process(set, MakeFrame()));
  EXPECT_EQ(agent.diagnostics().declined_by_estimator, 1);
}

// ---------------------------------------------------------------------------
// Latency accounting -- the R4 scheduling input
// ---------------------------------------------------------------------------

TEST(PoseAgent, LatencyIsRecordedPerEstimationAcrossFrames) {
  PoseAgent agent = MakeAgent();
  DetectionSet set;
  set.camera_id = "cam_front";
  set.detections = {MakeDetection("mustard_bottle", 1),
                    MakeDetection("bowl", 2, cv::Rect2i(50, 50, 100, 100))};

  agent.Process(set, MakeFrame("cam_front", 1'000'000'000ULL));
  agent.Process(set, MakeFrame("cam_front", 1'033'000'000ULL));

  EXPECT_EQ(agent.latency().count(), 4u);  // 2 objects x 2 frames
  EXPECT_GE(agent.latency().max_ms(), agent.latency().percentile_ms(0.5));
  EXPECT_NE(agent.latency().Format("pose").find("p95"), std::string::npos);

  agent.ResetLatency();
  EXPECT_EQ(agent.latency().count(), 0u);
}

TEST(LatencyStats, PercentilesAndFormattingAreSane) {
  LatencyStats stats;
  for (int i = 1; i <= 100; ++i) stats.Record(static_cast<double>(i));
  EXPECT_EQ(stats.count(), 100u);
  EXPECT_NEAR(stats.mean_ms(), 50.5, 1e-9);
  EXPECT_NEAR(stats.percentile_ms(0.5), 50.0, 1.0);
  EXPECT_NEAR(stats.percentile_ms(0.95), 95.0, 1.0);
  EXPECT_NEAR(stats.max_ms(), 100.0, 1e-9);

  LatencyStats empty;
  EXPECT_EQ(empty.mean_ms(), 0.0);
  EXPECT_NE(empty.Format("x").find("no samples"), std::string::npos);
  // A negative or non-finite sample is a clock bug, not a measurement.
  empty.Record(-5.0);
  empty.Record(std::nan(""));
  EXPECT_EQ(empty.count(), 0u);
}

// ---------------------------------------------------------------------------
// Mock replay -- the mode fusion_agent develops against
// ---------------------------------------------------------------------------

TEST(PoseAgent, ReplayMockReturnsRecordedGroundTruth) {
  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.translation() = Eigen::Vector3d(0.03, -0.02, 0.62);

  std::vector<GroundTruthPose> table;
  table.push_back({"cam_front", "mustard_bottle", 1'000'000'000ULL, 7, truth});

  PoseAgent agent = MakeAgent(MockPoseEstimator::Replay(table));
  DetectionSet set;
  set.camera_id = "cam_front";
  set.detections = {MakeDetection("mustard_bottle", 7)};

  const std::vector<PoseEstimate> out = agent.Process(set, MakeFrame("cam_front", 1'000'000'000ULL));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_LT((out[0].pose_cam.translation() - truth.translation()).norm(), 1e-12);
}

TEST(PoseAgent, ReplayMockDeclinesWhenGroundTruthHasNoMatch) {
  // Fabricating a pose for an unmatched request would silently invent data in
  // exactly the mode used to generate evaluation inputs.
  std::vector<GroundTruthPose> table;
  table.push_back({"cam_front", "mustard_bottle", 1'000'000'000ULL, 7,
                   Eigen::Isometry3d::Identity()});

  PoseAgent agent = MakeAgent(MockPoseEstimator::Replay(table));
  DetectionSet set;
  set.camera_id = "cam_front";
  set.detections = {MakeDetection("mustard_bottle", /*track_id=*/99)};

  EXPECT_TRUE(agent.Process(set, MakeFrame("cam_front", 1'000'000'000ULL)).empty());
  EXPECT_EQ(agent.diagnostics().declined_by_estimator, 1);
}

TEST(PoseAgent, ReplayNoiseIsReproducibleFromTheSeed) {
  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.translation() = Eigen::Vector3d(0.0, 0.0, 0.6);
  std::vector<GroundTruthPose> table;
  for (int i = 0; i < 20; ++i) {
    table.push_back({"cam_front", "mustard_bottle",
                     1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL, 1, truth});
  }

  MockNoise noise;
  noise.translation_sigma_m = 0.01;
  noise.seed = 12345;

  const auto run = [&]() {
    PoseAgent agent = MakeAgent(MockPoseEstimator::Replay(table, noise));
    std::vector<Eigen::Vector3d> positions;
    for (int i = 0; i < 20; ++i) {
      DetectionSet set;
      set.camera_id = "cam_front";
      set.detections = {MakeDetection("mustard_bottle", 1)};
      const auto out = agent.Process(
          set, MakeFrame("cam_front", 1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL));
      if (!out.empty()) positions.push_back(out[0].pose_cam.translation());
    }
    return positions;
  };

  const auto a = run();
  const auto b = run();
  ASSERT_EQ(a.size(), 20u);
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) EXPECT_LT((a[i] - b[i]).norm(), 1e-15);
  // ...and the noise is actually applied.
  EXPECT_GT((a[0] - a[1]).norm(), 0.0);
}

TEST(PoseAgent, ReplayDropoutProducesMissingViews) {
  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.translation() = Eigen::Vector3d(0.0, 0.0, 0.6);
  std::vector<GroundTruthPose> table;
  for (int i = 0; i < 200; ++i) {
    table.push_back({"cam_front", "mustard_bottle",
                     1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL, 1, truth});
  }
  MockNoise noise;
  noise.dropout_probability = 0.5;

  PoseAgent agent = MakeAgent(MockPoseEstimator::Replay(table, noise));
  int produced = 0;
  for (int i = 0; i < 200; ++i) {
    DetectionSet set;
    set.camera_id = "cam_front";
    set.detections = {MakeDetection("mustard_bottle", 1)};
    produced += static_cast<int>(agent.Process(
        set, MakeFrame("cam_front",
                       1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL)).size());
  }
  EXPECT_GT(produced, 50);
  EXPECT_LT(produced, 150);
}

// ---------------------------------------------------------------------------
// Determinism and config
// ---------------------------------------------------------------------------

TEST(PoseAgent, OutputOrderIsDeterministicRegardlessOfDetectorOrder) {
  PoseAgent agent = MakeAgent();
  const Frame frame = MakeFrame();

  DetectionSet forward;
  forward.camera_id = "cam_front";
  forward.detections = {MakeDetection("mustard_bottle", 2),
                        MakeDetection("bowl", 1, cv::Rect2i(50, 50, 100, 100)),
                        MakeDetection("mustard_bottle", 1, cv::Rect2i(400, 200, 90, 90))};

  DetectionSet reversed = forward;
  std::reverse(reversed.detections.begin(), reversed.detections.end());

  const auto a = agent.Process(forward, frame);
  const auto b = agent.Process(reversed, frame);
  ASSERT_EQ(a.size(), 3u);
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].cls, b[i].cls);
    EXPECT_EQ(a[i].track_id, b[i].track_id);
  }
}

TEST(PoseAgent, ShippedConfigLoadsAndMatchesInCodeDefaults) {
  const PoseAgentConfig cfg =
      LoadPoseAgentConfig(std::string(THE_EYES_CONFIG_DIR) + "/pose.yaml");
  const PoseAgentConfig defaults;
  EXPECT_DOUBLE_EQ(cfg.roi.margin_ratio, defaults.roi.margin_ratio);
  EXPECT_EQ(cfg.roi.min_size_px, defaults.roi.min_size_px);
  EXPECT_DOUBLE_EQ(cfg.min_score, defaults.min_score);
}

TEST(PoseAgent, NullEstimatorIsRejectedAtConstruction) {
  EXPECT_THROW(PoseAgent(TestRegistry(), PoseAgentConfig{}, nullptr), std::invalid_argument);
}

}  // namespace
}  // namespace the_eyes::pose
