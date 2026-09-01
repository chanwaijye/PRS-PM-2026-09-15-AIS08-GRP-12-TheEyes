// The whole contract chain, with no hardware and no models:
//
//   DetectionSet (§2) -> pose_agent x2 -> PoseEstimate (§3) -> fusion
//     -> SceneState (§5) -> scenegraph -> PDDL (:init ...) (§6)
//
// The earlier test_pipeline.cpp starts at fusion with hand-built PoseEstimates.
// This one starts at the detector boundary and runs every module in between, so
// a contract mismatch anywhere in the chain shows up here rather than at
// integration time on real hardware.
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "the_eyes/fusion/fusion.hpp"
#include "the_eyes/fusion/metrics.hpp"
#include "the_eyes/pose/mock_estimator.hpp"
#include "the_eyes/pose/pose_agent.hpp"
#include "the_eyes/scenegraph/pddl.hpp"

namespace the_eyes {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kWidth = 640;
constexpr int kHeight = 480;

Eigen::Isometry3d Pose(const Eigen::Vector3d& t, double yaw_deg = 0.0) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = Eigen::AngleAxisd(yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p.translation() = t;
  return p;
}

// One object as the scene defines it, before any module has seen it.
struct SceneObject {
  std::string cls;
  uint32_t track_id;
  Eigen::Isometry3d pose_w;
  Eigen::Vector3d extent;
  double diameter_m;
  bool symmetric;
};

fusion::ExtrinsicsSet Rig() {
  fusion::ExtrinsicsSet ex;
  ex.world_frame = "table_origin";

  CameraExtrinsics front;
  front.camera_id = "cam_front";
  front.serial = "FRONT001";
  front.T_world_cam.linear() = (Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitX()))
                                   .toRotationMatrix();
  front.T_world_cam.translation() = Eigen::Vector3d(0.60, 0.0, 0.40);
  front.reprojection_rmse_px = 0.12;
  ex.cameras.emplace(front.camera_id, front);

  CameraExtrinsics over;
  over.camera_id = "cam_over";
  over.serial = "OVER002";
  over.T_world_cam.linear() = Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  over.T_world_cam.translation() = Eigen::Vector3d(0.0, 0.0, 0.90);
  over.reprojection_rmse_px = 0.18;
  ex.cameras.emplace(over.camera_id, over);
  return ex;
}

std::vector<SceneObject> TableScene() {
  return {
      {"table", 1, Pose({0.0, 0.0, -0.02}), {0.60, 0.40, 0.02}, 1.44, false},
      {"mustard_bottle", 2, Pose({-0.10, 0.05, 0.093}, 20.0), {0.048, 0.043, 0.093}, 0.196, false},
      {"bin", 3, Pose({0.30, 0.20, 0.06}), {0.10, 0.10, 0.06}, 0.32, true},
      {"ball", 4, Pose({0.30, 0.20, 0.03}), {0.02, 0.02, 0.02}, 0.069, true},
  };
}

pose::MeshRegistry RegistryFor(const std::vector<SceneObject>& scene) {
  std::map<std::string, MeshEntry> entries;
  for (const SceneObject& o : scene) {
    MeshEntry entry;
    entry.cls = o.cls;
    entry.mesh_path = "assets/meshes/" + o.cls + ".obj";
    entry.symmetric = o.symmetric;
    entry.diameter_m = o.diameter_m;
    entry.extent = o.extent;
    entries.emplace(o.cls, entry);
  }
  return pose::MeshRegistry(std::move(entries));
}

pose::Frame MakeFrame(const std::string& camera_id, uint64_t stamp_ns) {
  pose::Frame frame;
  frame.camera_id = camera_id;
  frame.stamp_ns = stamp_ns;
  frame.rgb = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(20, 30, 40));
  frame.depth = cv::Mat(kHeight, kWidth, CV_16UC1, cv::Scalar(700));
  frame.depth_scale_m = 0.001;
  frame.K << 600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0;
  return frame;
}

// Stands in for the detector: one plausible box per object, spread across the
// frame so none is gated out for being too small.
DetectionSet MakeDetections(const std::vector<SceneObject>& scene,
                            const std::string& camera_id, uint64_t stamp_ns) {
  DetectionSet set;
  set.camera_id = camera_id;
  set.stamp_ns = stamp_ns;
  int column = 0;
  for (const SceneObject& o : scene) {
    Detection d;
    d.cls = o.cls;
    d.score = 0.9f;
    d.track_id = o.track_id;
    d.bbox = cv::Rect2i(40 + column * 130, 120, 90, 110);
    set.detections.push_back(d);
    ++column;
  }
  return set;
}

// Ground truth as each camera would see it: world pose pushed into that
// camera's optical frame, which is what a pose estimator reports (§3).
std::vector<pose::GroundTruthPose> GroundTruthTable(const std::vector<SceneObject>& scene,
                                                    const fusion::ExtrinsicsSet& ex,
                                                    const std::vector<uint64_t>& stamps,
                                                    const std::vector<std::string>& cameras) {
  std::vector<pose::GroundTruthPose> table;
  for (const std::string& camera_id : cameras) {
    const Eigen::Isometry3d T_cam_world = ex.at(camera_id).T_world_cam.inverse();
    for (uint64_t stamp : stamps) {
      for (const SceneObject& o : scene) {
        table.push_back({camera_id, o.cls, stamp, o.track_id, T_cam_world * o.pose_w});
      }
    }
  }
  return table;
}

// Runs one frame through both cameras' pose agents and returns the combined
// per-view estimates, exactly as fusion would receive them.
std::vector<PoseEstimate> RunPoseAgents(pose::PoseAgent& front, pose::PoseAgent& over,
                                        const std::vector<SceneObject>& scene, uint64_t stamp) {
  std::vector<PoseEstimate> all;
  const std::pair<pose::PoseAgent*, std::string> agents[] = {{&front, "cam_front"},
                                                             {&over, "cam_over"}};
  for (const auto& entry : agents) {
    const auto out = entry.first->Process(MakeDetections(scene, entry.second, stamp),
                                          MakeFrame(entry.second, stamp));
    all.insert(all.end(), out.begin(), out.end());
  }
  return all;
}

bool Has(const std::vector<std::string>& facts, const std::string& fact) {
  return std::find(facts.begin(), facts.end(), fact) != facts.end();
}

// ---------------------------------------------------------------------------

TEST(FullPipeline, DetectionsBecomePddlFactsThroughEveryModule) {
  const std::vector<SceneObject> scene = TableScene();
  const fusion::ExtrinsicsSet ex = Rig();
  const uint64_t stamp = 1'000'000'000ULL;

  const auto table = GroundTruthTable(scene, ex, {stamp}, {"cam_front", "cam_over"});
  auto estimator = pose::MockPoseEstimator::Replay(table);

  pose::PoseAgent front(RegistryFor(scene), pose::PoseAgentConfig{}, estimator);
  pose::PoseAgent over(RegistryFor(scene), pose::PoseAgentConfig{}, estimator);

  const std::vector<PoseEstimate> estimates = RunPoseAgents(front, over, scene, stamp);
  ASSERT_EQ(estimates.size(), scene.size() * 2) << "every object seen by both cameras";

  fusion::Fusion fuser(fusion::FusionConfig{}, ex);
  const SceneState state = fuser.Process(estimates, stamp);
  ASSERT_EQ(state.objects.size(), scene.size());

  const scenegraph::PddlResult pddl =
      scenegraph::EmitInit(state, scenegraph::PredicateConfig{});

  EXPECT_TRUE(Has(pddl.facts, "(on mustard_bottle_01 table_01)"));
  EXPECT_TRUE(Has(pddl.facts, "(on bin_01 table_01)"));
  EXPECT_TRUE(Has(pddl.facts, "(in ball_01 bin_01)"));
  EXPECT_FALSE(Has(pddl.facts, "(on ball_01 table_01)")) << "containment is exclusive";
  EXPECT_FALSE(Has(pddl.facts, "(clear table_01)"));
  EXPECT_TRUE(pddl.warnings.empty());
}

TEST(FullPipeline, ExtentAndSymmetryTravelFromTheMeshRegistryToTheSceneGraph) {
  // The registry is the only source of both. If either were dropped at a module
  // boundary, predicates would degenerate (extent) or fusion would average a
  // symmetric object's orientation (symmetric).
  const std::vector<SceneObject> scene = TableScene();
  const fusion::ExtrinsicsSet ex = Rig();
  const uint64_t stamp = 1'000'000'000ULL;

  auto estimator = pose::MockPoseEstimator::Replay(
      GroundTruthTable(scene, ex, {stamp}, {"cam_front", "cam_over"}));
  pose::PoseAgent front(RegistryFor(scene), pose::PoseAgentConfig{}, estimator);
  pose::PoseAgent over(RegistryFor(scene), pose::PoseAgentConfig{}, estimator);

  const std::vector<PoseEstimate> estimates = RunPoseAgents(front, over, scene, stamp);
  for (const PoseEstimate& e : estimates) {
    EXPECT_FALSE(e.extent.isZero()) << e.cls << " lost its extent in pose_agent";
  }
  const auto bin = std::find_if(estimates.begin(), estimates.end(),
                                [](const PoseEstimate& e) { return e.cls == "bin"; });
  ASSERT_NE(bin, estimates.end());
  EXPECT_TRUE(bin->symmetric);

  fusion::Fusion fuser(fusion::FusionConfig{}, ex);
  const SceneState state = fuser.Process(estimates, stamp);
  for (const ObjectState& o : state.objects) {
    EXPECT_FALSE(o.extent.isZero()) << o.id << " lost its extent in fusion";
  }
  EXPECT_TRUE(fuser.diagnostics().warnings.empty());
}

TEST(FullPipeline, DropoutInOneViewSurfacesAsAnOccludedFromFact) {
  // The end-to-end version of the occlusion path: the pose estimator declines
  // for one camera, and that has to travel through fusion's visibility bitmask
  // into a symbolic fact PM1 can plan around.
  const std::vector<SceneObject> scene = TableScene();
  const fusion::ExtrinsicsSet ex = Rig();
  const uint64_t stamp = 1'000'000'000ULL;

  // Ground truth for cam_over deliberately omits the mustard bottle, so that
  // view alone cannot resolve it.
  std::vector<pose::GroundTruthPose> table =
      GroundTruthTable(scene, ex, {stamp}, {"cam_front"});
  for (const SceneObject& o : scene) {
    if (o.cls == "mustard_bottle") continue;
    table.push_back({"cam_over", o.cls, stamp, o.track_id,
                     ex.at("cam_over").T_world_cam.inverse() * o.pose_w});
  }

  auto estimator = pose::MockPoseEstimator::Replay(table);
  pose::PoseAgent front(RegistryFor(scene), pose::PoseAgentConfig{}, estimator);
  pose::PoseAgent over(RegistryFor(scene), pose::PoseAgentConfig{}, estimator);

  fusion::Fusion fuser(fusion::FusionConfig{}, ex);
  const SceneState state = fuser.Process(RunPoseAgents(front, over, scene, stamp), stamp);

  const auto mustard = std::find_if(state.objects.begin(), state.objects.end(),
                                    [](const ObjectState& o) { return o.cls == "mustard_bottle"; });
  ASSERT_NE(mustard, state.objects.end());
  EXPECT_EQ(mustard->visible_views, kViewCamFront);

  const scenegraph::PddlResult pddl =
      scenegraph::EmitInit(state, scenegraph::PredicateConfig{});
  EXPECT_TRUE(Has(pddl.facts, "(occluded_from mustard_bottle_01 cam_over)"));
  EXPECT_FALSE(Has(pddl.facts, "(occluded_from mustard_bottle_01 cam_front)"));
  EXPECT_FALSE(Has(pddl.facts, "(occluded_from bin_01 cam_over)"));
}

TEST(FullPipeline, UnregisteredClassIsDroppedAtPoseAgentAndNeverReachesTheSceneGraph) {
  const std::vector<SceneObject> scene = TableScene();
  const fusion::ExtrinsicsSet ex = Rig();
  const uint64_t stamp = 1'000'000'000ULL;

  auto estimator = pose::MockPoseEstimator::Replay(
      GroundTruthTable(scene, ex, {stamp}, {"cam_front", "cam_over"}));
  // Registry deliberately missing the bin.
  std::vector<SceneObject> registered;
  for (const SceneObject& o : scene) {
    if (o.cls != "bin") registered.push_back(o);
  }
  pose::PoseAgent front(RegistryFor(registered), pose::PoseAgentConfig{}, estimator);
  pose::PoseAgent over(RegistryFor(registered), pose::PoseAgentConfig{}, estimator);

  const std::vector<PoseEstimate> estimates = RunPoseAgents(front, over, scene, stamp);
  for (const PoseEstimate& e : estimates) EXPECT_NE(e.cls, "bin");
  EXPECT_EQ(front.diagnostics().dropped_no_mesh, 1);

  fusion::Fusion fuser(fusion::FusionConfig{}, ex);
  const SceneState state = fuser.Process(estimates, stamp);
  const scenegraph::PddlResult pddl =
      scenegraph::EmitInit(state, scenegraph::PredicateConfig{});

  for (const std::string& fact : pddl.facts) {
    EXPECT_EQ(fact.find("bin"), std::string::npos) << fact;
  }
  // With no bin, the ball is no longer contained -- it rests on the table.
  EXPECT_TRUE(Has(pddl.facts, "(on ball_01 table_01)"));
}

TEST(FullPipeline, FusedErrorBeatsSingleViewThroughTheRealModuleChain) {
  // fusion's own tests prove this against hand-built PoseEstimates. This proves
  // it survives pose_agent's ROI gating, registry join, and ordering too.
  const std::vector<SceneObject> scene = {
      {"mustard_bottle", 2, Pose({-0.10, 0.05, 0.093}, 20.0), {0.048, 0.043, 0.093}, 0.196, false}};
  const fusion::ExtrinsicsSet ex = Rig();

  std::vector<uint64_t> stamps;
  for (int i = 0; i < 120; ++i) {
    stamps.push_back(1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL);
  }

  pose::MockNoise noise;
  noise.translation_sigma_m = 0.010;
  noise.seed = 20260820;

  fusion::ModelRegistry models;
  models["mustard_bottle"] = fusion::ModelEntry{
      fusion::BoxModelPoints(scene[0].extent, 4), scene[0].diameter_m};

  auto estimator = pose::MockPoseEstimator::Replay(
      GroundTruthTable(scene, ex, stamps, {"cam_front", "cam_over"}), noise);
  pose::PoseAgent front(RegistryFor(scene), pose::PoseAgentConfig{}, estimator);
  pose::PoseAgent over(RegistryFor(scene), pose::PoseAgentConfig{}, estimator);

  std::vector<fusion::AblationFrame> frames;
  for (uint64_t stamp : stamps) {
    fusion::AblationFrame frame;
    frame.stamp_ns = stamp;
    frame.estimates = RunPoseAgents(front, over, scene, stamp);
    frame.ground_truth.push_back({scene[0].cls, scene[0].pose_w});
    frames.push_back(std::move(frame));
  }

  const fusion::AblationResult result =
      fusion::RunOcclusionAblation(frames, ex, fusion::FusionConfig{}, {}, models);
  ASSERT_EQ(result.rows.size(), 3u);

  const auto row = [&](const std::string& label) {
    return *std::find_if(result.rows.begin(), result.rows.end(),
                         [&](const fusion::AblationRow& r) { return r.label == label; });
  };
  EXPECT_LT(row("fused").mean_add_s_m, row("cam_front").mean_add_s_m);
  EXPECT_LT(row("fused").mean_add_s_m, row("cam_over").mean_add_s_m);
  EXPECT_EQ(row("fused").missed, 0);
}

}  // namespace
}  // namespace the_eyes
