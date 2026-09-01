// End-to-end pipeline test: per-view PoseEstimate -> fusion -> SceneState ->
// scenegraph -> PDDL (:init ...).
//
// This is MVP item 3 ("scene-graph bridge -> PDDL facts") exercised with zero
// hardware and zero models. It is deliberately owned at the integration level
// rather than by either module: AGENTS.md forbids an agent editing
// another agent's files, so the seam between them needs a test that belongs to
// neither.
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "the_eyes/fusion/fusion.hpp"
#include "the_eyes/scenegraph/pddl.hpp"
#include "the_eyes/scenegraph/predicates.hpp"

namespace the_eyes {
namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d Pose(const Eigen::Vector3d& t, double yaw_deg = 0.0) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() =
      Eigen::AngleAxisd(yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p.translation() = t;
  return p;
}

fusion::ExtrinsicsSet Rig() {
  fusion::ExtrinsicsSet ex;
  ex.world_frame = "table_origin";

  CameraExtrinsics front;
  front.camera_id = "cam_front";
  front.serial = "FRONT001";
  front.T_world_cam.linear() =
      (Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  front.T_world_cam.translation() = Eigen::Vector3d(0.60, 0.0, 0.40);
  front.reprojection_rmse_px = 0.12;
  ex.cameras.emplace(front.camera_id, front);

  CameraExtrinsics over;
  over.camera_id = "cam_over";
  over.serial = "OVER002";
  over.T_world_cam.linear() =
      Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  over.T_world_cam.translation() = Eigen::Vector3d(0.0, 0.0, 0.90);
  over.reprojection_rmse_px = 0.18;
  ex.cameras.emplace(over.camera_id, over);
  return ex;
}

PoseEstimate Observe(const fusion::ExtrinsicsSet& ex, const std::string& camera_id,
                     const std::string& cls, const Eigen::Isometry3d& world_pose,
                     const Eigen::Vector3d& half_extent, double score = 0.9) {
  PoseEstimate e;
  e.camera_id = camera_id;
  e.cls = cls;
  e.pose_cam = ex.at(camera_id).T_world_cam.inverse() * world_pose;
  e.score = score;
  e.extent = half_extent;
  return e;
}

bool Has(const std::vector<std::string>& facts, const std::string& fact) {
  return std::find(facts.begin(), facts.end(), fact) != facts.end();
}

// Table top at z = 0. A cube rests on it, visible only from cam_front. A bin
// sits on the table with a ball inside it, both cameras seeing both.
struct Scene {
  fusion::ExtrinsicsSet ex = Rig();
  Eigen::Isometry3d table = Pose({0.0, 0.0, -0.02});
  Eigen::Isometry3d cube = Pose({-0.10, 0.05, 0.025}, 20.0);
  Eigen::Isometry3d bin = Pose({0.30, 0.20, 0.06});
  Eigen::Isometry3d ball = Pose({0.30, 0.20, 0.03});

  Eigen::Vector3d table_extent{0.60, 0.40, 0.02};
  Eigen::Vector3d cube_extent{0.025, 0.025, 0.025};
  Eigen::Vector3d bin_extent{0.10, 0.10, 0.06};
  Eigen::Vector3d ball_extent{0.02, 0.02, 0.02};

  std::vector<PoseEstimate> Estimates() const {
    return {
        Observe(ex, "cam_front", "table", table, table_extent),
        Observe(ex, "cam_over", "table", table, table_extent),
        // The cube is occluded from the overhead view.
        Observe(ex, "cam_front", "cube", cube, cube_extent),
        Observe(ex, "cam_front", "bin", bin, bin_extent),
        Observe(ex, "cam_over", "bin", bin, bin_extent),
        Observe(ex, "cam_front", "ball", ball, ball_extent),
        Observe(ex, "cam_over", "ball", ball, ball_extent),
    };
  }
};

TEST(Pipeline, FusedSceneProducesTheExpectedPddlInitBlock) {
  const Scene scene;
  fusion::Fusion fuser(fusion::FusionConfig{}, scene.ex);

  const SceneState state = fuser.Process(scene.Estimates(), 1'000'000'000ULL);
  ASSERT_EQ(state.objects.size(), 4u);

  const scenegraph::PddlResult pddl =
      scenegraph::EmitInit(state, scenegraph::PredicateConfig{});

  EXPECT_TRUE(Has(pddl.facts, "(on cube_01 table_01)"));
  EXPECT_TRUE(Has(pddl.facts, "(on bin_01 table_01)"));
  EXPECT_TRUE(Has(pddl.facts, "(in ball_01 bin_01)"));
  EXPECT_TRUE(Has(pddl.facts, "(clear cube_01)"));
  EXPECT_TRUE(Has(pddl.facts, "(clear ball_01)"));
  EXPECT_TRUE(Has(pddl.facts, "(clear bin_01)"));
  EXPECT_FALSE(Has(pddl.facts, "(clear table_01)"));

  // A ball inside a bin is not on the table, even though the bin is thin enough
  // that the ball's base sits within the `on` tolerance of the table top.
  EXPECT_FALSE(Has(pddl.facts, "(on ball_01 table_01)"));

  EXPECT_TRUE(pddl.warnings.empty()) << pddl.warnings.front();
}

TEST(Pipeline, VisibilityBitsSurviveFusionAndBecomeOccludedFromFacts) {
  const Scene scene;
  fusion::Fusion fuser(fusion::FusionConfig{}, scene.ex);
  const SceneState state = fuser.Process(scene.Estimates(), 1'000'000'000ULL);

  for (const ObjectState& o : state.objects) {
    if (o.cls == "cube") {
      EXPECT_EQ(o.visible_views, kViewCamFront) << "cube is occluded from cam_over";
    } else {
      EXPECT_EQ(o.visible_views, kViewCamFront | kViewCamOver) << o.id;
    }
  }

  const scenegraph::PddlResult pddl =
      scenegraph::EmitInit(state, scenegraph::PredicateConfig{});
  EXPECT_TRUE(Has(pddl.facts, "(occluded_from cube_01 cam_over)"));
  EXPECT_FALSE(Has(pddl.facts, "(occluded_from cube_01 cam_front)"));
  EXPECT_FALSE(Has(pddl.facts, "(occluded_from ball_01 cam_over)"));
}

TEST(Pipeline, ExtentFlowsFromEstimateThroughFusionIntoPredicates) {
  // Predicates are pure geometry over ObjectState::extent. If extent were lost
  // in fusion, every object would have zero footprint and no `on` fact could
  // ever hold -- so this asserts the field survives the whole path.
  const Scene scene;
  fusion::Fusion fuser(fusion::FusionConfig{}, scene.ex);
  const SceneState state = fuser.Process(scene.Estimates(), 1'000'000'000ULL);

  for (const ObjectState& o : state.objects) {
    EXPECT_FALSE(o.extent.isZero()) << o.id << " lost its extent in fusion";
  }
  EXPECT_TRUE(fuser.diagnostics().warnings.empty());
}

TEST(Pipeline, FusionLeavesRelationsForScenegraphToFill) {
  const Scene scene;
  fusion::Fusion fuser(fusion::FusionConfig{}, scene.ex);
  SceneState state = fuser.Process(scene.Estimates(), 1'000'000'000ULL);

  for (const ObjectState& o : state.objects) EXPECT_TRUE(o.relations.empty());

  scenegraph::Annotate(state, scenegraph::PredicateConfig{});
  for (const ObjectState& o : state.objects) {
    // Every object is at least `clear` or `occluded_from` something, except the
    // table, which is neither clear nor occluded.
    if (o.cls != "table") {
      EXPECT_FALSE(o.relations.empty()) << o.id;
    }
  }
}

TEST(Pipeline, ObjectIdsAreStableAcrossFramesAndPddlNamesFollowThem) {
  const Scene scene;
  fusion::Fusion fuser(fusion::FusionConfig{}, scene.ex);

  const SceneState first = fuser.Process(scene.Estimates(), 1'000'000'000ULL);
  const scenegraph::PddlResult a = scenegraph::EmitInit(first, scenegraph::PredicateConfig{});

  const SceneState second = fuser.Process(scene.Estimates(), 1'033'000'000ULL);
  const scenegraph::PddlResult b = scenegraph::EmitInit(second, scenegraph::PredicateConfig{});

  // A static scene must yield an identical fact set frame to frame; churn here
  // would make PM1 replan every frame for no reason.
  EXPECT_EQ(a.facts, b.facts);
}

TEST(Pipeline, RefusesToRunOnUncalibratedExtrinsics) {
  // The gate is the first thing the pipeline hits: bad extrinsics poison every
  // fused pose, and every predicate derived from one.
  fusion::ExtrinsicsSet bad = Rig();
  bad.cameras.at("cam_over").reprojection_rmse_px = 3.0;
  EXPECT_THROW(fusion::Fusion(fusion::FusionConfig{}, bad), std::runtime_error);
}

}  // namespace
}  // namespace the_eyes
