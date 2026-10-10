#include <cstdlib>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "the_eyes/runtime/static_scene.hpp"
#include "the_eyes/scenegraph/pddl.hpp"

namespace {

using namespace the_eyes;
using namespace the_eyes::runtime;

std::string WriteTemp(const std::string& body) {
  char tmpl[] = "/tmp/the_eyes_scene_XXXXXX";
  const char* d = mkdtemp(tmpl);
  if (d == nullptr) throw std::runtime_error("mkdtemp failed");
  const std::string path = std::string(d) + "/scene.yaml";
  std::ofstream(path) << body;
  return path;
}

}  // namespace

TEST(StaticScene, LoadsTheShippedConfigAndInjectsATable) {
  const StaticScene scene = LoadStaticScene(std::string(THE_EYES_CONFIG_DIR) + "/scene.yaml");
  EXPECT_EQ(scene.world_frame, "table_origin");
  ASSERT_EQ(scene.objects.size(), 1u);
  EXPECT_EQ(scene.objects[0].id, "table_01");
  EXPECT_EQ(scene.objects[0].cls, "table");
  EXPECT_GT(scene.objects[0].extent.minCoeff(), 0.0);

  SceneState state;
  state.stamp_ns = 123;
  scene.Inject(state, {"cam_front", "cam_over"});
  ASSERT_EQ(state.objects.size(), 1u);
  const ObjectState& t = state.objects[0];
  EXPECT_EQ(t.visible_views, 0b11u);
  EXPECT_EQ(t.observed_views, 0b11u) << "a fixed object is hidden from no camera";
  EXPECT_DOUBLE_EQ(t.confidence, 1.0);
  EXPECT_EQ(t.stamp_ns, 123u);
  EXPECT_FALSE(has_uncertainty(t)) << "a static object has no estimator, so no sigma";
  EXPECT_LT(t.position_sigma_m, 0.0);
  EXPECT_TRUE(t.velocity_w.isZero());

  const scenegraph::PddlResult pddl = scenegraph::EmitInit(state, scenegraph::PredicateConfig{});
  EXPECT_TRUE(pddl.warnings.empty());
  bool clear = false;
  for (const std::string& f : pddl.facts) clear = clear || f == "(clear table_01)";
  EXPECT_TRUE(clear) << pddl.init_block;
}

TEST(StaticScene, ACubeRestingOnTheInjectedTableIsOnIt) {
  const StaticScene scene = LoadStaticScene(std::string(THE_EYES_CONFIG_DIR) + "/scene.yaml");
  SceneState state;
  ObjectState cube;
  cube.id = "red_cube_01";
  cube.cls = "red_cube";
  cube.pose_w.translation() = Eigen::Vector3d(0.05, -0.02, 0.025);   // base on z = 0
  cube.extent = Eigen::Vector3d(0.025, 0.025, 0.025);
  cube.confidence = 0.8;
  cube.visible_views = 0b11;
  state.objects.push_back(cube);
  scene.Inject(state, {"cam_front", "cam_over"});
  const scenegraph::PddlResult pddl = scenegraph::EmitInit(state, scenegraph::PredicateConfig{});
  bool on = false;
  for (const std::string& f : pddl.facts) on = on || f == "(on red_cube_01 table_01)";
  EXPECT_TRUE(on) << pddl.init_block;
}

TEST(StaticScene, RejectsBadConfigs) {
  EXPECT_THROW(LoadStaticScene(WriteTemp("objects: []\n")), std::runtime_error);
  EXPECT_THROW(LoadStaticScene(WriteTemp(
                   "world_frame: w\nobjects:\n  - {id: a, cls: c, pose_w: {t: [0,0,0]}, "
                   "extent: [0.1, 0.0, 0.1]}\n")),
               std::runtime_error);
  EXPECT_THROW(LoadStaticScene(WriteTemp(
                   "world_frame: w\nobjects:\n"
                   "  - {id: a, cls: c, pose_w: {t: [0,0,0]}, extent: [0.1, 0.1, 0.1]}\n"
                   "  - {id: a, cls: c, pose_w: {t: [0,0,0]}, extent: [0.1, 0.1, 0.1]}\n")),
               std::runtime_error);
  EXPECT_THROW(LoadStaticScene(WriteTemp(
                   "world_frame: w\nobjects:\n  - {id: a, cls: c, pose_w: {t: [0,0]}, "
                   "extent: [0.1, 0.1, 0.1]}\n")),
               std::runtime_error);

  const StaticScene ok = LoadStaticScene(WriteTemp(
      "world_frame: w\nobjects:\n  - {id: a, cls: c, pose_w: {t: [1,2,3], rpy_deg: [0,0,90]}, "
      "extent: [0.1, 0.2, 0.3]}\n"));
  EXPECT_TRUE(ok.objects[0].pose_w.translation().isApprox(Eigen::Vector3d(1, 2, 3)));
  EXPECT_TRUE((ok.objects[0].pose_w.linear() * Eigen::Vector3d::UnitX())
                  .isApprox(Eigen::Vector3d::UnitY(), 1e-12));

  SceneState state;
  ObjectState clash;
  clash.id = "a";
  clash.visible_views = 1;
  state.objects.push_back(clash);
  EXPECT_THROW(ok.Inject(state, {"cam_front"}), std::runtime_error);
}
