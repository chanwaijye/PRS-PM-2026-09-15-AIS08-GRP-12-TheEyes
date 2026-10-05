#include "the_eyes/scenegraph/pddl.hpp"

#include <gtest/gtest.h>

namespace the_eyes::scenegraph {
namespace {

ObjectState Make(const std::string& id, const Eigen::Vector3d& centre,
                 const Eigen::Vector3d& half_extent,
                 uint32_t views = kViewCamFront | kViewCamOver) {
  ObjectState o;
  o.id = id;
  o.cls = "cube";
  o.pose_w.translation() = centre;
  o.extent = half_extent;
  o.visible_views = views;
  return o;
}

TEST(Pddl, SanitizeAcceptsWellFormedIdsUnchanged) {
  bool changed = true;
  EXPECT_EQ(SanitizePddlName("red_cube_01", &changed), "red_cube_01");
  EXPECT_FALSE(changed);

  EXPECT_EQ(SanitizePddlName("mustard-bottle", &changed), "mustard-bottle");
  EXPECT_FALSE(changed);
}

TEST(Pddl, SanitizeRewritesIllegalCharactersAndLeadingDigits) {
  bool changed = false;
  EXPECT_EQ(SanitizePddlName("red cube (1)", &changed), "red_cube__1_");
  EXPECT_TRUE(changed);

  changed = false;
  EXPECT_EQ(SanitizePddlName("01_cube", &changed), "o01_cube");
  EXPECT_TRUE(changed);

  changed = false;
  EXPECT_EQ(SanitizePddlName("", &changed), "obj");
  EXPECT_TRUE(changed);
}

TEST(Pddl, InitBlockIsWellFormedAndSorted) {
  SceneState scene;
  scene.objects = {
      Make("table_01", {0.0, 0.0, -0.02}, {0.60, 0.40, 0.02}),
      Make("red_cube_01", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025}, kViewCamFront),
  };

  const PddlResult r = EmitInit(scene, PredicateConfig{});
  EXPECT_EQ(r.init_block,
            "(:init\n"
            "  (clear red_cube_01)\n"
            "  (occluded_from red_cube_01 cam_over)\n"
            "  (on red_cube_01 table_01)\n"
            ")");
  EXPECT_TRUE(r.warnings.empty());

  // Balanced parentheses -- the cheapest proxy for "PM1 can parse this".
  int depth = 0;
  for (const char c : r.init_block) {
    if (c == '(') ++depth;
    if (c == ')') --depth;
    ASSERT_GE(depth, 0);
  }
  EXPECT_EQ(depth, 0);
}

TEST(Pddl, IllegalIdsAreRenamedAndReported) {
  SceneState scene;
  scene.objects = {Make("red cube!", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025})};

  const PddlResult r = EmitInit(scene, PredicateConfig{});
  EXPECT_EQ(r.facts, (std::vector<std::string>{"(clear red_cube_)"}));
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("renamed"), std::string::npos);
}

TEST(Pddl, SanitizingCollisionsAreDisambiguatedNotMerged) {
  SceneState scene;
  // Two distinct fusion instances that sanitize onto the same PDDL name.
  scene.objects = {
      Make("cube 1", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025}),
      Make("cube+1", {0.5, 0.0, 0.025}, {0.025, 0.025, 0.025}),
  };

  const PddlResult r = EmitInit(scene, PredicateConfig{});
  // Two objects in, two `clear` facts out -- they must not collapse into one.
  EXPECT_EQ(r.facts, (std::vector<std::string>{"(clear cube_1)", "(clear cube_1_2)"}));
}

TEST(Pddl, EmptySceneStillProducesAParseableInitBlock) {
  const PddlResult r = EmitInit(SceneState{}, PredicateConfig{});
  EXPECT_EQ(r.init_block, "(:init\n)");
  EXPECT_TRUE(r.facts.empty());
}

TEST(Pddl, AnObjectThatRestsOnNothingIsLeftOutOfTheInitBlockAndNamed) {
  // The cube on the table, and a second one hanging 95 mm above it.
  ObjectState table = Make("table_01", {0.0, 0.0, -0.02}, {0.60, 0.40, 0.02});
  table.cls = "table";
  SceneState scene;
  scene.objects = {
      table,
      Make("red_cube_01", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025}),
      Make("red_cube_02", {0.08, -0.12, 0.120}, {0.025, 0.025, 0.025}, kViewCamOver),
  };
  PredicateConfig cfg;
  cfg.require_support = true;

  const PddlResult r = EmitInit(scene, cfg);
  EXPECT_EQ(r.init_block.find("red_cube_02"), std::string::npos) << r.init_block;
  EXPECT_NE(r.init_block.find("(on red_cube_01 table_01)"), std::string::npos) << r.init_block;
  EXPECT_EQ(r.unsupported, std::vector<std::string>{"red_cube_02"});
  EXPECT_TRUE(r.warnings.empty());

  // Without the check it is a constant in the planner's world.
  const PddlResult before = EmitInit(scene, PredicateConfig{});
  EXPECT_NE(before.init_block.find("(clear red_cube_02)"), std::string::npos);
  EXPECT_TRUE(before.unsupported.empty());
}

}  // namespace
}  // namespace the_eyes::scenegraph
