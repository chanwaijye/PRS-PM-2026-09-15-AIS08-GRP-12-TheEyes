// Fixture-driven unit tests. Zero hardware, zero models: this module is a pure
// function from SceneState to a fact set, and that is what makes it the first
// thing buildable in the project (AGENTS.md, Phase A).
#include "the_eyes/scenegraph/predicates.hpp"

#include <algorithm>
#include <random>

#include <gtest/gtest.h>

namespace the_eyes::scenegraph {
namespace {

ObjectState Make(const std::string& id, const std::string& cls,
                 const Eigen::Vector3d& centre, const Eigen::Vector3d& half_extent,
                 uint32_t views = kViewCamFront | kViewCamOver) {
  ObjectState o;
  o.id = id;
  o.cls = cls;
  o.pose_w = Eigen::Isometry3d::Identity();
  o.pose_w.translation() = centre;
  o.extent = half_extent;
  o.visible_views = views;
  o.confidence = 0.9;
  o.stamp_ns = 1'000'000'000ULL;
  return o;
}

// Table top sits exactly at z = 0, so object base z is its height above table.
ObjectState Table() {
  return Make("table_01", "table", {0.0, 0.0, -0.02}, {0.60, 0.40, 0.02});
}

bool Has(const PredicateResult& r, const std::string& fact) {
  return std::find(r.facts.begin(), r.facts.end(), fact) != r.facts.end();
}

PredicateConfig Cfg() { return PredicateConfig{}; }

TEST(Predicates, CubeRestingOnTableIsOnAndClear) {
  SceneState scene;
  scene.objects = {Table(), Make("red_cube_01", "cube", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025})};

  const PredicateResult r = Extract(scene, Cfg());
  EXPECT_TRUE(Has(r, "(on red_cube_01 table_01)"));
  EXPECT_TRUE(Has(r, "(clear red_cube_01)"));
  EXPECT_FALSE(Has(r, "(clear table_01)"));  // the cube is on it
  EXPECT_TRUE(r.warnings.empty());
}

TEST(Predicates, ObjectBesideTableIsNotOnIt) {
  SceneState scene;
  // Same height as a resting cube, but horizontally clear of the table entirely.
  scene.objects = {Table(), Make("red_cube_01", "cube", {1.5, 0.0, 0.025}, {0.025, 0.025, 0.025})};

  const PredicateResult r = Extract(scene, Cfg());
  EXPECT_FALSE(Has(r, "(on red_cube_01 table_01)"));
  EXPECT_TRUE(Has(r, "(clear table_01)"));
}

TEST(Predicates, PartialOverlapBelowRatioIsNotOn) {
  const PredicateConfig cfg = Cfg();
  const ObjectState table = Table();
  // Cube 0.05 wide, shifted so only ~10% of its footprint is over the table.
  const ObjectState cube = Make("red_cube_01", "cube", {0.62, 0.0, 0.025}, {0.025, 0.025, 0.025});
  EXPECT_FALSE(IsOn(cube, table, cfg));

  // Shift it back so most of the footprint overlaps.
  const ObjectState centred = Make("red_cube_01", "cube", {0.58, 0.0, 0.025}, {0.025, 0.025, 0.025});
  EXPECT_TRUE(IsOn(centred, table, cfg));
}

TEST(Predicates, StackedObjectsChainCorrectly) {
  SceneState scene;
  scene.objects = {
      Table(),
      Make("box_01", "box", {0.0, 0.0, 0.04}, {0.06, 0.06, 0.04}),         // on table
      Make("red_cube_01", "cube", {0.0, 0.0, 0.105}, {0.025, 0.025, 0.025}),  // on box
  };

  const PredicateResult r = Extract(scene, Cfg());
  EXPECT_TRUE(Has(r, "(on box_01 table_01)"));
  EXPECT_TRUE(Has(r, "(on red_cube_01 box_01)"));
  EXPECT_FALSE(Has(r, "(on red_cube_01 table_01)"));  // not resting on the table
  EXPECT_TRUE(Has(r, "(clear red_cube_01)"));
  EXPECT_FALSE(Has(r, "(clear box_01)"));
  EXPECT_FALSE(Has(r, "(clear table_01)"));
}

TEST(Predicates, ObjectInContainerIsInNotOn) {
  SceneState scene;
  // Open box: interior spans z in [0, 0.12], footprint 0.20 x 0.20.
  scene.objects = {
      Table(),
      Make("bin_01", "bin", {0.30, 0.20, 0.06}, {0.10, 0.10, 0.06}),
      Make("red_cube_01", "cube", {0.30, 0.20, 0.03}, {0.025, 0.025, 0.025}),
  };

  const PredicateResult r = Extract(scene, Cfg());
  EXPECT_TRUE(Has(r, "(in red_cube_01 bin_01)"));
  EXPECT_FALSE(Has(r, "(on red_cube_01 bin_01)"));
  // Containment is not support: the bin still has nothing stacked on top.
  EXPECT_TRUE(Has(r, "(clear bin_01)"));
}

TEST(Predicates, ContainedObjectIsOnNothingAtAll) {
  // Regression from the end-to-end pipeline test. A ball resting in a shallow
  // bin has its base within the `on` tolerance of the table top beneath the
  // bin, so the pairwise rule would emit (on ball table) alongside
  // (in ball bin). Containment is exclusive: PM1 must not be told it can pick
  // the ball straight off the table.
  SceneState scene;
  scene.objects = {
      Table(),
      Make("bin_01", "bin", {0.30, 0.20, 0.06}, {0.10, 0.10, 0.06}),
      Make("ball_01", "ball", {0.30, 0.20, 0.03}, {0.02, 0.02, 0.02}),
  };

  const PredicateConfig cfg = Cfg();
  // The pairwise rule genuinely does hold for this geometry...
  EXPECT_TRUE(IsOn(scene.objects[2], scene.objects[0], cfg));

  // ...and Extract must still suppress it.
  const PredicateResult r = Extract(scene, cfg);
  EXPECT_TRUE(Has(r, "(in ball_01 bin_01)"));
  EXPECT_FALSE(Has(r, "(on ball_01 table_01)"));
  EXPECT_FALSE(Has(r, "(on ball_01 bin_01)"));
  // The bin itself is not contained, so it still rests on the table.
  EXPECT_TRUE(Has(r, "(on bin_01 table_01)"));
  // And the table is not clear -- the bin is on it.
  EXPECT_FALSE(Has(r, "(clear table_01)"));
}

TEST(Predicates, ContainerTooSmallToHoldIsNotIn) {
  const PredicateConfig cfg = Cfg();
  // Coincident centroids, but the "container" is smaller than the object.
  const ObjectState big = Make("big_01", "box", {0.0, 0.0, 0.10}, {0.10, 0.10, 0.10});
  const ObjectState small = Make("small_01", "box", {0.0, 0.0, 0.10}, {0.02, 0.02, 0.02});
  EXPECT_TRUE(IsIn(small, big, cfg));
  EXPECT_FALSE(IsIn(big, small, cfg));  // volume ratio gate
}

TEST(Predicates, ObjectOccludedFromExactlyOneView) {
  SceneState scene;
  scene.objects = {
      Table(),
      Make("red_cube_01", "cube", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025}, kViewCamFront),
  };

  const PredicateResult r = Extract(scene, Cfg());
  EXPECT_TRUE(Has(r, "(occluded_from red_cube_01 cam_over)"));
  EXPECT_FALSE(Has(r, "(occluded_from red_cube_01 cam_front)"));
  EXPECT_FALSE(Has(r, "(occluded_from table_01 cam_front)"));
  EXPECT_FALSE(Has(r, "(occluded_from table_01 cam_over)"));
  // Occlusion does not suppress the geometric relations.
  EXPECT_TRUE(Has(r, "(on red_cube_01 table_01)"));
}

TEST(Predicates, UnknownObjectParticipatesUsingCentroidAndExtentOnly) {
  SceneState scene;
  ObjectState unknown =
      Make("unknown_object_01", "unknown_object", {0.10, 0.0, 0.04}, {0.03, 0.03, 0.04});
  // Rotation is meaningless for the DBSCAN path (INTERFACES.md §5); setting a
  // wild one must not change any predicate, because extent is world-axis-aligned.
  unknown.pose_w.linear() =
      Eigen::AngleAxisd(1.1, Eigen::Vector3d(0.3, 0.7, 0.2).normalized()).toRotationMatrix();
  scene.objects = {Table(), unknown};

  const PredicateResult r = Extract(scene, Cfg());
  EXPECT_TRUE(Has(r, "(on unknown_object_01 table_01)"));
  EXPECT_TRUE(Has(r, "(clear unknown_object_01)"));

  SceneState upright = scene;
  upright.objects[1].pose_w.linear() = Eigen::Matrix3d::Identity();
  EXPECT_EQ(Extract(upright, Cfg()).facts, r.facts);
}

TEST(Predicates, InvisibleObjectIsDroppedWithAWarning) {
  SceneState scene;
  scene.objects = {Table(), Make("ghost_01", "cube", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025}, 0)};

  const PredicateResult r = Extract(scene, Cfg());
  for (const std::string& f : r.facts) {
    EXPECT_EQ(f.find("ghost_01"), std::string::npos) << f;
  }
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("ghost_01"), std::string::npos);
  EXPECT_TRUE(Has(r, "(clear table_01)"));
}

TEST(Predicates, DuplicateIdIsDroppedWithAWarning) {
  SceneState scene;
  scene.objects = {
      Make("red_cube_01", "cube", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025}),
      Make("red_cube_01", "cube", {0.4, 0.0, 0.025}, {0.025, 0.025, 0.025}),
  };
  const PredicateResult r = Extract(scene, Cfg());
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("duplicate id"), std::string::npos);
}

TEST(Predicates, OutputIsDeterministicUnderInputPermutation) {
  SceneState scene;
  scene.objects = {
      Table(),
      Make("box_01", "box", {0.0, 0.0, 0.04}, {0.06, 0.06, 0.04}),
      Make("red_cube_01", "cube", {0.0, 0.0, 0.105}, {0.025, 0.025, 0.025}, kViewCamOver),
      Make("bin_01", "bin", {0.30, 0.20, 0.06}, {0.10, 0.10, 0.06}),
      Make("ball_01", "ball", {0.30, 0.20, 0.03}, {0.02, 0.02, 0.02}),
  };

  const std::vector<std::string> reference = Extract(scene, Cfg()).facts;
  ASSERT_FALSE(reference.empty());

  std::mt19937 rng(20260819);
  for (int trial = 0; trial < 25; ++trial) {
    SceneState shuffled = scene;
    std::shuffle(shuffled.objects.begin(), shuffled.objects.end(), rng);
    EXPECT_EQ(Extract(shuffled, Cfg()).facts, reference) << "trial " << trial;
  }
}

TEST(Predicates, AnnotateFillsRelationsForTheSubjectObjectOnly) {
  SceneState scene;
  scene.objects = {Table(), Make("red_cube_01", "cube", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025},
                                 kViewCamFront)};
  Annotate(scene, Cfg());

  const ObjectState& table = scene.objects[0];
  const ObjectState& cube = scene.objects[1];
  EXPECT_EQ(cube.relations,
            (std::vector<std::string>{"(clear red_cube_01)",
                                      "(occluded_from red_cube_01 cam_over)",
                                      "(on red_cube_01 table_01)"}));
  // The table is the object of "(on red_cube_01 table_01)", not its subject.
  EXPECT_TRUE(table.relations.empty());
}

TEST(Predicates, ToleranceComesFromConfigNotConstants) {
  const ObjectState table = Table();
  // Floating 8 mm above the table: inside the shipped 15 mm tolerance...
  const ObjectState hovering = Make("red_cube_01", "cube", {0.0, 0.0, 0.033}, {0.025, 0.025, 0.025});

  PredicateConfig loose = Cfg();
  EXPECT_TRUE(IsOn(hovering, table, loose));

  PredicateConfig tight = Cfg();
  tight.on_surface_tol_m = 0.002;  // ...and outside a 2 mm one
  EXPECT_FALSE(IsOn(hovering, table, tight));
}

TEST(Predicates, ShippedConfigFileLoadsAndMatchesDocumentedDefaults) {
  const PredicateConfig cfg =
      LoadPredicateConfig(std::string(THE_EYES_CONFIG_DIR) + "/predicates.yaml");
  EXPECT_DOUBLE_EQ(cfg.on_surface_tol_m, 0.015);
  EXPECT_DOUBLE_EQ(cfg.on_min_overlap_ratio, 0.25);
  EXPECT_DOUBLE_EQ(cfg.in_containment_margin_m, 0.005);
  EXPECT_DOUBLE_EQ(cfg.in_max_volume_ratio, 0.5);
  // View order defines the visible_views bit positions.
  ASSERT_EQ(cfg.view_names.size(), 2u);
  EXPECT_EQ(cfg.view_names[0], "cam_front");
  EXPECT_EQ(cfg.view_names[1], "cam_over");
}

TEST(Predicates, MissingConfigFileThrowsRatherThanUsingDefaults) {
  EXPECT_THROW(LoadPredicateConfig("/nonexistent/predicates.yaml"), std::runtime_error);
}

}  // namespace
}  // namespace the_eyes::scenegraph
