// Fixture-driven unit tests. Zero hardware, zero models: this module is a pure
// function from SceneState to a fact set, and that is what makes it the first
// thing buildable in the project (AGENTS.md, Phase A).
#include "the_eyes/scenegraph/predicates.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include <unistd.h>

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

// The support check is off in a bare config; a fixture that wants it says so.
PredicateConfig CfgWithSupport(std::vector<std::string> containers = {"bowl"}) {
  PredicateConfig cfg;
  cfg.in_container_classes = std::move(containers);
  cfg.require_support = true;
  cfg.ground_classes = {"table"};
  return cfg;
}

bool Mentions(const PredicateResult& r, const std::string& id) {
  for (const std::string& f : r.facts) {
    if (f.find(" " + id + " ") != std::string::npos || f.find(" " + id + ")") != std::string::npos) {
      return true;
    }
  }
  return false;
}

// `in` is default-deny on the container class, so a fixture that wants a
// container has to name it. That is the point of the flag: the declaration is
// visible at the call site rather than inferred from a box's proportions.
PredicateConfig CfgWithContainers(std::vector<std::string> classes) {
  PredicateConfig cfg;
  cfg.in_container_classes = std::move(classes);
  return cfg;
}

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

  const PredicateResult r = Extract(scene, CfgWithContainers({"bin"}));
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

  const PredicateConfig cfg = CfgWithContainers({"bin"});
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
  const PredicateConfig cfg = CfgWithContainers({"box"});
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

TEST(Predicates, AViewThatSawTheObjectButWasGatedOutIsNotOccluded) {
  // The bottle of t1_bottle_b: both cameras estimate it on every pair, the
  // disagreement gate keeps cam_front's estimate, and nothing hides the bottle
  // from cam_over. visible_views says which view was fused; observed_views
  // says which views saw it, and occluded_from is about seeing.
  ObjectState bottle =
      Make("mustard_bottle_01", "bottle", {0.0, 0.0, 0.095}, {0.03, 0.05, 0.095}, kViewCamFront);
  bottle.observed_views = kViewCamFront | kViewCamOver;
  SceneState scene;
  scene.objects = {Table(), bottle};

  const PredicateResult r = Extract(scene, Cfg());
  EXPECT_FALSE(Has(r, "(occluded_from mustard_bottle_01 cam_over)"));
  EXPECT_FALSE(Has(r, "(occluded_from mustard_bottle_01 cam_front)"));
  EXPECT_TRUE(Has(r, "(on mustard_bottle_01 table_01)"));
}

TEST(Predicates, OccludedFromFollowsObservedViewsWhenSupplied) {
  // Fused from both, seen by both: nothing. Fused from one and seen by one:
  // the other camera has no estimate of it, which is what the fact reports.
  ObjectState cube =
      Make("red_cube_01", "cube", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025}, kViewCamFront);
  cube.observed_views = kViewCamFront;
  SceneState scene;
  scene.objects = {Table(), cube};
  const PredicateResult r = Extract(scene, Cfg());
  EXPECT_TRUE(Has(r, "(occluded_from red_cube_01 cam_over)"));
  EXPECT_FALSE(Has(r, "(occluded_from red_cube_01 cam_front)"));
}

TEST(Predicates, OccludedFromFallsBackToVisibleViewsWhenObservedIsNotSupplied) {
  // A SceneState from a producer older than observed_views (0 = not supplied)
  // means what it meant: the missing visible_views bit. ObjectOccludedFrom-
  // ExactlyOneView above is that case; this one pins the helper.
  ObjectState cube =
      Make("red_cube_01", "cube", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025}, kViewCamFront);
  ASSERT_EQ(cube.observed_views, 0u);
  EXPECT_TRUE(observed_from(cube, kViewCamFront));
  EXPECT_FALSE(observed_from(cube, kViewCamOver));
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
  // The shipped scene has exactly one container class, and `table` is not it.
  ASSERT_EQ(cfg.in_container_classes.size(), 1u);
  EXPECT_EQ(cfg.in_container_classes[0], "bowl");
  // View order defines the visible_views bit positions.
  ASSERT_EQ(cfg.view_names.size(), 2u);
  EXPECT_EQ(cfg.view_names[0], "cam_front");
  EXPECT_EQ(cfg.view_names[1], "cam_over");
}

TEST(Predicates, MissingConfigFileThrowsRatherThanUsingDefaults) {
  EXPECT_THROW(LoadPredicateConfig("/nonexistent/predicates.yaml"), std::runtime_error);
}

// ---------------------------------------------------------------------------
// The container flag. Regression from the 2026-09-06 live run, which emitted
// `(in red_cube_01 table_01)` 26 times.
// ---------------------------------------------------------------------------

TEST(Predicates, TableIsNotAContainerEvenWhenTheCubeSinksIntoIt) {
  // The exact failing geometry: the shipped table is a 20 mm slab centred at
  // z = -0.010, and a pose ~35 mm low puts the cube's centroid between its
  // faces. Every geometric test passes -- the volume ratio is tiny, the
  // centroid is inside the inset box -- so only the class declaration can
  // reject it.
  const ObjectState table = Make("table_01", "table", {0.0, 0.0, -0.010}, {0.400, 0.300, 0.010});
  const ObjectState sunk = Make("red_cube_01", "cube", {0.0, 0.0, -0.010}, {0.025, 0.025, 0.025});

  const Aabb bt = AabbOf(table);
  const Aabb bc = AabbOf(sunk);
  ASSERT_LT(bc.volume(), 0.5 * bt.volume()) << "volume gate would not have caught it";
  ASSERT_GE(sunk.pose_w.translation().z(), bt.min.z() + 0.005);
  ASSERT_LE(sunk.pose_w.translation().z(), bt.max.z() - 0.005)
      << "centroid must be inside the slab for this to be the real regression";

  EXPECT_FALSE(IsIn(sunk, table, Cfg()));

  // And the whole scene still behaves: no `in`, and `on` is not suppressed.
  SceneState scene;
  scene.objects = {table, Make("red_cube_01", "cube", {0.0, 0.0, 0.025}, {0.025, 0.025, 0.025})};
  const PredicateResult r = Extract(scene, Cfg());
  EXPECT_FALSE(Has(r, "(in red_cube_01 table_01)"));
  EXPECT_TRUE(Has(r, "(on red_cube_01 table_01)"));
}

TEST(Predicates, DeclaringTheContainerIsWhatEnablesIn) {
  // Same geometry, two configs. Nothing but the declaration differs, which
  // pins that the flag is the deciding term and not some coincidence of shape.
  const ObjectState bowl = Make("bowl_01", "bowl", {0.0, 0.0, 0.03}, {0.08, 0.08, 0.03});
  const ObjectState cube = Make("red_cube_01", "cube", {0.0, 0.0, 0.03}, {0.02, 0.02, 0.02});

  EXPECT_FALSE(IsIn(cube, bowl, CfgWithContainers({})));
  EXPECT_TRUE(IsIn(cube, bowl, CfgWithContainers({"bowl"})));
}

TEST(Predicates, UnknownObjectIsNeverAContainer) {
  // The DBSCAN path produces a centroid and an extent and no semantics at all,
  // so it can be contained but can never contain. Falls out of default-deny
  // rather than needing a special case, and this pins that it stays that way.
  const ObjectState unknown =
      Make("unknown_object_01", "unknown_object", {0.0, 0.0, 0.06}, {0.10, 0.10, 0.06});
  const ObjectState cube = Make("red_cube_01", "cube", {0.0, 0.0, 0.06}, {0.02, 0.02, 0.02});

  EXPECT_FALSE(IsIn(cube, unknown, Cfg()));
  EXPECT_FALSE(IsIn(cube, unknown, CfgWithContainers({"bowl"})));
}

TEST(Predicates, EmptyContainerListIsValidAndEmitsNoInFacts) {
  SceneState scene;
  scene.objects = {
      Table(),
      Make("bowl_01", "bowl", {0.30, 0.20, 0.03}, {0.08, 0.08, 0.03}),
      Make("red_cube_01", "cube", {0.30, 0.20, 0.03}, {0.02, 0.02, 0.02}),
  };
  const PredicateResult r = Extract(scene, CfgWithContainers({}));
  for (const std::string& f : r.facts) {
    EXPECT_EQ(f.rfind("(in ", 0), std::string::npos) << "unexpected containment: " << f;
  }
  // With containment off, the cube is simply on the table.
  EXPECT_TRUE(Has(r, "(on red_cube_01 table_01)"));
}

TEST(Predicates, AbsentContainerClassesKeyThrowsRatherThanDefaulting) {
  // Absent and empty are different. A default here would either invent
  // containers or silently drop the predicate, and both still look like a
  // working scene graph -- the failure mode this module exists to avoid.
  char tmpl[] = "/tmp/the_eyes_predicates_XXXXXX";
  const int fd = ::mkstemp(tmpl);
  if (fd >= 0) ::close(fd);
  std::ofstream(tmpl) << "on:\n  surface_tol_m: 0.015\n  min_overlap_ratio: 0.25\n"
                      << "in:\n  containment_margin_m: 0.005\n  max_volume_ratio: 0.5\n"
                      << "views:\n  names: [\"cam_front\", \"cam_over\"]\n";
  EXPECT_THROW(LoadPredicateConfig(tmpl), std::runtime_error);
  std::remove(tmpl);
}

// ---------------------------------------------------------------------------
// The support check. Every number in the first three fixtures is a median from
// the 2026-10-05 recordings (docs/OCTOBER_PLAN.md, "The objects that are not
// there"), with the table top moved to z = 0 as Table() has it.
// ---------------------------------------------------------------------------
const Eigen::Vector3d kCubeHalf(0.025, 0.025, 0.025);
const Eigen::Vector3d kBowlHalf(0.0807, 0.0806, 0.0275);

TEST(SupportCheck, APoseAtTheOccludersRangeFloatsAndIsNotReported) {
  // t3_cube_over: cam_front has the cube where it is; cam_over, half hidden,
  // puts it 171 mm away and 120 mm up, with the higher pose score of the two.
  SceneState scene;
  scene.objects = {
      Table(),
      Make("red_cube_01", "red_cube", {-0.0542, 0.0134, 0.0290}, kCubeHalf, kViewCamFront),
      Make("red_cube_02", "red_cube", {0.0265, -0.1042, 0.1202}, kCubeHalf, kViewCamOver),
  };
  const PredicateResult r = Extract(scene, CfgWithSupport());
  EXPECT_TRUE(Has(r, "(on red_cube_01 table_01)"));
  EXPECT_FALSE(Mentions(r, "red_cube_02"));
  EXPECT_EQ(r.unsupported, std::vector<std::string>{"red_cube_02"});
  EXPECT_TRUE(r.warnings.empty()) << "a steady state, not an event";

  // The same scene without the check reports both, as it did on the day.
  const PredicateResult before = Extract(scene, Cfg());
  EXPECT_TRUE(Has(before, "(clear red_cube_02)"));
  EXPECT_TRUE(Has(before, "(occluded_from red_cube_02 cam_front)"));
  EXPECT_TRUE(before.unsupported.empty());
}

TEST(SupportCheck, ATrackUnderTheTableIsNotReported) {
  // t3_cube_over, the stray detection at the image edge: 92 mm below the top.
  SceneState scene;
  scene.objects = {
      Table(),
      Make("red_cube_01", "red_cube", {-0.0542, 0.0134, 0.0290}, kCubeHalf),
      Make("red_cube_03", "red_cube", {0.1484, 0.3010, -0.0919}, kCubeHalf, kViewCamOver),
  };
  const PredicateResult r = Extract(scene, CfgWithSupport());
  EXPECT_FALSE(Mentions(r, "red_cube_03"));
  EXPECT_EQ(r.unsupported, std::vector<std::string>{"red_cube_03"});
}

TEST(SupportCheck, TheTrayTakenForABowlGoesAndTakesItsFalseFactWithIt) {
  // Every cube scene: the tray is a `bowl` 20 mm below the table top, and the
  // cube, which is on the table, also reads as on it.
  SceneState scene;
  scene.objects = {
      Table(),
      Make("bowl_01", "bowl", {-0.0116, 0.0174, -0.0198}, kBowlHalf, kViewCamFront),
      Make("red_cube_01", "red_cube", {-0.0542, 0.0134, 0.0290}, kCubeHalf, kViewCamFront),
  };
  const PredicateResult before = Extract(scene, Cfg());
  ASSERT_TRUE(Has(before, "(on red_cube_01 bowl_01)")) << "the false fact this fixture is about";
  ASSERT_TRUE(Has(before, "(on red_cube_01 table_01)"));

  const PredicateResult r = Extract(scene, CfgWithSupport());
  EXPECT_TRUE(Has(r, "(on red_cube_01 table_01)"));
  EXPECT_FALSE(Mentions(r, "bowl_01"));
  EXPECT_EQ(r.unsupported, std::vector<std::string>{"bowl_01"});
  EXPECT_TRUE(Has(r, "(clear red_cube_01)"));
}

TEST(SupportCheck, SupportRunsThroughAStackAndThroughAContainer) {
  // Nothing here touches the table except the bottom block and the bowl.
  SceneState scene;
  scene.objects = {
      Table(),
      Make("block_01", "block", {0.0, 0.0, 0.025}, kCubeHalf),
      Make("block_02", "block", {0.0, 0.0, 0.075}, kCubeHalf),
      Make("block_03", "block", {0.0, 0.0, 0.125}, kCubeHalf),
      Make("bowl_01", "bowl", {0.30, 0.20, 0.03}, {0.08, 0.08, 0.03}),
      Make("ball_01", "ball", {0.30, 0.20, 0.03}, {0.02, 0.02, 0.02}),
  };
  const PredicateResult r = Extract(scene, CfgWithSupport());
  EXPECT_TRUE(r.unsupported.empty());
  EXPECT_TRUE(Has(r, "(on block_03 block_02)"));
  EXPECT_TRUE(Has(r, "(in ball_01 bowl_01)"));
  // With everything grounded the check changes nothing at all.
  EXPECT_EQ(r.facts, Extract(scene, CfgWithContainers({"bowl"})).facts);
}

TEST(SupportCheck, RestingOnSomethingUnsupportedIsNotSupport) {
  // A phantom with a second phantom on top of it, well clear of the table.
  SceneState scene;
  scene.objects = {
      Table(),
      Make("red_cube_01", "red_cube", {0.0, 0.0, 0.025}, kCubeHalf),
      Make("red_cube_02", "red_cube", {0.2, 0.0, 0.125}, kCubeHalf),
      Make("red_cube_03", "red_cube", {0.2, 0.0, 0.175}, kCubeHalf),
  };
  ASSERT_TRUE(Has(Extract(scene, Cfg()), "(on red_cube_03 red_cube_02)"));
  const PredicateResult r = Extract(scene, CfgWithSupport());
  EXPECT_EQ(r.unsupported, (std::vector<std::string>{"red_cube_02", "red_cube_03"}));
  EXPECT_FALSE(Mentions(r, "red_cube_02"));
  EXPECT_FALSE(Mentions(r, "red_cube_03"));
  EXPECT_TRUE(Has(r, "(on red_cube_01 table_01)"));
}

TEST(SupportCheck, AnUnsupportedContainerDoesNotTakeItsContentsWithIt) {
  // The cube is on the table. A phantom bowl hangs around it, off the table,
  // and by the pairwise rules the cube is IN that bowl -- which, containment
  // being exclusive, would strip the cube of its (on ... table). Grounding is
  // decided before that happens: the bowl goes, the cube stays on the table.
  SceneState scene;
  scene.objects = {
      Table(),
      Make("red_cube_01", "red_cube", {0.0, 0.0, 0.025}, kCubeHalf),
      // Its floor is 18 mm above the table top: outside the 15 mm tolerance.
      Make("bowl_01", "bowl", {0.0, 0.0, 0.068}, {0.10, 0.10, 0.05}),
  };
  const PredicateResult before = Extract(scene, CfgWithContainers({"bowl"}));
  ASSERT_TRUE(Has(before, "(in red_cube_01 bowl_01)"));
  ASSERT_FALSE(Has(before, "(on red_cube_01 table_01)"));

  const PredicateResult r = Extract(scene, CfgWithSupport());
  EXPECT_EQ(r.unsupported, std::vector<std::string>{"bowl_01"});
  EXPECT_TRUE(Has(r, "(on red_cube_01 table_01)"));
  EXPECT_FALSE(Mentions(r, "bowl_01"));
}

TEST(SupportCheck, WithNoGroundInTheSceneEverythingGoesAndItSaysWhy) {
  SceneState scene;
  scene.objects = {Make("red_cube_01", "red_cube", {0.0, 0.0, 0.025}, kCubeHalf),
                   Make("red_cube_02", "red_cube", {0.0, 0.0, 0.075}, kCubeHalf)};
  const PredicateResult r = Extract(scene, CfgWithSupport());
  EXPECT_TRUE(r.facts.empty());
  EXPECT_EQ(r.unsupported, (std::vector<std::string>{"red_cube_01", "red_cube_02"}));
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("no object of a ground class"), std::string::npos) << r.warnings[0];

  // An empty scene is not a misconfiguration.
  EXPECT_TRUE(Extract(SceneState{}, CfgWithSupport()).warnings.empty());
}

TEST(SupportCheck, AnnotateLeavesAnUnsupportedObjectInTheSceneWithNoRelations) {
  SceneState scene;
  scene.objects = {
      Table(),
      Make("red_cube_01", "red_cube", {-0.0542, 0.0134, 0.0290}, kCubeHalf, kViewCamFront),
      Make("red_cube_02", "red_cube", {0.0265, -0.1042, 0.1202}, kCubeHalf, kViewCamOver),
  };
  const PredicateResult r = Annotate(scene, CfgWithSupport());
  ASSERT_EQ(scene.objects.size(), 3u) << "the scene is fusion's account; nothing is removed";
  EXPECT_FALSE(scene.objects[1].relations.empty());
  EXPECT_TRUE(scene.objects[2].relations.empty());
  EXPECT_EQ(r.unsupported, std::vector<std::string>{"red_cube_02"});
  EXPECT_EQ(r.facts, Extract(scene, CfgWithSupport()).facts);
}

TEST(SupportCheck, IsDeterministicUnderInputPermutation) {
  SceneState scene;
  scene.objects = {
      Table(),
      Make("bowl_01", "bowl", {-0.0116, 0.0174, -0.0198}, kBowlHalf),
      Make("red_cube_01", "red_cube", {-0.0542, 0.0134, 0.0290}, kCubeHalf),
      Make("red_cube_02", "red_cube", {0.0265, -0.1042, 0.1202}, kCubeHalf),
      Make("red_cube_03", "red_cube", {0.1484, 0.3010, -0.0919}, kCubeHalf),
      Make("block_02", "block", {-0.0542, 0.0134, 0.0790}, kCubeHalf),
  };
  const PredicateResult reference = Extract(scene, CfgWithSupport());
  std::mt19937 rng(7);
  for (int i = 0; i < 20; ++i) {
    std::shuffle(scene.objects.begin(), scene.objects.end(), rng);
    const PredicateResult r = Extract(scene, CfgWithSupport());
    EXPECT_EQ(r.facts, reference.facts);
    EXPECT_EQ(r.unsupported, reference.unsupported);
  }
  EXPECT_EQ(reference.unsupported,
            (std::vector<std::string>{"bowl_01", "red_cube_02", "red_cube_03"}));
  EXPECT_TRUE(Has(reference, "(on block_02 red_cube_01)"));
}

std::string PredicatesYamlWith(const std::string& support_section) {
  char tmpl[] = "/tmp/the_eyes_predicates_XXXXXX";
  const int fd = ::mkstemp(tmpl);
  if (fd >= 0) ::close(fd);
  std::ofstream(tmpl) << "on:\n  surface_tol_m: 0.015\n  min_overlap_ratio: 0.25\n"
                      << "in:\n  containment_margin_m: 0.005\n  max_volume_ratio: 0.5\n"
                      << "  container_classes: [bowl]\n"
                      << "views:\n  names: [\"cam_front\", \"cam_over\"]\n"
                      << support_section;
  return tmpl;
}

TEST(SupportCheck, ConfigSectionIsOptionalButStrictWhenPresent) {
  // A predicates.yaml saved with a run from before the check still loads, and
  // still means what it meant.
  const std::string none = PredicatesYamlWith("");
  EXPECT_FALSE(LoadPredicateConfig(none).require_support);
  std::remove(none.c_str());

  const std::string on = PredicatesYamlWith(
      "support:\n  require_support: true\n  ground_classes: [table, shelf]\n");
  const PredicateConfig cfg = LoadPredicateConfig(on);
  EXPECT_TRUE(cfg.require_support);
  EXPECT_EQ(cfg.ground_classes, (std::vector<std::string>{"table", "shelf"}));
  std::remove(on.c_str());

  for (const char* bad : {
           "support:\n  require_support: true\n",                       // no ground classes
           "support:\n  ground_classes: [table]\n",                     // no switch
           "support:\n  require_support: true\n  ground_classes: []\n",  // nothing to stand on
           "support:\n  require_support: maybe\n  ground_classes: [table]\n",
       }) {
    const std::string path = PredicatesYamlWith(bad);
    EXPECT_THROW(LoadPredicateConfig(path), std::runtime_error) << bad;
    std::remove(path.c_str());
  }
  // Off with an empty list is a legitimate way to say "no check".
  const std::string off = PredicatesYamlWith(
      "support:\n  require_support: false\n  ground_classes: []\n");
  EXPECT_FALSE(LoadPredicateConfig(off).require_support);
  std::remove(off.c_str());
}

TEST(SupportCheck, TheShippedConfigTurnsItOnWithTheTableAsGround) {
  const PredicateConfig cfg =
      LoadPredicateConfig(std::string(THE_EYES_CONFIG_DIR) + "/predicates.yaml");
  EXPECT_TRUE(cfg.require_support);
  EXPECT_EQ(cfg.ground_classes, std::vector<std::string>{"table"});
}

}  // namespace
}  // namespace the_eyes::scenegraph
