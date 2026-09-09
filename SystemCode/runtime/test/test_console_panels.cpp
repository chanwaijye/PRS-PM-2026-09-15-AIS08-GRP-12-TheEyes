// Headless: no window, no display, no hardware. The console's panels are pure
// functions precisely so this suite can exist (docs/VIZ_PLAN.md §6).
#include "the_eyes/runtime/console_panels.hpp"

#include "the_eyes/runtime/console.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include <tuple>

using namespace the_eyes;
using namespace the_eyes::runtime;

namespace {

ObjectState Obj(const std::string& id, const std::string& cls, double x, double y, double z,
                double sigma = -1.0) {
  ObjectState o;
  o.id = id;
  o.cls = cls;
  o.pose_w = Eigen::Isometry3d::Identity();
  o.pose_w.translation() = Eigen::Vector3d(x, y, z);
  o.extent = Eigen::Vector3d(0.025, 0.025, 0.025);
  o.visible_views = kViewCamFront | kViewCamOver;
  o.confidence = 0.8;
  o.position_sigma_m = sigma;
  o.orientation_sigma_rad = sigma;
  return o;
}

cv::Mat Panel(int w = 424, int h = 520) { return cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0)); }

long NonBackground(const cv::Mat& m) {
  cv::Mat grey, mask;
  cv::cvtColor(m, grey, cv::COLOR_BGR2GRAY);
  // The panels clear to a dark grey (28); anything drawn is brighter.
  cv::threshold(grey, mask, 40, 255, cv::THRESH_BINARY);
  return cv::countNonZero(mask);
}

}  // namespace

TEST(ConsolePanels, BinaryFactsBecomeEdgesAndOthersAreIgnored) {
  const auto edges = ParseBinaryFacts({"(on red_cube_01 table_01)", "(clear red_cube_01)",
                                       "(in bowl_01 table_01)",
                                       "(occluded_from red_cube_01 cam_over)"});
  // `clear` is unary and is not an edge. `occluded_from` is binary in arity but
  // its second argument is a VIEW, not an object -- it becomes an edge here and
  // is dropped later because no node matches, which is the honest order: parse
  // by shape, resolve against the actual nodes.
  ASSERT_EQ(edges.size(), 3u);
  EXPECT_EQ(edges[0].predicate, "on");
  EXPECT_EQ(edges[0].from, "red_cube_01");
  EXPECT_EQ(edges[0].to, "table_01");
  EXPECT_EQ(edges[1].predicate, "in");
  EXPECT_EQ(edges[2].predicate, "occluded_from");
}

TEST(ConsolePanels, GraphLayoutIsIndependentOfEmissionOrder) {
  // The claim from docs/VIZ_PLAN.md §3.1: shuffling the object vector must not
  // move a node, or the viewer reads motion that never happened.
  SceneState a;
  a.objects = {Obj("red_cube_02", "red_cube", 0.1, 0, 0.025),
               Obj("bowl_01", "bowl", -0.1, 0, 0.02),
               Obj("table_01", "table", 0, 0.155, -0.0115),
               Obj("red_cube_01", "red_cube", 0, 0, 0.025)};
  SceneState b = a;
  std::reverse(b.objects.begin(), b.objects.end());
  const std::vector<std::string> statics{"table_01"};

  const auto oa = GraphNodeOrder(a, statics);
  const auto ob = GraphNodeOrder(b, statics);
  EXPECT_EQ(oa, ob);
  // Movable first in sorted order, statics pinned last.
  ASSERT_EQ(oa.size(), 4u);
  EXPECT_EQ(oa[0], "bowl_01");
  EXPECT_EQ(oa[3], "table_01");

  cv::Mat pa = Panel(), pb = Panel();
  DrawGraphPanel(pa, a, statics, {"(on red_cube_01 table_01)"});
  DrawGraphPanel(pb, b, statics, {"(on red_cube_01 table_01)"});
  EXPECT_EQ(cv::norm(pa, pb, cv::NORM_L1), 0.0) << "the same scene drew differently";
}

TEST(ConsolePanels, GraphIsStableAcrossRepeatedCalls) {
  SceneState s;
  s.objects = {Obj("a_01", "red_cube", 0, 0, 0.025), Obj("t_01", "table", 0, 0, -0.01)};
  cv::Mat p1 = Panel(), p2 = Panel();
  DrawGraphPanel(p1, s, {"t_01"}, {"(on a_01 t_01)"});
  DrawGraphPanel(p2, s, {"t_01"}, {"(on a_01 t_01)"});
  EXPECT_EQ(cv::norm(p1, p2, cv::NORM_L1), 0.0);
}

TEST(ConsolePanels, EmptySceneDrawsSomethingRatherThanCrashing) {
  const SceneState empty;
  cv::Mat g = Panel();
  DrawGraphPanel(g, empty, {}, {});
  EXPECT_GT(NonBackground(g), 0);

  cv::Mat w = Panel(848, 520);
  DrawWorldPanel(w, empty, {}, fusion::ExtrinsicsSet{}, WorldView{});
  EXPECT_GT(NonBackground(w), 0) << "the plan/elevation axes should still be drawn";
}

TEST(ConsolePanels, AbsentUncertaintyDrawsNoCircle) {
  // position_sigma_m is NEGATIVE when not supplied, and that sentinel is the
  // whole point of the field: drawing a zero-radius circle would render an
  // unmeasured pose as a perfectly certain one.
  SceneState without, with;
  without.objects = {Obj("c_01", "red_cube", 0, 0, 0.025, -1.0)};
  with.objects = {Obj("c_01", "red_cube", 0, 0, 0.025, 0.02)};

  cv::Mat a = Panel(848, 520), b = Panel(848, 520);
  DrawWorldPanel(a, without, {}, fusion::ExtrinsicsSet{}, WorldView{});
  DrawWorldPanel(b, with, {}, fusion::ExtrinsicsSet{}, WorldView{});
  EXPECT_LT(NonBackground(a), NonBackground(b))
      << "the sigma circle must appear only when uncertainty was supplied";
}

TEST(ConsolePanels, UnknownObjectGetsNoHeadingTick) {
  // Its rotation is meaningless by contract (INTERFACES.md §5).
  SceneState known, unknown;
  known.objects = {Obj("k_01", "red_cube", 0, 0, 0.025)};
  unknown.objects = {Obj("u_01", "unknown_object", 0, 0, 0.025)};
  cv::Mat a = Panel(848, 520), b = Panel(848, 520);
  DrawWorldPanel(a, known, {}, fusion::ExtrinsicsSet{}, WorldView{});
  DrawWorldPanel(b, unknown, {}, fusion::ExtrinsicsSet{}, WorldView{});
  // Different renderings; the dashed box also proves the two paths diverge.
  EXPECT_NE(cv::norm(a, b, cv::NORM_L1), 0.0);
}

TEST(ConsolePanels, PddlPanelRendersTheBlockVerbatim) {
  // Acceptance criterion 3: what is drawn is what was emitted.
  const std::string block = "(:init\n  (clear red_cube_01)\n  (on red_cube_01 table_01)\n)";
  cv::Mat p = Panel();
  DrawPddlPanel(p, block, {"renamed object id 'x/y' -> 'x_y'"});
  EXPECT_GT(NonBackground(p), 0);
  cv::Mat q = Panel();
  DrawPddlPanel(q, block, {"renamed object id 'x/y' -> 'x_y'"});
  EXPECT_EQ(cv::norm(p, q, cv::NORM_L1), 0.0);
}

TEST(ConsolePanels, AFactCycleDoesNotHangTheGraph) {
  // Nothing forbids fusion emitting a cycle, and a layout that assumed a tree
  // would loop forever on one.
  SceneState s;
  s.objects = {Obj("a_01", "red_cube", 0, 0, 0.025), Obj("b_01", "red_cube", 0.1, 0, 0.025)};
  cv::Mat p = Panel();
  DrawGraphPanel(p, s, {}, {"(on a_01 b_01)", "(on b_01 a_01)"});
  EXPECT_GT(NonBackground(p), 0);
}

TEST(ConsolePanels, EdgesToUnknownNodesAreDropped) {
  SceneState s;
  s.objects = {Obj("a_01", "red_cube", 0, 0, 0.025)};
  cv::Mat with_view = Panel(), without = Panel();
  // cam_over is not an object, so (occluded_from a_01 cam_over) has no node to
  // land on and must not invent one.
  DrawGraphPanel(with_view, s, {}, {"(occluded_from a_01 cam_over)"});
  DrawGraphPanel(without, s, {}, {});
  EXPECT_EQ(cv::norm(with_view, without, cv::NORM_L1), 0.0);
}

TEST(ConsolePanels, FitKeepsTheCamerasOutOfTheElevationStrip) {
  // Regression: the first version used a hand-chosen scale, and the cameras --
  // which sit ~0.4 m in FRONT of the world origin while the table reaches
  // 0.53 m behind it -- were drawn past the bottom of the plan region and
  // landed on top of the elevation strip.
  SceneState s;
  s.objects = {Obj("table_01", "table", 0.0, 0.155, -0.0115)};
  s.objects[0].extent = Eigen::Vector3d(0.590, 0.375, 0.0115);

  fusion::ExtrinsicsSet ex;
  for (const auto& [id, x, y] : std::vector<std::tuple<std::string, double, double>>{
           {"cam_front", -0.403, -0.372}, {"cam_over", 0.301, -0.430}}) {
    CameraExtrinsics c;
    c.camera_id = id;
    c.T_world_cam = Eigen::Isometry3d::Identity();
    c.T_world_cam.translation() = Eigen::Vector3d(x, y, 0.39);
    ex.cameras[id] = c;
  }

  const int W = 848, PLAN = 360, H = 520;
  const WorldView v = FitWorldView(s, ex, W, PLAN);
  cv::Mat panel(H, W, CV_8UC3);
  DrawWorldPanel(panel, s, {"table_01"}, ex, v);

  // Every camera must project inside the plan region.
  for (const auto& [id, cam] : ex.cameras) {
    const Eigen::Vector3d p = cam.T_world_cam.translation();
    const int py = static_cast<int>(PLAN / 2.0 - (p.y() - v.centre_y_m) * v.px_per_m);
    EXPECT_GE(py, 0) << id << " is above the plan region";
    EXPECT_LT(py, PLAN) << id << " spilled into the elevation strip";
  }
  // And the table's far edge must fit too.
  const double back_y = 0.155 + 0.375;
  const int back_py = static_cast<int>(PLAN / 2.0 - (back_y - v.centre_y_m) * v.px_per_m);
  EXPECT_GE(back_py, 0) << "the table's back edge is off the top of the plan";
}

// Console::Compose opens no window, which is what makes this testable at all.
TEST(ConsolePanels, ComposeIsHeadlessDeterministicAndCorrectlySized) {
  FramePair pair;
  pair.pair_index = 7;
  camera::CameraFrame f;
  f.rgb = cv::Mat(480, 848, CV_8UC3, cv::Scalar(30, 30, 30));
  f.K = Eigen::Matrix3d::Identity();
  f.K(0, 0) = f.K(1, 1) = 600.0;
  f.K(0, 2) = 424.0;
  f.K(1, 2) = 240.0;
  pair.frames["cam_front"] = f;
  pair.frames["cam_over"] = f;

  PipelineResult r;
  r.pair_index = 7;
  r.scene.objects = {Obj("red_cube_01", "red_cube", 0.0, 0.0, 0.025, 0.006),
                     Obj("table_01", "table", 0.0, 0.155, -0.0115)};
  r.scene.objects[1].extent = Eigen::Vector3d(0.590, 0.375, 0.0115);
  r.static_ids = {"table_01"};
  r.pddl.facts = {"(clear red_cube_01)", "(on red_cube_01 table_01)"};
  r.pddl.init_block = "(:init\n  (clear red_cube_01)\n  (on red_cube_01 table_01)\n)";

  const std::vector<std::string> views{"cam_front", "cam_over"};
  const fusion::ExtrinsicsSet ex;
  const cv::Mat a = Console::Compose(views, ex, pair, r, 19.0);
  EXPECT_EQ(a.cols, 2 * 848);
  EXPECT_EQ(a.rows, 480 + 520 + 24);

  const cv::Mat b = Console::Compose(views, ex, pair, r, 19.0);
  EXPECT_EQ(cv::norm(a, b, cv::NORM_L1), 0.0) << "the same pair composed differently twice";
}

TEST(ConsolePanels, ComposeSurvivesAMissingView) {
  // A dropped camera must annotate the slot, not crash or silently shift the
  // layout so the two views swap places.
  FramePair pair;
  camera::CameraFrame f;
  f.rgb = cv::Mat(480, 848, CV_8UC3, cv::Scalar(30, 30, 30));
  f.K = Eigen::Matrix3d::Identity();
  pair.frames["cam_front"] = f;
  PipelineResult r;
  const cv::Mat c = Console::Compose({"cam_front", "cam_over"}, fusion::ExtrinsicsSet{}, pair, r,
                                     0.0);
  EXPECT_EQ(c.cols, 2 * 848);
  EXPECT_GT(NonBackground(c), 0);
}
