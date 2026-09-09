// The console's non-camera panels, as free functions that draw into a cv::Mat.
//
// Free functions and nothing else, deliberately: they open no window, hold no
// state, and read only const inputs, so the whole panel layer is testable in
// ctest on a headless box. The window lives in console.hpp, which is the only
// part that needs a display.
//
// WHAT THESE PANELS ARE FOR (docs/VIZ_PLAN.md §3.1). The existing `--show`
// viewer draws detections and reprojected poses, which shows the pipeline's
// answer but not the two things that are graded: the world-frame geometry
// fusion produced, and the symbolic facts the scene graph derived from it.
// These three panels are those two things.
#ifndef THE_EYES_RUNTIME_CONSOLE_PANELS_HPP
#define THE_EYES_RUNTIME_CONSOLE_PANELS_HPP

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/runtime/frame_pair.hpp"
#include "the_eyes/runtime/pipeline_result.hpp"
#include "the_eyes/runtime/static_scene.hpp"
#include "the_eyes/scenegraph/pddl.hpp"
#include "the_eyes/types.hpp"

namespace the_eyes::runtime {

// Maps world metres to panel pixels for the plan view. Kept explicit rather
// than derived per call so the plan and elevation views agree on scale and an
// object cannot appear in different places in the two.
struct WorldView {
  double centre_x_m = 0.0;   // world point at the panel's centre
  double centre_y_m = 0.0;
  double px_per_m = 500.0;
  int plan_height_px = 360;  // the rest of the panel is the elevation strip
};

// A view that fits everything worth seeing -- every object footprint AND both
// camera positions -- into the plan region, with a margin.
//
// Hand-tuning the scale does not survive a rig change: the cameras sit ~0.43 m
// in front of the world origin while the table reaches 0.53 m behind it, so a
// view chosen for one is wrong for the other, and the first version of this
// panel drew the cameras on top of the elevation strip because of exactly that.
WorldView FitWorldView(const SceneState& scene, const fusion::ExtrinsicsSet& extrinsics,
                       int panel_w, int plan_h);

// Text with a black outline, legible over an arbitrary camera frame. Shared so
// the viewer's status strip and the console's overlays match.
void DrawOutlinedText(cv::Mat& img, const std::string& text, cv::Point at,
                      const cv::Scalar& colour);

// PANEL A -- one camera's colour frame with detections, reprojected fused poses
// and per-view timings.
//
// Extracted from Viewer so the console and `--show` draw the same picture from
// the same code. Two copies of this would drift, and the difference would show
// up as "the console disagrees with the viewer" long before anyone suspected
// duplicated drawing.
//
// `panel` must be the frame's own size; the caller owns the layout.
void DrawCameraPanel(cv::Mat& panel, const std::string& view, const camera::CameraFrame& frame,
                     const PipelineResult& result, const fusion::ExtrinsicsSet& extrinsics,
                     const std::vector<std::string>& static_ids);

// PANEL B -- world view: plan (XY) above, elevation (XZ) below.
//
// Draws, per object: the world-axis-aligned footprint from `extent`, a heading
// tick from pose_w's rotation, and -- only when has_uncertainty() -- a 1-sigma
// circle. The sentinel is negative precisely so absent uncertainty cannot be
// drawn as a confident point estimate, so this must never fall back to zero.
//
// `unknown_object` gets a dashed box and NO heading tick: its rotation is
// meaningless by contract (INTERFACES.md §5) and drawing one would assert
// something the pipeline never claimed.
//
// The elevation strip is not decoration. The only checkable accuracy figure
// this project has produced is a 50 mm cube whose fused z read 29.7 mm against
// a true 25 mm, and height error is invisible in a plan view.
void DrawWorldPanel(cv::Mat& panel, const SceneState& scene,
                    const std::vector<std::string>& static_ids,
                    const fusion::ExtrinsicsSet& extrinsics, const WorldView& view);

// PANEL C -- the scene graph as nodes and edges.
//
// Edges come from `facts` -- the same strings the planner would receive -- so
// the panel has no predicate rules of its own and cannot drift from what was
// emitted.
//
// Layout is DETERMINISTIC: slots are assigned by sorted object id, with static
// objects pinned to the bottom row. A force-directed layout that re-solves each
// frame is unreadable at 19 pairs/s, and worse, a viewer reads the motion as a
// change in the scene when nothing changed.
void DrawGraphPanel(cv::Mat& panel, const SceneState& scene,
                    const std::vector<std::string>& static_ids,
                    const std::vector<std::string>& facts);

// PANEL D -- the literal (:init ...) block.
//
// Rendered verbatim, never reformatted: this panel exists to answer "what does
// the consumer actually get", and a prettified version would answer a different
// question. Warnings are drawn beneath it rather than folded in.
void DrawPddlPanel(cv::Mat& panel, const std::string& init_block,
                   const std::vector<std::string>& warnings);

// Exposed for the test, and because the graph panel's determinism is a claim
// worth being able to check directly rather than by pixel comparison.
std::vector<std::string> GraphNodeOrder(const SceneState& scene,
                                        const std::vector<std::string>& static_ids);

// Parsed `(pred a b)` -> {pred, a, b}. Facts of other arities are ignored
// rather than guessed at.
struct FactEdge {
  std::string predicate;
  std::string from;
  std::string to;
};
std::vector<FactEdge> ParseBinaryFacts(const std::vector<std::string>& facts);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_CONSOLE_PANELS_HPP
