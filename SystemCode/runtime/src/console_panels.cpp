#include "the_eyes/runtime/console_panels.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>

#include <iomanip>

#include <opencv2/imgproc.hpp>

namespace the_eyes::runtime {
namespace {

const cv::Scalar kInk(230, 230, 230);
const cv::Scalar kDim(120, 120, 120);
const cv::Scalar kTable(70, 90, 70);
const cv::Scalar kObject(80, 200, 255);
const cv::Scalar kStatic(140, 140, 140);
const cv::Scalar kUnknown(200, 120, 255);
const cv::Scalar kSigma(60, 160, 220);
const cv::Scalar kCamera(255, 200, 90);
const cv::Scalar kAmber(60, 190, 255);
const cv::Scalar kBg(28, 28, 28);

void Label(cv::Mat& m, const std::string& s, cv::Point at, const cv::Scalar& c,
           double scale = 0.4) {
  cv::putText(m, s, at, cv::FONT_HERSHEY_SIMPLEX, scale, c, 1, cv::LINE_AA);
}

bool IsStatic(const std::string& id, const std::vector<std::string>& static_ids) {
  return std::find(static_ids.begin(), static_ids.end(), id) != static_ids.end();
}

void DashedRect(cv::Mat& m, const cv::Rect& r, const cv::Scalar& c) {
  const int step = 6;
  for (int x = r.x; x < r.x + r.width; x += step * 2) {
    const int x2 = std::min(x + step, r.x + r.width);
    cv::line(m, {x, r.y}, {x2, r.y}, c, 1, cv::LINE_AA);
    cv::line(m, {x, r.y + r.height}, {x2, r.y + r.height}, c, 1, cv::LINE_AA);
  }
  for (int y = r.y; y < r.y + r.height; y += step * 2) {
    const int y2 = std::min(y + step, r.y + r.height);
    cv::line(m, {r.x, y}, {r.x, y2}, c, 1, cv::LINE_AA);
    cv::line(m, {r.x + r.width, y}, {r.x + r.width, y2}, c, 1, cv::LINE_AA);
  }
}


bool Project(const Eigen::Matrix3d& K, const Eigen::Isometry3d& T_cam_world,
             const Eigen::Vector3d& p_w, cv::Point* out) {
  const Eigen::Vector3d p = T_cam_world * p_w;
  if (p.z() <= 1e-6) return false;
  const Eigen::Vector3d uv = K * (p / p.z());
  *out = cv::Point(static_cast<int>(std::lround(uv.x())), static_cast<int>(std::lround(uv.y())));
  return true;
}

}  // namespace

void DrawCameraPanel(cv::Mat& panel, const std::string& view,
                     const camera::CameraFrame& frame, const PipelineResult& result,
                     const fusion::ExtrinsicsSet& extrinsics,
                     const std::vector<std::string>& static_ids) {
    auto dets = result.detections.find(view);
    if (dets != result.detections.end()) {
      for (const Detection& d : dets->second.detections) {
        cv::rectangle(panel, d.bbox, cv::Scalar(0, 200, 255), 1);
        std::ostringstream os;
        os << d.cls << " " << std::fixed << std::setprecision(2) << d.score << " #" << d.track_id;
        Label(panel, os.str(), d.bbox.tl(), cv::Scalar(0, 200, 255));
      }
    }

    auto ex = extrinsics.cameras.find(view);
    if (ex != extrinsics.cameras.end()) {
      const Eigen::Isometry3d T_cam_world = ex->second.T_world_cam.inverse();
      for (const ObjectState& o : result.scene.objects) {
        bool is_static = false;
        for (const std::string& id : static_ids) is_static = is_static || id == o.id;
        if (is_static) continue;
        cv::Point origin;
        if (!Project(frame.K, T_cam_world, o.pose_w.translation(), &origin)) continue;
        const double axis = 0.03;
        const cv::Scalar colours[3] = {cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0),
                                       cv::Scalar(255, 0, 0)};
        for (int a = 0; a < 3; ++a) {
          cv::Point tip;
          const Eigen::Vector3d p = o.pose_w * (Eigen::Vector3d::Unit(a) * axis);
          if (Project(frame.K, T_cam_world, p, &tip)) cv::line(panel, origin, tip, colours[a], 2);
        }
        std::ostringstream os;
        os << o.id << " c=" << std::fixed << std::setprecision(2) << o.confidence;
        Label(panel, os.str(), origin + cv::Point(6, 0), cv::Scalar(255, 255, 255));
      }
    }

    std::ostringstream hud;
    hud << view << "  det " << std::fixed << std::setprecision(1);
    auto dm = result.timings.detect_ms.find(view);
    hud << (dm != result.timings.detect_ms.end() ? dm->second : 0.0) << " ms  pose ";
    auto pm = result.timings.pose_ms.find(view);
    hud << (pm != result.timings.pose_ms.end() ? pm->second : 0.0) << " ms";
    Label(panel, hud.str(), cv::Point(8, 22), cv::Scalar(255, 255, 255));
}

void DrawOutlinedText(cv::Mat& img, const std::string& text, cv::Point at,
                      const cv::Scalar& colour) {
  cv::putText(img, text, at + cv::Point(0, -4), cv::FONT_HERSHEY_SIMPLEX, 0.45,
              cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
  cv::putText(img, text, at + cv::Point(0, -4), cv::FONT_HERSHEY_SIMPLEX, 0.45, colour, 1,
              cv::LINE_AA);
}

std::vector<FactEdge> ParseBinaryFacts(const std::vector<std::string>& facts) {
  std::vector<FactEdge> out;
  for (const std::string& f : facts) {
    std::string body = f;
    if (!body.empty() && body.front() == '(') body.erase(body.begin());
    if (!body.empty() && body.back() == ')') body.pop_back();
    std::istringstream is(body);
    std::vector<std::string> tok;
    for (std::string t; is >> t;) tok.push_back(t);
    // Only binary relations are edges. `(clear x)` and `(occluded_from x view)`
    // are node properties or view facts, not links between two objects, and
    // guessing at them would draw relationships the scene graph never asserted.
    if (tok.size() == 3) out.push_back({tok[0], tok[1], tok[2]});
  }
  return out;
}

std::vector<std::string> GraphNodeOrder(const SceneState& scene,
                                        const std::vector<std::string>& static_ids) {
  std::vector<std::string> movable, fixed;
  for (const ObjectState& o : scene.objects) {
    (IsStatic(o.id, static_ids) ? fixed : movable).push_back(o.id);
  }
  // Sorted, so the layout depends on the scene's CONTENT and not on the order
  // fusion happened to emit it in. Shuffling the object vector must not move a
  // node, or the viewer reads motion that did not happen.
  std::sort(movable.begin(), movable.end());
  std::sort(fixed.begin(), fixed.end());
  movable.insert(movable.end(), fixed.begin(), fixed.end());
  return movable;
}

WorldView FitWorldView(const SceneState& scene, const fusion::ExtrinsicsSet& extrinsics,
                       int panel_w, int plan_h) {
  WorldView v;
  v.plan_height_px = plan_h;
  double x0 = 0.0, x1 = 0.0, y0 = 0.0, y1 = 0.0;
  bool any = false;
  auto grow = [&](double x, double y) {
    if (!any) { x0 = x1 = x; y0 = y1 = y; any = true; return; }
    x0 = std::min(x0, x); x1 = std::max(x1, x);
    y0 = std::min(y0, y); y1 = std::max(y1, y);
  };
  for (const ObjectState& o : scene.objects) {
    const Eigen::Vector3d t = o.pose_w.translation();
    grow(t.x() - o.extent.x(), t.y() - o.extent.y());
    grow(t.x() + o.extent.x(), t.y() + o.extent.y());
  }
  for (const auto& [id, cam] : extrinsics.cameras) {
    const Eigen::Vector3d p = cam.T_world_cam.translation();
    grow(p.x(), p.y());
  }
  if (!any) { grow(-0.5, -0.5); grow(0.5, 0.5); }
  v.centre_x_m = (x0 + x1) / 2.0;
  v.centre_y_m = (y0 + y1) / 2.0;
  const double margin = 1.12;
  const double span_x = std::max(1e-3, (x1 - x0) * margin);
  const double span_y = std::max(1e-3, (y1 - y0) * margin);
  v.px_per_m = std::min(panel_w / span_x, plan_h / span_y);
  return v;
}

void DrawWorldPanel(cv::Mat& panel, const SceneState& scene,
                    const std::vector<std::string>& static_ids,
                    const fusion::ExtrinsicsSet& extrinsics, const WorldView& view) {
  panel.setTo(kBg);
  const int W = panel.cols;
  const int plan_h = std::min(view.plan_height_px, panel.rows - 40);
  const double px_per_m = view.px_per_m;

  // Sub-images, so nothing drawn for the plan can bleed into the elevation and
  // vice versa. The first version put the cameras on top of the elevation strip.
  cv::Mat plan_roi = panel(cv::Rect(0, 0, W, plan_h));
  cv::Mat elev_roi = panel(cv::Rect(0, plan_h, W, panel.rows - plan_h));

  // Plan view: +x right, +y UP the panel, so the picture matches the world
  // frame's handedness rather than image coordinates.
  auto plan = [&](double x, double y) {
    return cv::Point(static_cast<int>(W / 2.0 + (x - view.centre_x_m) * px_per_m),
                     static_cast<int>(plan_h / 2.0 - (y - view.centre_y_m) * px_per_m));
  };
  const int elev_zero = elev_roi.rows - 24;
  auto elev = [&](double x, double z) {
    return cv::Point(static_cast<int>(W / 2.0 + (x - view.centre_x_m) * px_per_m),
                     static_cast<int>(elev_zero - z * px_per_m));
  };

  cv::line(panel, {0, plan_h}, {W, plan_h}, kDim, 1);
  Label(panel, "plan (x right, y up)", {8, 16}, kDim, 0.38);
  Label(panel, "elevation (x, z)", {8, plan_h + 16}, kDim, 0.38);
  cv::line(elev_roi, {0, elev_zero}, {W, elev_zero}, kDim, 1);   // the table top, z = 0

  for (const auto& [id, cam] : extrinsics.cameras) {
    const Eigen::Vector3d p = cam.T_world_cam.translation();
    const cv::Point c = plan(p.x(), p.y());
    cv::drawMarker(plan_roi, c, kCamera, cv::MARKER_TRIANGLE_UP, 9, 1, cv::LINE_AA);
    Label(plan_roi, id, c + cv::Point(7, 4), kCamera, 0.34);
    // Optical axis, projected onto the table plane: which way each camera looks
    // is the whole reason two views help.
    const Eigen::Vector3d fwd = cam.T_world_cam.linear() * Eigen::Vector3d::UnitZ();
    cv::line(plan_roi, c, plan(p.x() + fwd.x() * 0.15, p.y() + fwd.y() * 0.15), kCamera, 1,
             cv::LINE_AA);
  }

  for (const ObjectState& o : scene.objects) {
    const bool is_static = IsStatic(o.id, static_ids);
    const bool unknown = o.cls == "unknown_object";
    const cv::Scalar colour = is_static ? kStatic : (unknown ? kUnknown : kObject);
    const Eigen::Vector3d t = o.pose_w.translation();
    const Eigen::Vector3d e = o.extent;

    const cv::Point a = plan(t.x() - e.x(), t.y() + e.y());
    const cv::Point b = plan(t.x() + e.x(), t.y() - e.y());
    const cv::Rect r(a, b);
    if (unknown) {
      DashedRect(plan_roi, r, colour);
    } else {
      cv::rectangle(plan_roi, r, is_static ? kTable : colour, 1, cv::LINE_AA);
    }

    if (!is_static && !unknown) {
      // Heading only: the box itself is world-axis-aligned by contract, so
      // rotating it would assert an extent the pipeline never produced.
      const Eigen::Vector3d h = o.pose_w.linear() * Eigen::Vector3d::UnitX();
      cv::line(plan_roi, plan(t.x(), t.y()),
               plan(t.x() + h.x() * (e.x() + 0.02), t.y() + h.y() * (e.y() + 0.02)), colour, 1,
               cv::LINE_AA);
    }
    if (has_uncertainty(o)) {
      cv::circle(plan_roi, plan(t.x(), t.y()),
                 std::max(1, static_cast<int>(o.position_sigma_m * px_per_m)), kSigma, 1,
                 cv::LINE_AA);
    }
    if (!is_static) Label(plan_roi, o.id, plan(t.x(), t.y()) + cv::Point(4, -4), colour, 0.34);

    const cv::Rect er(elev(t.x() - e.x(), t.z() + e.z()), elev(t.x() + e.x(), t.z() - e.z()));
    cv::rectangle(elev_roi, er, is_static ? kTable : colour, 1, cv::LINE_AA);
  }
}

void DrawGraphPanel(cv::Mat& panel, const SceneState& scene,
                    const std::vector<std::string>& static_ids,
                    const std::vector<std::string>& facts) {
  panel.setTo(kBg);
  Label(panel, "scene graph", {8, 16}, kDim, 0.38);
  const std::vector<std::string> order = GraphNodeOrder(scene, static_ids);
  if (order.empty()) {
    Label(panel, "(no objects)", {8, 40}, kDim);
    return;
  }

  std::map<std::string, cv::Point> at;
  const int n_static = static_cast<int>(
      std::count_if(order.begin(), order.end(),
                    [&](const std::string& id) { return IsStatic(id, static_ids); }));
  const int n_movable = static_cast<int>(order.size()) - n_static;
  const int top_y = 60;
  const int bottom_y = panel.rows - 40;
  for (size_t i = 0; i < order.size(); ++i) {
    const bool is_static = IsStatic(order[i], static_ids);
    const int count = is_static ? std::max(1, n_static) : std::max(1, n_movable);
    const int idx = is_static ? static_cast<int>(i) - n_movable : static_cast<int>(i);
    const int x = static_cast<int>(panel.cols * (idx + 1.0) / (count + 1.0));
    at[order[i]] = {x, is_static ? bottom_y : top_y};
  }

  for (const FactEdge& e : ParseBinaryFacts(facts)) {
    auto a = at.find(e.from), b = at.find(e.to);
    if (a == at.end() || b == at.end()) continue;
    cv::line(panel, a->second, b->second, kDim, 1, cv::LINE_AA);
    const cv::Point mid((a->second.x + b->second.x) / 2, (a->second.y + b->second.y) / 2);
    Label(panel, e.predicate, mid + cv::Point(3, -3), kAmber, 0.34);
  }

  for (const auto& [id, p] : at) {
    const bool is_static = IsStatic(id, static_ids);
    cv::circle(panel, p, 5, is_static ? kStatic : kObject, cv::FILLED, cv::LINE_AA);
    Label(panel, id, p + cv::Point(-8, is_static ? 18 : -10), kInk, 0.34);
  }
}

void DrawPddlPanel(cv::Mat& panel, const std::string& init_block,
                   const std::vector<std::string>& warnings) {
  panel.setTo(kBg);
  Label(panel, "(:init ...) as emitted", {8, 16}, kDim, 0.38);
  int y = 38;
  std::istringstream is(init_block);
  for (std::string line; std::getline(is, line) && y < panel.rows - 16;) {
    Label(panel, line, {8, y}, kInk, 0.38);
    y += 15;
  }
  for (const std::string& w : warnings) {
    if (y >= panel.rows - 6) break;
    Label(panel, "! " + w, {8, y}, kAmber, 0.34);
    y += 14;
  }
}

}  // namespace the_eyes::runtime
