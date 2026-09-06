#include "the_eyes/runtime/viewer.hpp"

#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace the_eyes::runtime {
namespace {

bool Project(const Eigen::Matrix3d& K, const Eigen::Isometry3d& T_cam_world,
             const Eigen::Vector3d& p_w, cv::Point* out) {
  const Eigen::Vector3d p = T_cam_world * p_w;
  if (p.z() <= 1e-6) return false;
  const Eigen::Vector3d uv = K * (p / p.z());
  *out = cv::Point(static_cast<int>(std::lround(uv.x())), static_cast<int>(std::lround(uv.y())));
  return true;
}

void Label(cv::Mat& img, const std::string& text, cv::Point at, const cv::Scalar& colour) {
  cv::putText(img, text, at + cv::Point(0, -4), cv::FONT_HERSHEY_SIMPLEX, 0.45,
              cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
  cv::putText(img, text, at + cv::Point(0, -4), cv::FONT_HERSHEY_SIMPLEX, 0.45, colour, 1,
              cv::LINE_AA);
}

}  // namespace

Viewer::Viewer(std::vector<std::string> view_names, fusion::ExtrinsicsSet extrinsics,
               std::string window_name)
    : view_names_(std::move(view_names)),
      extrinsics_(std::move(extrinsics)),
      window_(std::move(window_name)) {
  if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr) {
    throw std::runtime_error("--show: no DISPLAY and no WAYLAND_DISPLAY, so no window can open");
  }
  if (std::getenv("WAYLAND_DISPLAY") != nullptr && std::getenv("QT_QPA_PLATFORM") == nullptr) {
    ::setenv("QT_QPA_PLATFORM", "xcb", 0);
  }
  cv::namedWindow(window_, cv::WINDOW_AUTOSIZE);
}

Viewer::~Viewer() {
  try {
    cv::destroyWindow(window_);
  } catch (...) {
  }
}

bool Viewer::Show(const FramePair& pair, const PipelineResult& result, double fps) {
  std::vector<cv::Mat> panels;
  for (const std::string& view : view_names_) {
    auto it = pair.frames.find(view);
    if (it == pair.frames.end() || it->second.rgb.empty()) continue;
    const camera::CameraFrame& frame = it->second;
    cv::Mat img;
    cv::cvtColor(frame.rgb, img, cv::COLOR_RGB2BGR);

    auto dets = result.detections.find(view);
    if (dets != result.detections.end()) {
      for (const Detection& d : dets->second.detections) {
        cv::rectangle(img, d.bbox, cv::Scalar(0, 200, 255), 1);
        std::ostringstream os;
        os << d.cls << " " << std::fixed << std::setprecision(2) << d.score << " #" << d.track_id;
        Label(img, os.str(), d.bbox.tl(), cv::Scalar(0, 200, 255));
      }
    }

    auto ex = extrinsics_.cameras.find(view);
    if (ex != extrinsics_.cameras.end()) {
      const Eigen::Isometry3d T_cam_world = ex->second.T_world_cam.inverse();
      for (const ObjectState& o : result.scene.objects) {
        bool is_static = false;
        for (const std::string& id : result.static_ids) is_static = is_static || id == o.id;
        if (is_static) continue;
        cv::Point origin;
        if (!Project(frame.K, T_cam_world, o.pose_w.translation(), &origin)) continue;
        const double axis = 0.03;
        const cv::Scalar colours[3] = {cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0),
                                       cv::Scalar(255, 0, 0)};
        for (int a = 0; a < 3; ++a) {
          cv::Point tip;
          const Eigen::Vector3d p = o.pose_w * (Eigen::Vector3d::Unit(a) * axis);
          if (Project(frame.K, T_cam_world, p, &tip)) cv::line(img, origin, tip, colours[a], 2);
        }
        std::ostringstream os;
        os << o.id << " c=" << std::fixed << std::setprecision(2) << o.confidence;
        Label(img, os.str(), origin + cv::Point(6, 0), cv::Scalar(255, 255, 255));
      }
    }

    std::ostringstream hud;
    hud << view << "  det " << std::fixed << std::setprecision(1);
    auto dm = result.timings.detect_ms.find(view);
    hud << (dm != result.timings.detect_ms.end() ? dm->second : 0.0) << " ms  pose ";
    auto pm = result.timings.pose_ms.find(view);
    hud << (pm != result.timings.pose_ms.end() ? pm->second : 0.0) << " ms";
    Label(img, hud.str(), cv::Point(8, 22), cv::Scalar(255, 255, 255));
    panels.push_back(img);
  }
  if (panels.empty()) return true;

  cv::Mat canvas;
  cv::hconcat(panels, canvas);
  std::ostringstream status;
  status << "pair " << result.pair_index << "  skew " << std::fixed << std::setprecision(1)
         << result.skew_ms << " ms  fusion " << result.timings.fusion_ms << " ms  total "
         << result.timings.total_ms << " ms  " << std::setprecision(1) << fps << " fps  objects "
         << result.scene.objects.size() << "  facts " << result.pddl.facts.size();
  Label(canvas, status.str(), cv::Point(8, canvas.rows - 8), cv::Scalar(0, 255, 255));

  cv::imshow(window_, canvas);
  const int key = cv::waitKey(1) & 0xff;
  return !(key == 'q' || key == 27);
}

}  // namespace the_eyes::runtime
