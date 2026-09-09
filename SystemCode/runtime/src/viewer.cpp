#include "the_eyes/runtime/viewer.hpp"

#include "the_eyes/runtime/console_panels.hpp"

#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace the_eyes::runtime {
namespace {



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

    DrawCameraPanel(img, view, frame, result, extrinsics_, result.static_ids);
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
  DrawOutlinedText(canvas, status.str(), cv::Point(8, canvas.rows - 8), cv::Scalar(0, 255, 255));

  cv::imshow(window_, canvas);
  const int key = cv::waitKey(1) & 0xff;
  return !(key == 'q' || key == 27);
}

}  // namespace the_eyes::runtime
