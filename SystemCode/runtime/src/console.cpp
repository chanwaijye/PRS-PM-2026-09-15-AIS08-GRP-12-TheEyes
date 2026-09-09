#include "the_eyes/runtime/console.hpp"

#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "the_eyes/runtime/console_panels.hpp"

namespace the_eyes::runtime {
namespace {

constexpr int kCamW = 848;
constexpr int kCamH = 480;
constexpr int kLowerH = 520;
constexpr int kStatusH = 24;
constexpr int kGraphW = 424;

}  // namespace

Console::Console(std::vector<std::string> view_names, fusion::ExtrinsicsSet extrinsics,
                 ArmConfig arms, std::string window_name, bool window)
    : view_names_(std::move(view_names)),
      extrinsics_(std::move(extrinsics)),
      arms_(std::move(arms)),
      window_(std::move(window_name)),
      window_open_(window) {
  if (!window_open_) return;
  if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr) {
    throw std::runtime_error("--show=console: no DISPLAY and no WAYLAND_DISPLAY, so no window "
                             "can open");
  }
  // OpenCV 4.6 opens no window on Wayland; the same trap Viewer documents.
  if (std::getenv("WAYLAND_DISPLAY") != nullptr && std::getenv("QT_QPA_PLATFORM") == nullptr) {
    ::setenv("QT_QPA_PLATFORM", "xcb", 0);
  }
  cv::namedWindow(window_, cv::WINDOW_NORMAL);
  cv::resizeWindow(window_, 1696 / 2 * 2, (kCamH + kLowerH + kStatusH));
}

Console::~Console() {
  // Flush the video before the window goes, so a truncated file cannot be the
  // last thing a run leaves behind.
  recorder_.reset();
  try {
    if (window_open_) cv::destroyWindow(window_);
  } catch (...) {
  }
}

void Console::StartRecording(const std::string& path, cv::Size frame_size, double fps,
                             bool lossless) {
  recorder_ = std::make_unique<ConsoleRecorder>(path, frame_size, fps, !lossless);
}

cv::Mat Console::Compose(const std::vector<std::string>& view_names,
                         const fusion::ExtrinsicsSet& extrinsics, const FramePair& pair,
                         const PipelineResult& result, double fps, const ArmConfig& arms) {
  const int n = static_cast<int>(view_names.size());
  const int width = std::max(1, n) * kCamW;
  cv::Mat canvas(kCamH + kLowerH + kStatusH, width, CV_8UC3, cv::Scalar(20, 20, 20));

  for (int i = 0; i < n; ++i) {
    const std::string& view = view_names[i];
    cv::Mat slot = canvas(cv::Rect(i * kCamW, 0, kCamW, kCamH));
    auto it = pair.frames.find(view);
    if (it == pair.frames.end() || it->second.rgb.empty()) {
      DrawOutlinedText(slot, view + ": no frame", {12, 28}, cv::Scalar(90, 90, 200));
      continue;
    }
    cv::Mat bgr;
    cv::cvtColor(it->second.rgb, bgr, cv::COLOR_RGB2BGR);
    DrawCameraPanel(bgr, view, it->second, result, extrinsics, result.static_ids);
    // A frame that is not the layout's size is letterboxed rather than
    // stretched: a stretched panel would put the reprojected axes somewhere
    // they are not.
    if (bgr.size() != cv::Size(kCamW, kCamH)) {
      const double s = std::min(static_cast<double>(kCamW) / bgr.cols,
                                static_cast<double>(kCamH) / bgr.rows);
      cv::Mat fitted;
      cv::resize(bgr, fitted, cv::Size(), s, s, cv::INTER_AREA);
      fitted.copyTo(slot(cv::Rect(0, 0, fitted.cols, fitted.rows)));
    } else {
      bgr.copyTo(slot);
    }
  }

  const int lower_y = kCamH;
  const int world_w = std::max(1, width - 2 * kGraphW);
  cv::Mat world = canvas(cv::Rect(0, lower_y, world_w, kLowerH));
  cv::Mat graph = canvas(cv::Rect(world_w, lower_y, kGraphW, kLowerH));
  cv::Mat pddl = canvas(cv::Rect(world_w + kGraphW, lower_y, kGraphW, kLowerH));

  const WorldView view = FitWorldView(result.scene, extrinsics, world_w, kLowerH - 150);
  DrawWorldPanel(world, result.scene, result.static_ids, extrinsics, view);
  if (arms.enabled) {
    // Two extra geometric fuses on estimates the pipeline already produced.
    // Drawn AFTER the fused poses so the solid answer sits on top of the ghosts.
    DrawArmOverlay(world,
                   SingleViewArms(result, extrinsics, arms.fusion, arms.extents, view_names),
                   result.scene.objects.size() - result.static_ids.size(), view);
  }
  DrawGraphPanel(graph, result.scene, result.static_ids, result.pddl.facts);
  DrawPddlPanel(pddl, result.pddl.init_block, result.pddl.warnings);

  cv::Mat status = canvas(cv::Rect(0, lower_y + kLowerH, width, kStatusH));
  status.setTo(cv::Scalar(12, 12, 12));
  std::ostringstream os;
  os << "pair " << result.pair_index << "   skew " << std::fixed << std::setprecision(1)
     << result.skew_ms << " ms   fusion " << result.timings.fusion_ms << "   graph "
     << result.timings.scenegraph_ms << "   total " << result.timings.total_ms << " ms   "
     << std::setprecision(1) << fps << " fps   dropped "
     << result.frames_dropped_since_last << "   objects " << result.scene.objects.size()
     << "   facts " << result.pddl.facts.size();
  // Ground truth is what ADD-S is read against, and nothing fills that table
  // yet. Saying so on screen stops a screenshot implying otherwise.
  os << "   GT: none";
  DrawOutlinedText(status, os.str(), {8, 20}, cv::Scalar(0, 220, 220));
  return canvas;
}

bool Console::Show(const FramePair& pair, const PipelineResult& result, double fps) {
  const cv::Mat canvas = Compose(view_names_, extrinsics_, pair, result, fps, arms_);
  if (recorder_) recorder_->Offer(canvas);
  if (!window_open_) return true;
  cv::imshow(window_, canvas);
  const int key = cv::waitKey(1) & 0xff;
  return !(key == 'q' || key == 27);
}

}  // namespace the_eyes::runtime
