// The console: one window showing the whole chain for one pair.
//
// `--show` draws the camera views. `--show=console` adds the three panels that
// carry what is actually graded -- the world-frame geometry fusion produced,
// the scene graph derived from it, and the literal (:init ...) a consumer
// receives (docs/VIZ_PLAN.md).
//
//   +---------------------------+---------------------------+
//   | cam_front 848x480         | cam_over 848x480          |
//   +---------------------------+-------------+-------------+
//   | world: plan + elevation   | scene graph | (:init ...) |
//   |                    848    |         424 |         424 |
//   +---------------------------+-------------+-------------+
//   | status: pair, skew, per-stage ms, fps, drops          |
//   +-------------------------------------------------------+
//
// A SINK, NEVER A STAGE. Same status as Viewer: it reads const inputs, cannot
// signal back-pressure, and `Show` returning false means only that the user
// asked to quit. The measured live budget is p50 51.7 ms per pair and a
// demonstration window must not spend any of it.
#ifndef THE_EYES_RUNTIME_CONSOLE_HPP
#define THE_EYES_RUNTIME_CONSOLE_HPP

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/runtime/frame_pair.hpp"
#include "the_eyes/runtime/pipeline_result.hpp"

namespace the_eyes::runtime {

class Console {
 public:
  // Throws std::runtime_error if no display is available.
  Console(std::vector<std::string> view_names, fusion::ExtrinsicsSet extrinsics,
          std::string window_name = "The Eyes - console");
  ~Console();

  Console(const Console&) = delete;
  Console& operator=(const Console&) = delete;

  // Returns false when the user asked to quit (q or ESC).
  bool Show(const FramePair& pair, const PipelineResult& result, double fps);

  // The canvas, with no window anywhere in the call. Public because it is what
  // makes the layout testable headlessly, and because `--show-record` writes
  // exactly this.
  static cv::Mat Compose(const std::vector<std::string>& view_names,
                         const fusion::ExtrinsicsSet& extrinsics, const FramePair& pair,
                         const PipelineResult& result, double fps);

 private:
  std::vector<std::string> view_names_;
  fusion::ExtrinsicsSet extrinsics_;
  std::string window_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_CONSOLE_HPP
