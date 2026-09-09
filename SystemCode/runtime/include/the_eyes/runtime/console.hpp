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

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/fusion/fusion.hpp"
#include "the_eyes/runtime/console_recorder.hpp"
#include "the_eyes/runtime/frame_pair.hpp"
#include "the_eyes/runtime/pipeline_result.hpp"

namespace the_eyes::runtime {

// What the A/B arms need. Optional: without it the console draws everything
// else and simply shows no ghosts, which is what the headless tests use.
struct ArmConfig {
  bool enabled = false;
  fusion::FusionConfig fusion;
  fusion::ExtentRegistry extents;
};

class Console {
 public:
  // Throws std::runtime_error if no display is available.
  // `window` false composes and records with no display at all, which is what
  // lets a video be re-shot from a replay over ssh.
  Console(std::vector<std::string> view_names, fusion::ExtrinsicsSet extrinsics,
          ArmConfig arms = {}, std::string window_name = "The Eyes - console",
          bool window = true);
  ~Console();

  Console(const Console&) = delete;
  Console& operator=(const Console&) = delete;

  // Returns false when the user asked to quit (q or ESC).
  bool Show(const FramePair& pair, const PipelineResult& result, double fps);

  // Also write every composed canvas to `path`. `lossless` blocks rather than
  // dropping when the encoder falls behind -- correct for a replay, wrong for
  // a live run (see console_recorder.hpp). Throws if the file cannot be opened.
  void StartRecording(const std::string& path, cv::Size frame_size, double fps, bool lossless);
  const ConsoleRecorder* recorder() const { return recorder_.get(); }

  // The canvas, with no window anywhere in the call. Public because it is what
  // makes the layout testable headlessly, and because `--show-record` writes
  // exactly this.
  static cv::Mat Compose(const std::vector<std::string>& view_names,
                         const fusion::ExtrinsicsSet& extrinsics, const FramePair& pair,
                         const PipelineResult& result, double fps, const ArmConfig& arms = {});

 private:
  std::vector<std::string> view_names_;
  fusion::ExtrinsicsSet extrinsics_;
  ArmConfig arms_;
  std::string window_;
  bool window_open_ = false;
  std::unique_ptr<ConsoleRecorder> recorder_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_CONSOLE_HPP
