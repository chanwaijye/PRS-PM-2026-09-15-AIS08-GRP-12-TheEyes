// Optional OpenCV window (--show): both views side by side with detections,
// projected fused poses and the per-stage timings. For the demo video and for
// seeing what the detector does on real frames; it is not on the data path.
#ifndef THE_EYES_RUNTIME_VIEWER_HPP
#define THE_EYES_RUNTIME_VIEWER_HPP

#include <string>
#include <vector>

#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/runtime/frame_pair.hpp"
#include "the_eyes/runtime/pipeline_result.hpp"

namespace the_eyes::runtime {

class Viewer {
 public:
  // Throws std::runtime_error if no display is available. On a Wayland
  // session forces QT_QPA_PLATFORM=xcb (OpenCV 4.6 opens no window
  // otherwise; see docs/STATUS.md "Known nuisances").
  Viewer(std::vector<std::string> view_names, fusion::ExtrinsicsSet extrinsics,
         std::string window_name = "The Eyes");
  ~Viewer();

  // Returns false when the user asked to quit (q or ESC).
  bool Show(const FramePair& pair, const PipelineResult& result, double fps);

 private:
  std::vector<std::string> view_names_;
  fusion::ExtrinsicsSet extrinsics_;
  std::string window_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_VIEWER_HPP
