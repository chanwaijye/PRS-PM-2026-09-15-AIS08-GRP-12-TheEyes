// One synchronised frame from every camera -- the runner's unit of work.
//
// Fusion (INTERFACES.md §5) consumes both views' estimates in one call with
// one timestamp, so the runner never processes a single camera's frame on its
// own. A FramePair is what the live pairer emits, what the recorder writes,
// what replay reads back, and what the pipeline consumes.
#ifndef THE_EYES_RUNTIME_FRAME_PAIR_HPP
#define THE_EYES_RUNTIME_FRAME_PAIR_HPP

#include <cstdint>
#include <map>
#include <string>

#include "the_eyes/camera/frame.hpp"
#include "the_eyes/detector/detector.hpp"
#include "the_eyes/pose/pose_estimator.hpp"

namespace the_eyes::runtime {

struct FramePair {
  uint64_t pair_index = 0;   // monotonic within a run or a recorded session
  // The reference camera's capture time (view_names[0], the sync master).
  // This is the stamp fusion advances its filters by.
  uint64_t stamp_ns = 0;
  // Signed skew, ms: frames[view_names[0]] - frames[view_names[1]]. Sign is
  // kept because a fixed offset is a known constant and jitter is what matters.
  double skew_ms = 0.0;
  std::map<std::string, camera::CameraFrame> frames;   // keyed by camera_id
  // Frames from either camera that went unused since the previous pair --
  // trimmed from the ring, rejected by the skew gate, or missing from the
  // source's own frame numbering. Zero means nothing was lost.
  uint64_t frames_dropped_since_last = 0;
};

// The pose and detector modules each define their own frame struct rather
// than depending on the camera module (AGENTS.md rule 1). cv::Mat copies are
// refcounted headers, so these are cheap and share pixels.
pose::Frame ToPoseFrame(const camera::CameraFrame& frame);
detector::DetectorInput ToDetectorInput(const camera::CameraFrame& frame);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_FRAME_PAIR_HPP
