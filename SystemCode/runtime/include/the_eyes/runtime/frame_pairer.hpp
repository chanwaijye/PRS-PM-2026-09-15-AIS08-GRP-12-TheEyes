// Pairs frames from independently-buffered cameras by nearest timestamp.
//
// librealsense gives one pipeline per camera with its own queue, and the two
// never start at the same instant. A grabber thread per camera drains its
// pipeline at full rate and pushes here; the consumer asks for the next pair.
// Between two consumer calls the rings keep turning over, so a main loop that
// is slower than the cameras (a 1.3 s FoundationPose registration, say) gets
// the freshest agreeing pair next time, not a backlog. Everything it did not
// use is counted, never silently discarded.
//
// Pure C++ -- no librealsense -- so the whole policy is unit-testable.
#ifndef THE_EYES_RUNTIME_FRAME_PAIRER_HPP
#define THE_EYES_RUNTIME_FRAME_PAIRER_HPP

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "the_eyes/runtime/frame_pair.hpp"

namespace the_eyes::runtime {

struct PairerConfig {
  std::vector<std::string> camera_ids;   // view order; [0] is the reference. Two cameras.
  // Frames further apart than this never pair (INTERFACES.md §1: one frame
  // period). Comes from cameras.yaml sync.max_skew_ms.
  double max_skew_ms = 33.0;
  // Nominal frame period (1000 / fps). A candidate partner is only final once
  // no frame that has not arrived yet could be closer, and the next frame from
  // a camera is at least this far from its last -- so a partner within half a
  // period is final immediately, one further away waits for the next frame.
  // Without this rule the gate alone accepts an off-by-one pairing (32.5 ms on
  // a 30 fps rig) as readily as the true partner 0.5 ms away.
  double nominal_period_ms = 1000.0 / 30.0;
  // Frames kept per camera. Older frames are trimmed and counted.
  size_t ring_depth = 4;
};

struct PairerStats {
  uint64_t paired = 0;
  uint64_t dropped_skew = 0;    // could never pair within max_skew_ms
  uint64_t dropped_stale = 0;   // arrived late for an emitted pair, or trimmed
  uint64_t source_gaps = 0;     // frame_number discontinuities from the camera
  uint64_t timeouts = 0;        // WaitPair calls that returned nothing
  std::map<std::string, uint64_t> pushed;
};

class FramePairer {
 public:
  // Throws std::invalid_argument on fewer than one camera_id, a non-positive
  // skew, or an empty ring.
  explicit FramePairer(PairerConfig cfg);

  // Producer side; safe from any thread.
  void Push(camera::CameraFrame frame);
  // A grabber that died reports here; every later WaitPair/TryPair throws
  // std::runtime_error naming the camera, so the failure cannot be mistaken
  // for a quiet stall.
  void Fail(const std::string& camera_id, const std::string& what);

  // Consumer side. Blocks up to timeout_ms for an agreeing pair.
  std::optional<FramePair> WaitPair(int timeout_ms);
  // Non-blocking form of the same policy.
  std::optional<FramePair> TryPair();

  PairerStats stats() const;
  const PairerConfig& config() const { return cfg_; }

 private:
  std::optional<FramePair> TryPairLocked();
  void ThrowIfFailedLocked() const;

  PairerConfig cfg_;
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::map<std::string, std::deque<camera::CameraFrame>> rings_;
  std::map<std::string, uint64_t> last_frame_number_;
  std::map<std::string, uint64_t> last_consumed_stamp_ns_;
  uint64_t next_pair_index_ = 0;
  uint64_t pending_dropped_ = 0;   // unused frames since the last emitted pair
  uint64_t push_generation_ = 0;
  std::optional<std::string> failure_;
  PairerStats stats_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_FRAME_PAIRER_HPP
