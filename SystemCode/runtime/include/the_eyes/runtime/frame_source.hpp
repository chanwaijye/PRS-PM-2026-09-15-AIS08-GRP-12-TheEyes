// Where FramePairs come from: live cameras or a recorded session. The pipeline
// and the recorder see only this interface, which is what makes the replay
// path a faithful stand-in for the cameras (the camera module's mock mode,
// AGENTS.md §1).
#ifndef THE_EYES_RUNTIME_FRAME_SOURCE_HPP
#define THE_EYES_RUNTIME_FRAME_SOURCE_HPP

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "the_eyes/runtime/frame_pair.hpp"

namespace the_eyes::runtime {

// Per-camera facts that do not change within a session. Recorded into
// session.yaml and the metrics log so a replay reproduces them exactly.
struct CameraMeta {
  std::string serial;
  double depth_scale_m = 0.001;
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  int sync_mode_requested = 0;
  int sync_mode_actual = 0;
  std::string timestamp_domain;
  // Colour controls as the device reported them. Recorded so a session can say
  // what it was shot at -- two captures at different exposures are not
  // comparable, and until 2026-09-09 nothing recorded which was which.
  bool auto_exposure = true;
  int exposure = 0;   // device units, see camera/config.hpp
  int gain = 0;
  bool auto_white_balance = true;
  int white_balance_k = 0;
};

class FrameSource {
 public:
  virtual ~FrameSource() = default;
  // nullopt means no pair arrived within timeout_ms. For a live source that is
  // a stall; for replay it means the session is exhausted (see exhausted()).
  virtual std::optional<FramePair> NextPair(int timeout_ms) = 0;
  virtual bool exhausted() const = 0;
  // In view order: [0] is the reference camera.
  virtual const std::vector<std::string>& camera_ids() const = 0;
  virtual const std::map<std::string, CameraMeta>& meta() const = 0;
  // One line for logs: "live serials ..." or "replay <dir>".
  virtual std::string describe() const = 0;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_FRAME_SOURCE_HPP
