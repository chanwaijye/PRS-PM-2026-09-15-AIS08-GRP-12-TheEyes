// Two RealSense cameras, opened by serial, drained by one grabber thread
// each into a FramePairer. Compiled only with librealsense.
//
// The constructor does the whole bring-up and refuses to return a source
// that is not synchronised: sync mode read back from each device, warm-up
// frames discarded, then sync.startup_check_frames pairs analysed with
// camera::AnalyseSkew (INTERFACES.md §1: within one frame period, asserted
// at startup and logged).
#ifndef THE_EYES_RUNTIME_LIVE_FRAME_SOURCE_HPP
#define THE_EYES_RUNTIME_LIVE_FRAME_SOURCE_HPP

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>
#include <thread>
#include <vector>

#include "the_eyes/camera/config.hpp"
#include "the_eyes/camera/realsense_camera.hpp"
#include "the_eyes/camera/sync.hpp"
#include "the_eyes/runtime/frame_pairer.hpp"
#include "the_eyes/runtime/frame_source.hpp"
#include "the_eyes/runtime/runtime_config.hpp"

namespace the_eyes::runtime {

class LiveFrameSource : public FrameSource {
 public:
  // Throws std::runtime_error with the full reason on any bring-up failure.
  LiveFrameSource(const camera::CameraAgentConfig& cameras, PairerConfig pairer, LiveConfig live,
                  std::ostream& log);
  ~LiveFrameSource() override;

  std::optional<FramePair> NextPair(int timeout_ms) override;
  bool exhausted() const override { return false; }
  const std::vector<std::string>& camera_ids() const override { return camera_ids_; }
  const std::map<std::string, CameraMeta>& meta() const override { return meta_; }
  std::string describe() const override;

  const camera::SyncReport& startup_sync_report() const { return startup_sync_; }
  PairerStats pairer_stats() const { return pairer_.stats(); }
  const camera::StreamConfig& streams() const { return streams_; }

 private:
  void GrabLoop(const std::string& camera_id);

  std::vector<std::string> camera_ids_;
  camera::StreamConfig streams_;
  LiveConfig live_;
  std::map<std::string, std::unique_ptr<camera::RealSenseCamera>> cameras_;
  FramePairer pairer_;
  std::vector<std::thread> grabbers_;
  std::atomic<bool> stop_{false};
  std::mutex meta_mutex_;
  std::map<std::string, CameraMeta> meta_;
  camera::SyncReport startup_sync_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_LIVE_FRAME_SOURCE_HPP
