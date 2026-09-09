// librealsense capture wrapper: one camera, opened by serial.
#ifndef THE_EYES_CAMERA_REALSENSE_CAMERA_HPP
#define THE_EYES_CAMERA_REALSENSE_CAMERA_HPP

#include <memory>
#include <string>
#include <vector>

#include "the_eyes/camera/config.hpp"
#include "the_eyes/camera/frame.hpp"

namespace the_eyes::camera {

class RealSenseCamera {
 public:
  // Throws std::runtime_error if the serial is not attached, or if the
  // requested stream profile is unavailable.
  RealSenseCamera(const CameraEntry& entry, const StreamConfig& streams);
  ~RealSenseCamera();

  RealSenseCamera(const RealSenseCamera&) = delete;
  RealSenseCamera& operator=(const RealSenseCamera&) = delete;

  // Blocks for the next aligned frame. Throws on timeout.
  //
  // Under StreamConfig::color_only the returned frame carries an EMPTY depth
  // Mat and will not satisfy CameraFrame::valid() -- that predicate asserts the
  // RGB-D contract, and a colour-only capture deliberately is not one.
  CameraFrame NextFrame(int timeout_ms = 5000);

  const std::string& camera_id() const;
  const std::string& serial() const;

  // What inter_cam_sync_mode the device REPORTS after being set. A unit with no
  // sync cable can accept the option and do nothing, so the value is read back
  // rather than assumed -- risk R4b depends on this actually taking effect.
  int actual_sync_mode() const;
  // Colour controls as the device reports them after the requested values were
  // applied. Recorded with every session: a capture whose exposure is unknown
  // cannot be compared with another one.
  bool actual_auto_exposure() const;
  int actual_exposure() const;
  int actual_gain() const;
  bool actual_auto_white_balance() const;
  int actual_white_balance_k() const;
  double depth_scale_m() const;
  // librealsense timestamp domain, e.g. "Global Time" or "System Time". Skew
  // between two cameras is only meaningful if both report the same domain.
  std::string timestamp_domain() const;
  const std::vector<std::string>& warnings() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace the_eyes::camera

#endif  // THE_EYES_CAMERA_REALSENSE_CAMERA_HPP
