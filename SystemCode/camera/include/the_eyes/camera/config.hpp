// camera_agent configuration, loaded from config/cameras.yaml.
#ifndef THE_EYES_CAMERA_CONFIG_HPP
#define THE_EYES_CAMERA_CONFIG_HPP

#include <map>
#include <string>
#include <vector>

namespace the_eyes::camera {

struct CameraEntry {
  std::string camera_id;
  std::string serial;
  int inter_cam_sync_mode = 0;  // 0 default, 1 master, 2 slave
};

// Colour-sensor controls.
//
// The D435i colour stream runs auto-exposure and auto-white-balance by default,
// and until 2026-09-09 nothing here touched either. That is not a neutral
// default, it is an active control loop: turning the room light on moved the
// table surface by 2.7 grey levels because the camera stopped down to cancel it
// (docs/STATUS.md "Sim-to-real"). Auto also lets the two units pick different
// exposures for the same scene, which makes the views incomparable, and makes a
// recorded session unreproducible frame to frame.
//
// So the settings are stated rather than inherited. `auto_*` true keeps the old
// behaviour; setting it false requires the corresponding value, because a
// manual mode with no value is the one combination that silently does nothing.
struct ColorControl {
  bool auto_exposure = true;
  // DEVICE UNITS, not microseconds. Measured on this rig's D435i colour sensor:
  // range [1, 10000], step 1, default 166. The depth sensor's exposure option
  // IS in microseconds; the colour sensor's is not, and the two are easy to
  // confuse. Naming it `_us` would be a precision this project cannot verify,
  // so it is named for what it is and the measured range is written down.
  int exposure = 0;           // required when auto_exposure is false
  int gain = 0;               // required when auto_exposure is false; range [0, 128]
  bool auto_white_balance = true;
  int white_balance_k = 0;    // kelvin, range [2800, 6500] step 10

  bool manual_exposure() const { return !auto_exposure; }
  bool manual_white_balance() const { return !auto_white_balance; }
};

struct StreamConfig {
  int depth_width = 848;
  int depth_height = 480;
  int color_width = 848;
  int color_height = 480;
  int fps = 30;
  bool align_depth_to_color = true;
  // Open the colour stream only, leaving depth closed.
  //
  // NOT loaded from cameras.yaml: this is a capture-mode choice made by one
  // tool, not a property of the rig. Calibration needs it -- MEASURED on this
  // rig, two D435i units cannot both run 1920x1080 colour AND 848x480 depth at
  // once: the second pipeline.start() blocks indefinitely. Colour-only, both
  // units start in ~0.2 s and deliver frames. ChArUco corners come from the
  // colour sensor, so nothing is lost.
  //
  // Frames produced in this mode carry an EMPTY depth Mat and therefore do not
  // satisfy CameraFrame::valid(), which asserts the RGB-D contract.
  bool color_only = false;

  ColorControl color;
};

struct SyncConfig {
  double max_skew_ms = 33.0;
  int startup_check_frames = 30;
};

struct NoiseConfig {
  std::vector<double> distances_m{0.4, 0.6, 0.8, 1.0};
  double roi_fraction = 0.5;
  int frames_per_distance = 30;
};

struct CameraAgentConfig {
  // Ordered, because the order fixes nothing here -- lookups are by id -- but a
  // deterministic order keeps logs and reports stable.
  std::map<std::string, CameraEntry> cameras;
  StreamConfig streams;
  SyncConfig sync;
  NoiseConfig noise;

  // Throws std::out_of_range if the id is unknown.
  const CameraEntry& at(const std::string& camera_id) const;
};

// Throws std::runtime_error on a missing file, a missing serial, or two
// cameras sharing one serial.
CameraAgentConfig LoadCameraAgentConfig(const std::string& yaml_path);

}  // namespace the_eyes::camera

#endif  // THE_EYES_CAMERA_CONFIG_HPP
