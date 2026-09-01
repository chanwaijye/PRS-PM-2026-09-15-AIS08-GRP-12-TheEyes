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
