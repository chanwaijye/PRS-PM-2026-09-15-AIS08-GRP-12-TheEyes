// config/runtime.yaml -- what the runner needs that no other module owns --
// plus the pose backend selection that config/pose.yaml has always carried
// under estimator.* but that no loader in the pose module reads.
#ifndef THE_EYES_RUNTIME_RUNTIME_CONFIG_HPP
#define THE_EYES_RUNTIME_RUNTIME_CONFIG_HPP

#include <cstddef>
#include <string>

#include "the_eyes/runtime/brightness.hpp"

namespace the_eyes::runtime {

struct DetectorBackendConfig {
  std::string backend = "onnx";   // "onnx" | "mock" | "fixed"
  std::string mock_fixture;       // for mock; relative to the repo root
  std::string fixed_boxes;        // for fixed; see fixed_box_detector.hpp
};

struct PairingConfig {
  size_t ring_depth = 4;
};

struct LiveConfig {
  int warmup_frames = 15;
  int stall_timeout_ms = 1000;
  int max_consecutive_stalls = 5;
};

struct RecorderTuning {
  size_t queue_capacity = 64;
  int png_compression = 1;
  int writer_threads = 2;
};

struct OutputConfig {
  std::string captures_dir = "captures/runtime";
  std::string runs_dir = "runs/runtime";
  int print_every = 30;
};

struct RuntimeConfig {
  DetectorBackendConfig detector;
  PairingConfig pairing;
  LiveConfig live;
  RecorderTuning recorder;
  OutputConfig output;
  BrightnessConfig brightness;  // disabled when the file has no `brightness:` section
};

// Strict: every section and key must be present and in range. The one
// exception is `brightness`, which may be absent -- configs written before it
// existed must still load, and mean "off" -- but is strict once it is there.
RuntimeConfig LoadRuntimeConfig(const std::string& yaml_path);

struct PoseBackendConfig {
  std::string backend = "mock";   // "mock" | "foundationpose"
  std::string socket_path = "/tmp/the_eyes_foundationpose.sock";
  int request_timeout_ms = 5000;
  double reregister_area_ratio = 0.0;     // 0 = off; see FoundationPoseClientConfig
  double reregister_min_box_cover = 0.0;  // 0 = off; the same
  int reregister_after_off_box = 0;       // 0 = off; the same
};

// Reads estimator.backend / socket_path / request_timeout_ms /
// reregister_area_ratio / reregister_min_box_cover / reregister_after_off_box
// from pose.yaml.
PoseBackendConfig LoadPoseBackendConfig(const std::string& pose_yaml_path);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_RUNTIME_CONFIG_HPP
