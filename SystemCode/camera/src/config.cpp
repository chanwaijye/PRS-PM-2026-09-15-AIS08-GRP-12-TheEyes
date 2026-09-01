#include "the_eyes/camera/config.hpp"

#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::camera {

const CameraEntry& CameraAgentConfig::at(const std::string& camera_id) const {
  const auto it = cameras.find(camera_id);
  if (it == cameras.end()) {
    throw std::out_of_range("cameras.yaml: no camera '" + camera_id + "'");
  }
  return it->second;
}

CameraAgentConfig LoadCameraAgentConfig(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("cameras.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }

  CameraAgentConfig config;

  const YAML::Node cameras = root["cameras"];
  if (!cameras || !cameras.IsMap() || cameras.size() == 0) {
    throw std::runtime_error("cameras.yaml: 'cameras' must be a non-empty map");
  }
  for (const auto& kv : cameras) {
    CameraEntry entry;
    entry.camera_id = kv.first.as<std::string>();
    const YAML::Node& node = kv.second;

    if (!node["serial"] || !node["serial"].IsScalar()) {
      // CLAUDE.md constraint 4. Without a serial there is no stable way to
      // address the device, and falling back to enumeration order is exactly
      // the failure this rule exists to prevent.
      throw std::runtime_error("cameras.yaml: '" + entry.camera_id +
                               ".serial' is required -- cameras are addressed by serial, "
                               "never by enumeration order");
    }
    entry.serial = node["serial"].as<std::string>();
    if (entry.serial.empty()) {
      throw std::runtime_error("cameras.yaml: '" + entry.camera_id + ".serial' is empty");
    }
    if (node["inter_cam_sync_mode"]) {
      entry.inter_cam_sync_mode = node["inter_cam_sync_mode"].as<int>();
      if (entry.inter_cam_sync_mode < 0 || entry.inter_cam_sync_mode > 2) {
        throw std::runtime_error("cameras.yaml: '" + entry.camera_id +
                                 ".inter_cam_sync_mode' must be 0, 1, or 2");
      }
    }
    config.cameras.emplace(entry.camera_id, entry);
  }

  // Two entries pointing at one physical unit would make every "dual-view"
  // result a comparison of a camera with itself.
  for (auto a = config.cameras.begin(); a != config.cameras.end(); ++a) {
    for (auto b = std::next(a); b != config.cameras.end(); ++b) {
      if (a->second.serial == b->second.serial) {
        throw std::runtime_error("cameras.yaml: '" + a->first + "' and '" + b->first +
                                 "' share serial '" + a->second.serial + "'");
      }
    }
  }

  if (const YAML::Node streams = root["streams"]) {
    const auto get = [&streams](const char* key, int fallback) {
      return streams[key] ? streams[key].as<int>() : fallback;
    };
    config.streams.depth_width = get("depth_width", config.streams.depth_width);
    config.streams.depth_height = get("depth_height", config.streams.depth_height);
    config.streams.color_width = get("color_width", config.streams.color_width);
    config.streams.color_height = get("color_height", config.streams.color_height);
    config.streams.fps = get("fps", config.streams.fps);
    if (streams["align_depth_to_color"]) {
      config.streams.align_depth_to_color = streams["align_depth_to_color"].as<bool>();
    }
    if (!config.streams.align_depth_to_color) {
      // INTERFACES.md §1 requires it, and FoundationPose assumes registered
      // RGB-D. Allowing it off would silently break the pose estimator.
      throw std::runtime_error("cameras.yaml: 'streams.align_depth_to_color' must be true -- "
                               "depth aligned to colour is required by INTERFACES.md §1");
    }
    if (config.streams.fps <= 0 || config.streams.depth_width <= 0) {
      throw std::runtime_error("cameras.yaml: stream dimensions and fps must be positive");
    }
  }

  if (const YAML::Node sync = root["sync"]) {
    if (sync["max_skew_ms"]) config.sync.max_skew_ms = sync["max_skew_ms"].as<double>();
    if (sync["startup_check_frames"]) {
      config.sync.startup_check_frames = sync["startup_check_frames"].as<int>();
    }
    if (!(config.sync.max_skew_ms > 0.0) || config.sync.startup_check_frames < 1) {
      throw std::runtime_error("cameras.yaml: sync thresholds must be positive");
    }
  }

  if (const YAML::Node noise = root["noise_characterisation"]) {
    if (noise["distances_m"] && noise["distances_m"].IsSequence()) {
      config.noise.distances_m.clear();
      for (const YAML::Node& d : noise["distances_m"]) {
        config.noise.distances_m.push_back(d.as<double>());
      }
    }
    if (noise["roi_fraction"]) config.noise.roi_fraction = noise["roi_fraction"].as<double>();
    if (noise["frames_per_distance"]) {
      config.noise.frames_per_distance = noise["frames_per_distance"].as<int>();
    }
    if (!(config.noise.roi_fraction > 0.0 && config.noise.roi_fraction <= 1.0)) {
      throw std::runtime_error("cameras.yaml: 'noise_characterisation.roi_fraction' must be "
                               "in (0, 1]");
    }
  }
  return config;
}

}  // namespace the_eyes::camera
