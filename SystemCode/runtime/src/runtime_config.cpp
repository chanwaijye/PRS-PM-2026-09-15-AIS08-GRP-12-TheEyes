#include "the_eyes/runtime/runtime_config.hpp"

#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::runtime {
namespace {

YAML::Node RequireSection(const YAML::Node& root, const std::string& key) {
  const YAML::Node node = root[key];
  if (!node || !node.IsMap()) throw std::runtime_error("runtime.yaml: missing section '" + key + "'");
  return node;
}

template <typename T>
T RequireInRange(const YAML::Node& parent, const std::string& section, const std::string& key,
                 T lo, T hi) {
  const YAML::Node node = parent[key];
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("runtime.yaml: missing '" + section + "." + key + "'");
  }
  const T v = node.as<T>();
  if (v < lo || v > hi) {
    std::ostringstream os;
    os << "runtime.yaml: '" << section << "." << key << "' = " << v << " outside [" << lo << ", "
       << hi << "]";
    throw std::runtime_error(os.str());
  }
  return v;
}

std::string RequireString(const YAML::Node& parent, const std::string& section,
                          const std::string& key) {
  const YAML::Node node = parent[key];
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("runtime.yaml: missing '" + section + "." + key + "'");
  }
  return node.as<std::string>();
}

}  // namespace

RuntimeConfig LoadRuntimeConfig(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("runtime.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }
  RuntimeConfig cfg;

  const YAML::Node det = RequireSection(root, "detector");
  cfg.detector.backend = RequireString(det, "detector", "backend");
  if (cfg.detector.backend != "onnx" && cfg.detector.backend != "mock") {
    throw std::runtime_error("runtime.yaml: detector.backend must be 'onnx' or 'mock', got '" +
                             cfg.detector.backend + "'");
  }
  cfg.detector.mock_fixture = det["mock_fixture"] ? det["mock_fixture"].as<std::string>() : "";

  const YAML::Node pairing = RequireSection(root, "pairing");
  cfg.pairing.ring_depth =
      static_cast<size_t>(RequireInRange<int>(pairing, "pairing", "ring_depth", 1, 64));

  const YAML::Node live = RequireSection(root, "live");
  cfg.live.warmup_frames = RequireInRange<int>(live, "live", "warmup_frames", 0, 1000);
  cfg.live.stall_timeout_ms = RequireInRange<int>(live, "live", "stall_timeout_ms", 1, 60000);
  cfg.live.max_consecutive_stalls =
      RequireInRange<int>(live, "live", "max_consecutive_stalls", 1, 1000);

  const YAML::Node rec = RequireSection(root, "recorder");
  cfg.recorder.queue_capacity =
      static_cast<size_t>(RequireInRange<int>(rec, "recorder", "queue_capacity", 1, 4096));
  cfg.recorder.png_compression = RequireInRange<int>(rec, "recorder", "png_compression", 0, 9);
  cfg.recorder.writer_threads = RequireInRange<int>(rec, "recorder", "writer_threads", 1, 32);

  const YAML::Node out = RequireSection(root, "output");
  cfg.output.captures_dir = RequireString(out, "output", "captures_dir");
  cfg.output.runs_dir = RequireString(out, "output", "runs_dir");
  cfg.output.print_every = RequireInRange<int>(out, "output", "print_every", 1, 100000);
  return cfg;
}

PoseBackendConfig LoadPoseBackendConfig(const std::string& pose_yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(pose_yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("pose.yaml: cannot parse '" + pose_yaml_path + "': " + e.what());
  }
  const YAML::Node est = root["estimator"];
  if (!est || !est.IsMap()) throw std::runtime_error("pose.yaml: missing section 'estimator'");
  PoseBackendConfig cfg;
  if (!est["backend"]) throw std::runtime_error("pose.yaml: missing 'estimator.backend'");
  cfg.backend = est["backend"].as<std::string>();
  if (cfg.backend != "mock" && cfg.backend != "foundationpose") {
    throw std::runtime_error("pose.yaml: estimator.backend must be 'mock' or 'foundationpose', got '" +
                             cfg.backend + "'");
  }
  if (est["socket_path"]) cfg.socket_path = est["socket_path"].as<std::string>();
  if (est["request_timeout_ms"]) cfg.request_timeout_ms = est["request_timeout_ms"].as<int>();
  if (cfg.request_timeout_ms < 1) {
    throw std::runtime_error("pose.yaml: 'estimator.request_timeout_ms' must be >= 1");
  }
  return cfg;
}

}  // namespace the_eyes::runtime
