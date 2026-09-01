#include "the_eyes/detector/detector.hpp"

#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::detector {
namespace {

const YAML::Node RequireSection(const YAML::Node& root, const std::string& key) {
  const YAML::Node node = root[key];
  if (!node || !node.IsMap()) {
    throw std::runtime_error("detector.yaml: missing section '" + key + "'");
  }
  return node;
}

double RequireDouble(const YAML::Node& parent, const std::string& section,
                     const std::string& key, double lo, double hi) {
  const YAML::Node node = parent[key];
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("detector.yaml: missing '" + section + "." + key + "'");
  }
  const double v = node.as<double>();
  if (v < lo || v > hi) {
    std::ostringstream os;
    os << "detector.yaml: '" << section << "." << key << "' = " << v << " outside [" << lo
       << ", " << hi << "]";
    throw std::runtime_error(os.str());
  }
  return v;
}

}  // namespace

DetectorConfig LoadDetectorConfig(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("detector.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }

  DetectorConfig cfg;

  const YAML::Node model = RequireSection(root, "model");
  if (!model["onnx_path"]) throw std::runtime_error("detector.yaml: missing 'model.onnx_path'");
  cfg.model.onnx_path = model["onnx_path"].as<std::string>();
  cfg.model.input_size = static_cast<int>(RequireDouble(model, "model", "input_size", 32, 4096));
  if (model["input_channels"]) {
    cfg.model.input_channels =
        static_cast<int>(RequireDouble(model, "model", "input_channels", 3, 4));
  }

  const YAML::Node names = model["class_names"];
  if (!names || !names.IsSequence() || names.size() == 0) {
    throw std::runtime_error("detector.yaml: 'model.class_names' must be a non-empty sequence");
  }
  for (const YAML::Node& n : names) cfg.model.class_names.push_back(n.as<std::string>());

  // A duplicate name makes two class indices collapse onto one mesh-registry
  // key, which would silently merge two object types downstream.
  for (size_t i = 0; i < cfg.model.class_names.size(); ++i) {
    for (size_t j = i + 1; j < cfg.model.class_names.size(); ++j) {
      if (cfg.model.class_names[i] == cfg.model.class_names[j]) {
        throw std::runtime_error("detector.yaml: duplicate class name '" +
                                 cfg.model.class_names[i] + "'");
      }
    }
  }

  const YAML::Node inference = RequireSection(root, "inference");
  if (inference["provider"]) cfg.inference.provider = inference["provider"].as<std::string>();
  if (cfg.inference.provider != "cuda" && cfg.inference.provider != "tensorrt" &&
      cfg.inference.provider != "cpu") {
    throw std::runtime_error("detector.yaml: 'inference.provider' must be cuda, tensorrt, or cpu");
  }
  if (inference["allow_cpu_fallback"]) {
    cfg.inference.allow_cpu_fallback = inference["allow_cpu_fallback"].as<bool>();
  }
  if (inference["intra_op_threads"]) {
    cfg.inference.intra_op_threads =
        static_cast<int>(RequireDouble(inference, "inference", "intra_op_threads", 1, 256));
  }

  if (const YAML::Node depth = root["depth"]) {
    const auto get = [&depth](const char* key, double fallback) {
      return depth[key] ? depth[key].as<double>() : fallback;
    };
    cfg.model.depth.min_m = get("min_m", cfg.model.depth.min_m);
    cfg.model.depth.max_m = get("max_m", cfg.model.depth.max_m);
    cfg.model.depth.valid_floor =
        static_cast<float>(get("valid_floor", cfg.model.depth.valid_floor));
    cfg.model.depth.sentinel = static_cast<float>(get("sentinel", cfg.model.depth.sentinel));
    cfg.model.depth.letterbox_pad_u8 =
        static_cast<int>(get("letterbox_pad_u8", cfg.model.depth.letterbox_pad_u8));
  }
  // Validate at load, not at first inference: a sentinel that collides with the
  // valid band silently destroys the distinction the encoding exists for.
  try {
    cfg.model.depth.Validate();
  } catch (const std::invalid_argument& e) {
    throw std::runtime_error(std::string("detector.yaml: ") + e.what());
  }

  const YAML::Node post = RequireSection(root, "postprocess");
  cfg.postprocess.score_threshold =
      static_cast<float>(RequireDouble(post, "postprocess", "score_threshold", 0.0, 1.0));
  cfg.postprocess.iou_threshold =
      static_cast<float>(RequireDouble(post, "postprocess", "iou_threshold", 0.0, 1.0));
  cfg.postprocess.max_detections =
      static_cast<int>(RequireDouble(post, "postprocess", "max_detections", 1, 100000));

  const YAML::Node tracking = RequireSection(root, "tracking");
  cfg.tracking.iou_match_threshold =
      static_cast<float>(RequireDouble(tracking, "tracking", "iou_match_threshold", 0.0, 1.0));
  cfg.tracking.max_age_frames =
      static_cast<int>(RequireDouble(tracking, "tracking", "max_age_frames", 0, 100000));

  return cfg;
}

}  // namespace the_eyes::detector
