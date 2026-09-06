// Constructs the detector and pose backends the configs name. The modules
// themselves ship no factory: detector.yaml has no backend key (it belongs to
// detector_agent and describes one network), and pose.yaml's estimator.*
// keys were parsed by nothing until runtime_config.hpp.
#ifndef THE_EYES_RUNTIME_BACKENDS_HPP
#define THE_EYES_RUNTIME_BACKENDS_HPP

#include <memory>
#include <string>

#include "the_eyes/detector/detector.hpp"
#include "the_eyes/pose/pose_estimator.hpp"
#include "the_eyes/runtime/runtime_config.hpp"

namespace the_eyes::detector {
class OnnxDetector;
}
namespace the_eyes::pose {
class FoundationPoseClient;
}

namespace the_eyes::runtime {

struct DetectorHandle {
  std::unique_ptr<detector::Detector> detector;
  detector::OnnxDetector* onnx = nullptr;   // non-owning; null for mock
  std::string label;                        // "onnx" | "mock:<fixture>"
  std::string requested_provider;           // detector.yaml inference.provider
};

// Relative paths in the configs (onnx_path, mock_fixture) resolve against
// repo_root, never the working directory. Throws std::runtime_error when the
// onnx backend is requested and ONNX Runtime was not compiled in, when the
// model or fixture file is missing, or on an unknown backend name.
DetectorHandle MakeDetector(const DetectorBackendConfig& cfg, detector::DetectorConfig detector_cfg,
                            const std::string& repo_root);

struct PoseHandle {
  std::shared_ptr<pose::PoseEstimator> estimator;
  pose::FoundationPoseClient* client = nullptr;   // non-owning; null for mock
  std::string label;                              // "foundationpose" | "mock:identity"
};

PoseHandle MakePoseEstimator(const PoseBackendConfig& cfg);

// Joins a config-relative path onto the repo root unless it is already absolute.
std::string ResolveAgainst(const std::string& root, const std::string& path);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_BACKENDS_HPP
