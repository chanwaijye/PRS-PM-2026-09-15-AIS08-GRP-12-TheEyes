#include "the_eyes/runtime/backends.hpp"

#include <filesystem>
#include <stdexcept>

#include "the_eyes/detector/mock_detector.hpp"
#include "the_eyes/pose/foundationpose_client.hpp"
#include "the_eyes/pose/mock_estimator.hpp"
#if THE_EYES_WITH_ONNXRUNTIME
#include "the_eyes/detector/onnx_detector.hpp"
#endif

namespace the_eyes::runtime {

std::string ResolveAgainst(const std::string& root, const std::string& path) {
  const std::filesystem::path p(path);
  if (p.is_absolute() || root.empty()) return p.string();
  return (std::filesystem::path(root) / p).lexically_normal().string();
}

DetectorHandle MakeDetector(const DetectorBackendConfig& cfg, detector::DetectorConfig detector_cfg,
                            const std::string& repo_root) {
  DetectorHandle h;
  h.requested_provider = detector_cfg.inference.provider;
  if (cfg.backend == "mock") {
    if (cfg.mock_fixture.empty()) {
      throw std::runtime_error("detector backend 'mock' needs a fixture path "
                               "(runtime.yaml detector.mock_fixture or --detector mock:<path>)");
    }
    const std::string fixture = ResolveAgainst(repo_root, cfg.mock_fixture);
    if (!std::filesystem::exists(fixture)) {
      throw std::runtime_error("detector fixture not found: " + fixture);
    }
    h.detector = std::make_unique<detector::MockDetector>(detector::LoadDetectionFixture(fixture),
                                                          detector_cfg.tracking);
    h.label = "mock:" + cfg.mock_fixture;
    return h;
  }
  if (cfg.backend == "onnx") {
    detector_cfg.model.onnx_path = ResolveAgainst(repo_root, detector_cfg.model.onnx_path);
    if (!std::filesystem::exists(detector_cfg.model.onnx_path)) {
      throw std::runtime_error(
          "ONNX model not found: " + detector_cfg.model.onnx_path +
          "\n  regenerate with: python training/detector/export_onnx.py "
          "--weights runs/detector/rgbd_50k_v2/weights/best.pt --imgsz 640 "
          "--out assets/models/yolov8_the_eyes.onnx");
    }
#if THE_EYES_WITH_ONNXRUNTIME
    auto onnx = std::make_unique<detector::OnnxDetector>(std::move(detector_cfg));
    h.onnx = onnx.get();
    h.detector = std::move(onnx);
    h.label = "onnx";
    return h;
#else
    throw std::runtime_error(
        "detector backend 'onnx' requested but ONNX Runtime was not compiled in "
        "(set THE_EYES_ONNXRUNTIME_ROOT at configure time)");
#endif
  }
  throw std::runtime_error("unknown detector backend '" + cfg.backend + "' (onnx | mock)");
}

PoseHandle MakePoseEstimator(const PoseBackendConfig& cfg) {
  PoseHandle h;
  if (cfg.backend == "mock") {
    h.estimator = pose::MockPoseEstimator::Identity();
    h.label = "mock:identity";
    return h;
  }
  if (cfg.backend == "foundationpose") {
    pose::FoundationPoseClientConfig client_cfg;
    client_cfg.socket_path = cfg.socket_path;
    client_cfg.request_timeout_ms = cfg.request_timeout_ms;
    auto client = std::make_shared<pose::FoundationPoseClient>(client_cfg);
    h.client = client.get();
    h.estimator = client;
    h.label = "foundationpose";
    return h;
  }
  throw std::runtime_error("unknown pose backend '" + cfg.backend + "' (foundationpose | mock)");
}

}  // namespace the_eyes::runtime
