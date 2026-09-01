#include "the_eyes/pose/pose_agent.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::pose {
namespace {

// Frame timestamps across the two cameras must agree within one frame period
// (INTERFACES.md §1). A detection set paired with a frame further apart than
// this describes a different image than the depth being used.
constexpr uint64_t kStampWarnThresholdNs = 33'000'000ULL;  // one frame at 30 fps

}  // namespace

PoseAgentConfig LoadPoseAgentConfig(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("pose.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }

  PoseAgentConfig cfg;

  const YAML::Node roi = root["roi"];
  if (!roi || !roi.IsMap()) throw std::runtime_error("pose.yaml: missing section 'roi'");
  if (!roi["margin_ratio"] || !roi["min_size_px"]) {
    throw std::runtime_error("pose.yaml: 'roi' needs margin_ratio and min_size_px");
  }
  cfg.roi.margin_ratio = roi["margin_ratio"].as<double>();
  cfg.roi.min_size_px = roi["min_size_px"].as<int>();
  if (!(cfg.roi.margin_ratio >= 0.0 && cfg.roi.margin_ratio <= 2.0)) {
    throw std::runtime_error("pose.yaml: 'roi.margin_ratio' outside [0, 2]");
  }
  if (cfg.roi.min_size_px < 1) {
    throw std::runtime_error("pose.yaml: 'roi.min_size_px' must be >= 1");
  }

  const YAML::Node est = root["estimator"];
  if (!est || !est.IsMap()) throw std::runtime_error("pose.yaml: missing section 'estimator'");
  if (!est["min_score"]) throw std::runtime_error("pose.yaml: missing 'estimator.min_score'");
  cfg.min_score = est["min_score"].as<double>();
  if (!(cfg.min_score >= 0.0 && cfg.min_score <= 1.0)) {
    throw std::runtime_error("pose.yaml: 'estimator.min_score' outside [0, 1]");
  }
  return cfg;
}

PoseAgent::PoseAgent(MeshRegistry registry, PoseAgentConfig config,
                     std::shared_ptr<PoseEstimator> estimator)
    : registry_(std::move(registry)),
      config_(config),
      estimator_(std::move(estimator)) {
  if (estimator_ == nullptr) {
    throw std::invalid_argument("PoseAgent: null estimator backend");
  }
}

std::vector<PoseEstimate> PoseAgent::Process(const DetectionSet& detections,
                                             const Frame& frame) {
  diagnostics_ = PoseAgentDiagnostics{};

  if (detections.camera_id != frame.camera_id) {
    throw std::invalid_argument("PoseAgent: detections are from camera '" +
                                detections.camera_id + "' but the frame is from '" +
                                frame.camera_id + "'");
  }
  if (!frame.valid()) {
    throw std::invalid_argument("PoseAgent: frame from '" + frame.camera_id +
                                "' is invalid (empty, wrong type, or depth not the same "
                                "size as colour -- depth must be aligned to colour)");
  }

  const uint64_t delta = (detections.stamp_ns > frame.stamp_ns)
                             ? detections.stamp_ns - frame.stamp_ns
                             : frame.stamp_ns - detections.stamp_ns;
  if (delta > kStampWarnThresholdNs) {
    std::ostringstream os;
    os << "detections stamped " << detections.stamp_ns << " paired with frame stamped "
       << frame.stamp_ns << " (" << (delta / 1'000'000.0)
       << " ms apart, more than one frame period) -- the detections may describe a "
          "different image than this depth";
    diagnostics_.warnings.push_back(os.str());
  }

  std::vector<PoseEstimate> out;
  out.reserve(detections.detections.size());

  for (const Detection& detection : detections.detections) {
    // INTERFACES.md §2: a detection whose class has no registered mesh is
    // dropped with a warning and MUST NOT reach the pose estimator.
    const MeshEntry* mesh = registry_.Find(detection.cls);
    if (mesh == nullptr) {
      ++diagnostics_.dropped_no_mesh;
      diagnostics_.warnings.push_back("dropped detection of class '" + detection.cls +
                                      "': no mesh registered, so no pose can be estimated");
      continue;
    }

    const cv::Rect2i roi =
        ComputeRoi(detection.bbox, frame.rgb.cols, frame.rgb.rows, config_.roi);
    if (roi.width == 0 || roi.height == 0) {
      ++diagnostics_.dropped_roi_too_small;
      continue;
    }

    PoseRequest request;
    request.frame = &frame;
    request.detection = detection;
    request.roi = roi;
    request.mesh = mesh;

    const auto start = std::chrono::steady_clock::now();
    const std::optional<PoseEstimate> estimate = estimator_->Estimate(request);
    const auto finish = std::chrono::steady_clock::now();
    latency_.Record(std::chrono::duration<double, std::milli>(finish - start).count());

    if (!estimate.has_value()) {
      ++diagnostics_.declined_by_estimator;
      continue;
    }
    if (estimate->score < config_.min_score) {
      ++diagnostics_.dropped_low_score;
      continue;
    }
    out.push_back(*estimate);
  }

  // Deterministic order regardless of detector output order, so a downstream
  // diff between two runs reflects the poses and not the iteration order.
  std::sort(out.begin(), out.end(), [](const PoseEstimate& a, const PoseEstimate& b) {
    if (a.cls != b.cls) return a.cls < b.cls;
    return a.track_id < b.track_id;
  });
  return out;
}

}  // namespace the_eyes::pose
