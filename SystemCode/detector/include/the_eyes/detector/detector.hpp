// detector_agent: per-view object detection.
//
// Produces DetectionSet (INTERFACES.md §2). The `cls` string is the join
// key to the mesh registry and travels unchanged all the way to the pose
// estimator, so class naming is a contract, not a label.
#ifndef THE_EYES_DETECTOR_DETECTOR_HPP
#define THE_EYES_DETECTOR_DETECTOR_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "the_eyes/detection.hpp"
#include "the_eyes/detector/depth_encoding.hpp"

namespace the_eyes::detector {

// Narrower than pose::Frame -- detection needs no intrinsics, and depending on
// the pose module's type would couple them. It DOES need depth: the detector is
// a 4-channel RGB-D network (CLAUDE.md), not RGB-only.
struct DetectorInput {
  std::string camera_id;
  uint64_t stamp_ns = 0;  // SOURCE frame timestamp, not receive time
  cv::Mat rgb;            // CV_8UC3, RGB order
  // CV_16UC1, ALIGNED TO COLOUR (INTERFACES.md §1) -- same pixel grid as rgb.
  // Required when the model has 4 input channels.
  cv::Mat depth;
  double depth_scale_m = 0.001;  // metres per depth unit; D435i reports 0.001

  bool has_depth() const { return !depth.empty(); }
};

struct ModelConfig {
  std::string onnx_path;
  int input_size = 640;
  // 3 = RGB baseline (kept as the modality-ablation control), 4 = RGB-D early
  // fusion (the deployed network). Validated against the loaded model's actual
  // input channel count -- a mismatch would feed the depth channel into a
  // network that never saw one.
  int input_channels = 4;
  DepthEncoding depth;
  // Index -> name. Order must match the training data; names must match the
  // keys in config/meshes.yaml.
  std::vector<std::string> class_names;
};

struct InferenceConfig {
  std::string provider = "cuda";  // "cuda" | "tensorrt" | "cpu"
  bool allow_cpu_fallback = true;
  int intra_op_threads = 4;
};

struct PostprocessConfig {
  float score_threshold = 0.25f;
  float iou_threshold = 0.45f;
  int max_detections = 64;
};

struct TrackingConfig {
  float iou_match_threshold = 0.30f;
  int max_age_frames = 15;
};

struct DetectorConfig {
  ModelConfig model;
  InferenceConfig inference;
  PostprocessConfig postprocess;
  TrackingConfig tracking;
};

// Throws std::runtime_error on a missing file or an out-of-range value.
DetectorConfig LoadDetectorConfig(const std::string& yaml_path);

class Detector {
 public:
  virtual ~Detector() = default;
  virtual DetectionSet Detect(const DetectorInput& input) = 0;
  virtual std::string name() const = 0;
};

}  // namespace the_eyes::detector

#endif  // THE_EYES_DETECTOR_DETECTOR_HPP
