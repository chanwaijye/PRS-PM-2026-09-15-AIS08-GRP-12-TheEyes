#include "the_eyes/detector/onnx_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <map>
#include <sstream>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>

#include "the_eyes/detector/letterbox.hpp"
#include "the_eyes/detector/postprocess.hpp"
#include "the_eyes/detector/tracker.hpp"

namespace the_eyes::detector {

struct OnnxDetector::Impl {
  DetectorConfig config;
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "the_eyes_detector"};
  Ort::SessionOptions options;
  std::unique_ptr<Ort::Session> session;
  Ort::AllocatorWithDefaultOptions allocator;

  std::string input_name;
  std::string output_name;
  std::string provider = "cpu";
  double last_inference_ms = 0.0;
  std::vector<std::string> warnings;

  // One tracker per camera: ids are stable within a camera only (§2).
  std::map<std::string, IouTracker> trackers;

  // Reused across frames so a 30 fps loop is not reallocating 4.9 MB per call.
  std::vector<float> input_tensor;

  explicit Impl(DetectorConfig cfg) : config(std::move(cfg)) {}
};

namespace {

// Attempts the requested provider, recording what actually happened. ORT throws
// when a provider's libraries are missing, and a silent fall back to CPU would
// make a latency number meaningless.
void AppendProvider(Ort::SessionOptions& options, const std::string& requested,
                    std::string* active, std::vector<std::string>* warnings,
                    bool allow_cpu_fallback) {
  if (requested == "cpu") {
    *active = "cpu";
    return;
  }
  try {
    if (requested == "tensorrt") {
      OrtTensorRTProviderOptions trt{};
      options.AppendExecutionProvider_TensorRT(trt);
      *active = "tensorrt";
      return;
    }
    OrtCUDAProviderOptions cuda{};
    options.AppendExecutionProvider_CUDA(cuda);
    *active = "cuda";
  } catch (const Ort::Exception& e) {
    if (!allow_cpu_fallback) {
      throw std::runtime_error("detector: execution provider '" + requested +
                               "' unavailable and allow_cpu_fallback is false: " + e.what());
    }
    warnings->push_back("execution provider '" + requested + "' unavailable (" + e.what() +
                        "); running on CPU -- do not quote GPU latency from this session");
    *active = "cpu";
  }
}

}  // namespace

OnnxDetector::OnnxDetector(DetectorConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
  impl_->options.SetIntraOpNumThreads(impl_->config.inference.intra_op_threads);
  impl_->options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  AppendProvider(impl_->options, impl_->config.inference.provider, &impl_->provider,
                 &impl_->warnings, impl_->config.inference.allow_cpu_fallback);

  try {
    impl_->session = std::make_unique<Ort::Session>(impl_->env, impl_->config.model.onnx_path.c_str(),
                                                    impl_->options);
  } catch (const Ort::Exception& e) {
    throw std::runtime_error("detector: cannot load model '" + impl_->config.model.onnx_path +
                             "': " + e.what());
  }

  if (impl_->session->GetInputCount() != 1 || impl_->session->GetOutputCount() < 1) {
    throw std::runtime_error("detector: expected a single-input YOLOv8 graph");
  }
  impl_->input_name = impl_->session->GetInputNameAllocated(0, impl_->allocator).get();
  impl_->output_name = impl_->session->GetOutputNameAllocated(0, impl_->allocator).get();

  // The model's own input channel count is authoritative. A 3-channel model fed
  // a 4-channel tensor (or the reverse) fails in ONNX Runtime with a shape
  // error, but only at the first Detect() -- catching it at load turns a
  // mid-run crash into a startup message that names the actual mismatch.
  {
    const auto input_shape =
        impl_->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape.size() == 4 && input_shape[1] > 0) {
      const int model_channels = static_cast<int>(input_shape[1]);
      if (model_channels != impl_->config.model.input_channels) {
        std::ostringstream os;
        os << "detector: model expects " << model_channels << " input channels but "
           << "config/detector.yaml says " << impl_->config.model.input_channels
           << ". A 4-channel RGB-D model and a 3-channel RGB model are not "
              "interchangeable.";
        throw std::runtime_error(os.str());
      }
    }
  }
  impl_->config.model.depth.Validate();

  // Validate the head against the configured class list. A model trained on a
  // different class set emits correct-looking boxes under wrong labels, and the
  // label is the join key all the way to the pose estimator.
  const auto shape =
      impl_->session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  if (shape.size() == 3) {
    const int64_t channels = shape[1];
    const int64_t expected = 4 + static_cast<int64_t>(impl_->config.model.class_names.size());
    if (channels > 0 && channels != expected) {
      std::ostringstream os;
      os << "detector: model output has " << channels << " channels but config lists "
         << impl_->config.model.class_names.size() << " classes (expected " << expected
         << "). The model and config/detector.yaml disagree about the class set.";
      throw std::runtime_error(os.str());
    }
  }
}

OnnxDetector::~OnnxDetector() = default;

std::string OnnxDetector::name() const { return "yolov8:onnxruntime"; }
std::string OnnxDetector::active_provider() const { return impl_->provider; }
double OnnxDetector::last_inference_ms() const { return impl_->last_inference_ms; }
const std::vector<std::string>& OnnxDetector::warnings() const { return impl_->warnings; }

DetectionSet OnnxDetector::Detect(const DetectorInput& input) {
  DetectionSet set;
  set.camera_id = input.camera_id;
  set.stamp_ns = input.stamp_ns;  // source frame time, never receive time

  if (input.rgb.empty() || input.rgb.type() != CV_8UC3) {
    throw std::invalid_argument("detector: input must be a non-empty CV_8UC3 RGB image");
  }

  const int channels = impl_->config.model.input_channels;
  const bool wants_depth = channels == 4;

  if (wants_depth) {
    if (!input.has_depth()) {
      // AGENTS.md: skip the frame rather than substitute zeros. A zero-filled
      // depth channel is indistinguishable from "object at zero range" to the
      // network, so faking it produces confident nonsense instead of a gap.
      throw std::invalid_argument(
          "detector: model is RGB-D but this frame carries no depth. Skip the frame -- "
          "do not substitute zeros, which the network reads as 'object at zero range'.");
    }
    if (input.depth.type() != CV_16UC1) {
      throw std::invalid_argument("detector: depth must be CV_16UC1");
    }
    if (input.depth.size() != input.rgb.size()) {
      throw std::invalid_argument(
          "detector: depth is not the same size as colour -- depth must be ALIGNED to "
          "colour (INTERFACES.md §1), not merely present");
    }
  }

  const DepthEncoding& encoding = impl_->config.model.depth;
  const int size = impl_->config.model.input_size;
  cv::Mat letterboxed;
  const LetterboxTransform transform = Letterbox(input.rgb, &letterboxed, size);

  cv::Mat letterboxed_depth;
  if (wants_depth) {
    // Pad to match what TRAINING used, not what is semantically tidy.
    // ultralytics pads all channels with 114; a model trained through it reads
    // the border as a surface at ~0.70 m, and feeding the sentinel instead
    // would present the network with a border it never saw. See
    // DepthEncoding::letterbox_pad_u8.
    //
    // The pad is applied in RAW units, so convert the configured uint8 back
    // through the encoding to the raw value that reproduces it.
    const double pad_metres =
        encoding.min_m + ((encoding.letterbox_pad_u8 / 255.0) - encoding.valid_floor) /
                             (1.0 - encoding.valid_floor) * (encoding.max_m - encoding.min_m);
    const double pad_raw = (encoding.letterbox_pad_u8 <= 0)
                               ? 0.0
                               : std::round(pad_metres / input.depth_scale_m);
    Letterbox(input.depth, &letterboxed_depth, size, cv::Scalar(pad_raw));
  }

  // NCHW float32. RGB in [0,1]; depth via the sentinel encoding in metric
  // metres. ultralytics exports expect RGB order -- BGR in would produce
  // confident detections of the wrong classes.
  impl_->input_tensor.resize(static_cast<size_t>(channels) * size * size);
  for (int y = 0; y < size; ++y) {
    const uint8_t* rgb_row = letterboxed.ptr<uint8_t>(y);
    for (int c = 0; c < 3; ++c) {
      float* out = impl_->input_tensor.data() + (static_cast<size_t>(c) * size + y) * size;
      for (int x = 0; x < size; ++x) out[x] = rgb_row[x * 3 + c] / 255.0f;
    }
    if (wants_depth) {
      const uint16_t* depth_row = letterboxed_depth.ptr<uint16_t>(y);
      float* out = impl_->input_tensor.data() + (static_cast<size_t>(3) * size + y) * size;
      for (int x = 0; x < size; ++x) {
        out[x] = encoding.Encode(depth_row[x], input.depth_scale_m);
      }
    }
  }

  const std::array<int64_t, 4> input_shape{1, channels, size, size};
  const Ort::MemoryInfo memory =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input_value = Ort::Value::CreateTensor<float>(
      memory, impl_->input_tensor.data(), impl_->input_tensor.size(), input_shape.data(),
      input_shape.size());

  const char* input_names[] = {impl_->input_name.c_str()};
  const char* output_names[] = {impl_->output_name.c_str()};

  const auto start = std::chrono::steady_clock::now();
  auto outputs = impl_->session->Run(Ort::RunOptions{nullptr}, input_names, &input_value, 1,
                                     output_names, 1);
  impl_->last_inference_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

  const auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
  if (shape.size() != 3) {
    throw std::runtime_error("detector: expected a rank-3 YOLOv8 output tensor");
  }
  // Output channels (4 box + one score per class), distinct from the INPUT
  // channel count above -- they are unrelated numbers that both want the name.
  const int output_channels = static_cast<int>(shape[1]);
  const int anchors = static_cast<int>(shape[2]);
  const int num_classes = static_cast<int>(impl_->config.model.class_names.size());

  std::vector<RawDetection> raw =
      DecodeYolov8(outputs[0].GetTensorData<float>(), output_channels, anchors, num_classes,
                   impl_->config.postprocess.score_threshold);
  raw = NonMaxSuppression(std::move(raw), impl_->config.postprocess.iou_threshold,
                          impl_->config.postprocess.max_detections);

  for (const RawDetection& detection : raw) {
    const cv::Rect2i box = transform.ToSource(detection.box);
    if (box.width <= 0 || box.height <= 0) continue;  // fell entirely into padding

    Detection out;
    out.cls = impl_->config.model.class_names[static_cast<size_t>(detection.class_id)];
    out.score = detection.score;
    out.bbox = box;
    set.detections.push_back(out);
  }

  const auto tracker =
      impl_->trackers.emplace(input.camera_id, IouTracker(impl_->config.tracking)).first;
  tracker->second.Assign(&set.detections);
  return set;
}

}  // namespace the_eyes::detector
