// YOLOv8 inference via ONNX Runtime.
//
// PIMPL so that ONNX Runtime's headers stay out of this one -- fusion,
// scenegraph, and the tests that only need postprocessing must not acquire a
// transitive dependency on an inference runtime.
#ifndef THE_EYES_DETECTOR_ONNX_DETECTOR_HPP
#define THE_EYES_DETECTOR_ONNX_DETECTOR_HPP

#include <memory>
#include <string>
#include <vector>

#include "the_eyes/detector/detector.hpp"

namespace the_eyes::detector {

class OnnxDetector : public Detector {
 public:
  // Throws std::runtime_error if the model cannot be loaded, or if its output
  // shape disagrees with model.class_names -- a model trained on a different
  // class set would otherwise emit correct-looking boxes under wrong labels,
  // and the label is the join key all the way to the pose estimator.
  explicit OnnxDetector(DetectorConfig config);
  ~OnnxDetector() override;

  OnnxDetector(const OnnxDetector&) = delete;
  OnnxDetector& operator=(const OnnxDetector&) = delete;

  DetectionSet Detect(const DetectorInput& input) override;
  std::string name() const override;

  // The provider actually in use, which may differ from the one requested when
  // allow_cpu_fallback let it fall back. Reported so a benchmark can never be
  // attributed to a provider that did not run it.
  std::string active_provider() const;
  double last_inference_ms() const;
  const std::vector<std::string>& warnings() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace the_eyes::detector

#endif  // THE_EYES_DETECTOR_ONNX_DETECTOR_HPP
