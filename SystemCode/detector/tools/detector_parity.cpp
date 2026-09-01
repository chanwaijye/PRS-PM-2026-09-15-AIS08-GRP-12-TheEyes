// Parity harness: dumps the C++ inference path's detections for one image so
// the Python reference can be diffed against them.
//
// AGENTS.md §3 acceptance: "C++ inference output matches the Python
// reference within float tolerance." That criterion is only meaningful if both
// sides can be made to print the same thing, which is what this exists for.
// Pair it with training/detector/parity_reference.py.
//
// Usage: detector_parity <config.yaml> <image.png> [provider]
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "the_eyes/detector/onnx_detector.hpp"

int main(int argc, char** argv) {
  using namespace the_eyes;
  using namespace the_eyes::detector;

  if (argc < 3) {
    std::cerr << "usage: detector_parity <config.yaml> <image.png> [provider] "
                 "[--depth <depth.png>]\n"
                 "\n  --depth  16-bit aligned depth, required when the model is RGB-D\n";
    return EXIT_FAILURE;
  }
  try {
    DetectorConfig config = LoadDetectorConfig(argv[1]);
    std::string depth_path;
    for (int i = 3; i < argc; ++i) {
      if (std::string(argv[i]) == "--depth" && i + 1 < argc) depth_path = argv[++i];
    }
    if (argc > 3 && std::string(argv[3]) != "--depth") {
      config.inference.provider = argv[3];
      // Parity is a correctness check, so a silent CPU fallback must not be
      // mistaken for the provider under test.
      config.inference.allow_cpu_fallback = false;
    }

    const cv::Mat bgr = cv::imread(argv[2], cv::IMREAD_COLOR);
    if (bgr.empty()) throw std::runtime_error(std::string("cannot read image ") + argv[2]);

    DetectorInput input;
    input.camera_id = "parity";
    input.stamp_ns = 1;
    cv::cvtColor(bgr, input.rgb, cv::COLOR_BGR2RGB);

    if (config.model.input_channels == 4) {
      if (depth_path.empty()) {
        throw std::runtime_error(
            "model is RGB-D (input_channels: 4) but no --depth was given. Supply aligned "
            "16-bit depth; substituting zeros would read as 'object at zero range'.");
      }
      input.depth = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
      if (input.depth.empty()) throw std::runtime_error("cannot read depth " + depth_path);
    }

    OnnxDetector detector(config);
    const DetectionSet set = detector.Detect(input);

    for (const std::string& w : detector.warnings()) std::cerr << "warning: " << w << "\n";

    // One detection per line, fixed precision, sorted by the detector's own
    // deterministic NMS order -- so a diff against the Python side is exact.
    std::cout << "# provider=" << detector.active_provider()
              << " inference_ms=" << std::fixed << std::setprecision(3)
              << detector.last_inference_ms() << "\n";
    std::cout << "# cls score x y w h\n";
    for (const Detection& d : set.detections) {
      std::cout << d.cls << " " << std::fixed << std::setprecision(6) << d.score << " "
                << d.bbox.x << " " << d.bbox.y << " " << d.bbox.width << " " << d.bbox.height
                << "\n";
    }
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "detector_parity: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
