#include "the_eyes/detector/mock_detector.hpp"

#include <algorithm>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::detector {

std::vector<FixtureFrame> LoadDetectionFixture(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("detection fixture: cannot parse '" + yaml_path + "': " + e.what());
  }

  const YAML::Node frames = root["frames"];
  if (!frames || !frames.IsSequence()) {
    throw std::runtime_error("detection fixture: 'frames' must be a sequence");
  }

  std::vector<FixtureFrame> out;
  for (const YAML::Node& node : frames) {
    FixtureFrame frame;
    if (!node["camera_id"] || !node["stamp_ns"]) {
      throw std::runtime_error("detection fixture: every frame needs camera_id and stamp_ns");
    }
    frame.camera_id = node["camera_id"].as<std::string>();
    frame.stamp_ns = node["stamp_ns"].as<uint64_t>();

    const YAML::Node detections = node["detections"];
    if (detections && detections.IsSequence()) {
      for (const YAML::Node& d : detections) {
        if (!d["cls"] || !d["bbox"]) {
          throw std::runtime_error("detection fixture: every detection needs cls and bbox");
        }
        const YAML::Node bbox = d["bbox"];
        if (!bbox.IsSequence() || bbox.size() != 4) {
          throw std::runtime_error("detection fixture: bbox must be [x, y, w, h]");
        }
        Detection detection;
        detection.cls = d["cls"].as<std::string>();
        detection.score = d["score"] ? d["score"].as<float>() : 1.0f;
        detection.bbox = cv::Rect2i(bbox[0].as<int>(), bbox[1].as<int>(), bbox[2].as<int>(),
                                    bbox[3].as<int>());
        frame.detections.push_back(detection);
      }
    }
    out.push_back(std::move(frame));
  }
  return out;
}

MockDetector::MockDetector(std::vector<FixtureFrame> frames, TrackingConfig tracking)
    : frames_(std::move(frames)), tracking_(tracking) {}

DetectionSet MockDetector::Detect(const DetectorInput& input) {
  DetectionSet set;
  set.camera_id = input.camera_id;
  set.stamp_ns = input.stamp_ns;

  const auto match = std::find_if(frames_.begin(), frames_.end(), [&input](const FixtureFrame& f) {
    return f.camera_id == input.camera_id && f.stamp_ns == input.stamp_ns;
  });
  if (match != frames_.end()) set.detections = match->detections;

  // Track even on a miss, so the tracker ages its tracks consistently whether
  // or not the fixture covers this frame.
  const auto tracker = trackers_.emplace(input.camera_id, IouTracker(tracking_)).first;
  tracker->second.Assign(&set.detections);
  return set;
}

}  // namespace the_eyes::detector
