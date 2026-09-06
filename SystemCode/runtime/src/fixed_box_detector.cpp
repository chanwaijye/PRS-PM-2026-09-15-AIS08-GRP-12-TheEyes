#include "the_eyes/runtime/fixed_box_detector.hpp"

#include <sstream>
#include <stdexcept>

namespace the_eyes::runtime {
namespace {

std::vector<std::string> Split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::string field;
  std::istringstream is(s);
  while (std::getline(is, field, sep)) out.push_back(field);
  return out;
}

}  // namespace

FixedBoxes ParseFixedBoxes(const std::string& spec) {
  FixedBoxes boxes;
  if (spec.empty()) throw std::runtime_error("fixed boxes: empty spec");
  for (const std::string& entry : Split(spec, ';')) {
    if (entry.empty()) continue;
    const size_t eq = entry.find('=');
    if (eq == std::string::npos || eq == 0) {
      throw std::runtime_error("fixed boxes: entry '" + entry +
                               "' must be <camera_id>=<cls>,<x>,<y>,<w>,<h>");
    }
    const std::string camera_id = entry.substr(0, eq);
    const std::vector<std::string> f = Split(entry.substr(eq + 1), ',');
    if (f.size() != 5 || f[0].empty()) {
      throw std::runtime_error("fixed boxes: entry '" + entry +
                               "' must be <camera_id>=<cls>,<x>,<y>,<w>,<h>");
    }
    Detection d;
    d.cls = f[0];
    d.score = 1.0f;
    try {
      d.bbox = cv::Rect2i(std::stoi(f[1]), std::stoi(f[2]), std::stoi(f[3]), std::stoi(f[4]));
    } catch (const std::exception&) {
      throw std::runtime_error("fixed boxes: non-integer box in '" + entry + "'");
    }
    if (d.bbox.width <= 0 || d.bbox.height <= 0 || d.bbox.x < 0 || d.bbox.y < 0) {
      throw std::runtime_error("fixed boxes: box in '" + entry + "' must have x,y >= 0 and w,h > 0");
    }
    boxes[camera_id].push_back(d);
  }
  if (boxes.empty()) throw std::runtime_error("fixed boxes: no entries in '" + spec + "'");
  return boxes;
}

FixedBoxDetector::FixedBoxDetector(FixedBoxes boxes, detector::TrackingConfig tracking)
    : boxes_(std::move(boxes)), tracking_(tracking) {}

DetectionSet FixedBoxDetector::Detect(const detector::DetectorInput& input) {
  DetectionSet set;
  set.camera_id = input.camera_id;
  set.stamp_ns = input.stamp_ns;
  auto it = boxes_.find(input.camera_id);
  if (it != boxes_.end()) {
    for (const Detection& d : it->second) {
      Detection clipped = d;
      // Keep the box inside the frame; ComputeRoi would drop one hanging out.
      const cv::Rect2i frame(0, 0, input.rgb.cols, input.rgb.rows);
      clipped.bbox = d.bbox & frame;
      if (clipped.bbox.width > 0 && clipped.bbox.height > 0) set.detections.push_back(clipped);
    }
  }
  auto tracker = trackers_.emplace(input.camera_id, detector::IouTracker(tracking_)).first;
  tracker->second.Assign(&set.detections);
  return set;
}

}  // namespace the_eyes::runtime
