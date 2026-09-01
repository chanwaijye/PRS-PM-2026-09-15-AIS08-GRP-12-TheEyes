#include "the_eyes/detector/postprocess.hpp"

#include <algorithm>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace the_eyes::detector {

float IoU(const cv::Rect2f& a, const cv::Rect2f& b) {
  const float x0 = std::max(a.x, b.x);
  const float y0 = std::max(a.y, b.y);
  const float x1 = std::min(a.x + a.width, b.x + b.width);
  const float y1 = std::min(a.y + a.height, b.y + b.height);
  if (x1 <= x0 || y1 <= y0) return 0.0f;

  const float intersection = (x1 - x0) * (y1 - y0);
  const float union_area = a.area() + b.area() - intersection;
  if (union_area <= 0.0f) return 0.0f;
  return intersection / union_area;
}

std::vector<RawDetection> DecodeYolov8(const float* data, int channels, int num_anchors,
                                       int num_classes, float score_threshold) {
  if (data == nullptr) throw std::invalid_argument("DecodeYolov8: null tensor");
  if (num_anchors <= 0 || num_classes <= 0) {
    throw std::invalid_argument("DecodeYolov8: num_anchors and num_classes must be positive");
  }
  // 4 box channels + one score per class, and NO objectness channel. If this
  // does not hold, the tensor is not a YOLOv8 head and decoding it with these
  // strides would produce plausible-looking nonsense rather than an error.
  if (channels != 4 + num_classes) {
    std::ostringstream os;
    os << "DecodeYolov8: expected " << (4 + num_classes) << " channels for " << num_classes
       << " classes, got " << channels
       << " -- is this a YOLOv8 head, or a v5-style tensor with objectness?";
    throw std::invalid_argument(os.str());
  }

  std::vector<RawDetection> out;
  for (int anchor = 0; anchor < num_anchors; ++anchor) {
    // Channel-first: element (c, a) lives at c * num_anchors + a.
    int best_class = -1;
    float best_score = score_threshold;
    for (int c = 0; c < num_classes; ++c) {
      const float score = data[(4 + c) * num_anchors + anchor];
      if (score > best_score) {
        best_score = score;
        best_class = c;
      }
    }
    if (best_class < 0) continue;

    const float cx = data[0 * num_anchors + anchor];
    const float cy = data[1 * num_anchors + anchor];
    const float w = data[2 * num_anchors + anchor];
    const float h = data[3 * num_anchors + anchor];
    if (!(w > 0.0f) || !(h > 0.0f)) continue;

    RawDetection detection;
    detection.class_id = best_class;
    detection.score = best_score;
    detection.box = cv::Rect2f(cx - w * 0.5f, cy - h * 0.5f, w, h);
    out.push_back(detection);
  }
  return out;
}

std::vector<RawDetection> NonMaxSuppression(std::vector<RawDetection> detections,
                                            float iou_threshold, int max_detections) {
  // Sort by score descending, breaking ties deterministically so two runs on
  // identical input cannot disagree about which of two equal boxes survives.
  std::stable_sort(detections.begin(), detections.end(),
                   [](const RawDetection& a, const RawDetection& b) {
                     if (a.score != b.score) return a.score > b.score;
                     if (a.class_id != b.class_id) return a.class_id < b.class_id;
                     if (a.box.x != b.box.x) return a.box.x < b.box.x;
                     return a.box.y < b.box.y;
                   });

  std::vector<RawDetection> kept;
  std::vector<bool> suppressed(detections.size(), false);
  for (size_t i = 0; i < detections.size(); ++i) {
    if (suppressed[i]) continue;
    kept.push_back(detections[i]);
    if (max_detections > 0 && static_cast<int>(kept.size()) >= max_detections) break;

    for (size_t j = i + 1; j < detections.size(); ++j) {
      if (suppressed[j]) continue;
      // Per class only: a bowl overlapping a bottle is two objects, and
      // suppressing across classes would delete one of them.
      if (detections[j].class_id != detections[i].class_id) continue;
      if (IoU(detections[i].box, detections[j].box) > iou_threshold) suppressed[j] = true;
    }
  }
  return kept;
}

}  // namespace the_eyes::detector
