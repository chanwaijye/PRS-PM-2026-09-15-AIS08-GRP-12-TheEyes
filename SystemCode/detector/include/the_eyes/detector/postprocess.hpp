// YOLOv8 raw-output decoding and non-maximum suppression.
//
// Kept free of any inference dependency so it is unit-testable without a model,
// a GPU, or ONNX Runtime -- which is most of what can actually go wrong here.
#ifndef THE_EYES_DETECTOR_POSTPROCESS_HPP
#define THE_EYES_DETECTOR_POSTPROCESS_HPP

#include <vector>

#include <opencv2/core.hpp>

namespace the_eyes::detector {

// One box in NETWORK pixel coordinates, before letterbox inversion.
struct RawDetection {
  int class_id = -1;
  float score = 0.0f;
  cv::Rect2f box;
};

float IoU(const cv::Rect2f& a, const cv::Rect2f& b);

// Decodes YOLOv8's output tensor.
//
// Layout is [1, 4 + num_classes, num_anchors], CHANNEL-FIRST -- the transpose of
// YOLOv5's [1, num_anchors, 5 + num_classes]. There is also no objectness
// channel: class scores stand alone. Reading a v8 tensor with v5 indexing
// silently produces plausible-looking garbage, so `channels` is validated
// against num_classes rather than assumed.
//
// Boxes arrive as (cx, cy, w, h) in network pixels and are returned as corner
// rects. Throws std::invalid_argument if the shape is inconsistent.
std::vector<RawDetection> DecodeYolov8(const float* data, int channels, int num_anchors,
                                       int num_classes, float score_threshold);

// Greedy NMS, applied PER CLASS: a bowl overlapping a bottle is two objects,
// and suppressing across classes would delete one of them.
std::vector<RawDetection> NonMaxSuppression(std::vector<RawDetection> detections,
                                            float iou_threshold, int max_detections);

}  // namespace the_eyes::detector

#endif  // THE_EYES_DETECTOR_POSTPROCESS_HPP
