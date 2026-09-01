#include "the_eyes/detector/tracker.hpp"

#include <algorithm>

namespace the_eyes::detector {
namespace {

cv::Rect2f ToRect2f(const cv::Rect2i& r) {
  return cv::Rect2f(static_cast<float>(r.x), static_cast<float>(r.y),
                    static_cast<float>(r.width), static_cast<float>(r.height));
}

}  // namespace

void IouTracker::Reset() {
  tracks_.clear();
  // next_id_ deliberately NOT reset: an id must never name two different
  // physical objects over the life of a run.
}

void IouTracker::Assign(std::vector<Detection>* detections) {
  if (detections == nullptr) return;

  for (Track& track : tracks_) ++track.age_frames;

  std::vector<bool> track_taken(tracks_.size(), false);

  // Greedy by detection order, which is NMS's score-descending order -- so the
  // most confident detection gets first claim on a track.
  for (Detection& detection : *detections) {
    const cv::Rect2f box = ToRect2f(detection.bbox);

    int best = -1;
    float best_iou = config_.iou_match_threshold;
    for (size_t t = 0; t < tracks_.size(); ++t) {
      if (track_taken[t] || tracks_[t].cls != detection.cls) continue;
      const float iou = IoU(tracks_[t].box, box);
      if (iou >= best_iou) {
        best_iou = iou;
        best = static_cast<int>(t);
      }
    }

    if (best >= 0) {
      Track& track = tracks_[static_cast<size_t>(best)];
      track.box = box;
      track.age_frames = 0;
      track_taken[static_cast<size_t>(best)] = true;
      detection.track_id = track.id;
    } else {
      Track track;
      track.id = next_id_++;
      track.cls = detection.cls;
      track.box = box;
      track.age_frames = 0;
      tracks_.push_back(track);
      track_taken.push_back(true);
      detection.track_id = track.id;
    }
  }

  const int max_age = config_.max_age_frames;
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [max_age](const Track& t) { return t.age_frames > max_age; }),
                tracks_.end());
}

}  // namespace the_eyes::detector
