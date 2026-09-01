// IoU tracker producing stable per-camera track ids.
//
// This is not cosmetic. pose_agent keys FoundationPose registration on
// (camera_id, class, track_id), and registration measured ~1.3 s against ~11 ms
// for tracking (docs/R1_STATUS.md). An id that changes every frame means paying
// registration every frame -- a ~120x cost for the same result.
//
// Deliberately simple: greedy IoU association against the previous frame, no
// motion model. The objects are on a table and the cameras are static, so
// frame-to-frame displacement is small; a Kalman-based tracker would add
// tuning surface for very little here. The per-object EKF that does exist lives
// in fusion, where it operates on 6-DoF pose rather than pixels.
#ifndef THE_EYES_DETECTOR_TRACKER_HPP
#define THE_EYES_DETECTOR_TRACKER_HPP

#include <cstdint>
#include <vector>

#include "the_eyes/detector/detector.hpp"
#include "the_eyes/detector/postprocess.hpp"

namespace the_eyes::detector {

class IouTracker {
 public:
  explicit IouTracker(TrackingConfig config) : config_(config) {}

  // Assigns a stable track_id to each detection, in place. Detections are
  // matched to existing tracks of the SAME CLASS only.
  void Assign(std::vector<Detection>* detections);

  void Reset();
  size_t active_track_count() const { return tracks_.size(); }

 private:
  struct Track {
    uint32_t id = 0;
    std::string cls;
    cv::Rect2f box;
    int age_frames = 0;  // frames since last match
  };

  TrackingConfig config_;
  std::vector<Track> tracks_;
  // Never reused, even after a track retires: a recycled id would make two
  // different physical objects share FoundationPose state.
  uint32_t next_id_ = 1;
};

}  // namespace the_eyes::detector

#endif  // THE_EYES_DETECTOR_TRACKER_HPP
