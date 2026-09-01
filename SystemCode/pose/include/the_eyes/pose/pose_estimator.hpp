// Per-view 6-DoF pose estimation: the backend interface, the frame it operates
// on, and the latency accounting that feeds the R4 scheduling decision.
//
// INTERFACES.md §3: poses are emitted in the CAMERA OPTICAL FRAME. Nothing in
// this module knows about a world frame or about extrinsics -- transformation
// is the fusion module's job, and keeping the knowledge out of here is what
// makes that separation real rather than nominal.
#ifndef THE_EYES_POSE_POSE_ESTIMATOR_HPP
#define THE_EYES_POSE_POSE_ESTIMATOR_HPP

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "the_eyes/detection.hpp"
#include "the_eyes/types.hpp"

namespace the_eyes::pose {

// One camera's aligned RGB-D frame. Depth must already be registered to colour
// (INTERFACES.md §1) -- FoundationPose assumes the two share a pixel grid.
struct Frame {
  std::string camera_id;
  uint64_t stamp_ns = 0;      // SOURCE frame timestamp, never receive time
  cv::Mat rgb;                // CV_8UC3, RGB order
  cv::Mat depth;              // CV_16UC1, raw sensor units
  double depth_scale_m = 0.001;  // metres per depth unit; D435i reports 0.001
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();  // colour intrinsics

  bool valid() const;
};

// Everything one estimation needs. Holds views into the frame rather than
// copies -- a crop per detection per view per frame is not free at 30 fps.
struct PoseRequest {
  const Frame* frame = nullptr;
  Detection detection;
  cv::Rect2i roi;             // already expanded and clamped
  const MeshEntry* mesh = nullptr;  // never null: unregistered classes are
                                    // dropped before a request is built
};

// Latency accounting. An explicit acceptance criterion for this module: the
// per-object cost is what decides between running both views every frame,
// alternating them, or gating on detections (risk R4).
class LatencyStats {
 public:
  void Record(double milliseconds);
  void Reset();

  size_t count() const { return samples_.size(); }
  double mean_ms() const;
  double percentile_ms(double p) const;  // p in [0,1]; 0.5 = median
  double max_ms() const;
  std::string Format(const std::string& label) const;

 private:
  std::vector<double> samples_;
};

class PoseEstimator {
 public:
  virtual ~PoseEstimator() = default;

  // Returns nullopt when the backend declines to produce a pose -- a failed
  // hypothesis is not an error, it is a detection this view could not resolve,
  // and fusion handles a missing view natively.
  virtual std::optional<PoseEstimate> Estimate(const PoseRequest& request) = 0;

  // Identifies the backend in logs and in the evaluation record, so a metric
  // can never be attributed to the wrong estimator.
  virtual std::string name() const = 0;
};

}  // namespace the_eyes::pose

#endif  // THE_EYES_POSE_POSE_ESTIMATOR_HPP
