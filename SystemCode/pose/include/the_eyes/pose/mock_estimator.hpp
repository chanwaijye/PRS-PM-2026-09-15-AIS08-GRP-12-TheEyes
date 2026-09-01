// Mock pose estimation backend.
//
// AGENTS.md orchestration rule 2: every module runs standalone against
// synthetic input before it is wired to its upstream neighbour. This backend is
// also what lets fusion_agent develop against realistic input -- its replay
// mode with injected noise and dropout IS fusion's primary development mode.
#ifndef THE_EYES_POSE_MOCK_ESTIMATOR_HPP
#define THE_EYES_POSE_MOCK_ESTIMATOR_HPP

#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "the_eyes/pose/pose_estimator.hpp"

namespace the_eyes::pose {

// One ground-truth pose, as an Isaac Sim capture would record it.
struct GroundTruthPose {
  std::string camera_id;
  std::string cls;
  uint64_t stamp_ns = 0;
  uint32_t track_id = 0;   // disambiguates two instances of one class
  Eigen::Isometry3d pose_cam = Eigen::Isometry3d::Identity();
};

// Perturbation applied to replayed ground truth. Deterministic given `seed`:
// an ablation that cannot be re-run to the same numbers is not evidence.
struct MockNoise {
  double translation_sigma_m = 0.0;
  double rotation_sigma_rad = 0.0;
  // Fraction of requests that return no pose, simulating a view that failed to
  // resolve the object. Fusion must handle a missing view natively, so its
  // tests need a way to produce one.
  double dropout_probability = 0.0;
  uint32_t seed = 20260820;
};

class MockPoseEstimator : public PoseEstimator {
 public:
  // Returns nullopt to simulate a declined hypothesis.
  using PoseFn = std::function<std::optional<Eigen::Isometry3d>(const PoseRequest&)>;

  MockPoseEstimator(PoseFn fn, std::string name, double score);

  // Identity rotation at a fixed distance along the optical axis. The cheapest
  // possible stand-in: lets downstream modules run before any pose source
  // exists at all.
  static std::shared_ptr<MockPoseEstimator> Identity(double distance_m = 0.5,
                                                     double score = 0.9);

  // Replays recorded ground truth, optionally perturbed. Unmatched requests
  // return no pose rather than a fabricated one.
  static std::shared_ptr<MockPoseEstimator> Replay(std::vector<GroundTruthPose> table,
                                                   MockNoise noise = {},
                                                   double score = 0.9);

  std::optional<PoseEstimate> Estimate(const PoseRequest& request) override;
  std::string name() const override { return name_; }

 private:
  PoseFn fn_;
  std::string name_;
  double score_;
};

}  // namespace the_eyes::pose

#endif  // THE_EYES_POSE_MOCK_ESTIMATOR_HPP
