#include "the_eyes/pose/mock_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace the_eyes::pose {
namespace {

// Small-angle-safe exponential map. Duplicated rather than shared with the
// fusion module's so3.hpp on purpose: AGENTS.md forbids one module's
// agent editing another's files, and a mock backend reaching into the graded
// fusion contribution for a helper would couple them for no benefit.
Eigen::Quaterniond ExpSO3(const Eigen::Vector3d& omega) {
  const double theta = omega.norm();
  Eigen::Quaterniond q;
  if (theta < 1e-8) {
    q.w() = 1.0;
    q.vec() = 0.5 * omega;
  } else {
    q.w() = std::cos(0.5 * theta);
    q.vec() = (std::sin(0.5 * theta) / theta) * omega;
  }
  q.normalize();
  return q;
}

}  // namespace

MockPoseEstimator::MockPoseEstimator(PoseFn fn, std::string name, double score)
    : fn_(std::move(fn)), name_(std::move(name)), score_(score) {
  if (!fn_) throw std::invalid_argument("MockPoseEstimator: null pose function");
}

std::shared_ptr<MockPoseEstimator> MockPoseEstimator::Identity(double distance_m,
                                                               double score) {
  auto fn = [distance_m](const PoseRequest&) -> std::optional<Eigen::Isometry3d> {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    // +z is forward in the camera optical frame.
    pose.translation() = Eigen::Vector3d(0.0, 0.0, distance_m);
    return pose;
  };
  return std::make_shared<MockPoseEstimator>(std::move(fn), "mock:identity", score);
}

std::shared_ptr<MockPoseEstimator> MockPoseEstimator::Replay(
    std::vector<GroundTruthPose> table, MockNoise noise, double score) {
  // Shared state so the generator advances across calls; seeded once so a whole
  // run is reproducible from the seed alone.
  auto rng = std::make_shared<std::mt19937>(noise.seed);

  auto fn = [table = std::move(table), noise, rng](
                const PoseRequest& request) -> std::optional<Eigen::Isometry3d> {
    if (request.frame == nullptr) return std::nullopt;

    const auto match = std::find_if(
        table.begin(), table.end(), [&request](const GroundTruthPose& gt) {
          return gt.camera_id == request.frame->camera_id &&
                 gt.cls == request.detection.cls &&
                 gt.stamp_ns == request.frame->stamp_ns &&
                 gt.track_id == request.detection.track_id;
        });
    if (match == table.end()) return std::nullopt;

    // Dropout is drawn before noise so the same seed produces the same dropout
    // pattern regardless of whether noise is enabled.
    if (noise.dropout_probability > 0.0) {
      std::uniform_real_distribution<double> uniform(0.0, 1.0);
      if (uniform(*rng) < noise.dropout_probability) return std::nullopt;
    }

    Eigen::Isometry3d pose = match->pose_cam;
    if (noise.translation_sigma_m > 0.0) {
      std::normal_distribution<double> gauss(0.0, noise.translation_sigma_m);
      pose.translation() += Eigen::Vector3d(gauss(*rng), gauss(*rng), gauss(*rng));
    }
    if (noise.rotation_sigma_rad > 0.0) {
      std::normal_distribution<double> gauss(0.0, noise.rotation_sigma_rad);
      const Eigen::Vector3d omega(gauss(*rng), gauss(*rng), gauss(*rng));
      pose.linear() = (ExpSO3(omega) * Eigen::Quaterniond(pose.rotation())).toRotationMatrix();
    }
    return pose;
  };
  return std::make_shared<MockPoseEstimator>(std::move(fn), "mock:replay", score);
}

std::optional<PoseEstimate> MockPoseEstimator::Estimate(const PoseRequest& request) {
  if (request.frame == nullptr || request.mesh == nullptr) return std::nullopt;

  const std::optional<Eigen::Isometry3d> pose = fn_(request);
  if (!pose.has_value()) return std::nullopt;

  PoseEstimate estimate;
  estimate.camera_id = request.frame->camera_id;
  estimate.cls = request.detection.cls;
  estimate.track_id = request.detection.track_id;
  // Source frame timestamp, never receive time (INTERFACES.md §2).
  estimate.stamp_ns = request.frame->stamp_ns;
  estimate.pose_cam = *pose;
  estimate.score = score_;
  // Both come from the mesh registry, not from the estimator's own opinion:
  // fusion must not average a symmetric object's orientation, and it fills
  // ObjectState::extent from here.
  estimate.symmetric = request.mesh->symmetric;
  estimate.extent = request.mesh->extent;
  return estimate;
}

}  // namespace the_eyes::pose
