// Runs the occlusion ablation on a synthetic dual-view scene and prints the
// table in the shape the report needs.
//
// THE NUMBERS THIS PRINTS ARE SYNTHETIC. The per-view noise is drawn from a
// Gaussian, not measured from FoundationPose, and the occlusion pattern is
// scripted rather than observed. It demonstrates that the harness produces a
// comparable three-way result; it is not the project's evaluation result, and
// must be labelled as synthetic anywhere it is quoted (CLAUDE.md constraint 5).
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>

#include "the_eyes/fusion/config.hpp"
#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/fusion/metrics.hpp"

namespace {

using namespace the_eyes;
using namespace the_eyes::fusion;

constexpr double kPi = 3.14159265358979323846;
const Eigen::Vector3d kHalfExtent(0.025, 0.025, 0.025);

Eigen::Isometry3d Pose(const Eigen::Vector3d& t, double yaw_deg = 0.0) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() =
      Eigen::AngleAxisd(yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p.translation() = t;
  return p;
}

PoseEstimate Observe(const ExtrinsicsSet& ex, const std::string& camera_id,
                     const std::string& cls, const Eigen::Isometry3d& world_pose) {
  PoseEstimate e;
  e.camera_id = camera_id;
  e.cls = cls;
  e.pose_cam = ex.at(camera_id).T_world_cam.inverse() * world_pose;
  e.score = 0.9;
  e.extent = kHalfExtent;
  return e;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string config_dir = (argc > 1) ? argv[1] : "config";
  try {
    const FusionConfig cfg = LoadFusionConfig(config_dir + "/fusion.yaml");
    const ExtrinsicsSet ex = LoadExtrinsics(config_dir + "/extrinsics.yaml");

    ModelRegistry models;
    models["cube"] = ModelEntry{BoxModelPoints(kHalfExtent, 5), BoxDiameter(kHalfExtent)};

    // Three objects. cam_front sees all of them; cam_over loses two of them to
    // occlusion for most frames -- the situation the second camera exists for.
    const std::vector<Eigen::Isometry3d> truth = {
        Pose({0.10, 0.05, 0.03}, 15.0),
        Pose({-0.12, 0.02, 0.03}, 40.0),
        Pose({0.02, -0.14, 0.03}, 75.0),
    };

    std::mt19937 rng(20260819);
    std::normal_distribution<double> noise(0.0, 0.010);  // 10 mm per axis per view

    std::vector<AblationFrame> frames;
    for (int i = 0; i < 200; ++i) {
      AblationFrame f;
      f.stamp_ns = static_cast<uint64_t>(i) * 33'000'000ULL + 1'000'000'000ULL;
      for (size_t o = 0; o < truth.size(); ++o) {
        const auto perturb = [&](const Eigen::Isometry3d& p) {
          Eigen::Isometry3d out = p;
          out.translation() += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
          return out;
        };
        f.estimates.push_back(Observe(ex, "cam_front", "cube", perturb(truth[o])));
        const bool visible_from_over = (o == 0) || (i % 4 == 0);
        if (visible_from_over) {
          f.estimates.push_back(Observe(ex, "cam_over", "cube", perturb(truth[o])));
        }
        f.ground_truth.push_back(GroundTruthObject{"cube", truth[o]});
      }
      frames.push_back(std::move(f));
    }

    const AblationResult result = RunOcclusionAblation(frames, ex, cfg, {}, models);
    std::cout << "Occlusion ablation -- SYNTHETIC INPUT, NOT AN EVALUATION RESULT\n"
              << frames.size() << " frames, " << truth.size()
              << " objects, 10 mm/axis Gaussian per-view noise\n\n"
              << result.Format()
              << "\nArms: cam_* are single views. `naive` is both views under "
                 "association +\nunweighted mean with no gate and no filter -- the "
                 "baseline. `fused` is the\nfull policy. The claim rests on "
                 "fused-vs-naive, NOT on fused-vs-single-view:\ntwo cameras beat "
                 "one whoever writes the average.\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "ablation_demo: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
