// The whole system, end to end, with no hardware and no models.
//
//   DetectionSet -> pose_agent x2 -> fusion -> scenegraph -> PDDL
//
// Prints the occlusion ablation through the REAL module chain and the PDDL
// (:init ...) block PM1 would consume.
//
// THE NUMBERS ARE SYNTHETIC. Per-view noise is Gaussian, not measured from
// FoundationPose, and the occlusion pattern is scripted. This demonstrates that
// the contract chain holds and that the harness produces a comparable three-way
// result; it is not the project's evaluation result (CLAUDE.md constraint 5).
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "the_eyes/fusion/fusion.hpp"
#include "the_eyes/fusion/metrics.hpp"
#include "the_eyes/pose/mock_estimator.hpp"
#include "the_eyes/pose/pose_agent.hpp"
#include "the_eyes/scenegraph/pddl.hpp"

namespace {

using namespace the_eyes;

constexpr double kPi = 3.14159265358979323846;
constexpr int kWidth = 640;
constexpr int kHeight = 480;

Eigen::Isometry3d Pose(const Eigen::Vector3d& t, double yaw_deg = 0.0) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = Eigen::AngleAxisd(yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p.translation() = t;
  return p;
}

struct SceneObject {
  std::string cls;
  uint32_t track_id;
  Eigen::Isometry3d pose_w;
  Eigen::Vector3d extent;
  double diameter_m;
  bool symmetric;
};

fusion::ExtrinsicsSet Rig() {
  fusion::ExtrinsicsSet ex;
  ex.world_frame = "table_origin";
  CameraExtrinsics front;
  front.camera_id = "cam_front";
  front.serial = "FRONT001";
  front.T_world_cam.linear() = (Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitX()))
                                   .toRotationMatrix();
  front.T_world_cam.translation() = Eigen::Vector3d(0.60, 0.0, 0.40);
  front.reprojection_rmse_px = 0.12;
  ex.cameras.emplace(front.camera_id, front);

  CameraExtrinsics over;
  over.camera_id = "cam_over";
  over.serial = "OVER002";
  over.T_world_cam.linear() = Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  over.T_world_cam.translation() = Eigen::Vector3d(0.0, 0.0, 0.90);
  over.reprojection_rmse_px = 0.18;
  ex.cameras.emplace(over.camera_id, over);
  return ex;
}

std::vector<SceneObject> Scene() {
  return {
      {"table", 1, Pose({0.0, 0.0, -0.02}), {0.60, 0.40, 0.02}, 1.44, false},
      {"mustard_bottle", 2, Pose({-0.10, 0.05, 0.093}, 20.0), {0.048, 0.043, 0.093}, 0.196, false},
      {"bin", 3, Pose({0.30, 0.20, 0.06}), {0.10, 0.10, 0.06}, 0.32, true},
      {"ball", 4, Pose({0.30, 0.20, 0.03}), {0.02, 0.02, 0.02}, 0.069, true},
  };
}

pose::MeshRegistry Registry(const std::vector<SceneObject>& scene) {
  std::map<std::string, MeshEntry> entries;
  for (const SceneObject& o : scene) {
    MeshEntry e;
    e.cls = o.cls;
    e.mesh_path = "assets/meshes/" + o.cls + ".obj";
    e.symmetric = o.symmetric;
    e.diameter_m = o.diameter_m;
    e.extent = o.extent;
    entries.emplace(o.cls, e);
  }
  return pose::MeshRegistry(std::move(entries));
}

pose::Frame MakeFrame(const std::string& camera_id, uint64_t stamp_ns) {
  pose::Frame f;
  f.camera_id = camera_id;
  f.stamp_ns = stamp_ns;
  f.rgb = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(20, 30, 40));
  f.depth = cv::Mat(kHeight, kWidth, CV_16UC1, cv::Scalar(700));
  f.depth_scale_m = 0.001;
  f.K << 600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0;
  return f;
}

DetectionSet MakeDetections(const std::vector<SceneObject>& scene, const std::string& camera_id,
                            uint64_t stamp_ns) {
  DetectionSet set;
  set.camera_id = camera_id;
  set.stamp_ns = stamp_ns;
  int column = 0;
  for (const SceneObject& o : scene) {
    Detection d;
    d.cls = o.cls;
    d.score = 0.9f;
    d.track_id = o.track_id;
    d.bbox = cv::Rect2i(40 + column * 130, 120, 90, 110);
    set.detections.push_back(d);
    ++column;
  }
  return set;
}

}  // namespace

int main() {
  try {
    const std::vector<SceneObject> scene = Scene();
    const fusion::ExtrinsicsSet ex = Rig();
    constexpr int kFrames = 150;

    std::vector<uint64_t> stamps;
    for (int i = 0; i < kFrames; ++i) {
      stamps.push_back(1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL);
    }

    // Ground truth per camera. cam_over loses the mustard bottle for most
    // frames -- the occlusion the second viewpoint exists to cover.
    std::vector<pose::GroundTruthPose> table;
    fusion::ModelRegistry models;
    for (const SceneObject& o : scene) {
      models[o.cls] = fusion::ModelEntry{fusion::BoxModelPoints(o.extent, 4), o.diameter_m};
      for (size_t i = 0; i < stamps.size(); ++i) {
        table.push_back({"cam_front", o.cls, stamps[i], o.track_id,
                         ex.at("cam_front").T_world_cam.inverse() * o.pose_w});
        const bool occluded = (o.cls == "mustard_bottle") && (i % 4 != 0);
        if (!occluded) {
          table.push_back({"cam_over", o.cls, stamps[i], o.track_id,
                           ex.at("cam_over").T_world_cam.inverse() * o.pose_w});
        }
      }
    }

    pose::MockNoise noise;
    noise.translation_sigma_m = 0.010;
    noise.rotation_sigma_rad = 3.0 * kPi / 180.0;
    noise.seed = 20260820;

    auto estimator = pose::MockPoseEstimator::Replay(table, noise);
    pose::PoseAgent front(Registry(scene), pose::PoseAgentConfig{}, estimator);
    pose::PoseAgent over(Registry(scene), pose::PoseAgentConfig{}, estimator);

    std::vector<fusion::AblationFrame> frames;
    for (uint64_t stamp : stamps) {
      fusion::AblationFrame frame;
      frame.stamp_ns = stamp;
      for (const auto& entry :
           std::vector<std::pair<pose::PoseAgent*, std::string>>{{&front, "cam_front"},
                                                                 {&over, "cam_over"}}) {
        const auto out = entry.first->Process(MakeDetections(scene, entry.second, stamp),
                                              MakeFrame(entry.second, stamp));
        frame.estimates.insert(frame.estimates.end(), out.begin(), out.end());
      }
      for (const SceneObject& o : scene) {
        frame.ground_truth.push_back({o.cls, o.pose_w});
      }
      frames.push_back(std::move(frame));
    }

    std::cout << "The Eyes -- full pipeline, SYNTHETIC INPUT, NOT AN EVALUATION RESULT\n"
              << kFrames << " frames, " << scene.size()
              << " objects, 10 mm / 3 deg per-view noise,\n"
              << "mustard_bottle occluded from cam_over on 3 of every 4 frames\n\n"
              << "chain: DetectionSet -> pose_agent x2 -> fusion -> scenegraph -> PDDL\n\n";

    const fusion::AblationResult ablation =
        fusion::RunOcclusionAblation(frames, ex, fusion::FusionConfig{}, {}, models);
    std::cout << "--- occlusion ablation -------------------------------------------------\n"
              << ablation.Format() << "\n";

    // The mock backend is a table lookup, so these are ~0 and measure the
    // orchestration overhead only. The real FoundationPose backend measured
    // p50 11.1 ms end to end -- see SystemCode/pose/README.md.
    std::cout << front.latency().Format("pose_agent cam_front (MOCK backend)") << "\n"
              << over.latency().Format("pose_agent cam_over  (MOCK backend)") << "\n"
              << "  ^ mock is a table lookup; the real FoundationPose backend measured "
                 "p50 11.1 ms\n\n";

    // Final frame through fusion + scenegraph, to show what PM1 receives.
    fusion::Fusion fuser(fusion::FusionConfig{}, ex);
    SceneState state;
    for (const fusion::AblationFrame& frame : frames) {
      state = fuser.Process(frame.estimates, frame.stamp_ns);
    }
    const scenegraph::PddlResult pddl =
        scenegraph::EmitInit(state, scenegraph::PredicateConfig{});

    std::cout << "--- scene graph, final frame -------------------------------------------\n";
    for (const ObjectState& o : state.objects) {
      std::cout << "  " << o.id << "  views=0b";
      for (int bit = 1; bit >= 0; --bit) std::cout << ((o.visible_views >> bit) & 1u);
      std::cout << "  conf=" << o.confidence << "\n";
    }
    std::cout << "\n--- PDDL for PM1 -------------------------------------------------------\n"
              << pddl.init_block << "\n";
    for (const std::string& w : pddl.warnings) std::cout << "warning: " << w << "\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "pipeline_demo: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
