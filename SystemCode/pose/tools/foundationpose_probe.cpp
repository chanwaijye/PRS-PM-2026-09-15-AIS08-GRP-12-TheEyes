// End-to-end probe: the C++ pose_agent driving the real FoundationPose server.
//
// The unit tests cover the wire protocol against a fake server. This is the
// other half -- proof that the C++ client and the Python server actually
// interoperate, on real RGB-D frames, producing poses that satisfy the §3
// contract. It needs the server running:
//
//   ~/venvs/the-eyes-r1/bin/python SystemCode/pose/server/foundationpose_server.py --asset-root ~/src/FoundationPose
//
// Usage:
//   foundationpose_probe <mustard0_dir> [n_frames] [socket_path]
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <algorithm>
#include <filesystem>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "the_eyes/pose/foundationpose_client.hpp"
#include "the_eyes/pose/pose_agent.hpp"

namespace {

using namespace the_eyes;
using namespace the_eyes::pose;

Eigen::Matrix3d LoadIntrinsics(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open " + path);
  Eigen::Matrix3d K;
  for (int i = 0; i < 9; ++i) {
    if (!(file >> K(i / 3, i % 3))) throw std::runtime_error("bad intrinsics in " + path);
  }
  return K;
}

// The YCBInEOAT layout names each frame by its capture timestamp in
// nanoseconds, not by an index -- so the filenames ARE the source timestamps
// INTERFACES.md §2 asks for, and are used as such below.
std::vector<std::string> FrameStems(const std::string& dir) {
  std::vector<std::string> stems;
  for (const auto& entry : std::filesystem::directory_iterator(dir + "/rgb")) {
    if (entry.path().extension() == ".png") stems.push_back(entry.path().stem().string());
  }
  std::sort(stems.begin(), stems.end());
  return stems;
}

// Tightest box containing the non-zero mask pixels -- stands in for the
// detector, which does not exist yet.
cv::Rect2i BboxFromMask(const cv::Mat& mask) {
  std::vector<cv::Point> points;
  cv::findNonZero(mask, points);
  if (points.empty()) throw std::runtime_error("mask is empty");
  return cv::boundingRect(points);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: foundationpose_probe <mustard0_dir> [n_frames] [socket_path]\n";
    return EXIT_FAILURE;
  }
  const std::string dir = argv[1];
  const int n_frames = (argc > 2) ? std::atoi(argv[2]) : 10;
  const std::string socket_path =
      (argc > 3) ? argv[3] : "/tmp/the_eyes_foundationpose.sock";

  try {
    const Eigen::Matrix3d K = LoadIntrinsics(dir + "/cam_K.txt");

    // The demo ships one mask, for frame 0. Every later frame reuses that box:
    // this probe is testing the bridge, not tracking quality.
    const std::vector<std::string> stems = FrameStems(dir);
    if (stems.empty()) throw std::runtime_error("no frames under " + dir + "/rgb");
    std::cout << "frames available: " << stems.size() << "\n";

    cv::Mat mask = cv::imread(dir + "/masks/" + stems.front() + ".png", cv::IMREAD_GRAYSCALE);
    if (mask.empty()) throw std::runtime_error("cannot read the frame-0 mask");
    const cv::Rect2i bbox = BboxFromMask(mask);
    std::cout << "detector stand-in bbox from mask: " << bbox << "\n";

    MeshEntry mesh;
    mesh.cls = "mustard_bottle";
    mesh.mesh_path = "demo_data/mustard0/mesh/textured_simple.obj";
    mesh.symmetric = false;
    mesh.diameter_m = 0.196;
    mesh.extent = Eigen::Vector3d(0.048, 0.043, 0.093);
    MeshRegistry registry({{mesh.cls, mesh}});

    auto client = std::make_shared<FoundationPoseClient>(
        FoundationPoseClientConfig{socket_path, /*request_timeout_ms=*/60000});
    PoseAgent agent(registry, PoseAgentConfig{}, client);

    std::cout << "backend: " << agent.estimator_name() << "\n\n";

    int produced = 0;
    const int limit = std::min<int>(n_frames, static_cast<int>(stems.size()));
    for (int i = 0; i < limit; ++i) {
      const cv::Mat bgr = cv::imread(dir + "/rgb/" + stems[i] + ".png", cv::IMREAD_COLOR);
      const cv::Mat depth = cv::imread(dir + "/depth/" + stems[i] + ".png", cv::IMREAD_UNCHANGED);
      if (bgr.empty() || depth.empty()) {
        std::cerr << "missing frame " << stems[i] << ", stopping\n";
        break;
      }

      Frame frame;
      frame.camera_id = "cam_front";
      frame.stamp_ns = std::stoull(stems[i]);  // the filename is the capture time
      cv::cvtColor(bgr, frame.rgb, cv::COLOR_BGR2RGB);
      frame.depth = depth;
      frame.depth_scale_m = 0.001;
      frame.K = K;

      DetectionSet detections;
      detections.camera_id = frame.camera_id;
      detections.stamp_ns = frame.stamp_ns;
      Detection detection;
      detection.cls = mesh.cls;
      detection.score = 0.95f;
      detection.bbox = bbox;
      detection.track_id = 1;
      detections.detections = {detection};

      const std::vector<PoseEstimate> out = agent.Process(detections, frame);
      if (out.empty()) {
        std::cout << "frame " << i << ": no pose\n";
        continue;
      }
      ++produced;
      const Eigen::Vector3d t = out[0].pose_cam.translation();
      std::cout << "frame " << i << ": t = [" << t.x() << ", " << t.y() << ", " << t.z()
                << "] m, score " << out[0].score << "\n";
    }

    std::cout << "\n" << agent.latency().Format("pose_agent end-to-end") << "\n";
    std::cout << "poses produced: " << produced << " / " << limit << "\n";
    for (const std::string& w : client->warnings()) std::cout << "warning: " << w << "\n";
    return produced > 0 ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception& e) {
    std::cerr << "foundationpose_probe: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
