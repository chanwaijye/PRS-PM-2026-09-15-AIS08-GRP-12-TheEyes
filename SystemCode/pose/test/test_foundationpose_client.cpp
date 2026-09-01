// Protocol tests for the FoundationPose client, against an in-test fake server.
//
// Testing against the real server would need the venv, CUDA, and 250 MB of
// weights -- none of which belong in a unit test. What matters here is the
// wire contract: framing, the register/track transition, and what the client
// does when the server misbehaves.
#include "the_eyes/pose/foundationpose_client.hpp"

#include <atomic>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <gtest/gtest.h>

namespace the_eyes::pose {
namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;

std::string TempSocketPath() {
  char tmpl[] = "/tmp/the_eyes_fp_test_XXXXXX";
  const int fd = ::mkstemp(tmpl);
  if (fd >= 0) ::close(fd);
  std::string path = tmpl;
  ::unlink(path.c_str());  // bind() needs the path free
  return path;
}

uint32_t ReadU32(const unsigned char* b) {
  return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
         (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24);
}

// What the fake server should do with each request it receives.
enum class Behaviour {
  kReturnIdentity,
  kDecline,
  kReturnNonRigidPose,
  kReturnOutOfRangeScore,
  kCloseMidStream,
};

class FakeServer {
 public:
  explicit FakeServer(Behaviour behaviour) : behaviour_(behaviour), path_(TempSocketPath()) {
    listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path) - 1);
    ::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    ::listen(listen_fd_, 1);
    thread_ = std::thread([this] { Run(); });
  }

  ~FakeServer() {
    stop_ = true;
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    if (thread_.joinable()) thread_.join();
    ::unlink(path_.c_str());
  }

  const std::string& path() const { return path_; }
  int request_count() const { return requests_; }
  // Modes seen, in order -- the register/track transition is the thing worth
  // asserting, and only the server can observe it.
  std::vector<std::string> modes() {
    std::lock_guard<std::mutex> lock(mutex_);
    return modes_;
  }
  size_t last_rgb_bytes() const { return last_rgb_bytes_; }

 private:
  bool RecvAll(int fd, void* data, size_t size) {
    char* cursor = static_cast<char*>(data);
    size_t remaining = size;
    while (remaining > 0) {
      const ssize_t n = ::recv(fd, cursor, remaining, 0);
      if (n <= 0) return false;
      cursor += n;
      remaining -= static_cast<size_t>(n);
    }
    return true;
  }

  bool ReadBlock(int fd, std::string* out) {
    unsigned char length_bytes[4];
    if (!RecvAll(fd, length_bytes, 4)) return false;
    const uint32_t length = ReadU32(length_bytes);
    out->assign(length, '\0');
    return length == 0 || RecvAll(fd, out->data(), length);
  }

  void Run() {
    const int conn = ::accept(listen_fd_, nullptr, nullptr);
    if (conn < 0) return;

    while (!stop_) {
      char magic[4];
      if (!RecvAll(conn, magic, 4)) break;
      std::string header, rgb, depth;
      if (!ReadBlock(conn, &header)) break;
      if (!ReadBlock(conn, &rgb)) break;
      if (!ReadBlock(conn, &depth)) break;

      ++requests_;
      last_rgb_bytes_ = rgb.size();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        modes_.push_back(header.find("mode: \"register\"") != std::string::npos ? "register"
                                                                               : "track");
      }

      if (behaviour_ == Behaviour::kCloseMidStream) break;

      std::string body;
      if (behaviour_ == Behaviour::kDecline) {
        body = "{ok: false, reason: \"no valid depth inside the ROI\"}";
      } else if (behaviour_ == Behaviour::kReturnOutOfRangeScore) {
        // FoundationPose's raw scorer emits unbounded logits -- ~81 was observed
        // on the demo sequence before the server learned to normalise them.
        body = "{ok: true, score: 81.0156, "
               "pose: [1, 0, 0, 0.05, 0, 1, 0, -0.02, 0, 0, 1, 0.63, 0, 0, 0, 1]}";
      } else if (behaviour_ == Behaviour::kReturnNonRigidPose) {
        // Scaled rotation block -- a plausible-looking but non-rigid transform.
        body = "{ok: true, score: 0.9, pose: [2.0, 0, 0, 0.1, 0, 2.0, 0, 0.2, "
               "0, 0, 2.0, 0.6, 0, 0, 0, 1]}";
      } else {
        body = "{ok: true, score: 0.87, score_source: \"register\", "
               "pose: [1, 0, 0, 0.05, 0, 1, 0, -0.02, 0, 0, 1, 0.63, 0, 0, 0, 1]}";
      }

      std::string message = "FPS1";
      const uint32_t length = static_cast<uint32_t>(body.size());
      const char length_bytes[4] = {static_cast<char>(length & 0xFF),
                                    static_cast<char>((length >> 8) & 0xFF),
                                    static_cast<char>((length >> 16) & 0xFF),
                                    static_cast<char>((length >> 24) & 0xFF)};
      message.append(length_bytes, 4);
      message.append(body);
      if (::send(conn, message.data(), message.size(), MSG_NOSIGNAL) <= 0) break;
    }
    ::close(conn);
  }

  Behaviour behaviour_;
  std::string path_;
  int listen_fd_ = -1;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<int> requests_{0};
  std::atomic<size_t> last_rgb_bytes_{0};
  std::mutex mutex_;
  std::vector<std::string> modes_;
};

Frame MakeFrame(uint64_t stamp_ns = 1'000'000'000ULL) {
  Frame frame;
  frame.camera_id = "cam_front";
  frame.stamp_ns = stamp_ns;
  frame.rgb = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(10, 20, 30));
  frame.depth = cv::Mat(kHeight, kWidth, CV_16UC1, cv::Scalar(650));
  frame.depth_scale_m = 0.001;
  frame.K << 300.0, 0.0, 160.0, 0.0, 300.0, 120.0, 0.0, 0.0, 1.0;
  return frame;
}

MeshEntry MakeMesh() {
  MeshEntry mesh;
  mesh.cls = "mustard_bottle";
  mesh.mesh_path = "assets/meshes/mustard_bottle.obj";
  mesh.symmetric = false;
  mesh.diameter_m = 0.196;
  mesh.extent = Eigen::Vector3d(0.048, 0.043, 0.093);
  return mesh;
}

PoseRequest MakeRequest(const Frame& frame, const MeshEntry& mesh, uint32_t track_id = 1,
                        cv::Rect2i roi = cv::Rect2i(100, 80, 64, 64)) {
  PoseRequest request;
  request.frame = &frame;
  request.detection.cls = mesh.cls;
  request.detection.track_id = track_id;
  request.detection.score = 0.9f;
  request.roi = roi;
  request.mesh = &mesh;
  return request;
}

FoundationPoseClientConfig ConfigFor(const FakeServer& server, int timeout_ms = 2000) {
  FoundationPoseClientConfig config;
  config.socket_path = server.path();
  config.request_timeout_ms = timeout_ms;
  return config;
}

TEST(FoundationPoseClient, RoundTripsAPoseAndPopulatesTheContract) {
  FakeServer server(Behaviour::kReturnIdentity);
  FoundationPoseClient client(ConfigFor(server));

  const Frame frame = MakeFrame(7'000'000'000ULL);
  const MeshEntry mesh = MakeMesh();
  const std::optional<PoseEstimate> estimate = client.Estimate(MakeRequest(frame, mesh));

  ASSERT_TRUE(estimate.has_value());
  EXPECT_EQ(estimate->camera_id, "cam_front");
  EXPECT_EQ(estimate->cls, "mustard_bottle");
  EXPECT_EQ(estimate->track_id, 1u);
  EXPECT_EQ(estimate->stamp_ns, 7'000'000'000ULL);
  EXPECT_NEAR(estimate->score, 0.87, 1e-9);
  EXPECT_LT((estimate->pose_cam.translation() - Eigen::Vector3d(0.05, -0.02, 0.63)).norm(), 1e-12);
  // Symmetry and extent come from the mesh registry, never from the server.
  EXPECT_FALSE(estimate->symmetric);
  EXPECT_LT((estimate->extent - mesh.extent).norm(), 1e-12);
}

TEST(FoundationPoseClient, SendsOnlyTheRoiCropNotTheWholeFrame) {
  FakeServer server(Behaviour::kReturnIdentity);
  FoundationPoseClient client(ConfigFor(server));

  const Frame frame = MakeFrame();
  const MeshEntry mesh = MakeMesh();
  ASSERT_TRUE(client.Estimate(MakeRequest(frame, mesh, 1, cv::Rect2i(100, 80, 64, 48))).has_value());

  EXPECT_EQ(server.last_rgb_bytes(), 64u * 48u * 3u);
  EXPECT_LT(server.last_rgb_bytes(), static_cast<size_t>(kWidth) * kHeight * 3);
}

TEST(FoundationPoseClient, FirstSightingRegistersThenSubsequentFramesTrack) {
  // FoundationPose registration measured ~1258 ms against ~17 ms for tracking,
  // so re-registering every frame would be a 70x cost for nothing.
  FakeServer server(Behaviour::kReturnIdentity);
  FoundationPoseClient client(ConfigFor(server));

  const MeshEntry mesh = MakeMesh();
  for (int i = 0; i < 3; ++i) {
    const Frame frame = MakeFrame(1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL);
    ASSERT_TRUE(client.Estimate(MakeRequest(frame, mesh)).has_value()) << "i=" << i;
  }
  EXPECT_EQ(server.modes(), (std::vector<std::string>{"register", "track", "track"}));
}

TEST(FoundationPoseClient, EachTrackedInstanceRegistersIndependently) {
  FakeServer server(Behaviour::kReturnIdentity);
  FoundationPoseClient client(ConfigFor(server));

  const Frame frame = MakeFrame();
  const MeshEntry mesh = MakeMesh();
  ASSERT_TRUE(client.Estimate(MakeRequest(frame, mesh, /*track_id=*/1)).has_value());
  ASSERT_TRUE(client.Estimate(MakeRequest(frame, mesh, /*track_id=*/2)).has_value());
  ASSERT_TRUE(client.Estimate(MakeRequest(frame, mesh, /*track_id=*/1)).has_value());

  EXPECT_EQ(server.modes(), (std::vector<std::string>{"register", "register", "track"}));
}

TEST(FoundationPoseClient, DeclinedRequestReturnsNulloptWithTheServersReason) {
  FakeServer server(Behaviour::kDecline);
  FoundationPoseClient client(ConfigFor(server));

  const Frame frame = MakeFrame();
  const MeshEntry mesh = MakeMesh();
  EXPECT_FALSE(client.Estimate(MakeRequest(frame, mesh)).has_value());

  ASSERT_FALSE(client.warnings().empty());
  EXPECT_NE(client.warnings().back().find("no valid depth"), std::string::npos);
  // A decline is not a connection fault -- the socket stays up.
  EXPECT_TRUE(client.connected());
}

TEST(FoundationPoseClient, DeclinedRequestDoesNotMarkTheObjectRegistered) {
  // Otherwise the next call would ask the server to refine state it never built.
  FakeServer server(Behaviour::kDecline);
  FoundationPoseClient client(ConfigFor(server));

  const Frame frame = MakeFrame();
  const MeshEntry mesh = MakeMesh();
  client.Estimate(MakeRequest(frame, mesh));
  client.Estimate(MakeRequest(frame, mesh));

  EXPECT_EQ(server.modes(), (std::vector<std::string>{"register", "register"}));
}

TEST(FoundationPoseClient, NonRigidPoseIsRejected) {
  // A subtly broken server returning a scaled matrix would otherwise poison
  // every downstream ADD-S number while looking like a successful run.
  FakeServer server(Behaviour::kReturnNonRigidPose);
  FoundationPoseClient client(ConfigFor(server));

  const Frame frame = MakeFrame();
  const MeshEntry mesh = MakeMesh();
  EXPECT_FALSE(client.Estimate(MakeRequest(frame, mesh)).has_value());
  ASSERT_FALSE(client.warnings().empty());
  EXPECT_NE(client.warnings().back().find("non-rigid"), std::string::npos);
}

TEST(FoundationPoseClient, ScoreOutsideTheContractRangeIsRejected) {
  // Regression from the first live run against the real server. Fusion weights
  // by score, so an unnormalised 81 next to another view's 0.9 would let one
  // view dominate every fuse.
  FakeServer server(Behaviour::kReturnOutOfRangeScore);
  FoundationPoseClient client(ConfigFor(server));

  const Frame frame = MakeFrame();
  const MeshEntry mesh = MakeMesh();
  EXPECT_FALSE(client.Estimate(MakeRequest(frame, mesh)).has_value());
  ASSERT_FALSE(client.warnings().empty());
  EXPECT_NE(client.warnings().back().find("outside the [0,1]"), std::string::npos);
}

TEST(FoundationPoseClient, NoServerYieldsNulloptAndAnActionableWarning) {
  FoundationPoseClientConfig config;
  config.socket_path = "/tmp/the_eyes_definitely_no_server.sock";
  config.request_timeout_ms = 500;
  FoundationPoseClient client(config);

  const Frame frame = MakeFrame();
  const MeshEntry mesh = MakeMesh();
  EXPECT_FALSE(client.Estimate(MakeRequest(frame, mesh)).has_value());
  EXPECT_FALSE(client.connected());
  ASSERT_FALSE(client.warnings().empty());
  EXPECT_NE(client.warnings().back().find("foundationpose_server.py"), std::string::npos);
}

TEST(FoundationPoseClient, ServerDeathDropsTheConnectionAndForgetsRegistration) {
  // The server owns the per-object estimator state. A reconnected server has
  // none, so tracking against it would refine from nothing.
  FakeServer server(Behaviour::kCloseMidStream);
  FoundationPoseClient client(ConfigFor(server, 500));

  const Frame frame = MakeFrame();
  const MeshEntry mesh = MakeMesh();
  EXPECT_FALSE(client.Estimate(MakeRequest(frame, mesh)).has_value());
  EXPECT_FALSE(client.connected());
  ASSERT_FALSE(client.warnings().empty());
}

TEST(FoundationPoseClient, NullFrameOrMeshIsRejectedWithoutTouchingTheSocket) {
  FakeServer server(Behaviour::kReturnIdentity);
  FoundationPoseClient client(ConfigFor(server));

  PoseRequest empty;
  EXPECT_FALSE(client.Estimate(empty).has_value());
  EXPECT_EQ(server.request_count(), 0);
}

}  // namespace
}  // namespace the_eyes::pose
