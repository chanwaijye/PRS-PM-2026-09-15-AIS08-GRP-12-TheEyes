#include "the_eyes/pose/foundationpose_client.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>

#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

namespace the_eyes::pose {
namespace {

constexpr char kRequestMagic[4] = {'F', 'P', 'R', '1'};
constexpr char kResponseMagic[4] = {'F', 'P', 'S', '1'};
// A header is metadata, never pixels. Anything larger means the stream has
// desynchronised and we are reading pixel bytes as a length.
constexpr uint32_t kMaxHeaderBytes = 64 * 1024;

void AppendU32(std::string* buffer, uint32_t value) {
  const char bytes[4] = {static_cast<char>(value & 0xFF),
                         static_cast<char>((value >> 8) & 0xFF),
                         static_cast<char>((value >> 16) & 0xFF),
                         static_cast<char>((value >> 24) & 0xFF)};
  buffer->append(bytes, 4);
}

uint32_t ReadU32(const unsigned char* bytes) {
  return static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) | (static_cast<uint32_t>(bytes[3]) << 24);
}

// Contiguity matters: the payload is sent as one flat block, and a cv::Mat that
// is a view into a larger image has row padding.
cv::Mat EnsureContiguous(const cv::Mat& m) { return m.isContinuous() ? m : m.clone(); }

}  // namespace

bool BoxChangedBeyond(double registered_area, double area, double ratio) {
  if (!(ratio > 0.0) || !(registered_area > 0.0) || !(area > 0.0)) return false;
  return area > ratio * registered_area || area * ratio < registered_area;
}

double BoxCover(const Eigen::Isometry3d& pose_cam, const Eigen::Vector3d& half_extent,
                const Eigen::Matrix3d& K, const cv::Rect2i& box) {
  if (box.area() <= 0 || !(half_extent.minCoeff() > 0.0)) return -1.0;
  double u0 = std::numeric_limits<double>::infinity(), v0 = u0;
  double u1 = -u0, v1 = -u0;
  for (int i = 0; i < 8; ++i) {
    const Eigen::Vector3d corner(i & 1 ? half_extent.x() : -half_extent.x(),
                                 i & 2 ? half_extent.y() : -half_extent.y(),
                                 i & 4 ? half_extent.z() : -half_extent.z());
    const Eigen::Vector3d p = K * (pose_cam * corner);
    if (!(p.z() > 0.0)) return 0.0;
    u0 = std::min(u0, p.x() / p.z());
    u1 = std::max(u1, p.x() / p.z());
    v0 = std::min(v0, p.y() / p.z());
    v1 = std::max(v1, p.y() / p.z());
  }
  const double w = std::min<double>(box.x + box.width, u1) - std::max<double>(box.x, u0);
  const double h = std::min<double>(box.y + box.height, v1) - std::max<double>(box.y, v0);
  if (w <= 0.0 || h <= 0.0) return 0.0;
  return w * h / static_cast<double>(box.area());
}

FoundationPoseClient::FoundationPoseClient(FoundationPoseClientConfig config)
    : config_(std::move(config)) {}

FoundationPoseClient::~FoundationPoseClient() {
  if (fd_ >= 0) ::close(fd_);
}

void FoundationPoseClient::Disconnect(const std::string& reason) {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  // The server owns per-object estimator state. A reconnected server has none,
  // so every object must re-register or it would refine against nothing.
  registered_.clear();
  off_box_.clear();
  if (!reason.empty()) warnings_.push_back("disconnected: " + reason);
}

bool FoundationPoseClient::EnsureConnected() {
  if (fd_ >= 0) return true;

  const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    warnings_.push_back(std::string("socket() failed: ") + std::strerror(errno));
    return false;
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (config_.socket_path.size() >= sizeof(addr.sun_path)) {
    ::close(fd);
    warnings_.push_back("socket path too long: " + config_.socket_path);
    return false;
  }
  std::strncpy(addr.sun_path, config_.socket_path.c_str(), sizeof(addr.sun_path) - 1);

  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    warnings_.push_back("connect to '" + config_.socket_path +
                        "' failed: " + std::strerror(errno) +
                        " (is foundationpose_server.py running?)");
    return false;
  }

  // Without timeouts a wedged server stalls the whole perception pipeline
  // indefinitely, with no signal that anything is wrong.
  timeval tv{};
  tv.tv_sec = config_.request_timeout_ms / 1000;
  tv.tv_usec = (config_.request_timeout_ms % 1000) * 1000;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  fd_ = fd;
  ++reconnect_count_;
  return true;
}

bool FoundationPoseClient::SendAll(const void* data, size_t size) {
  const char* cursor = static_cast<const char*>(data);
  size_t remaining = size;
  while (remaining > 0) {
    // MSG_NOSIGNAL: a server that died mid-write must surface as EPIPE, not as
    // SIGPIPE killing the perception process.
    const ssize_t n = ::send(fd_, cursor, remaining, MSG_NOSIGNAL);
    if (n <= 0) {
      if (n < 0 && errno == EINTR) continue;
      return false;
    }
    cursor += n;
    remaining -= static_cast<size_t>(n);
  }
  return true;
}

bool FoundationPoseClient::RecvAll(void* data, size_t size) {
  char* cursor = static_cast<char*>(data);
  size_t remaining = size;
  while (remaining > 0) {
    const ssize_t n = ::recv(fd_, cursor, remaining, 0);
    if (n == 0) return false;  // orderly shutdown mid-message
    if (n < 0) {
      if (errno == EINTR) continue;
      return false;  // includes EAGAIN from the receive timeout
    }
    cursor += n;
    remaining -= static_cast<size_t>(n);
  }
  return true;
}

std::optional<PoseEstimate> FoundationPoseClient::Estimate(const PoseRequest& request) {
  if (request.frame == nullptr || request.mesh == nullptr) return std::nullopt;
  if (!EnsureConnected()) return std::nullopt;

  const Frame& frame = *request.frame;
  const cv::Mat rgb = EnsureContiguous(frame.rgb(request.roi));
  const cv::Mat depth = EnsureContiguous(frame.depth(request.roi));

  const ObjectKey key{frame.camera_id, request.detection.cls, request.detection.track_id};
  const double box_area = static_cast<double>(request.detection.bbox.area());
  const auto known = registered_.find(key);
  const bool register_now =
      known == registered_.end() ||
      BoxChangedBeyond(known->second, box_area, config_.reregister_area_ratio);

  std::ostringstream header;
  header << std::setprecision(17);
  header << "camera_id: \"" << frame.camera_id << "\"\n"
         << "cls: \"" << request.detection.cls << "\"\n"
         << "track_id: " << request.detection.track_id << "\n"
         << "stamp_ns: " << frame.stamp_ns << "\n"
         << "mode: \"" << (register_now ? "register" : "track") << "\"\n"
         << "mesh: \"" << request.mesh->mesh_path << "\"\n"
         << "roi: [" << request.roi.x << ", " << request.roi.y << ", " << request.roi.width
         << ", " << request.roi.height << "]\n"
         << "rgb_shape: [" << rgb.rows << ", " << rgb.cols << ", 3]\n"
         << "depth_shape: [" << depth.rows << ", " << depth.cols << "]\n"
         << "depth_scale_m: " << frame.depth_scale_m << "\n"
         << "K: [";
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (r != 0 || c != 0) header << ", ";
      header << frame.K(r, c);
    }
  }
  header << "]\n";
  const std::string header_bytes = header.str();

  const size_t rgb_bytes = rgb.total() * rgb.elemSize();
  const size_t depth_bytes = depth.total() * depth.elemSize();

  std::string message;
  message.reserve(16 + header_bytes.size() + rgb_bytes + depth_bytes);
  message.append(kRequestMagic, 4);
  AppendU32(&message, static_cast<uint32_t>(header_bytes.size()));
  message.append(header_bytes);
  AppendU32(&message, static_cast<uint32_t>(rgb_bytes));
  message.append(reinterpret_cast<const char*>(rgb.data), rgb_bytes);
  AppendU32(&message, static_cast<uint32_t>(depth_bytes));
  message.append(reinterpret_cast<const char*>(depth.data), depth_bytes);

  if (!SendAll(message.data(), message.size())) {
    Disconnect(std::string("send failed: ") + std::strerror(errno));
    return std::nullopt;
  }

  unsigned char response_head[8];
  if (!RecvAll(response_head, sizeof(response_head))) {
    Disconnect("no response header (timeout or server closed)");
    return std::nullopt;
  }
  if (std::memcmp(response_head, kResponseMagic, 4) != 0) {
    Disconnect("bad response magic -- protocol desynchronised");
    return std::nullopt;
  }
  const uint32_t header_len = ReadU32(response_head + 4);
  if (header_len == 0 || header_len > kMaxHeaderBytes) {
    Disconnect("implausible response header length");
    return std::nullopt;
  }

  std::string response(header_len, '\0');
  if (!RecvAll(response.data(), header_len)) {
    Disconnect("truncated response header");
    return std::nullopt;
  }

  YAML::Node node;
  try {
    node = YAML::Load(response);
  } catch (const YAML::Exception& e) {
    Disconnect(std::string("unparseable response: ") + e.what());
    return std::nullopt;
  }

  if (!node["ok"] || !node["ok"].as<bool>()) {
    const std::string reason =
        node["reason"] ? node["reason"].as<std::string>() : "unspecified";
    warnings_.push_back("server declined " + request.detection.cls + ": " + reason);
    return std::nullopt;
  }

  const YAML::Node pose_node = node["pose"];
  if (!pose_node || !pose_node.IsSequence() || pose_node.size() != 16) {
    warnings_.push_back("server returned a malformed pose for " + request.detection.cls);
    return std::nullopt;
  }

  Eigen::Matrix4d m;
  for (int i = 0; i < 16; ++i) m(i / 4, i % 4) = pose_node[i].as<double>();

  // Trusting a returned matrix blindly would let a subtly broken server poison
  // every downstream ADD-S number while looking like a successful run.
  const Eigen::Matrix3d rotation = m.topLeftCorner<3, 3>();
  const double orthonormality =
      (rotation * rotation.transpose() - Eigen::Matrix3d::Identity()).norm();
  if (!m.allFinite() || orthonormality > 1e-4 || rotation.determinant() < 0.0) {
    std::ostringstream os;
    os << "server returned a non-rigid pose for " << request.detection.cls
       << " (||RR^T - I|| = " << orthonormality << "); discarded";
    warnings_.push_back(os.str());
    return std::nullopt;
  }

  // INTERFACES.md §3 bounds the score to [0,1], and fusion weights by it. An
  // out-of-range value is a server contract violation, and accepting it would
  // let one view silently dominate every fused pose -- louder is better.
  const double score = node["score"] ? node["score"].as<double>() : 0.0;
  if (!std::isfinite(score) || score < 0.0 || score > 1.0) {
    std::ostringstream os;
    os << "server returned score " << score << " for " << request.detection.cls
       << ", outside the [0,1] the contract requires; discarded";
    warnings_.push_back(os.str());
    return std::nullopt;
  }

  // Only now is the object known-registered on the server side. Recording it
  // earlier would leave the client believing in state a failed call never made.
  // A tracking call keeps the box of the registration it refines from.
  if (register_now) registered_[key] = box_area;

  PoseEstimate estimate;
  estimate.camera_id = frame.camera_id;
  estimate.cls = request.detection.cls;
  estimate.track_id = request.detection.track_id;
  estimate.stamp_ns = frame.stamp_ns;  // source frame time, not receive time
  estimate.pose_cam.linear() = rotation;
  estimate.pose_cam.translation() = m.topRightCorner<3, 1>();
  estimate.score = score;
  estimate.symmetric = request.mesh->symmetric;
  estimate.extent = request.mesh->extent;

  if (config_.reregister_min_box_cover > 0.0 && config_.reregister_after_off_box > 0) {
    const double cover =
        BoxCover(estimate.pose_cam, estimate.extent, frame.K, request.detection.bbox);
    OffBox& off = off_box_[key];
    const int needed = config_.reregister_after_off_box << std::min(off.forced, 10);
    if (cover < 0.0 || cover >= config_.reregister_min_box_cover) {
      off = OffBox{};
    } else if (++off.running >= needed) {
      off.running = 0;
      ++off.forced;
      registered_.erase(key);
      ++forced_registrations_;
      std::ostringstream os;
      os << std::fixed << std::setprecision(2) << estimate.camera_id << " " << estimate.cls << " #"
         << estimate.track_id << ": covered " << cover << " of its detection box, under "
         << config_.reregister_min_box_cover << " on " << needed
         << " estimates running, registering again";
      warnings_.push_back(os.str());
    }
  }
  return estimate;
}

}  // namespace the_eyes::pose
