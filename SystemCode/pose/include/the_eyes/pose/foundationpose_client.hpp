// FoundationPose backend, over a Unix domain socket.
//
// WHY OUT-OF-PROCESS. CLAUDE.md constraint 1 puts all runtime code in C++17,
// and FoundationPose has no C++ implementation -- it is a Python library
// (docs/R1_STATUS.md). Embedding an interpreter would put the GIL, NumPy's ABI,
// and CUDA context ownership inside our runtime process. Running it as a model
// server keeps every line of OUR runtime logic in C++ and treats FoundationPose
// exactly as a TensorRT engine would be treated: an external inference service
// behind a byte protocol.
//
// PROTOCOL. Length-prefixed frames on SOCK_STREAM. Headers are YAML -- both
// sides already have a parser (yaml-cpp here, PyYAML there), so the bridge adds
// no dependency to either. Pixel data travels as raw bytes beside the header
// rather than inside it, because base64 in a 30 fps path is wasted work.
//
//   request : "FPR1" | u32 header_len | header | u32 rgb_len | rgb | u32 depth_len | depth
//   response: "FPS1" | u32 header_len | header
//
// All u32 are little-endian. See server/foundationpose_server.py.
#ifndef THE_EYES_POSE_FOUNDATIONPOSE_CLIENT_HPP
#define THE_EYES_POSE_FOUNDATIONPOSE_CLIENT_HPP

#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "the_eyes/pose/pose_estimator.hpp"

namespace the_eyes::pose {

struct FoundationPoseClientConfig {
  std::string socket_path = "/tmp/the_eyes_foundationpose.sock";
  int request_timeout_ms = 5000;
};

class FoundationPoseClient : public PoseEstimator {
 public:
  explicit FoundationPoseClient(FoundationPoseClientConfig config);
  ~FoundationPoseClient() override;

  FoundationPoseClient(const FoundationPoseClient&) = delete;
  FoundationPoseClient& operator=(const FoundationPoseClient&) = delete;

  std::optional<PoseEstimate> Estimate(const PoseRequest& request) override;
  std::string name() const override { return "foundationpose"; }

  bool connected() const { return fd_ >= 0; }
  int reconnect_count() const { return reconnect_count_; }
  const std::vector<std::string>& warnings() const { return warnings_; }
  void ClearWarnings() { warnings_.clear(); }

 private:
  // Object identity for the register/track distinction: FoundationPose does an
  // expensive global registration on first sight (~1258 ms measured) and cheap
  // refinement thereafter (~17 ms).
  using ObjectKey = std::tuple<std::string, std::string, uint32_t>;

  bool EnsureConnected();
  void Disconnect(const std::string& reason);
  bool SendAll(const void* data, size_t size);
  bool RecvAll(void* data, size_t size);

  FoundationPoseClientConfig config_;
  int fd_ = -1;
  int reconnect_count_ = 0;
  // Which objects the SERVER believes it has registered. Cleared on every
  // disconnect: the server holds the per-object estimator state, so a
  // reconnected server has forgotten it and would refine against nothing.
  std::set<ObjectKey> registered_;
  std::vector<std::string> warnings_;
};

}  // namespace the_eyes::pose

#endif  // THE_EYES_POSE_FOUNDATIONPOSE_CLIENT_HPP
