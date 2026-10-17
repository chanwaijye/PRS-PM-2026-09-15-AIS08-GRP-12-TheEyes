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

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "the_eyes/pose/pose_estimator.hpp"

namespace the_eyes::pose {

struct FoundationPoseClientConfig {
  std::string socket_path = "/tmp/the_eyes_foundationpose.sock";
  int request_timeout_ms = 5000;
  // A tracked object is registered again when its detection box has grown or
  // shrunk by more than this factor in area since it was last registered.
  // Tracking only refines the previous pose, so a track that the 2D tracker
  // carried from one thing onto another (a print on an occluding card, then
  // the real object behind it once the card is gone) keeps the old pose for
  // good. 0 is off.
  double reregister_area_ratio = 0.0;
  // A tracked object is registered again once its estimate has covered less
  // than reregister_min_box_cover of its own detection box this many times
  // running (BoxCover): the box has moved on and the pose has not. Tracking
  // refines from the last pose, so it can be outrun -- on the 10 Oct clean
  // take the cube was lifted out of the bowl and its pose stayed in the bowl,
  // where no depth check can refute it -- and a pose registered while a hand
  // was still setting the object down never recovers. A box on the visible
  // part of a half hidden object lies inside the footprint, so an occluder
  // that stays put does not trigger it (one sweeping across can). Each forced
  // registration doubles the count needed for the next, so a pose that stays
  // off is not registered again every few frames. Either at 0 is off.
  double reregister_min_box_cover = 0.0;
  int reregister_after_off_box = 0;
};

// True when a box of `area` px has changed from the `registered_area` px it was
// registered on by more than `ratio` either way. Off when ratio <= 0; a zero
// area (no box given) never triggers.
bool BoxChangedBeyond(double registered_area, double area, double ratio);

// The share of `box` that the estimate's footprint covers: the bounding
// rectangle, in the image, of the eight corners of the `half_extent` box at
// `pose_cam`. 0 when a corner is behind the camera; -1 (unknown) for an empty
// box or a zero extent.
double BoxCover(const Eigen::Isometry3d& pose_cam, const Eigen::Vector3d& half_extent,
                const Eigen::Matrix3d& K, const cv::Rect2i& box);

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
  int forced_registrations() const { return forced_registrations_; }
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
  // Which objects the SERVER believes it has registered, each with the area of
  // the detection box it was last registered on. Cleared on every disconnect:
  // the server holds the per-object estimator state, so a reconnected server
  // has forgotten it and would refine against nothing.
  std::map<ObjectKey, double> registered_;
  // Per object: estimates off their box in a row, and the registrations that
  // has forced so far.
  struct OffBox {
    int running = 0;
    int forced = 0;
  };
  std::map<ObjectKey, OffBox> off_box_;
  int forced_registrations_ = 0;
  std::vector<std::string> warnings_;
};

}  // namespace the_eyes::pose

#endif  // THE_EYES_POSE_FOUNDATIONPOSE_CLIENT_HPP
