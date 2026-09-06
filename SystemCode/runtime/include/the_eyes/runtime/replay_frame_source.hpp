// Replays a recorded session (session.hpp) as a FrameSource. Walks index.csv
// in file order with no clock unless asked, so the same session produces the
// same FramePairs every time -- the property the real-data ablation and every
// regression run depend on.
#ifndef THE_EYES_RUNTIME_REPLAY_FRAME_SOURCE_HPP
#define THE_EYES_RUNTIME_REPLAY_FRAME_SOURCE_HPP

#include <chrono>
#include <filesystem>
#include <optional>

#include "the_eyes/runtime/frame_source.hpp"
#include "the_eyes/runtime/session.hpp"

namespace the_eyes::runtime {

struct ReplayConfig {
  // Sleep between pairs by their recorded stamp delta. Never skips.
  bool realtime = false;
  // Skip rows whose pair_index is below this.
  uint64_t start_pair = 0;
  // Stop after this many pairs; 0 = all.
  uint64_t max_pairs = 0;
};

class ReplayFrameSource : public FrameSource {
 public:
  // Throws std::runtime_error on a missing or unreadable session.
  explicit ReplayFrameSource(std::filesystem::path session_dir, ReplayConfig cfg = {});

  std::optional<FramePair> NextPair(int timeout_ms) override;
  bool exhausted() const override { return cursor_ >= rows_.size() || LimitReached(); }
  const std::vector<std::string>& camera_ids() const override { return meta_.camera_ids; }
  const std::map<std::string, CameraMeta>& meta() const override { return meta_.cameras; }
  std::string describe() const override;

  const SessionMeta& session() const { return meta_; }
  size_t total_pairs() const { return rows_.size(); }

 private:
  bool LimitReached() const { return cfg_.max_pairs > 0 && emitted_ >= cfg_.max_pairs; }

  std::filesystem::path dir_;
  ReplayConfig cfg_;
  SessionMeta meta_;
  std::vector<IndexRow> rows_;
  size_t cursor_ = 0;
  uint64_t emitted_ = 0;
  std::optional<uint64_t> last_stamp_ns_;
  std::chrono::steady_clock::time_point last_emit_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_REPLAY_FRAME_SOURCE_HPP
