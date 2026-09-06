// On-disk format of a recorded session -- the camera module's replay mode.
//
//   captures/runtime/<session_id>/
//     session.yaml     everything constant for the session: serials, K,
//                      depth scale, sync modes, startup skew, pair counts
//     index.csv        one row per pair: stamps, frame numbers, skew, drops
//     <camera_id>/<pair_index:06>.rgb.png     CV_8UC3, stored BGR (OpenCV)
//     <camera_id>/<pair_index:06>.depth.png   CV_16UC1, raw sensor units
//
// PNG is lossless for both streams, so a replayed CameraFrame is bit-exact.
// Format changes bump format_version and the reader refuses anything else.
#ifndef THE_EYES_RUNTIME_SESSION_HPP
#define THE_EYES_RUNTIME_SESSION_HPP

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "the_eyes/camera/sync.hpp"
#include "the_eyes/runtime/frame_source.hpp"

namespace the_eyes::runtime {

constexpr int kSessionFormatVersion = 1;

struct SessionMeta {
  int format_version = kSessionFormatVersion;
  std::string session_id;
  std::string created_utc;
  std::string source;            // "live" | "replay:<path>"
  std::vector<std::string> camera_ids;   // view order; [0] is the reference
  double max_skew_ms = 33.0;
  int width = 0;
  int height = 0;
  int fps = 0;
  bool align_depth_to_color = true;
  std::map<std::string, CameraMeta> cameras;
  std::optional<camera::SyncReport> startup_sync;
  uint64_t pairs_written = 0;
  uint64_t pairs_dropped = 0;
};

void WriteSessionMeta(const SessionMeta& meta, const std::filesystem::path& yaml_path);
// Throws std::runtime_error on a missing file, a bad format_version, or a
// camera entry without serial / K / depth scale.
SessionMeta ReadSessionMeta(const std::filesystem::path& yaml_path);

struct IndexRow {
  uint64_t pair_index = 0;
  uint64_t stamp_ns = 0;
  double skew_ms = 0.0;
  // camera_id -> (stamp_ns, frame_number)
  std::map<std::string, std::pair<uint64_t, uint64_t>> per_camera;
  uint64_t frames_dropped_since_last = 0;
};

std::string IndexHeader(const std::vector<std::string>& camera_ids);
std::string FormatIndexRow(const IndexRow& row, const std::vector<std::string>& camera_ids);
// Reads every row, matching columns by header so the camera order in the
// file does not have to match the caller's.
std::vector<IndexRow> ReadIndex(const std::filesystem::path& csv_path,
                                const std::vector<std::string>& camera_ids);

std::string FrameFileStem(uint64_t pair_index);   // "000042"
std::filesystem::path RgbPath(const std::filesystem::path& session_dir,
                              const std::string& camera_id, uint64_t pair_index);
std::filesystem::path DepthPath(const std::filesystem::path& session_dir,
                                const std::string& camera_id, uint64_t pair_index);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_SESSION_HPP
