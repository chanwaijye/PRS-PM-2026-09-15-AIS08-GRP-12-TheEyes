#include "the_eyes/runtime/replay_frame_source.hpp"

#include <stdexcept>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace the_eyes::runtime {

ReplayFrameSource::ReplayFrameSource(std::filesystem::path session_dir, ReplayConfig cfg)
    : dir_(std::move(session_dir)), cfg_(cfg) {
  namespace fs = std::filesystem;
  if (!fs::is_directory(dir_)) {
    throw std::runtime_error("replay: '" + dir_.string() + "' is not a directory");
  }
  meta_ = ReadSessionMeta(dir_ / "session.yaml");
  rows_ = ReadIndex(dir_ / "index.csv", meta_.camera_ids);
  while (cursor_ < rows_.size() && rows_[cursor_].pair_index < cfg_.start_pair) ++cursor_;
}

std::string ReplayFrameSource::describe() const {
  return "replay " + dir_.string() + " (" + std::to_string(rows_.size()) + " pairs, session " +
         meta_.session_id + ")";
}

std::optional<FramePair> ReplayFrameSource::NextPair(int /*timeout_ms*/) {
  if (exhausted()) return std::nullopt;
  const IndexRow& row = rows_[cursor_++];

  if (cfg_.realtime && last_stamp_ns_) {
    const auto due = last_emit_ + std::chrono::nanoseconds(row.stamp_ns - *last_stamp_ns_);
    std::this_thread::sleep_until(due);
  }

  FramePair pair;
  pair.pair_index = row.pair_index;
  pair.stamp_ns = row.stamp_ns;
  pair.skew_ms = row.skew_ms;
  pair.frames_dropped_since_last = row.frames_dropped_since_last;
  for (const std::string& id : meta_.camera_ids) {
    const CameraMeta& cam = meta_.cameras.at(id);
    camera::CameraFrame f;
    f.camera_id = id;
    f.serial = cam.serial;
    f.depth_scale_m = cam.depth_scale_m;
    f.K = cam.K;
    const auto [stamp, number] = row.per_camera.at(id);
    f.stamp_ns = stamp;
    f.frame_number = number;

    const std::string rgb_path = RgbPath(dir_, id, row.pair_index).string();
    const cv::Mat bgr = cv::imread(rgb_path, cv::IMREAD_COLOR);
    if (bgr.empty()) throw std::runtime_error("replay: cannot read '" + rgb_path + "'");
    cv::cvtColor(bgr, f.rgb, cv::COLOR_BGR2RGB);

    const std::string depth_path = DepthPath(dir_, id, row.pair_index).string();
    f.depth = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
    if (f.depth.empty()) throw std::runtime_error("replay: cannot read '" + depth_path + "'");
    if (f.depth.type() != CV_16UC1) {
      throw std::runtime_error("replay: '" + depth_path + "' is not 16-bit single channel");
    }
    if (!f.valid()) {
      throw std::runtime_error("replay: pair " + std::to_string(row.pair_index) + " camera '" +
                               id + "' does not form a valid CameraFrame");
    }
    pair.frames.emplace(id, std::move(f));
  }

  last_stamp_ns_ = row.stamp_ns;
  last_emit_ = std::chrono::steady_clock::now();
  ++emitted_;
  return pair;
}

}  // namespace the_eyes::runtime
