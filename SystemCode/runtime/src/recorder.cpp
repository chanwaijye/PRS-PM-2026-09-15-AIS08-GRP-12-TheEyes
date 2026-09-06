#include "the_eyes/runtime/recorder.hpp"

#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace the_eyes::runtime {

Recorder::Recorder(RecorderConfig cfg, SessionMeta meta)
    : cfg_(std::move(cfg)), meta_(std::move(meta)) {
  if (cfg_.queue_capacity == 0) throw std::invalid_argument("Recorder: queue_capacity must be >= 1");
  if (cfg_.writer_threads < 1) throw std::invalid_argument("Recorder: writer_threads must be >= 1");
  if (cfg_.png_compression < 0 || cfg_.png_compression > 9) {
    throw std::invalid_argument("Recorder: png_compression must be in [0, 9]");
  }
  if (meta_.camera_ids.empty()) throw std::invalid_argument("Recorder: session has no cameras");

  namespace fs = std::filesystem;
  const fs::path yaml = cfg_.session_dir / "session.yaml";
  if (fs::exists(yaml)) {
    throw std::runtime_error("Recorder: '" + cfg_.session_dir.string() +
                             "' already holds a session; refusing to overwrite");
  }
  fs::create_directories(cfg_.session_dir);
  for (const std::string& id : meta_.camera_ids) fs::create_directories(cfg_.session_dir / id);

  meta_.pairs_written = 0;
  meta_.pairs_dropped = 0;
  WriteSessionMeta(meta_, yaml);

  index_.open(cfg_.session_dir / "index.csv");
  if (!index_) throw std::runtime_error("Recorder: cannot create index.csv in '" +
                                        cfg_.session_dir.string() + "'");
  index_ << IndexHeader(meta_.camera_ids) << "\n";
  index_.flush();

  for (int i = 0; i < cfg_.writer_threads; ++i) writers_.emplace_back([this] { WriterLoop(); });
}

Recorder::~Recorder() {
  try {
    Close();
  } catch (...) {
    // Nothing useful to do while unwinding.
  }
}

bool Recorder::Enqueue(const FramePair& pair) {
  std::lock_guard<std::mutex> lock(m_);
  if (closing_) return false;
  ++stats_.enqueued;
  if (!have_first_index_) {
    next_index_to_flush_ = pair.pair_index;
    have_first_index_ = true;
  }
  if (queue_.size() >= cfg_.queue_capacity) {
    ++stats_.dropped;
    CompleteLocked(pair.pair_index, std::nullopt);
    return false;
  }
  queue_.push_back(pair);   // shallow Mat copy; nobody mutates frames after pairing
  stats_.max_queue_fill = std::max(stats_.max_queue_fill, queue_.size());
  cv_.notify_one();
  return true;
}

void Recorder::WriterLoop() {
  for (;;) {
    FramePair pair;
    {
      std::unique_lock<std::mutex> lock(m_);
      cv_.wait(lock, [&] { return closing_ || !queue_.empty(); });
      if (queue_.empty()) return;   // closing and drained
      pair = std::move(queue_.front());
      queue_.pop_front();
    }
    WritePair(pair);
  }
}

void Recorder::WritePair(const FramePair& pair) {
  IndexRow row;
  row.pair_index = pair.pair_index;
  row.stamp_ns = pair.stamp_ns;
  row.skew_ms = pair.skew_ms;
  row.frames_dropped_since_last = pair.frames_dropped_since_last;
  std::string error;
  try {
    const std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, cfg_.png_compression};
    for (const std::string& id : meta_.camera_ids) {
      const camera::CameraFrame& f = pair.frames.at(id);
      cv::Mat bgr;
      cv::cvtColor(f.rgb, bgr, cv::COLOR_RGB2BGR);
      if (!cv::imwrite(RgbPath(cfg_.session_dir, id, pair.pair_index).string(), bgr, params)) {
        throw std::runtime_error("imwrite failed for " + id + " rgb");
      }
      if (!cv::imwrite(DepthPath(cfg_.session_dir, id, pair.pair_index).string(), f.depth,
                       params)) {
        throw std::runtime_error("imwrite failed for " + id + " depth");
      }
      row.per_camera[id] = {f.stamp_ns, f.frame_number};
    }
  } catch (const std::exception& e) {
    error = e.what();
  }
  std::lock_guard<std::mutex> lock(m_);
  if (!error.empty()) {
    stats_.errors.push_back("pair " + std::to_string(pair.pair_index) + ": " + error);
    ++stats_.dropped;
    CompleteLocked(pair.pair_index, std::nullopt);
    return;
  }
  ++stats_.written;
  CompleteLocked(pair.pair_index, std::move(row));
}

void Recorder::CompleteLocked(uint64_t pair_index, std::optional<IndexRow> row) {
  completed_[pair_index] = std::move(row);
  FlushIndexLocked();
}

void Recorder::FlushIndexLocked() {
  // Pair indices from the pairer are contiguous, so rows can be flushed in
  // order as soon as the next expected one is complete.
  while (!completed_.empty() && completed_.begin()->first == next_index_to_flush_) {
    auto it = completed_.begin();
    if (it->second) index_ << FormatIndexRow(*it->second, meta_.camera_ids) << "\n";
    completed_.erase(it);
    ++next_index_to_flush_;
  }
  index_.flush();
}

void Recorder::Close() {
  {
    std::lock_guard<std::mutex> lock(m_);
    if (closed_) return;
    closing_ = true;
    cv_.notify_all();
  }
  for (std::thread& t : writers_) {
    if (t.joinable()) t.join();
  }
  std::lock_guard<std::mutex> lock(m_);
  // Anything still pending has an index gap only if a writer errored; flush
  // whatever is in order, then force the rest so no row is lost.
  FlushIndexLocked();
  for (auto& [idx, row] : completed_) {
    if (row) index_ << FormatIndexRow(*row, meta_.camera_ids) << "\n";
  }
  completed_.clear();
  index_.flush();
  index_.close();
  meta_.pairs_written = stats_.written;
  meta_.pairs_dropped = stats_.dropped;
  WriteSessionMeta(meta_, cfg_.session_dir / "session.yaml");
  closed_ = true;
}

Recorder::Stats Recorder::stats() const {
  std::lock_guard<std::mutex> lock(m_);
  return stats_;
}

}  // namespace the_eyes::runtime
