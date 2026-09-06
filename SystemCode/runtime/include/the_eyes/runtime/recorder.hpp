// Writes FramePairs to a session directory (session.hpp) without ever
// blocking the capture loop: pairs go into a bounded queue, writer threads
// encode the PNGs, and a full queue DROPS the pair and counts it. Silent
// blocking would stall the pairer and make the recorded stream lie about
// what the cameras delivered; a counted drop does not.
#ifndef THE_EYES_RUNTIME_RECORDER_HPP
#define THE_EYES_RUNTIME_RECORDER_HPP

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "the_eyes/runtime/frame_pair.hpp"
#include "the_eyes/runtime/session.hpp"

namespace the_eyes::runtime {

struct RecorderConfig {
  std::filesystem::path session_dir;
  size_t queue_capacity = 64;
  int png_compression = 1;   // 0-9; lossless at every level
  int writer_threads = 2;
};

class Recorder {
 public:
  // Creates the directory tree, writes session.yaml and the index header.
  // Throws std::runtime_error if the directory already holds a session.
  Recorder(RecorderConfig cfg, SessionMeta meta);
  ~Recorder();
  Recorder(const Recorder&) = delete;
  Recorder& operator=(const Recorder&) = delete;

  // Returns false when the queue is full and the pair was dropped.
  bool Enqueue(const FramePair& pair);
  // Drains the queue, joins the writers, finalises index.csv and session.yaml.
  // Idempotent.
  void Close();

  struct Stats {
    uint64_t enqueued = 0;
    uint64_t written = 0;
    uint64_t dropped = 0;
    size_t max_queue_fill = 0;
    std::vector<std::string> errors;
  };
  Stats stats() const;
  const std::filesystem::path& session_dir() const { return cfg_.session_dir; }

 private:
  void WriterLoop();
  void WritePair(const FramePair& pair);
  void CompleteLocked(uint64_t pair_index, std::optional<IndexRow> row);
  void FlushIndexLocked();

  RecorderConfig cfg_;
  SessionMeta meta_;
  std::ofstream index_;
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<FramePair> queue_;
  std::vector<std::thread> writers_;
  bool closing_ = false;
  bool closed_ = false;
  // Rows are written to index.csv strictly in pair order even though writer
  // threads finish out of order; dropped pairs leave a nullopt so the
  // sequence can advance past them.
  std::map<uint64_t, std::optional<IndexRow>> completed_;
  uint64_t next_index_to_flush_ = 0;
  bool have_first_index_ = false;
  Stats stats_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_RECORDER_HPP
