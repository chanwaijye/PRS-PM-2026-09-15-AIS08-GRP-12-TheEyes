// Writes composed console canvases to a video file, off the calling thread.
//
// WHY THIS EXISTS. The demo video should come from the code that drew the
// window, at a known frame rate, rather than from a screen recorder capturing a
// compositor. `Console::Compose` is display-free precisely so the same canvas
// can go to a window, a file, or both.
//
// THE DROP POLICY DEPENDS ON THE SOURCE, and this is the one interesting
// decision here.
//
//   live    -- must never block. The measured budget is p50 51.7 ms per pair,
//              and an encoder stalling the capture loop would make the console
//              a stage rather than a sink (docs/VIZ_PLAN.md §3.4). Drops are
//              counted and shown, so a gappy video cannot be mistaken for a
//              slow system.
//   replay  -- must be lossless, or acceptance criterion 5 is unmeetable: the
//              same session has to produce the same video twice. Replay has no
//              real-time constraint, so blocking there costs nothing but wall
//              clock.
//
// Getting this backwards either drops frames from a deterministic re-shoot or
// lets an encoder back-pressure into a live capture, and both failures are
// quiet.
#ifndef THE_EYES_RUNTIME_CONSOLE_RECORDER_HPP
#define THE_EYES_RUNTIME_CONSOLE_RECORDER_HPP

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

namespace the_eyes::runtime {

class ConsoleRecorder {
 public:
  // Throws std::runtime_error if the file cannot be opened for writing.
  ConsoleRecorder(const std::string& path, cv::Size frame_size, double fps, bool drop_when_full,
                  size_t queue_capacity = 8);
  ~ConsoleRecorder();

  ConsoleRecorder(const ConsoleRecorder&) = delete;
  ConsoleRecorder& operator=(const ConsoleRecorder&) = delete;

  // Returns false when the frame was dropped. Never throws; a failing recorder
  // must not take the run down with it.
  bool Offer(const cv::Mat& canvas);

  // Blocks until everything offered has been written, then closes the file.
  // Called by the destructor; safe to call early and twice.
  void Finish();

  uint64_t written() const { return written_.load(); }
  uint64_t dropped() const { return dropped_.load(); }
  const std::string& path() const { return path_; }

 private:
  void Run();

  std::string path_;
  bool drop_when_full_;
  size_t capacity_;
  std::deque<cv::Mat> queue_;
  mutable std::mutex mutex_;
  std::condition_variable space_, work_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> written_{0};
  std::atomic<uint64_t> dropped_{0};
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::thread thread_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_CONSOLE_RECORDER_HPP
