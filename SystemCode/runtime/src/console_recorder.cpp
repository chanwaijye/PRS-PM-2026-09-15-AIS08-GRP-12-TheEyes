#include "the_eyes/runtime/console_recorder.hpp"

#include <stdexcept>

#include <opencv2/videoio.hpp>

namespace the_eyes::runtime {

struct ConsoleRecorder::Impl {
  cv::VideoWriter writer;
};

ConsoleRecorder::ConsoleRecorder(const std::string& path, cv::Size frame_size, double fps,
                                 bool drop_when_full, size_t queue_capacity)
    : path_(path),
      drop_when_full_(drop_when_full),
      capacity_(queue_capacity == 0 ? 1 : queue_capacity),
      impl_(std::make_unique<Impl>()) {
  // MJPG in an AVI container: every frame is encoded independently, so the
  // output is a deterministic function of the input frames. Inter-frame codecs
  // are not, which would make acceptance criterion 5 untestable.
  const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  if (!impl_->writer.open(path, fourcc, fps > 0.0 ? fps : 30.0, frame_size, true)) {
    throw std::runtime_error("console recorder: cannot open '" + path + "' for writing");
  }
  thread_ = std::thread(&ConsoleRecorder::Run, this);
}

ConsoleRecorder::~ConsoleRecorder() {
  try {
    Finish();
  } catch (...) {
  }
}

bool ConsoleRecorder::Offer(const cv::Mat& canvas) {
  if (canvas.empty()) return false;
  std::unique_lock<std::mutex> lock(mutex_);
  if (queue_.size() >= capacity_) {
    if (drop_when_full_) {
      dropped_.fetch_add(1);
      return false;
    }
    space_.wait(lock, [&] { return queue_.size() < capacity_ || stop_.load(); });
    if (stop_.load()) return false;
  }
  queue_.push_back(canvas.clone());   // the caller reuses its canvas
  work_.notify_one();
  return true;
}

void ConsoleRecorder::Run() {
  for (;;) {
    cv::Mat frame;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_.wait(lock, [&] { return !queue_.empty() || stop_.load(); });
      if (queue_.empty()) {
        if (stop_.load()) return;
        continue;
      }
      frame = std::move(queue_.front());
      queue_.pop_front();
      space_.notify_one();
    }
    impl_->writer.write(frame);
    written_.fetch_add(1);
  }
}

void ConsoleRecorder::Finish() {
  if (!thread_.joinable()) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_.store(true);
  }
  work_.notify_all();
  space_.notify_all();
  thread_.join();
  // Drain anything the writer thread did not reach before it saw the stop.
  while (!queue_.empty()) {
    impl_->writer.write(queue_.front());
    queue_.pop_front();
    written_.fetch_add(1);
  }
  impl_->writer.release();
}

}  // namespace the_eyes::runtime
