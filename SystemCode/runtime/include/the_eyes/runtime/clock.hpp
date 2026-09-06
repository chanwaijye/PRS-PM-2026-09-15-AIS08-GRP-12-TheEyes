// Small timing helpers for the runner. Header-only; nothing here is
// project-specific beyond the session-id format.
#ifndef THE_EYES_RUNTIME_CLOCK_HPP
#define THE_EYES_RUNTIME_CLOCK_HPP

#include <chrono>
#include <ctime>
#include <string>

namespace the_eyes::runtime {

using Clock = std::chrono::steady_clock;

inline double ElapsedMs(Clock::time_point since) {
  return std::chrono::duration<double, std::milli>(Clock::now() - since).count();
}

// Writes the elapsed wall time in milliseconds into *out when it goes out of
// scope, so a stage's timing is one line at the top of its block.
class ScopedTimer {
 public:
  explicit ScopedTimer(double* out_ms) : out_(out_ms), start_(Clock::now()) {}
  ~ScopedTimer() { *out_ = ElapsedMs(start_); }
  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

 private:
  double* out_;
  Clock::time_point start_;
};

// "2026-09-07T06:30:12Z"
inline std::string NowUtcString() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

// "20260907_143012" in local time -- the session id prefix. Local so the
// directory names sort the way the person at the desk experienced the day.
inline std::string NowSessionStamp() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
  return buf;
}

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_CLOCK_HPP
