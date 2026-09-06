#include "the_eyes/runtime/frame_pairer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace the_eyes::runtime {
namespace {

double AbsDeltaMs(uint64_t a, uint64_t b) {
  return std::fabs(static_cast<double>(static_cast<int64_t>(a) - static_cast<int64_t>(b))) / 1e6;
}

double SignedDeltaMs(uint64_t a, uint64_t b) {
  return static_cast<double>(static_cast<int64_t>(a) - static_cast<int64_t>(b)) / 1e6;
}

}  // namespace

FramePairer::FramePairer(PairerConfig cfg) : cfg_(std::move(cfg)) {
  if (cfg_.camera_ids.empty()) {
    throw std::invalid_argument("FramePairer: needs at least one camera_id");
  }
  if (!(cfg_.max_skew_ms > 0.0)) {
    throw std::invalid_argument("FramePairer: max_skew_ms must be positive");
  }
  if (cfg_.ring_depth == 0) {
    throw std::invalid_argument("FramePairer: ring_depth must be >= 1");
  }
  if (cfg_.camera_ids.size() > 2) {
    throw std::invalid_argument("FramePairer: pairs exactly two cameras");
  }
  if (!(cfg_.nominal_period_ms > 0.0)) {
    throw std::invalid_argument("FramePairer: nominal_period_ms must be positive");
  }
  for (const std::string& id : cfg_.camera_ids) {
    if (rings_.count(id)) {
      throw std::invalid_argument("FramePairer: duplicate camera_id '" + id + "'");
    }
    rings_[id];
    stats_.pushed[id] = 0;
  }
}

void FramePairer::Push(camera::CameraFrame frame) {
  std::lock_guard<std::mutex> lock(m_);
  auto ring = rings_.find(frame.camera_id);
  if (ring == rings_.end()) {
    throw std::invalid_argument("FramePairer: frame from unconfigured camera '" +
                                frame.camera_id + "'");
  }
  ++stats_.pushed[frame.camera_id];
  ++push_generation_;

  // Gaps in the camera's own numbering are frames the SDK dropped before we
  // ever saw them. They are still frames the pair did not use.
  auto last = last_frame_number_.find(frame.camera_id);
  if (last != last_frame_number_.end() && frame.frame_number > last->second + 1) {
    const uint64_t gap = frame.frame_number - last->second - 1;
    stats_.source_gaps += gap;
    pending_dropped_ += gap;
  }
  last_frame_number_[frame.camera_id] = frame.frame_number;

  // Too late: this camera's frame for an emitted pair was newer than this
  // one, and emitting an older pair would run fusion's filters backwards.
  auto consumed = last_consumed_stamp_ns_.find(frame.camera_id);
  if (consumed != last_consumed_stamp_ns_.end() && frame.stamp_ns <= consumed->second) {
    ++stats_.dropped_stale;
    ++pending_dropped_;
    cv_.notify_all();
    return;
  }

  ring->second.push_back(std::move(frame));
  while (ring->second.size() > cfg_.ring_depth) {
    ring->second.pop_front();
    ++stats_.dropped_stale;
    ++pending_dropped_;
  }
  cv_.notify_all();
}

void FramePairer::Fail(const std::string& camera_id, const std::string& what) {
  std::lock_guard<std::mutex> lock(m_);
  if (!failure_) failure_ = "camera '" + camera_id + "' grabber failed: " + what;
  cv_.notify_all();
}

void FramePairer::ThrowIfFailedLocked() const {
  if (failure_) throw std::runtime_error(*failure_);
}

std::optional<FramePair> FramePairer::TryPair() {
  std::lock_guard<std::mutex> lock(m_);
  ThrowIfFailedLocked();
  return TryPairLocked();
}

std::optional<FramePair> FramePairer::WaitPair(int timeout_ms) {
  std::unique_lock<std::mutex> lock(m_);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max(timeout_ms, 0));
  for (;;) {
    ThrowIfFailedLocked();
    if (auto pair = TryPairLocked()) return pair;
    const uint64_t seen = push_generation_;
    const bool woke = cv_.wait_until(lock, deadline, [&] {
      return failure_.has_value() || push_generation_ != seen;
    });
    if (!woke) {
      ++stats_.timeouts;
      return std::nullopt;
    }
  }
}

// The policy, in one place. Two rings, each ordered by arrival (= by stamp).
//
//   pivot p  = the LATER of the two oldest unconsumed frames
//   q        = the other ring's frame nearest to p; older frames that are
//              farther are dropped on the way (nothing later could want them)
//   gate     : |p - q| <= max_skew_ms, else whichever side can never pair is dropped
//   finality : a partner counts only when no frame still to come could beat
//              it. Arrivals are in order and at least one period apart, so a
//              partner within half a period is final at once; one further away
//              waits for the next frame on the side that is behind. If that
//              next frame turns out closer, the orphan is dropped and counted.
//   emit     : both fronts; stamp = reference camera's; signed skew kept.
//
// A slow consumer never sees a backlog: Push trims each ring to ring_depth,
// so by the time it asks again only the newest few frames remain and the same
// rules pick the freshest agreeing pair from those.
std::optional<FramePair> FramePairer::TryPairLocked() {
  auto drop_front = [&](std::deque<camera::CameraFrame>& ring) {
    ring.pop_front();
    ++stats_.dropped_skew;
    ++pending_dropped_;
  };
  const double half_period = 0.5 * cfg_.nominal_period_ms;

  for (;;) {
    for (const auto& [id, ring] : rings_) {
      if (ring.empty()) return std::nullopt;
    }
    if (cfg_.camera_ids.size() == 1) {
      auto& ring = rings_.at(cfg_.camera_ids[0]);
      FramePair pair;
      pair.pair_index = next_pair_index_++;
      pair.stamp_ns = ring.front().stamp_ns;
      last_consumed_stamp_ns_[cfg_.camera_ids[0]] = ring.front().stamp_ns;
      pair.frames.emplace(cfg_.camera_ids[0], std::move(ring.front()));
      ring.pop_front();
      pair.frames_dropped_since_last = pending_dropped_;
      pending_dropped_ = 0;
      ++stats_.paired;
      return pair;
    }

    auto& A = rings_.at(cfg_.camera_ids[0]);
    auto& B = rings_.at(cfg_.camera_ids[1]);
    const bool pivot_is_a = A.front().stamp_ns >= B.front().stamp_ns;
    auto& P = pivot_is_a ? A : B;
    auto& Q = pivot_is_a ? B : A;
    const uint64_t p = P.front().stamp_ns;

    // Walk Q up to its frame nearest p; the ones passed over are older and
    // farther, and every later pivot is later still, so they can never pair.
    while (Q.size() >= 2 && AbsDeltaMs(Q[1].stamp_ns, p) <= AbsDeltaMs(Q[0].stamp_ns, p)) {
      drop_front(Q);
    }
    const uint64_t q = Q.front().stamp_ns;
    const double d = AbsDeltaMs(p, q);

    if (d > cfg_.max_skew_ms) {
      if (q < p) {
        // Q's nearest is old and too far. If Q has anything after it, that is
        // farther still, so p has no partner. If not, only a future Q frame
        // can pair with p -- wait for it, but the old frame itself is spent.
        if (Q.size() >= 2) {
          drop_front(P);
          continue;
        }
        drop_front(Q);
        return std::nullopt;
      }
      // q is beyond p by more than the gate; everything after q is farther.
      drop_front(P);
      continue;
    }

    if (q < p) {
      // A later Q frame could only be closer if this one is more than half a
      // period behind p and nothing after it has arrived to rule that out.
      if (Q.size() == 1 && (static_cast<double>(p - q) / 1e6) > half_period) return std::nullopt;
    } else if (q > p) {
      if (P.size() >= 2 && AbsDeltaMs(P[1].stamp_ns, q) < d) {
        drop_front(P);   // q belongs to P's next frame; p is an orphan
        continue;
      }
      if (P.size() == 1 && (static_cast<double>(q - p) / 1e6) > half_period) return std::nullopt;
    }

    FramePair pair;
    pair.pair_index = next_pair_index_++;
    for (const std::string& id : cfg_.camera_ids) {
      auto& ring = rings_.at(id);
      last_consumed_stamp_ns_[id] = ring.front().stamp_ns;
      pair.frames.emplace(id, std::move(ring.front()));
      ring.pop_front();
    }
    const camera::CameraFrame& ref = pair.frames.at(cfg_.camera_ids[0]);
    pair.stamp_ns = ref.stamp_ns;
    pair.skew_ms = SignedDeltaMs(ref.stamp_ns, pair.frames.at(cfg_.camera_ids[1]).stamp_ns);
    pair.frames_dropped_since_last = pending_dropped_;
    pending_dropped_ = 0;
    ++stats_.paired;
    return pair;
  }
}

PairerStats FramePairer::stats() const {
  std::lock_guard<std::mutex> lock(m_);
  return stats_;
}

}  // namespace the_eyes::runtime
