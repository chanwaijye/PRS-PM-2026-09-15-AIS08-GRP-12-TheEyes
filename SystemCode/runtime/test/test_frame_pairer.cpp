// The pairing policy, pinned with synthetic stamps. No hardware.
#include <atomic>
#include <chrono>
#include <random>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include "the_eyes/runtime/frame_pairer.hpp"

namespace {

using namespace the_eyes;
using namespace the_eyes::runtime;

constexpr uint64_t kMs = 1'000'000ULL;
constexpr uint64_t kBase = 1'000'000'000ULL;

camera::CameraFrame Frame(const std::string& camera_id, uint64_t stamp_ns, uint64_t number) {
  camera::CameraFrame f;
  f.camera_id = camera_id;
  f.serial = camera_id == "cam_front" ? "F" : "O";
  f.stamp_ns = stamp_ns;
  f.frame_number = number;
  f.rgb = cv::Mat(1, 1, CV_8UC3, cv::Scalar(0, 0, 0));
  f.depth = cv::Mat(1, 1, CV_16UC1, cv::Scalar(0));
  return f;
}

PairerConfig TwoCameras() {
  PairerConfig cfg;
  cfg.camera_ids = {"cam_front", "cam_over"};
  cfg.max_skew_ms = 33.0;
  cfg.ring_depth = 4;
  return cfg;
}

}  // namespace

TEST(FramePairer, RejectsBadConfig) {
  PairerConfig none;
  EXPECT_THROW(FramePairer{none}, std::invalid_argument);
  PairerConfig cfg = TwoCameras();
  cfg.ring_depth = 0;
  EXPECT_THROW(FramePairer{cfg}, std::invalid_argument);
  cfg = TwoCameras();
  cfg.max_skew_ms = 0.0;
  EXPECT_THROW(FramePairer{cfg}, std::invalid_argument);
  cfg = TwoCameras();
  cfg.camera_ids = {"a", "a"};
  EXPECT_THROW(FramePairer{cfg}, std::invalid_argument);
  cfg = TwoCameras();
  cfg.camera_ids = {"a", "b", "c"};
  EXPECT_THROW(FramePairer{cfg}, std::invalid_argument);
}

TEST(FramePairer, PairsNearestWithFixedOffset) {
  // The hardware-synced rig: cam_over leads by ~11 ms every frame.
  FramePairer pairer(TwoCameras());
  const uint64_t front[] = {kBase, kBase + 33 * kMs, kBase + 66 * kMs};
  const uint64_t over[] = {kBase - 11 * kMs, kBase + 22 * kMs, kBase + 55 * kMs};

  for (int i = 0; i < 3; ++i) {
    pairer.Push(Frame("cam_front", front[i], 100 + i));
    pairer.Push(Frame("cam_over", over[i], 200 + i));
    auto pair = pairer.TryPair();
    ASSERT_TRUE(pair.has_value()) << "frame " << i;
    EXPECT_EQ(pair->pair_index, static_cast<uint64_t>(i));
    EXPECT_EQ(pair->stamp_ns, front[i]) << "stamp is the reference camera's";
    EXPECT_EQ(pair->frames.at("cam_front").stamp_ns, front[i]);
    EXPECT_EQ(pair->frames.at("cam_over").stamp_ns, over[i]);
    EXPECT_NEAR(pair->skew_ms, 11.0, 1e-9);
    EXPECT_EQ(pair->frames_dropped_since_last, 0u);
    EXPECT_FALSE(pairer.TryPair().has_value()) << "rings are consumed";
  }
  const PairerStats s = pairer.stats();
  EXPECT_EQ(s.paired, 3u);
  EXPECT_EQ(s.dropped_skew, 0u);
  EXPECT_EQ(s.dropped_stale, 0u);
}

TEST(FramePairer, WaitsForTheLaggingCamera) {
  FramePairer pairer(TwoCameras());
  pairer.Push(Frame("cam_front", kBase, 1));
  EXPECT_FALSE(pairer.TryPair().has_value());
  pairer.Push(Frame("cam_front", kBase + 33 * kMs, 2));
  EXPECT_FALSE(pairer.TryPair().has_value());
  // cam_over finally arrives, nearest to the first front frame.
  pairer.Push(Frame("cam_over", kBase + 5 * kMs, 1));
  auto pair = pairer.TryPair();
  ASSERT_TRUE(pair.has_value());
  EXPECT_EQ(pair->frames.at("cam_front").stamp_ns, kBase);
  EXPECT_EQ(pair->frames.at("cam_over").stamp_ns, kBase + 5 * kMs);
  // The second front frame is still waiting for its partner.
  EXPECT_FALSE(pairer.TryPair().has_value());
  pairer.Push(Frame("cam_over", kBase + 38 * kMs, 2));
  pair = pairer.TryPair();
  ASSERT_TRUE(pair.has_value());
  EXPECT_EQ(pair->frames.at("cam_front").stamp_ns, kBase + 33 * kMs);
  EXPECT_EQ(pair->frames_dropped_since_last, 0u);
}

TEST(FramePairer, GapInOneStreamDropsUnpairableFramesAndStaysMonotonic) {
  FramePairer pairer(TwoCameras());
  // cam_over misses 100 ms (frames 2..4 of 3 never arrive).
  std::vector<uint64_t> emitted;
  for (int i = 0; i < 8; ++i) {
    const uint64_t t = kBase + static_cast<uint64_t>(i) * 33 * kMs;
    pairer.Push(Frame("cam_front", t, 1 + i));
    const bool over_present = (i < 2) || (i >= 5);
    if (over_present) pairer.Push(Frame("cam_over", t - 11 * kMs, 1 + i));
    while (auto pair = pairer.TryPair()) {
      EXPECT_NEAR(pair->skew_ms, 11.0, 1e-9);
      emitted.push_back(pair->stamp_ns);
    }
  }
  ASSERT_EQ(emitted.size(), 5u);   // i = 0,1,5,6,7
  for (size_t i = 1; i < emitted.size(); ++i) EXPECT_GT(emitted[i], emitted[i - 1]);
  const PairerStats s = pairer.stats();
  EXPECT_EQ(s.paired, 5u);
  // Front frames 2,3,4 had no partner within 33 ms.
  EXPECT_EQ(s.dropped_skew + s.dropped_stale, 3u);
  // cam_over's own numbering jumped from 2 to 6: three source gaps.
  EXPECT_EQ(s.source_gaps, 3u);
}

TEST(FramePairer, LateFrameForAnEmittedPairIsStale) {
  FramePairer pairer(TwoCameras());
  pairer.Push(Frame("cam_front", kBase + 33 * kMs, 2));
  pairer.Push(Frame("cam_over", kBase + 30 * kMs, 2));
  ASSERT_TRUE(pairer.TryPair().has_value());
  // An older cam_over frame arrives after the pair was emitted.
  pairer.Push(Frame("cam_over", kBase, 1));
  EXPECT_FALSE(pairer.TryPair().has_value());
  EXPECT_EQ(pairer.stats().dropped_stale, 1u);
}

TEST(FramePairer, RingOverflowTrimsAndCounts) {
  FramePairer pairer(TwoCameras());
  for (int i = 0; i < 10; ++i) {
    pairer.Push(Frame("cam_front", kBase + static_cast<uint64_t>(i) * 33 * kMs, 1 + i));
  }
  EXPECT_FALSE(pairer.TryPair().has_value());
  EXPECT_EQ(pairer.stats().dropped_stale, 6u);
  // The partner for the newest surviving frame pairs immediately, and the
  // three older survivors are reported as unused on that pair.
  pairer.Push(Frame("cam_over", kBase + 9 * 33 * kMs - 11 * kMs, 1));
  auto pair = pairer.TryPair();
  ASSERT_TRUE(pair.has_value());
  EXPECT_EQ(pair->frames.at("cam_front").frame_number, 10u);
  EXPECT_EQ(pair->frames_dropped_since_last, 6u + 3u);
}

TEST(FramePairer, DoesNotPairOffByOneWhenTheTruePartnerIsStillToCome) {
  // The start-up shape on the real rig: the second grabber begins pushing
  // almost a full period after the first, so cam_over's frame lands 32.5 ms
  // after cam_front's -- inside the 33 ms gate, but the true partner is the
  // NEXT cam_front frame, 0.5 ms away. Observed live on 2026-09-06 as a
  // "fixed offset" of -32.50 ms with 0.08 ms jitter before this rule existed.
  FramePairer pairer(TwoCameras());
  pairer.Push(Frame("cam_front", kBase, 19));
  pairer.Push(Frame("cam_over", kBase + 32'500'000ULL, 16));
  EXPECT_FALSE(pairer.TryPair().has_value()) << "must wait for the next cam_front frame";
  pairer.Push(Frame("cam_front", kBase + 33 * kMs, 20));
  auto pair = pairer.TryPair();
  ASSERT_TRUE(pair.has_value());
  EXPECT_EQ(pair->frames.at("cam_front").frame_number, 20u);
  EXPECT_EQ(pair->frames.at("cam_over").frame_number, 16u);
  EXPECT_NEAR(pair->skew_ms, 0.5, 1e-9);
  EXPECT_EQ(pair->frames_dropped_since_last, 1u) << "frame 19 was the orphan";
  // And the steady state after that keeps pairing i with i-3 at +0.5 ms.
  pairer.Push(Frame("cam_over", kBase + 65'500'000ULL, 17));
  EXPECT_FALSE(pairer.TryPair().has_value());
  pairer.Push(Frame("cam_front", kBase + 66 * kMs, 21));
  pair = pairer.TryPair();
  ASSERT_TRUE(pair.has_value());
  EXPECT_NEAR(pair->skew_ms, 0.5, 1e-9);
  EXPECT_EQ(pair->frames_dropped_since_last, 0u);
}

TEST(FramePairer, FreeRunningOffsetBeyondHalfAPeriodSettlesOnTheNearerNeighbour) {
  // Unsynced cameras 20 ms apart: the first frame waits, its successor is
  // 13 ms from the other side and wins; from then on every pair is 13 ms.
  FramePairer pairer(TwoCameras());
  pairer.Push(Frame("cam_front", kBase, 1));
  pairer.Push(Frame("cam_over", kBase + 20 * kMs, 1));
  EXPECT_FALSE(pairer.TryPair().has_value());
  pairer.Push(Frame("cam_front", kBase + 33 * kMs, 2));
  auto pair = pairer.TryPair();
  ASSERT_TRUE(pair.has_value());
  EXPECT_NEAR(pair->skew_ms, 13.0, 1e-9);
  EXPECT_EQ(pair->frames_dropped_since_last, 1u);
  pairer.Push(Frame("cam_over", kBase + 53 * kMs, 2));
  pairer.Push(Frame("cam_front", kBase + 66 * kMs, 3));
  pair = pairer.TryPair();
  ASSERT_TRUE(pair.has_value());
  EXPECT_NEAR(pair->skew_ms, 13.0, 1e-9);
  EXPECT_EQ(pair->frames_dropped_since_last, 0u);
}

TEST(FramePairer, StallTimesOut) {
  FramePairer pairer(TwoCameras());
  pairer.Push(Frame("cam_front", kBase, 1));
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(pairer.WaitPair(50).has_value());
  const double waited_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_GE(waited_ms, 45.0);
  EXPECT_LT(waited_ms, 1000.0);
  EXPECT_EQ(pairer.stats().timeouts, 1u);
}

TEST(FramePairer, GrabberFailurePropagatesWithTheCameraName) {
  FramePairer pairer(TwoCameras());
  pairer.Fail("cam_over", "timed out after 5000 ms waiting for frames");
  try {
    pairer.WaitPair(10);
    FAIL() << "expected a throw";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("cam_over"), std::string::npos);
    EXPECT_NE(std::string(e.what()).find("5000 ms"), std::string::npos);
  }
  EXPECT_THROW(pairer.TryPair(), std::runtime_error);
}

TEST(FramePairer, ThreadedProducersWithASlowConsumer) {
  FramePairer pairer(TwoCameras());
  constexpr int kFrames = 60;
  std::atomic<bool> go{false};

  auto producer = [&](const std::string& id, int64_t offset_ns, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int64_t> jitter(-5 * static_cast<int64_t>(kMs),
                                                  5 * static_cast<int64_t>(kMs));
    while (!go.load()) std::this_thread::yield();
    for (int i = 0; i < kFrames; ++i) {
      const int64_t t = static_cast<int64_t>(kBase) + static_cast<int64_t>(i) * 33 *
                                                          static_cast<int64_t>(kMs) +
                        offset_ns + jitter(rng);
      pairer.Push(Frame(id, static_cast<uint64_t>(t), static_cast<uint64_t>(i + 1)));
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
  };
  std::thread front(producer, "cam_front", 0, 1);
  std::thread over(producer, "cam_over", -11 * static_cast<int64_t>(kMs), 2);
  go.store(true);

  std::vector<FramePair> pairs;
  for (;;) {
    auto pair = pairer.WaitPair(200);
    if (!pair) break;   // producers finished and the rings are drained
    pairs.push_back(std::move(*pair));
    std::this_thread::sleep_for(std::chrono::milliseconds(12));   // slower than the cameras
  }
  front.join();
  over.join();

  ASSERT_GE(pairs.size(), 10u);
  for (size_t i = 0; i < pairs.size(); ++i) {
    EXPECT_LE(std::fabs(pairs[i].skew_ms), 33.0);
    EXPECT_EQ(pairs[i].pair_index, i);
    if (i > 0) {
      EXPECT_GT(pairs[i].stamp_ns, pairs[i - 1].stamp_ns);
    }
  }
  const PairerStats s = pairer.stats();
  EXPECT_EQ(s.pushed.at("cam_front"), static_cast<uint64_t>(kFrames));
  EXPECT_EQ(s.pushed.at("cam_over"), static_cast<uint64_t>(kFrames));
  // Every pushed frame was either paired or counted as dropped.
  uint64_t dropped_reported = 0;
  for (const FramePair& p : pairs) dropped_reported += p.frames_dropped_since_last;
  EXPECT_EQ(s.paired * 2 + s.dropped_skew + s.dropped_stale + s.source_gaps,
            static_cast<uint64_t>(2 * kFrames) + s.source_gaps)
      << "conservation: pushed = paired*2 + dropped";
  EXPECT_LE(dropped_reported, s.dropped_skew + s.dropped_stale + s.source_gaps);
}
