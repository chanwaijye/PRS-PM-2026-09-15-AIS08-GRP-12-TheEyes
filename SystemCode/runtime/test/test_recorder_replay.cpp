// Record -> replay round trip must be bit-exact: the replayed stream is what
// every real-data number will be computed from.
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "the_eyes/runtime/recorder.hpp"
#include "the_eyes/runtime/replay_frame_source.hpp"

namespace {

using namespace the_eyes;
using namespace the_eyes::runtime;
namespace fs = std::filesystem;

class TempDir {
 public:
  TempDir() {
    char tmpl[] = "/tmp/the_eyes_session_XXXXXX";
    const char* d = mkdtemp(tmpl);
    if (d == nullptr) throw std::runtime_error("mkdtemp failed");
    path_ = d;
  }
  ~TempDir() { fs::remove_all(path_); }
  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

SessionMeta Meta() {
  SessionMeta m;
  m.session_id = "test_session";
  m.created_utc = "2026-09-06T00:00:00Z";
  m.source = "test";
  m.camera_ids = {"cam_front", "cam_over"};
  m.max_skew_ms = 33.0;
  m.width = 16;
  m.height = 12;
  m.fps = 30;
  CameraMeta front;
  front.serial = "261722073945";
  front.depth_scale_m = 0.001;
  front.K << 600.5, 0, 8.25, 0, 601.5, 6.125, 0, 0, 1;
  front.sync_mode_requested = 1;
  front.sync_mode_actual = 1;
  front.timestamp_domain = "Global Time";
  CameraMeta over = front;
  over.serial = "261822074567";
  over.depth_scale_m = 0.0001;
  over.K(0, 0) = 610.0;
  over.sync_mode_requested = 2;
  over.sync_mode_actual = 2;
  m.cameras = {{"cam_front", front}, {"cam_over", over}};
  camera::SyncReport s;
  s.camera_a = "cam_front";
  s.camera_b = "cam_over";
  s.samples = 30;
  s.jitter_ms = 4.51;
  s.mean_signed_skew_ms = -10.97;
  s.within_threshold = true;
  s.verdict = "ok";
  m.startup_sync = s;
  return m;
}

camera::CameraFrame RandomFrame(const std::string& id, const CameraMeta& cam, uint64_t stamp,
                                uint64_t number, std::mt19937& rng) {
  camera::CameraFrame f;
  f.camera_id = id;
  f.serial = cam.serial;
  f.stamp_ns = stamp;
  f.frame_number = number;
  f.depth_scale_m = cam.depth_scale_m;
  f.K = cam.K;
  f.rgb = cv::Mat(12, 16, CV_8UC3);
  f.depth = cv::Mat(12, 16, CV_16UC1);
  std::uniform_int_distribution<int> byte(0, 255);
  std::uniform_int_distribution<int> word(0, 65535);
  for (int r = 0; r < 12; ++r) {
    for (int c = 0; c < 16; ++c) {
      f.rgb.at<cv::Vec3b>(r, c) = cv::Vec3b(static_cast<uchar>(byte(rng)),
                                            static_cast<uchar>(byte(rng)),
                                            static_cast<uchar>(byte(rng)));
      f.depth.at<uint16_t>(r, c) = static_cast<uint16_t>(word(rng));
    }
  }
  f.depth.at<uint16_t>(0, 0) = 0;        // a stereo hole
  f.depth.at<uint16_t>(0, 1) = 65535;    // saturation
  return f;
}

std::vector<FramePair> MakePairs(const SessionMeta& meta, int n, std::mt19937& rng) {
  std::vector<FramePair> pairs;
  for (int i = 0; i < n; ++i) {
    FramePair p;
    p.pair_index = static_cast<uint64_t>(i);
    const uint64_t front = 1'000'000'000ULL + static_cast<uint64_t>(i) * 33'000'000ULL;
    const uint64_t over = front - 11'000'000ULL;
    p.stamp_ns = front;
    p.skew_ms = 11.0;
    p.frames_dropped_since_last = (i == 3) ? 2 : 0;
    p.frames.emplace("cam_front", RandomFrame("cam_front", meta.cameras.at("cam_front"), front,
                                               100 + static_cast<uint64_t>(i), rng));
    p.frames.emplace("cam_over", RandomFrame("cam_over", meta.cameras.at("cam_over"), over,
                                              200 + static_cast<uint64_t>(i), rng));
    pairs.push_back(std::move(p));
  }
  return pairs;
}

}  // namespace

TEST(RecorderReplay, RoundTripIsBitExact) {
  TempDir tmp;
  std::mt19937 rng(20260906);
  const SessionMeta meta = Meta();
  const std::vector<FramePair> pairs = MakePairs(meta, 5, rng);

  RecorderConfig rc;
  rc.session_dir = tmp.path() / "s1";
  rc.writer_threads = 2;
  {
    Recorder rec(rc, meta);
    for (const FramePair& p : pairs) EXPECT_TRUE(rec.Enqueue(p));
    rec.Close();
    const Recorder::Stats s = rec.stats();
    EXPECT_EQ(s.enqueued, 5u);
    EXPECT_EQ(s.written, 5u);
    EXPECT_EQ(s.dropped, 0u);
    EXPECT_TRUE(s.errors.empty());
  }

  ReplayFrameSource replay(rc.session_dir);
  EXPECT_EQ(replay.total_pairs(), 5u);
  EXPECT_EQ(replay.camera_ids(), meta.camera_ids);
  const SessionMeta& back = replay.session();
  EXPECT_EQ(back.session_id, "test_session");
  EXPECT_EQ(back.pairs_written, 5u);
  EXPECT_EQ(back.pairs_dropped, 0u);
  ASSERT_TRUE(back.startup_sync.has_value());
  EXPECT_NEAR(back.startup_sync->jitter_ms, 4.51, 1e-12);
  EXPECT_NEAR(back.startup_sync->mean_signed_skew_ms, -10.97, 1e-12);

  for (const FramePair& expected : pairs) {
    ASSERT_FALSE(replay.exhausted());
    auto got = replay.NextPair(0);
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->pair_index, expected.pair_index);
    EXPECT_EQ(got->stamp_ns, expected.stamp_ns);
    EXPECT_NEAR(got->skew_ms, expected.skew_ms, 1e-6);
    EXPECT_EQ(got->frames_dropped_since_last, expected.frames_dropped_since_last);
    for (const std::string& id : meta.camera_ids) {
      const camera::CameraFrame& e = expected.frames.at(id);
      const camera::CameraFrame& g = got->frames.at(id);
      EXPECT_TRUE(g.valid());
      EXPECT_EQ(g.camera_id, e.camera_id);
      EXPECT_EQ(g.serial, e.serial);
      EXPECT_EQ(g.stamp_ns, e.stamp_ns);
      EXPECT_EQ(g.frame_number, e.frame_number);
      EXPECT_DOUBLE_EQ(g.depth_scale_m, e.depth_scale_m);
      EXPECT_TRUE(g.K.isApprox(e.K, 0.0)) << id << " K differs";
      ASSERT_EQ(g.rgb.size(), e.rgb.size());
      ASSERT_EQ(g.rgb.type(), CV_8UC3);
      ASSERT_EQ(g.depth.type(), CV_16UC1);
      cv::Mat rgb_diff;
      cv::absdiff(g.rgb, e.rgb, rgb_diff);
      EXPECT_EQ(cv::countNonZero(rgb_diff.reshape(1)), 0) << id << " rgb differs";
      EXPECT_EQ(cv::countNonZero(g.depth != e.depth), 0) << id << " depth differs";
    }
  }
  EXPECT_TRUE(replay.exhausted());
  EXPECT_FALSE(replay.NextPair(0).has_value());
}

TEST(RecorderReplay, StartAndLimitAreHonoured) {
  TempDir tmp;
  std::mt19937 rng(7);
  const SessionMeta meta = Meta();
  RecorderConfig rc;
  rc.session_dir = tmp.path() / "s2";
  {
    Recorder rec(rc, meta);
    for (const FramePair& p : MakePairs(meta, 6, rng)) rec.Enqueue(p);
  }
  ReplayConfig cfg;
  cfg.start_pair = 2;
  cfg.max_pairs = 3;
  ReplayFrameSource replay(rc.session_dir, cfg);
  std::vector<uint64_t> seen;
  while (auto p = replay.NextPair(0)) seen.push_back(p->pair_index);
  EXPECT_EQ(seen, (std::vector<uint64_t>{2, 3, 4}));
}

TEST(RecorderReplay, RefusesToOverwriteAndRejectsBadSessions) {
  TempDir tmp;
  const SessionMeta meta = Meta();
  RecorderConfig rc;
  rc.session_dir = tmp.path() / "s3";
  { Recorder rec(rc, meta); }
  EXPECT_THROW(Recorder(rc, meta), std::runtime_error);

  EXPECT_THROW(ReplayFrameSource(tmp.path() / "missing"), std::runtime_error);

  // No index.
  fs::create_directories(tmp.path() / "s4");
  WriteSessionMeta(meta, tmp.path() / "s4" / "session.yaml");
  EXPECT_THROW(ReplayFrameSource(tmp.path() / "s4"), std::runtime_error);

  // Future format.
  SessionMeta future = meta;
  future.format_version = 2;
  fs::create_directories(tmp.path() / "s5");
  WriteSessionMeta(future, tmp.path() / "s5" / "session.yaml");
  std::ofstream(tmp.path() / "s5" / "index.csv") << IndexHeader(meta.camera_ids) << "\n";
  EXPECT_THROW(ReplayFrameSource(tmp.path() / "s5"), std::runtime_error);
}

TEST(RecorderReplay, FullQueueDropsAndCountsWithoutBlocking) {
  TempDir tmp;
  std::mt19937 rng(3);
  const SessionMeta meta = Meta();
  RecorderConfig rc;
  rc.session_dir = tmp.path() / "s6";
  rc.queue_capacity = 1;
  rc.writer_threads = 1;
  rc.png_compression = 9;
  Recorder rec(rc, meta);
  const std::vector<FramePair> pairs = MakePairs(meta, 40, rng);
  int accepted = 0;
  for (const FramePair& p : pairs) accepted += rec.Enqueue(p) ? 1 : 0;
  rec.Close();
  const Recorder::Stats s = rec.stats();
  EXPECT_EQ(s.enqueued, 40u);
  EXPECT_EQ(s.written + s.dropped, 40u);
  EXPECT_EQ(static_cast<int>(s.written), accepted);
  EXPECT_TRUE(s.errors.empty());

  // Whatever survived replays cleanly and in order, and the meta agrees.
  ReplayFrameSource replay(rc.session_dir);
  EXPECT_EQ(replay.total_pairs(), s.written);
  EXPECT_EQ(replay.session().pairs_dropped, s.dropped);
  uint64_t last = 0;
  bool first = true;
  while (auto p = replay.NextPair(0)) {
    if (!first) {
      EXPECT_GT(p->pair_index, last);
    }
    last = p->pair_index;
    first = false;
  }
}
