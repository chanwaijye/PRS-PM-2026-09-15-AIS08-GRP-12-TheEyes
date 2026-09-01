#include "the_eyes/camera/sync.hpp"

#include <gtest/gtest.h>

namespace the_eyes::camera {
namespace {

constexpr uint64_t kFrameNs = 33'333'333ULL;  // 30 fps

std::vector<uint64_t> Stream(uint64_t start_ns, int count, uint64_t period_ns = kFrameNs) {
  std::vector<uint64_t> out;
  for (int i = 0; i < count; ++i) out.push_back(start_ns + static_cast<uint64_t>(i) * period_ns);
  return out;
}

TEST(Sync, PairsNearestInTimeNotByIndex) {
  // Real streams are not lock-stepped. If B started half a frame late and
  // dropped its first frame, index pairing would report a whole extra frame of
  // skew that is not there.
  const std::vector<uint64_t> a = Stream(1'000'000'000ULL, 5);
  std::vector<uint64_t> b = Stream(1'000'000'000ULL + 2'000'000ULL, 5);
  b.erase(b.begin());

  const auto samples = PairNearest(a, b);
  ASSERT_EQ(samples.size(), 5u);
  // The first A frame has no earlier B partner, so it pairs with B's first --
  // one frame period away. Everything after pairs within 2 ms.
  for (size_t i = 1; i < samples.size(); ++i) {
    EXPECT_LT(std::abs(samples[i].skew_ms()), 5.0) << "sample " << i;
  }
}

TEST(Sync, WellSyncedStreamsPass) {
  const std::vector<uint64_t> a = Stream(1'000'000'000ULL, 30);
  const std::vector<uint64_t> b = Stream(1'000'000'000ULL + 1'000'000ULL, 30);  // 1 ms behind

  const SyncReport report = AnalyseSkew(PairNearest(a, b), "cam_front", "cam_over", 33.0);
  EXPECT_TRUE(report.within_threshold) << report.verdict;
  EXPECT_EQ(report.samples, 30);
  EXPECT_LT(report.max_skew_ms, 2.0);
}

TEST(Sync, HighJitterFailsAndPointsAtTheCable) {
  // Skew that swings wildly frame to frame -- the streams are not locked.
  // Gating is on JITTER, not on the raw maximum: one late frame in sixty makes
  // max exceed a frame period while the streams are genuinely synchronised.
  std::vector<SkewSample> samples;
  for (int i = 0; i < 30; ++i) {
    const uint64_t base = 1'000'000'000ULL + static_cast<uint64_t>(i) * kFrameNs;
    samples.push_back({base, base + (i % 2 == 0 ? 60'000'000ULL : 0ULL)});
  }
  const SyncReport report = AnalyseSkew(samples, "cam_front", "cam_over", 10.0);
  EXPECT_FALSE(report.within_threshold);
  EXPECT_GT(report.jitter_ms, 10.0);
  // Half the samples are out of tolerance, so this reports as "not locked"
  // rather than as a cable fault. Both are failures; naming which one is the
  // point, because they call for different actions.
  EXPECT_NE(report.verdict.find("not locked"), std::string::npos) << report.verdict;
  EXPECT_GT(report.outliers, report.samples / 4);
}

TEST(Sync, OneLateFrameDoesNotFailAnOtherwiseLockedPair) {
  // Regression from the live sync test. Gating on max skew failed a pair whose
  // jitter was under 2 ms, because a single sample landed a frame period out.
  std::vector<SkewSample> samples;
  for (int i = 0; i < 60; ++i) {
    const uint64_t base = 1'000'000'000ULL + static_cast<uint64_t>(i) * kFrameNs;
    samples.push_back({base, base + 1'000'000ULL});          // 1 ms, rock steady
  }
  samples[30].stamp_b_ns += 40'000'000ULL;                    // one late frame

  const SyncReport report = AnalyseSkew(samples, "a", "b", 33.0);
  EXPECT_GT(report.max_skew_ms, 33.0) << "the outlier is genuinely there";
  EXPECT_EQ(report.outliers, 1);
  EXPECT_TRUE(report.within_threshold) << "1/60 outliers must not fail a locked pair";
}

TEST(Sync, DistinguishesAFixedOffsetFromJitter) {
  // A one-sided offset is a fixed pipeline delay and is correctable; symmetric
  // jitter is not. The two call for completely different fixes, so the report
  // has to tell them apart.
  const std::vector<uint64_t> a = Stream(1'000'000'000ULL, 30);
  const std::vector<uint64_t> offset_b = Stream(1'000'000'000ULL - 10'000'000ULL, 30);

  const SyncReport offset = AnalyseSkew(PairNearest(a, offset_b), "a", "b", 33.0);
  EXPECT_TRUE(offset.within_threshold);
  EXPECT_GT(offset.mean_signed_skew_ms, 5.0) << "a consistent lead should show in the sign";
  EXPECT_LT(offset.jitter_ms, 1.0) << "a fixed offset has no jitter";
  EXPECT_NE(offset.verdict.find("FIXED OFFSET"), std::string::npos) << offset.verdict;

  // Alternating jitter: same magnitude, mean signed near zero.
  std::vector<uint64_t> jitter_b;
  for (int i = 0; i < 30; ++i) {
    const uint64_t base = 1'000'000'000ULL + static_cast<uint64_t>(i) * kFrameNs;
    jitter_b.push_back(i % 2 == 0 ? base + 10'000'000ULL : base - 10'000'000ULL);
  }
  const SyncReport jitter = AnalyseSkew(PairNearest(a, jitter_b), "a", "b", 33.0);
  EXPECT_LT(std::abs(jitter.mean_signed_skew_ms), 2.0);
  EXPECT_GT(jitter.jitter_ms, 5.0) << "alternating skew IS jitter";
  EXPECT_EQ(jitter.verdict.find("FIXED OFFSET"), std::string::npos) << jitter.verdict;
}

TEST(Sync, MeanAndMaxAreMagnitudesNotSignedAverages) {
  // Averaging signed skews would cancel a symmetric jitter to zero and report a
  // perfectly synced pair that is nothing of the sort.
  std::vector<SkewSample> samples = {{1'000'000'000ULL, 990'000'000ULL},
                                     {1'000'000'000ULL, 1'010'000'000ULL}};
  const SyncReport report = AnalyseSkew(samples, "a", "b", 33.0);
  EXPECT_NEAR(report.mean_skew_ms, 10.0, 1e-6);
  EXPECT_NEAR(report.max_skew_ms, 10.0, 1e-6);
  EXPECT_NEAR(report.mean_signed_skew_ms, 0.0, 1e-6);
}

TEST(Sync, EmptyInputIsReportedNotSilentlyPassed) {
  const SyncReport report = AnalyseSkew({}, "a", "b", 33.0);
  EXPECT_FALSE(report.within_threshold);
  EXPECT_EQ(report.samples, 0);
  EXPECT_NE(report.verdict.find("no frames"), std::string::npos);
  EXPECT_TRUE(PairNearest({}, {1, 2, 3}).empty());
  EXPECT_TRUE(PairNearest({1, 2, 3}, {}).empty());
}

}  // namespace
}  // namespace the_eyes::camera
