#include "the_eyes/detector/tracker.hpp"

#include <gtest/gtest.h>

namespace the_eyes::detector {
namespace {

Detection Make(const std::string& cls, cv::Rect2i bbox, float score = 0.9f) {
  Detection d;
  d.cls = cls;
  d.bbox = bbox;
  d.score = score;
  return d;
}

TrackingConfig Cfg() { return TrackingConfig{}; }  // IoU 0.30, max_age 15

TEST(IouTracker, AssignsStableIdsAcrossFramesForAMovingObject) {
  // The point of the whole class: pose_agent keys FoundationPose registration
  // on track_id, and registration costs ~1.3 s against ~11 ms for tracking.
  IouTracker tracker(Cfg());

  std::vector<Detection> frame1 = {Make("cube", cv::Rect2i(100, 100, 60, 60))};
  tracker.Assign(&frame1);
  const uint32_t id = frame1[0].track_id;
  EXPECT_NE(id, 0u);

  for (int step = 1; step <= 10; ++step) {
    std::vector<Detection> frame = {Make("cube", cv::Rect2i(100 + step * 5, 100, 60, 60))};
    tracker.Assign(&frame);
    EXPECT_EQ(frame[0].track_id, id) << "step " << step;
  }
}

TEST(IouTracker, TwoObjectsKeepDistinctIds) {
  IouTracker tracker(Cfg());
  std::vector<Detection> frame = {Make("cube", cv::Rect2i(100, 100, 60, 60)),
                                  Make("cube", cv::Rect2i(400, 100, 60, 60))};
  tracker.Assign(&frame);
  EXPECT_NE(frame[0].track_id, frame[1].track_id);

  const uint32_t left = frame[0].track_id;
  const uint32_t right = frame[1].track_id;

  std::vector<Detection> next = {Make("cube", cv::Rect2i(405, 102, 60, 60)),
                                 Make("cube", cv::Rect2i(103, 101, 60, 60))};
  tracker.Assign(&next);
  // Order swapped in the input; ids must follow geometry, not position in the list.
  EXPECT_EQ(next[0].track_id, right);
  EXPECT_EQ(next[1].track_id, left);
}

TEST(IouTracker, DoesNotMatchAcrossClasses) {
  IouTracker tracker(Cfg());
  std::vector<Detection> frame1 = {Make("cube", cv::Rect2i(100, 100, 60, 60))};
  tracker.Assign(&frame1);

  std::vector<Detection> frame2 = {Make("bowl", cv::Rect2i(100, 100, 60, 60))};
  tracker.Assign(&frame2);
  EXPECT_NE(frame2[0].track_id, frame1[0].track_id);
}

TEST(IouTracker, ANewIdIsIssuedWhenTheObjectJumpsTooFar) {
  IouTracker tracker(Cfg());
  std::vector<Detection> frame1 = {Make("cube", cv::Rect2i(100, 100, 60, 60))};
  tracker.Assign(&frame1);

  std::vector<Detection> frame2 = {Make("cube", cv::Rect2i(500, 400, 60, 60))};
  tracker.Assign(&frame2);
  EXPECT_NE(frame2[0].track_id, frame1[0].track_id);
}

TEST(IouTracker, TrackSurvivesABriefMissThenReattaches) {
  IouTracker tracker(Cfg());
  std::vector<Detection> frame1 = {Make("cube", cv::Rect2i(100, 100, 60, 60))};
  tracker.Assign(&frame1);
  const uint32_t id = frame1[0].track_id;

  for (int i = 0; i < 3; ++i) {
    std::vector<Detection> empty;
    tracker.Assign(&empty);  // object briefly undetected
  }
  std::vector<Detection> back = {Make("cube", cv::Rect2i(102, 101, 60, 60))};
  tracker.Assign(&back);
  EXPECT_EQ(back[0].track_id, id);
}

TEST(IouTracker, TrackIsRetiredAfterMaxAge) {
  TrackingConfig cfg = Cfg();
  cfg.max_age_frames = 3;
  IouTracker tracker(cfg);

  std::vector<Detection> frame1 = {Make("cube", cv::Rect2i(100, 100, 60, 60))};
  tracker.Assign(&frame1);
  EXPECT_EQ(tracker.active_track_count(), 1u);

  for (int i = 0; i < 5; ++i) {
    std::vector<Detection> empty;
    tracker.Assign(&empty);
  }
  EXPECT_EQ(tracker.active_track_count(), 0u);

  std::vector<Detection> back = {Make("cube", cv::Rect2i(100, 100, 60, 60))};
  tracker.Assign(&back);
  EXPECT_NE(back[0].track_id, frame1[0].track_id) << "a retired id must not be reused";
}

TEST(IouTracker, IdsAreNeverReusedEvenAfterReset) {
  // A recycled id would make two different physical objects share
  // FoundationPose's per-object estimator state on the server.
  IouTracker tracker(Cfg());
  std::vector<Detection> frame1 = {Make("cube", cv::Rect2i(100, 100, 60, 60))};
  tracker.Assign(&frame1);
  const uint32_t first = frame1[0].track_id;

  tracker.Reset();
  std::vector<Detection> frame2 = {Make("cube", cv::Rect2i(100, 100, 60, 60))};
  tracker.Assign(&frame2);
  EXPECT_GT(frame2[0].track_id, first);
}

TEST(IouTracker, HandlesNullAndEmptyInput) {
  IouTracker tracker(Cfg());
  EXPECT_NO_THROW(tracker.Assign(nullptr));
  std::vector<Detection> empty;
  EXPECT_NO_THROW(tracker.Assign(&empty));
}

}  // namespace
}  // namespace the_eyes::detector
