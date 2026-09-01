#include "the_eyes/pose/roi.hpp"

#include <gtest/gtest.h>

namespace the_eyes::pose {
namespace {

RoiConfig Cfg() { return RoiConfig{}; }  // 0.15 margin, 24 px minimum

TEST(Roi, ExpandsByTheConfiguredMargin) {
  const cv::Rect2i bbox(100, 100, 200, 100);
  const cv::Rect2i roi = ComputeRoi(bbox, 1280, 720, Cfg());
  // 15% of 200 = 30 px each side in x; 15% of 100 = 15 px each side in y.
  EXPECT_EQ(roi, cv::Rect2i(70, 85, 260, 130));
}

TEST(Roi, ClampsToTheImageAfterExpanding) {
  // Expanding first and clamping second is the order that matters: clamping
  // first would let a box on the edge grow back over it.
  const cv::Rect2i bbox(0, 0, 100, 100);
  const cv::Rect2i roi = ComputeRoi(bbox, 1280, 720, Cfg());
  EXPECT_EQ(roi.x, 0);
  EXPECT_EQ(roi.y, 0);
  EXPECT_EQ(roi.width, 115);
  EXPECT_EQ(roi.height, 115);
}

TEST(Roi, ClampsAtTheFarEdge) {
  const cv::Rect2i bbox(1200, 650, 80, 70);
  const cv::Rect2i roi = ComputeRoi(bbox, 1280, 720, Cfg());
  EXPECT_LE(roi.x + roi.width, 1280);
  EXPECT_LE(roi.y + roi.height, 720);
  EXPECT_GT(roi.width, 0);
}

TEST(Roi, RejectsACropTooSmallToConstrainSixDof) {
  const cv::Rect2i tiny(500, 500, 8, 8);  // 8px + 15% margin is still under 24
  const cv::Rect2i roi = ComputeRoi(tiny, 1280, 720, Cfg());
  EXPECT_EQ(roi.width, 0);
  EXPECT_EQ(roi.height, 0);
}

TEST(Roi, AcceptsABoxJustOverTheMinimum) {
  RoiConfig cfg = Cfg();
  cfg.margin_ratio = 0.0;
  EXPECT_EQ(ComputeRoi(cv::Rect2i(0, 0, 24, 24), 640, 480, cfg).width, 24);
  EXPECT_EQ(ComputeRoi(cv::Rect2i(0, 0, 23, 24), 640, 480, cfg).width, 0);
}

TEST(Roi, RejectsDegenerateAndOffFrameBoxes) {
  EXPECT_EQ(ComputeRoi(cv::Rect2i(10, 10, 0, 50), 640, 480, Cfg()).width, 0);
  EXPECT_EQ(ComputeRoi(cv::Rect2i(10, 10, 50, -5), 640, 480, Cfg()).width, 0);
  // Entirely past the right edge.
  EXPECT_EQ(ComputeRoi(cv::Rect2i(700, 10, 50, 50), 640, 480, Cfg()).width, 0);
  // Degenerate image.
  EXPECT_EQ(ComputeRoi(cv::Rect2i(10, 10, 50, 50), 0, 480, Cfg()).width, 0);
}

TEST(Roi, ZeroMarginReturnsTheBoxUnchanged) {
  RoiConfig cfg = Cfg();
  cfg.margin_ratio = 0.0;
  const cv::Rect2i bbox(100, 100, 200, 100);
  EXPECT_EQ(ComputeRoi(bbox, 1280, 720, cfg), bbox);
}

}  // namespace
}  // namespace the_eyes::pose
