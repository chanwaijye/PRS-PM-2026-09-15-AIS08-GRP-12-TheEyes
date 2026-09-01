#include "the_eyes/detector/letterbox.hpp"

#include <gtest/gtest.h>

namespace the_eyes::detector {
namespace {

TEST(Letterbox, PreservesAspectRatioAndFillsTheSquare) {
  const cv::Mat src(480, 640, CV_8UC3, cv::Scalar(10, 20, 30));
  cv::Mat dst;
  const LetterboxTransform t = Letterbox(src, &dst, 640);

  EXPECT_EQ(dst.size(), cv::Size(640, 640));
  EXPECT_DOUBLE_EQ(t.scale, 1.0);   // 640/640 is the binding dimension
  EXPECT_EQ(t.pad_x, 0);
  EXPECT_EQ(t.pad_y, 80);           // (640 - 480) / 2
}

TEST(Letterbox, ScalesDownWhenTheSourceIsLarger) {
  const cv::Mat src(1080, 1920, CV_8UC3, cv::Scalar::all(0));
  cv::Mat dst;
  const LetterboxTransform t = Letterbox(src, &dst, 640);

  EXPECT_EQ(dst.size(), cv::Size(640, 640));
  EXPECT_NEAR(t.scale, 640.0 / 1920.0, 1e-12);
  EXPECT_EQ(t.pad_x, 0);
  EXPECT_EQ(t.pad_y, (640 - 360) / 2);
}

TEST(Letterbox, RoundTripsABoxBackToSourceCoordinates) {
  // The property that matters: a box drawn around a known source region must
  // come back to that region. Getting the inverse wrong yields detections that
  // are subtly offset and read as a calibration fault.
  const cv::Mat src(480, 640, CV_8UC3, cv::Scalar::all(0));
  cv::Mat dst;
  const LetterboxTransform t = Letterbox(src, &dst, 640);

  const cv::Rect2i source_box(100, 50, 200, 150);
  const cv::Rect2f network_box(static_cast<float>(source_box.x * t.scale + t.pad_x),
                               static_cast<float>(source_box.y * t.scale + t.pad_y),
                               static_cast<float>(source_box.width * t.scale),
                               static_cast<float>(source_box.height * t.scale));
  EXPECT_EQ(t.ToSource(network_box), source_box);
}

TEST(Letterbox, RoundTripsOnANonSquareDownscale) {
  const cv::Mat src(720, 1280, CV_8UC3, cv::Scalar::all(0));
  cv::Mat dst;
  const LetterboxTransform t = Letterbox(src, &dst, 640);

  for (const cv::Rect2i& source_box :
       {cv::Rect2i(0, 0, 100, 100), cv::Rect2i(600, 300, 320, 240),
        cv::Rect2i(1180, 620, 100, 100)}) {
    const cv::Rect2f network_box(static_cast<float>(source_box.x * t.scale + t.pad_x),
                                 static_cast<float>(source_box.y * t.scale + t.pad_y),
                                 static_cast<float>(source_box.width * t.scale),
                                 static_cast<float>(source_box.height * t.scale));
    const cv::Rect2i back = t.ToSource(network_box);
    EXPECT_NEAR(back.x, source_box.x, 2);
    EXPECT_NEAR(back.y, source_box.y, 2);
    EXPECT_NEAR(back.width, source_box.width, 3);
    EXPECT_NEAR(back.height, source_box.height, 3);
  }
}

TEST(Letterbox, ClampsBoxesThatSpillIntoPadding) {
  const cv::Mat src(480, 640, CV_8UC3, cv::Scalar::all(0));
  cv::Mat dst;
  const LetterboxTransform t = Letterbox(src, &dst, 640);

  // A box sitting entirely in the top padding band maps to nothing.
  EXPECT_EQ(t.ToSource(cv::Rect2f(10.0f, 5.0f, 50.0f, 40.0f)).area(), 0);

  // One straddling the boundary is clipped to the image, not left negative.
  const cv::Rect2i clipped = t.ToSource(cv::Rect2f(10.0f, 60.0f, 50.0f, 60.0f));
  EXPECT_GE(clipped.x, 0);
  EXPECT_GE(clipped.y, 0);
  EXPECT_LE(clipped.y + clipped.height, 480);
}

TEST(Letterbox, PaddingUsesTheConfiguredValue) {
  const cv::Mat src(240, 640, CV_8UC3, cv::Scalar(1, 2, 3));
  cv::Mat dst;
  Letterbox(src, &dst, 640, cv::Scalar(114, 114, 114));
  // Top-left pixel is padding for this aspect ratio.
  const cv::Vec3b corner = dst.at<cv::Vec3b>(0, 0);
  EXPECT_EQ(corner[0], 114);
  EXPECT_EQ(corner[1], 114);
  EXPECT_EQ(corner[2], 114);
}

TEST(Letterbox, RejectsBadInput) {
  cv::Mat dst;
  EXPECT_THROW(Letterbox(cv::Mat(), &dst, 640), std::invalid_argument);
  const cv::Mat src(10, 10, CV_8UC3);
  EXPECT_THROW(Letterbox(src, &dst, 0), std::invalid_argument);
  EXPECT_THROW(Letterbox(src, nullptr, 640), std::invalid_argument);
}

}  // namespace
}  // namespace the_eyes::detector
