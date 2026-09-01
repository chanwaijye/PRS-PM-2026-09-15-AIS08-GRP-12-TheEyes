#include "the_eyes/detector/postprocess.hpp"

#include <vector>

#include <gtest/gtest.h>

namespace the_eyes::detector {
namespace {

// Builds a YOLOv8-shaped tensor: [1, 4 + num_classes, num_anchors], channel-first.
std::vector<float> MakeTensor(int num_classes, int num_anchors,
                              const std::vector<std::tuple<int, float, cv::Rect2f>>& entries) {
  const int channels = 4 + num_classes;
  std::vector<float> data(static_cast<size_t>(channels) * num_anchors, 0.0f);
  for (const auto& [anchor, score, box] : entries) {
    data[0 * num_anchors + anchor] = box.x + box.width * 0.5f;   // cx
    data[1 * num_anchors + anchor] = box.y + box.height * 0.5f;  // cy
    data[2 * num_anchors + anchor] = box.width;
    data[3 * num_anchors + anchor] = box.height;
    data[(4 + 0) * num_anchors + anchor] = score;                // class 0
  }
  return data;
}

TEST(IoU, MatchesHandComputedValues) {
  EXPECT_NEAR(IoU(cv::Rect2f(0, 0, 10, 10), cv::Rect2f(0, 0, 10, 10)), 1.0f, 1e-6);
  EXPECT_NEAR(IoU(cv::Rect2f(0, 0, 10, 10), cv::Rect2f(20, 20, 10, 10)), 0.0f, 1e-6);
  // Half overlap: intersection 50, union 150.
  EXPECT_NEAR(IoU(cv::Rect2f(0, 0, 10, 10), cv::Rect2f(5, 0, 10, 10)), 50.0f / 150.0f, 1e-6);
}

TEST(DecodeYolov8, ConvertsCentreFormToCornerRects) {
  const auto data = MakeTensor(1, 4, {{0, 0.9f, cv::Rect2f(100, 50, 40, 60)}});
  const auto out = DecodeYolov8(data.data(), 5, 4, 1, 0.25f);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].class_id, 0);
  EXPECT_NEAR(out[0].score, 0.9f, 1e-6);
  EXPECT_NEAR(out[0].box.x, 100.0f, 1e-4);
  EXPECT_NEAR(out[0].box.y, 50.0f, 1e-4);
  EXPECT_NEAR(out[0].box.width, 40.0f, 1e-4);
  EXPECT_NEAR(out[0].box.height, 60.0f, 1e-4);
}

TEST(DecodeYolov8, AppliesTheScoreThreshold) {
  const auto data = MakeTensor(1, 3,
                               {{0, 0.9f, cv::Rect2f(0, 0, 10, 10)},
                                {1, 0.10f, cv::Rect2f(20, 20, 10, 10)}});
  EXPECT_EQ(DecodeYolov8(data.data(), 5, 3, 1, 0.25f).size(), 1u);
  EXPECT_EQ(DecodeYolov8(data.data(), 5, 3, 1, 0.05f).size(), 2u);
}

TEST(DecodeYolov8, PicksTheHighestScoringClassPerAnchor) {
  const int num_classes = 3;
  const int num_anchors = 1;
  std::vector<float> data(static_cast<size_t>(4 + num_classes) * num_anchors, 0.0f);
  data[0] = 50.0f;  // cx
  data[1] = 50.0f;  // cy
  data[2] = 20.0f;  // w
  data[3] = 20.0f;  // h
  data[4] = 0.30f;  // class 0
  data[5] = 0.80f;  // class 1
  data[6] = 0.45f;  // class 2

  const auto out = DecodeYolov8(data.data(), 4 + num_classes, num_anchors, num_classes, 0.25f);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].class_id, 1);
  EXPECT_NEAR(out[0].score, 0.80f, 1e-6);
}

TEST(DecodeYolov8, RejectsAChannelCountThatIsNotAV8Head) {
  // A YOLOv5-style tensor carries an extra objectness channel. Decoding it with
  // v8 strides yields plausible-looking nonsense, so it must fail loudly.
  std::vector<float> data(static_cast<size_t>(6) * 10, 0.0f);
  EXPECT_THROW(DecodeYolov8(data.data(), /*channels=*/6, 10, /*num_classes=*/1, 0.25f),
               std::invalid_argument);
  EXPECT_THROW(DecodeYolov8(nullptr, 5, 10, 1, 0.25f), std::invalid_argument);
  EXPECT_THROW(DecodeYolov8(data.data(), 5, 0, 1, 0.25f), std::invalid_argument);
}

TEST(DecodeYolov8, SkipsDegenerateBoxes) {
  const auto data = MakeTensor(1, 2,
                               {{0, 0.9f, cv::Rect2f(10, 10, 0, 20)},
                                {1, 0.9f, cv::Rect2f(30, 30, 20, 20)}});
  const auto out = DecodeYolov8(data.data(), 5, 2, 1, 0.25f);
  EXPECT_EQ(out.size(), 1u);
}

TEST(Nms, SuppressesOverlappingBoxesOfTheSameClass) {
  std::vector<RawDetection> input = {
      {0, 0.90f, cv::Rect2f(100, 100, 50, 50)},
      {0, 0.85f, cv::Rect2f(105, 102, 50, 50)},  // heavy overlap, lower score
      {0, 0.80f, cv::Rect2f(300, 300, 50, 50)},  // elsewhere
  };
  const auto kept = NonMaxSuppression(std::move(input), 0.45f, 64);
  ASSERT_EQ(kept.size(), 2u);
  EXPECT_NEAR(kept[0].score, 0.90f, 1e-6);
  EXPECT_NEAR(kept[1].score, 0.80f, 1e-6);
}

TEST(Nms, DoesNotSuppressAcrossClasses) {
  // A bowl overlapping a bottle is two objects. Class-agnostic NMS would delete
  // one of them, and the pipeline would silently lose an object it detected.
  std::vector<RawDetection> input = {
      {0, 0.90f, cv::Rect2f(100, 100, 50, 50)},
      {1, 0.85f, cv::Rect2f(101, 101, 50, 50)},
  };
  EXPECT_EQ(NonMaxSuppression(std::move(input), 0.45f, 64).size(), 2u);
}

TEST(Nms, RespectsTheDetectionCap) {
  std::vector<RawDetection> input;
  for (int i = 0; i < 100; ++i) {
    input.push_back({0, 0.9f - i * 0.001f, cv::Rect2f(i * 100.0f, 0, 50, 50)});
  }
  EXPECT_EQ(NonMaxSuppression(std::move(input), 0.45f, 10).size(), 10u);
}

TEST(Nms, IsDeterministicForTiedScores) {
  // Two identical-scoring boxes must resolve the same way every run, or two
  // runs on one recording produce different track ids downstream.
  const std::vector<RawDetection> input = {
      {0, 0.5f, cv::Rect2f(200, 200, 50, 50)},
      {0, 0.5f, cv::Rect2f(100, 100, 50, 50)},
      {1, 0.5f, cv::Rect2f(100, 100, 50, 50)},
  };
  const auto first = NonMaxSuppression(input, 0.45f, 64);
  for (int i = 0; i < 10; ++i) {
    const auto again = NonMaxSuppression(input, 0.45f, 64);
    ASSERT_EQ(again.size(), first.size());
    for (size_t j = 0; j < first.size(); ++j) {
      EXPECT_EQ(again[j].class_id, first[j].class_id);
      EXPECT_EQ(again[j].box, first[j].box);
    }
  }
}

TEST(Nms, EmptyInputIsEmptyOutput) {
  EXPECT_TRUE(NonMaxSuppression({}, 0.45f, 64).empty());
}

}  // namespace
}  // namespace the_eyes::detector
