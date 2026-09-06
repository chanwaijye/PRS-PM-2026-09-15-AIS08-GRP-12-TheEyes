#include <gtest/gtest.h>

#include "the_eyes/runtime/backends.hpp"
#include "the_eyes/runtime/fixed_box_detector.hpp"

namespace {

using namespace the_eyes;
using namespace the_eyes::runtime;

detector::DetectorInput Input(const std::string& cam, uint64_t stamp) {
  detector::DetectorInput in;
  in.camera_id = cam;
  in.stamp_ns = stamp;
  in.rgb = cv::Mat(480, 848, CV_8UC3, cv::Scalar(0, 0, 0));
  in.depth = cv::Mat(480, 848, CV_16UC1, cv::Scalar(700));
  return in;
}

}  // namespace

TEST(FixedBoxDetector, ParsesASpecAndRejectsBadOnes) {
  const FixedBoxes b = ParseFixedBoxes("cam_front=red_cube,300,200,120,110;cam_over=red_cube,400,180,100,90");
  ASSERT_EQ(b.size(), 2u);
  EXPECT_EQ(b.at("cam_front")[0].cls, "red_cube");
  EXPECT_EQ(b.at("cam_front")[0].bbox, cv::Rect2i(300, 200, 120, 110));
  EXPECT_EQ(b.at("cam_over")[0].bbox, cv::Rect2i(400, 180, 100, 90));
  EXPECT_EQ(ParseFixedBoxes("cam_front=a,0,0,1,1;cam_front=b,5,5,2,2").at("cam_front").size(), 2u);

  EXPECT_THROW(ParseFixedBoxes(""), std::runtime_error);
  EXPECT_THROW(ParseFixedBoxes("cam_front"), std::runtime_error);
  EXPECT_THROW(ParseFixedBoxes("cam_front=red_cube,1,2,3"), std::runtime_error);
  EXPECT_THROW(ParseFixedBoxes("cam_front=red_cube,1,2,0,3"), std::runtime_error);
  EXPECT_THROW(ParseFixedBoxes("cam_front=red_cube,a,2,3,4"), std::runtime_error);
  EXPECT_THROW(ParseFixedBoxes("=red_cube,1,2,3,4"), std::runtime_error);
}

TEST(FixedBoxDetector, SameBoxEveryFrameWithAStableTrackId) {
  FixedBoxDetector det(ParseFixedBoxes("cam_front=red_cube,300,200,120,110"), detector::TrackingConfig{});
  uint32_t id = 0;
  for (int i = 0; i < 5; ++i) {
    const DetectionSet out = det.Detect(Input("cam_front", 1000 + static_cast<uint64_t>(i)));
    ASSERT_EQ(out.detections.size(), 1u);
    EXPECT_EQ(out.camera_id, "cam_front");
    EXPECT_EQ(out.stamp_ns, 1000u + static_cast<uint64_t>(i));
    EXPECT_EQ(out.detections[0].bbox, cv::Rect2i(300, 200, 120, 110));
    EXPECT_FLOAT_EQ(out.detections[0].score, 1.0f);
    EXPECT_NE(out.detections[0].track_id, 0u);
    if (i == 0) id = out.detections[0].track_id;
    EXPECT_EQ(out.detections[0].track_id, id) << "track id must not churn";
  }
  const DetectionSet none = det.Detect(Input("cam_over", 7));
  EXPECT_TRUE(none.detections.empty());
}

TEST(FixedBoxDetector, ClipsToTheFrameAndDropsBoxesOutsideIt) {
  FixedBoxDetector det(ParseFixedBoxes("cam_front=red_cube,800,400,120,110;cam_front=bowl,900,10,5,5"),
                       detector::TrackingConfig{});
  const DetectionSet out = det.Detect(Input("cam_front", 1));
  ASSERT_EQ(out.detections.size(), 1u);
  EXPECT_EQ(out.detections[0].bbox, cv::Rect2i(800, 400, 48, 80));
}

TEST(FixedBoxDetector, FactoryBuildsIt) {
  DetectorBackendConfig cfg;
  cfg.backend = "fixed";
  cfg.fixed_boxes = "cam_over=red_cube,10,10,50,50";
  const DetectorHandle h = MakeDetector(cfg, detector::DetectorConfig{}, "/nonexistent");
  ASSERT_TRUE(h.detector);
  EXPECT_EQ(h.onnx, nullptr);
  EXPECT_EQ(h.label, "fixed:cam_over=red_cube,10,10,50,50");
  EXPECT_EQ(h.detector->name(), "fixed:boxes");
  cfg.fixed_boxes = "";
  EXPECT_THROW(MakeDetector(cfg, detector::DetectorConfig{}, "/nonexistent"), std::runtime_error);
}
