#include "the_eyes/camera/depth_noise.hpp"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

namespace the_eyes::camera {
namespace {

Eigen::Matrix3d K() {
  Eigen::Matrix3d k = Eigen::Matrix3d::Identity();
  k << 600.0, 0.0, 424.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0;
  return k;
}

// A synthetic flat wall at `distance`, optionally tilted and noisy.
cv::Mat WallDepth(double distance_m, double depth_scale_m = 0.001, double tilt = 0.0,
                  double sigma_m = 0.0, uint32_t seed = 7, double invalid_fraction = 0.0) {
  cv::Mat depth(480, 848, CV_16UC1, cv::Scalar(0));
  const Eigen::Matrix3d k = K();
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, sigma_m);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  for (int y = 0; y < depth.rows; ++y) {
    for (int x = 0; x < depth.cols; ++x) {
      if (invalid_fraction > 0.0 && uniform(rng) < invalid_fraction) continue;  // stays 0
      // Plane z = distance + tilt * (x - cx) / fx * z, solved for z.
      const double u = (x - k(0, 2)) / k(0, 0);
      double z = distance_m / (1.0 - tilt * u);
      if (sigma_m > 0.0) z += noise(rng);
      if (z <= 0.0) continue;
      depth.at<uint16_t>(y, x) = static_cast<uint16_t>(std::lround(z / depth_scale_m));
    }
  }
  return depth;
}

TEST(PlaneFit, RecoversAKnownPlaneExactly) {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      points.emplace_back(i * 0.01, j * 0.01, 0.75);
    }
  }
  const PlaneFit fit = FitPlane(points);
  ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(std::abs(fit.normal.z()), 1.0, 1e-9);
  EXPECT_NEAR(fit.rms_error_m, 0.0, 1e-9);
  EXPECT_EQ(fit.points_used, 400);
  // Normal is oriented toward the camera so distances read positive.
  EXPECT_LT(fit.normal.z(), 0.0);
}

TEST(PlaneFit, RmsMatchesInjectedNoise) {
  // The metric must actually measure what it claims. Inject a known sigma and
  // check the reported RMS recovers it.
  std::mt19937 rng(11);
  std::normal_distribution<double> noise(0.0, 0.003);  // 3 mm
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 60; ++i) {
    for (int j = 0; j < 60; ++j) {
      points.emplace_back(i * 0.005, j * 0.005, 0.8 + noise(rng));
    }
  }
  EXPECT_NEAR(FitPlane(points).rms_error_m, 0.003, 0.0004);
}

TEST(PlaneFit, IsUnbiasedOnATiltedPlane) {
  // Total least squares minimises PERPENDICULAR distance. A z = ax+by+c fit
  // minimises vertical distance and inflates the RMS on a tilted plane -- and a
  // real wall is never perfectly fronto-parallel.
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 40; ++i) {
    for (int j = 0; j < 40; ++j) {
      const double x = i * 0.01;
      const double y = j * 0.01;
      points.emplace_back(x, y, 0.7 + 0.5 * x + 0.3 * y);  // steeply tilted
    }
  }
  const PlaneFit fit = FitPlane(points);
  ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(fit.rms_error_m, 0.0, 1e-9) << "a noiseless tilted plane must fit exactly";
}

TEST(PlaneFit, RefusesTooFewPoints) {
  // Three points fit any plane with zero residual, and that zero would be
  // reported as a perfect measurement.
  std::vector<Eigen::Vector3d> points = {{0, 0, 1}, {1, 0, 1}, {0, 1, 1}};
  EXPECT_FALSE(FitPlane(points).valid);
  EXPECT_FALSE(FitPlane({}).valid);
}

TEST(DepthToPoints, BackProjectsAWallToTheRightDistance) {
  const cv::Mat depth = WallDepth(0.75);
  const auto points = DepthToPoints(depth, K(), 0.001, 0.5);
  ASSERT_FALSE(points.empty());
  for (const Eigen::Vector3d& p : points) EXPECT_NEAR(p.z(), 0.75, 0.002);

  const PlaneFit fit = FitPlane(points);
  ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(std::abs(fit.offset_m), 0.75, 0.002);
}

TEST(DepthToPoints, SkipsZeroDepthPixels) {
  // Zero means "no depth here", not "a point at the camera origin". Including
  // those would drag the plane fit toward the sensor.
  const cv::Mat depth = WallDepth(0.75, 0.001, 0.0, 0.0, 7, /*invalid_fraction=*/0.4);
  const auto points = DepthToPoints(depth, K(), 0.001, 0.5);
  ASSERT_FALSE(points.empty());
  for (const Eigen::Vector3d& p : points) EXPECT_GT(p.z(), 0.1);

  const PlaneFit fit = FitPlane(points);
  EXPECT_NEAR(std::abs(fit.offset_m), 0.75, 0.002) << "invalid pixels leaked into the fit";
}

TEST(DepthToPoints, HonoursTheRoiFraction) {
  const cv::Mat depth = WallDepth(0.75);
  const auto full = DepthToPoints(depth, K(), 0.001, 1.0);
  const auto half = DepthToPoints(depth, K(), 0.001, 0.5);
  EXPECT_GT(full.size(), half.size());
  EXPECT_NEAR(static_cast<double>(half.size()) / static_cast<double>(full.size()), 0.25, 0.02);
}

TEST(DepthToPoints, RejectsMalformedInput) {
  EXPECT_TRUE(DepthToPoints(cv::Mat(), K(), 0.001, 0.5).empty());
  const cv::Mat wrong_type(10, 10, CV_8UC1, cv::Scalar(1));
  EXPECT_TRUE(DepthToPoints(wrong_type, K(), 0.001, 0.5).empty());
  const cv::Mat depth = WallDepth(0.75);
  EXPECT_TRUE(DepthToPoints(depth, K(), 0.0, 0.5).empty());
  EXPECT_TRUE(DepthToPoints(depth, K(), 0.001, 0.0).empty());
}

TEST(DepthNoise, EndToEndOnASyntheticNoisyWall) {
  // The whole measurement path: synthetic wall with known noise -> back-project
  // -> plane fit -> RMS. This is what makes the real report trustworthy.
  const double sigma = 0.004;
  const cv::Mat depth = WallDepth(0.6, 0.001, /*tilt=*/0.05, sigma);
  const auto points = DepthToPoints(depth, K(), 0.001, 0.5);
  const PlaneFit fit = FitPlane(points);

  ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(fit.rms_error_m, sigma, 0.001);
  EXPECT_GT(fit.points_used, 40000);
}

TEST(DepthNoiseMeasurement, PlanarFlagSeparatesAWallFromARoom) {
  // Regression from a live sensor check. The plane's perpendicular offset is a
  // valid distance ONLY when the scene really is a plane facing the camera. Fit
  // through a room spanning 0.5-3 m and the plane can come out nearly edge-on,
  // giving a tiny offset while every point is a metre away -- which reads as a
  // broken sensor when nothing is wrong.
  DepthNoiseMeasurement wall;
  wall.median_depth_m = 0.701;
  wall.measured_distance_m = 0.701;   // offset agrees with median
  wall.rms_error_m = 0.0029;          // 0.4% of distance
  EXPECT_TRUE(wall.planar());

  DepthNoiseMeasurement room;
  room.median_depth_m = 1.017;
  room.measured_distance_m = 0.170;   // offset wildly disagrees
  room.rms_error_m = 0.146;           // 14% of distance
  EXPECT_FALSE(room.planar());

  // A flat wall measured at an angle still counts, so long as offset and median
  // agree and the residual is small.
  DepthNoiseMeasurement tilted;
  tilted.median_depth_m = 0.80;
  tilted.measured_distance_m = 0.76;  // 5% disagreement, inside tolerance
  tilted.rms_error_m = 0.003;
  EXPECT_TRUE(tilted.planar());

  DepthNoiseMeasurement empty;
  EXPECT_FALSE(empty.planar()) << "no depth is not a plane";
}

TEST(DepthNoiseReport, FormatsMeasurements) {
  DepthNoiseReport report;
  report.camera_id = "cam_front";
  report.serial = "261722073945";
  report.measurements.push_back({0.4, 0.402, 0.401, 0.0012, 0.98, 30, 50000});
  const std::string text = report.Format();
  EXPECT_NE(text.find("cam_front"), std::string::npos);
  EXPECT_NE(text.find("261722073945"), std::string::npos);
  EXPECT_NE(text.find("RMS"), std::string::npos);
}

}  // namespace
}  // namespace the_eyes::camera
