// The normalisation decides what the detector is shown, so its three promises
// are pinned: it reaches the target, it never darkens, and it stops at the cap.
#include "the_eyes/runtime/brightness.hpp"

#include <unistd.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "the_eyes/runtime/runtime_config.hpp"

using namespace the_eyes::runtime;

namespace {

cv::Mat Flat(int r, int g, int b) { return cv::Mat(48, 64, CV_8UC3, cv::Scalar(r, g, b)); }

BrightnessConfig On(double target = 100.0, double max_gain = 4.0) {
  BrightnessConfig c;
  c.enabled = true;
  c.target_mean = target;
  c.max_gain = max_gain;
  return c;
}

std::string ShippedRuntimeYamlPlus(const std::string& extra) {
  std::ifstream in(std::string(THE_EYES_CONFIG_DIR) + "/runtime.yaml");
  std::stringstream ss;
  ss << in.rdbuf();
  std::string body = ss.str();
  // Drop any brightness section the shipped file carries, so each case below
  // states its own.
  const auto at = body.find("\nbrightness:");
  if (at != std::string::npos) body = body.substr(0, at + 1);
  char tmpl[] = "/tmp/the_eyes_bright_XXXXXX";
  const char* dir = mkdtemp(tmpl);
  if (dir == nullptr) throw std::runtime_error("mkdtemp failed");
  const std::string path = std::string(dir) + "/runtime.yaml";
  std::ofstream(path) << body << extra;
  return path;
}

}  // namespace

TEST(Brightness, MeanLumaWeighsTheChannelsAsLuma) {
  EXPECT_NEAR(MeanLuma(Flat(80, 80, 80)), 80.0, 1e-9);
  // RGB order: a pure green frame is far brighter to the eye than a pure blue.
  EXPECT_NEAR(MeanLuma(Flat(0, 200, 0)), 0.587 * 200, 1e-9);
  EXPECT_NEAR(MeanLuma(Flat(0, 0, 200)), 0.114 * 200, 1e-9);
  EXPECT_DOUBLE_EQ(MeanLuma(cv::Mat()), 0.0);
  EXPECT_THROW(MeanLuma(cv::Mat(4, 4, CV_8UC1)), std::invalid_argument);
}

TEST(Brightness, GainReachesTheTargetAndNoFurther) {
  // The night frames of 2026-10-05: cam_front at 48 and cam_over at 23.
  EXPECT_NEAR(BrightnessGain(Flat(48, 48, 48), On()), 100.0 / 48.0, 1e-9);
  EXPECT_NEAR(BrightnessGain(Flat(23, 23, 23), On()), 4.0, 1e-9) << "23 wants x4.3; capped";
  // A frame already at or past the target is left alone: it never darkens.
  EXPECT_DOUBLE_EQ(BrightnessGain(Flat(100, 100, 100), On()), 1.0);
  EXPECT_DOUBLE_EQ(BrightnessGain(Flat(160, 160, 160), On()), 1.0);
  // Nothing to scale.
  EXPECT_DOUBLE_EQ(BrightnessGain(Flat(0, 0, 0), On()), 1.0);
  EXPECT_DOUBLE_EQ(BrightnessGain(cv::Mat(), On()), 1.0);
}

TEST(Brightness, OffMeansOff) {
  BrightnessConfig off;  // the default
  EXPECT_FALSE(off.enabled);
  EXPECT_DOUBLE_EQ(BrightnessGain(Flat(10, 10, 10), off), 1.0);
}

TEST(Brightness, BrightenScalesSaturatesAndNeverTouchesItsInput) {
  cv::Mat in = Flat(40, 100, 200);
  const cv::Mat before = in.clone();
  const cv::Mat out = Brighten(in, 2.0);
  EXPECT_EQ(out.at<cv::Vec3b>(5, 5), cv::Vec3b(80, 200, 255)) << "200 x 2 saturates at 255";
  EXPECT_EQ(cv::norm(in, before, cv::NORM_INF), 0.0) << "the recorder may still hold this buffer";
  EXPECT_NE(out.data, in.data);
  // After brightening, the frame's mean is where the gain said it would be
  // (no channel saturates here).
  const cv::Mat dim = Flat(20, 30, 25);
  const double g = BrightnessGain(dim, On());
  EXPECT_NEAR(MeanLuma(Brighten(dim, g)), std::min(100.0, MeanLuma(dim) * 4.0), 1.0);
  // Gain 1 is a no-op that does not even copy.
  EXPECT_EQ(Brighten(in, 1.0).data, in.data);
}

TEST(Brightness, ConfigSectionIsOptionalButStrictWhenPresent) {
  // Absent: configs written before the section existed still load, as "off".
  const RuntimeConfig none = LoadRuntimeConfig(ShippedRuntimeYamlPlus(""));
  EXPECT_FALSE(none.brightness.enabled);

  const RuntimeConfig on = LoadRuntimeConfig(ShippedRuntimeYamlPlus(
      "brightness:\n  enabled: true\n  target_mean: 110\n  max_gain: 3.0\n  apply_to_pose: true\n"));
  EXPECT_TRUE(on.brightness.enabled);
  EXPECT_DOUBLE_EQ(on.brightness.target_mean, 110.0);
  EXPECT_DOUBLE_EQ(on.brightness.max_gain, 3.0);
  EXPECT_TRUE(on.brightness.apply_to_pose);

  EXPECT_THROW(LoadRuntimeConfig(ShippedRuntimeYamlPlus(
                   "brightness:\n  enabled: true\n  target_mean: 110\n")),
               std::runtime_error);
  EXPECT_THROW(LoadRuntimeConfig(ShippedRuntimeYamlPlus(
                   "brightness:\n  enabled: true\n  target_mean: 400\n  max_gain: 3.0\n"
                   "  apply_to_pose: false\n")),
               std::runtime_error);
  EXPECT_THROW(LoadRuntimeConfig(ShippedRuntimeYamlPlus(
                   "brightness:\n  enabled: true\n  target_mean: 100\n  max_gain: 0.5\n"
                   "  apply_to_pose: false\n")),
               std::runtime_error);
}
