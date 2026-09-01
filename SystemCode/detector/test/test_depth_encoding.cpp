// The sentinel encoding is the crux decision of the RGB-D rework, so it gets
// tested in isolation from any model, image, or inference runtime.
#include "the_eyes/detector/depth_encoding.hpp"

#include <gtest/gtest.h>

namespace the_eyes::detector {
namespace {

constexpr double kScale = 0.001;  // D435i: 1 mm per unit

DepthEncoding Cfg() { return DepthEncoding{}; }  // 0.20-1.50 m, floor 0.1, sentinel 0.0

TEST(DepthEncoding, RawZeroBecomesTheSentinelNotZeroMetres) {
  // The whole point. RealSense reports 0 for stereo holes; read as a distance it
  // means "object at zero range", which is the most confident possible claim
  // about a pixel carrying no information.
  const DepthEncoding e = Cfg();
  EXPECT_FLOAT_EQ(e.Encode(0, kScale), e.sentinel);
  EXPECT_TRUE(e.IsInvalid(e.Encode(0, kScale)));
  EXPECT_LT(e.Decode(e.Encode(0, kScale)), 0.0);
}

TEST(DepthEncoding, SentinelIsUnreachableByAnyValidMeasurement) {
  // If a valid depth could encode to the sentinel, "no data" and "very close"
  // would be the same input and the encoding would be pointless.
  const DepthEncoding e = Cfg();
  for (uint16_t raw = 1; raw < 5000; raw += 7) {
    const float v = e.Encode(raw, kScale);
    EXPECT_GE(v, e.valid_floor) << "raw=" << raw;
    EXPECT_FALSE(e.IsInvalid(v)) << "raw=" << raw;
  }
}

TEST(DepthEncoding, RangeEndpointsMapToFloorAndOne) {
  const DepthEncoding e = Cfg();
  EXPECT_NEAR(e.Encode(200, kScale), e.valid_floor, 1e-6);   // 0.20 m
  EXPECT_NEAR(e.Encode(1500, kScale), 1.0f, 1e-6);           // 1.50 m
  EXPECT_NEAR(e.Encode(850, kScale), 0.55f, 1e-3);           // midpoint
}

TEST(DepthEncoding, OutOfRangeIsClippedNotDiscarded) {
  // A reading outside the working range is a real observation that saturates.
  // Turning it into a hole would fabricate missing data the sensor never
  // reported, and holes are exactly what the sentinel is reserved for.
  const DepthEncoding e = Cfg();
  EXPECT_NEAR(e.Encode(50, kScale), e.valid_floor, 1e-6);    // 0.05 m, too close
  EXPECT_NEAR(e.Encode(4000, kScale), 1.0f, 1e-6);           // 4.0 m, too far
  EXPECT_FALSE(e.IsInvalid(e.Encode(50, kScale))) << "clipped, not invalid";
  EXPECT_FALSE(e.IsInvalid(e.Encode(4000, kScale)));
}

TEST(DepthEncoding, RoundTripsWithinTheWorkingRange) {
  const DepthEncoding e = Cfg();
  for (double metres : {0.20, 0.35, 0.60, 0.90, 1.20, 1.50}) {
    const uint16_t raw = static_cast<uint16_t>(metres / kScale);
    EXPECT_NEAR(e.Decode(e.Encode(raw, kScale)), metres, 1e-3) << metres << " m";
  }
}

TEST(DepthEncoding, IsMonotonicInDistance) {
  // The network can only learn depth ordering if the encoding preserves it.
  const DepthEncoding e = Cfg();
  float previous = -1.0f;
  for (uint16_t raw = 200; raw <= 1500; raw += 10) {
    const float v = e.Encode(raw, kScale);
    EXPECT_GT(v, previous) << "raw=" << raw;
    previous = v;
  }
}

TEST(DepthEncoding, HonoursTheDeviceDepthScale) {
  // Encoding in raw units rather than metres would silently break on a camera
  // with a different depth_scale. Two devices, same physical distance.
  const DepthEncoding e = Cfg();
  const float a = e.Encode(1000, 0.001);      // 1.0 m at 1 mm/unit
  const float b = e.Encode(10000, 0.0001);    // 1.0 m at 0.1 mm/unit
  EXPECT_NEAR(a, b, 1e-6);
  // A zero or negative scale is a broken device report, not a distance.
  EXPECT_TRUE(e.IsInvalid(e.Encode(1000, 0.0)));
}

TEST(DepthEncoding, LetterboxPadDefaultsToTheTrainingValueNotTheSentinel) {
  // Found by RGB-D parity testing. ultralytics pads EVERY channel with 114 --
  // a 4-channel array letterboxes to [114 114 114 114] -- so a model trained
  // through it has learned the border as a surface at ~0.70 m, not as missing
  // data. Padding with the semantically correct sentinel at inference presents
  // a border the network never saw, and parity failed on 1 of 6 frames until
  // the two were matched.
  const DepthEncoding e = Cfg();
  EXPECT_EQ(e.letterbox_pad_u8, 114) << "must match training, not semantics";

  // 114/255 decodes to roughly 0.70 m through this encoding.
  const double decoded = e.Decode(114.0f / 255.0f);
  EXPECT_NEAR(decoded, 0.70, 0.02);
  EXPECT_FALSE(e.IsInvalid(114.0f / 255.0f)) << "the pad reads as a real surface";
}

TEST(DepthEncoding, RejectsAConfigurationThatDefeatsTheSentinel) {
  DepthEncoding bad = Cfg();
  bad.sentinel = 0.1f;   // equal to valid_floor
  EXPECT_THROW(bad.Validate(), std::invalid_argument);

  bad = Cfg();
  bad.sentinel = 0.5f;   // inside the valid band
  EXPECT_THROW(bad.Validate(), std::invalid_argument);

  bad = Cfg();
  bad.max_m = bad.min_m;
  EXPECT_THROW(bad.Validate(), std::invalid_argument);

  bad = Cfg();
  bad.valid_floor = 1.0f;
  EXPECT_THROW(bad.Validate(), std::invalid_argument);

  EXPECT_NO_THROW(Cfg().Validate());
}

}  // namespace
}  // namespace the_eyes::detector
