// The depth correction is one number per camera that moves every pose, so the
// fit and the decision to apply it are both pinned on synthetic tables whose
// error is known.
#include "the_eyes/runtime/depth_correction.hpp"

#include <unistd.h>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

using namespace the_eyes;
using namespace the_eyes::runtime;

namespace {

std::string WriteTemp(const std::string& body) {
  char tmpl[] = "/tmp/the_eyes_depth_XXXXXX";
  const char* dir = mkdtemp(tmpl);
  if (dir == nullptr) throw std::runtime_error("mkdtemp failed");
  const std::string path = std::string(dir) + "/depth_correction.yaml";
  std::ofstream(path) << body;
  return path;
}

// A camera at `c` looking at the world origin, image y down.
Eigen::Isometry3d LookAtOrigin(const Eigen::Vector3d& c) {
  const Eigen::Vector3d z = (-c).normalized();
  const Eigen::Vector3d x = z.cross(Eigen::Vector3d::UnitZ()).normalized();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear().col(0) = x;
  T.linear().col(1) = z.cross(x);
  T.linear().col(2) = z;
  T.translation() = c;
  return T;
}

// A frame whose depth is the table plane z = 0 seen from T_world_cam, with each
// true depth passed through `distort` before being stored.
template <typename F>
camera::CameraFrame TableFrame(const Eigen::Isometry3d& T_world_cam, F distort) {
  camera::CameraFrame f;
  f.camera_id = "cam";
  f.serial = "S";
  f.depth_scale_m = 0.0001;  // fine units, so quantisation is not the thing measured
  f.K << 610, 0, 424, 0, 610, 240, 0, 0, 1;
  f.depth = cv::Mat(480, 848, CV_16UC1, cv::Scalar(0));
  const Eigen::Matrix3d Kinv = f.K.inverse();
  for (int v = 0; v < 480; ++v) {
    for (int u = 0; u < 848; ++u) {
      const Eigen::Vector3d dir = T_world_cam.linear() * (Kinv * Eigen::Vector3d(u, v, 1.0));
      if (dir.z() >= -1e-9) continue;
      const double z = -T_world_cam.translation().z() / dir.z();
      const double stored = distort(z, u, v);
      if (stored > 0.0 && stored < 6.5) {
        f.depth.at<uint16_t>(v, u) = static_cast<uint16_t>(std::lround(stored / f.depth_scale_m));
      }
    }
  }
  return f;
}

const Eigen::Isometry3d kCam = LookAtOrigin({-0.41, -0.37, 0.40});

}  // namespace

TEST(DepthCorrection, AMissingFileIsTheIdentity) {
  const DepthCorrection c = LoadDepthCorrection("/nonexistent/depth_correction.yaml");
  EXPECT_DOUBLE_EQ(c.For("cam_front"), 1.0);
  EXPECT_EQ(c.Describe(), "none");
}

TEST(DepthCorrection, LoadsScalesAndRefusesNonsense) {
  const DepthCorrection c = LoadDepthCorrection(
      WriteTemp("cameras:\n  cam_front:\n    scale: 1.0102\n  cam_over:\n    scale: 1.0\n"));
  EXPECT_DOUBLE_EQ(c.For("cam_front"), 1.0102);
  EXPECT_DOUBLE_EQ(c.For("cam_over"), 1.0);
  EXPECT_DOUBLE_EQ(c.For("cam_never_mentioned"), 1.0);
  EXPECT_EQ(c.Describe(), "cam_front x1.0102, cam_over x1.0000");

  // 15% is not a calibration error, it is a different unit or a broken sensor.
  EXPECT_THROW(LoadDepthCorrection(WriteTemp("cameras:\n  cam_front:\n    scale: 1.15\n")),
               std::runtime_error);
  EXPECT_THROW(LoadDepthCorrection(WriteTemp("cameras:\n  cam_front:\n    gain: 2\n")),
               std::runtime_error);
  EXPECT_THROW(LoadDepthCorrection(WriteTemp("scales: [1, 2]\n")), std::runtime_error);
}

TEST(DepthCorrection, ASensorReadingOnePercentShortIsRecovered) {
  // The cam_front case: every depth 1.0% short of the truth.
  const camera::CameraFrame f = TableFrame(kCam, [](double z, int, int) { return z / 1.0102; });
  PlaneSamples s;
  CollectTablePlaneSamples(f, kCam, 0.230, 0.185, 0.05, 3, &s);
  ASSERT_GT(s.z_measured.size(), 5000u);
  EXPECT_NEAR(FitDepthScale(s), 1.0102, 2e-4);
  EXPECT_TRUE(FitIsWorthApplying(s, FitDepthScale(s)));
  for (double e : BinnedDepthError(s, 1.0)) EXPECT_LT(e, -0.003) << "short at every range";
  for (double e : BinnedDepthError(s, FitDepthScale(s))) EXPECT_NEAR(e, 0.0, 3e-4);
}

TEST(DepthCorrection, OnlyTheTableInsideTheRegionIsUsed) {
  // Something standing on the desk (80 mm nearer over a block of pixels) must
  // not enter the fit, and neither must table outside the region.
  const camera::CameraFrame clean = TableFrame(kCam, [](double z, int, int) { return z; });
  const camera::CameraFrame cluttered = TableFrame(kCam, [](double z, int u, int v) {
    return (u > 380 && u < 470 && v > 200 && v < 280) ? z - 0.080 : z;
  });
  PlaneSamples a, b, wide;
  CollectTablePlaneSamples(clean, kCam, 0.230, 0.185, 0.05, 3, &a);
  CollectTablePlaneSamples(cluttered, kCam, 0.230, 0.185, 0.05, 3, &b);
  CollectTablePlaneSamples(clean, kCam, 0.460, 0.370, 0.05, 3, &wide);
  EXPECT_LT(b.z_measured.size(), a.z_measured.size()) << "the object's pixels are dropped";
  EXPECT_NEAR(FitDepthScale(b), 1.0, 1e-4) << "and what is left is still the table";
  EXPECT_GT(wide.z_measured.size(), a.z_measured.size());
}

TEST(DepthCorrection, AnErrorThatIsNotAScaleIsNotCorrected) {
  // The cam_over case: about right near the camera, long at the far edge. A
  // single multiplier can only trade the far error for a near one.
  const camera::CameraFrame f =
      TableFrame(kCam, [](double z, int, int) { return z + 0.022 * std::max(0.0, z - 0.50); });
  PlaneSamples s;
  CollectTablePlaneSamples(f, kCam, 0.230, 0.185, 0.05, 3, &s);
  const double fitted = FitDepthScale(s);
  EXPECT_LT(fitted, 1.0);
  EXPECT_FALSE(FitIsWorthApplying(s, fitted))
      << "fitted " << fitted << ": a scale here would be a number, not a correction";
}

TEST(DepthCorrection, RefusesEmptyOrMismatchedInput) {
  PlaneSamples empty;
  EXPECT_THROW(FitDepthScale(empty), std::invalid_argument);
  PlaneSamples bad;
  bad.z_measured = {0.5, 0.6};
  bad.z_plane = {0.5};
  EXPECT_THROW(FitDepthScale(bad), std::invalid_argument);
  EXPECT_THROW(BinnedDepthError(empty, 1.0), std::invalid_argument);
  camera::CameraFrame no_depth;
  EXPECT_THROW(CollectTablePlaneSamples(no_depth, kCam, 0.2, 0.2, 0.05, 3, &empty),
               std::invalid_argument);
}
