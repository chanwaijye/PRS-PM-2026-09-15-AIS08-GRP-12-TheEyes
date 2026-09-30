// The occlusion tiers are read off this arithmetic, so it is checked against
// geometry that can be written down: a cube whose projection and depth are
// known in closed form.
#include "the_eyes/runtime/visibility.hpp"

#include <unistd.h>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

using namespace the_eyes::runtime;

namespace {

constexpr int kW = 848, kH = 480;

Eigen::Matrix3d TestK() {
  Eigen::Matrix3d K;
  K << 600, 0, 424, 0, 600, 240, 0, 0, 1;
  return K;
}

// An axis-aligned cube of half-side h, 12 triangles, mixed winding on purpose.
TriMesh Cube(double h) {
  TriMesh m;
  for (int i = 0; i < 8; ++i) {
    m.vertices.emplace_back((i & 1) ? h : -h, (i & 2) ? h : -h, (i & 4) ? h : -h);
  }
  const int quads[6][4] = {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 4, 5, 1},
                           {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}};
  for (const auto& q : quads) {
    m.triangles.push_back({q[0], q[1], q[2]});
    m.triangles.push_back({q[0], q[3], q[2]});  // opposite winding to the first
  }
  return m;
}

Eigen::Isometry3d At(double z, double yaw_deg = 0.0, double tilt_deg = 0.0) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(tilt_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()))
                   .toRotationMatrix();
  T.translation() = Eigen::Vector3d(0, 0, z);
  return T;
}

// Depth of the first hit of pixel (u, v)'s ray on the cube, by the slab method
// in the cube's own frame -- an independent route to the same number. 0 on a miss.
double RayBoxDepth(const Eigen::Isometry3d& T, double h, double u, double v) {
  const Eigen::Matrix3d K = TestK();
  const Eigen::Vector3d dir_cam = K.inverse() * Eigen::Vector3d(u, v, 1.0);  // z component 1
  const Eigen::Isometry3d Ti = T.inverse();
  const Eigen::Vector3d o = Ti.translation();
  const Eigen::Vector3d d = Ti.linear() * dir_cam;
  double t_near = -1e30, t_far = 1e30;
  for (int a = 0; a < 3; ++a) {
    const double t1 = (-h - o[a]) / d[a], t2 = (h - o[a]) / d[a];
    t_near = std::max(t_near, std::min(t1, t2));
    t_far = std::min(t_far, std::max(t1, t2));
  }
  if (t_near > t_far || t_near <= 0) return 0.0;
  return t_near;  // dir_cam has z = 1, so t is the optical-axis depth
}

cv::Mat ToDepthU16(const cv::Mat& rendered_m, double scale) {
  cv::Mat d(rendered_m.size(), CV_16UC1, cv::Scalar(0));
  for (int y = 0; y < d.rows; ++y)
    for (int x = 0; x < d.cols; ++x)
      d.at<uint16_t>(y, x) =
          static_cast<uint16_t>(std::lround(rendered_m.at<float>(y, x) / scale));
  return d;
}

std::string WriteTemp(const std::string& body) {
  char tmpl[] = "/tmp/the_eyes_vis_XXXXXX";
  const char* dir = mkdtemp(tmpl);
  if (dir == nullptr) throw std::runtime_error("mkdtemp failed");
  const std::string path = std::string(dir) + "/m.obj";
  std::ofstream(path) << body;
  return path;
}

}  // namespace

TEST(Visibility, FaceOnCubeProjectsToTheSquareItShould) {
  // Front face at z = 0.675 m, half-side 25 mm: +-22.22 px about the principal
  // point, so pixel centres 402..446 and 218..262 -- 45 x 45.
  const cv::Mat r = RenderDepth(Cube(0.025), At(0.700), TestK(), kW, kH);
  EXPECT_EQ(cv::countNonZero(r), 45 * 45);
  EXPECT_NEAR(r.at<float>(240, 424), 0.675, 1e-6);
  EXPECT_EQ(r.at<float>(240, 401), 0.0f);
  EXPECT_EQ(r.at<float>(240, 447), 0.0f);
}

TEST(Visibility, DepthIsPerspectiveCorrectOnATiltedFace) {
  // Tilted 30 deg, the visible faces are not parallel to the image, so
  // interpolating z linearly in the image would be wrong everywhere except at
  // the vertices. Compare against an exact ray cast over the interior.
  const Eigen::Isometry3d T = At(0.700, 20.0, 30.0);
  const cv::Mat r = RenderDepth(Cube(0.025), T, TestK(), kW, kH);
  int checked = 0;
  for (int v = 200; v <= 280; v += 4) {
    for (int u = 384; u <= 464; u += 4) {
      const double truth = RayBoxDepth(T, 0.025, u, v);
      const float got = r.at<float>(v, u);
      if (truth == 0.0 || got == 0.0f) {
        // Silhouette boundary: allow a disagreement only right at the edge.
        continue;
      }
      EXPECT_NEAR(got, truth, 1e-6) << "(" << u << ", " << v << ")";
      ++checked;
    }
  }
  EXPECT_GT(checked, 150);
}

TEST(Visibility, MatchingDepthIsFullyVisible) {
  const cv::Mat r = RenderDepth(Cube(0.025), At(0.700), TestK(), kW, kH);
  const VisibilityCounts c = MeasureVisibility(r, ToDepthU16(r, 0.001), 0.001, 0.020, 0);
  EXPECT_EQ(c.silhouette, 45 * 45);
  EXPECT_EQ(c.consistent, c.silhouette);
  EXPECT_DOUBLE_EQ(c.visible_fraction(), 1.0);
  EXPECT_DOUBLE_EQ(c.behind_fraction(), 0.0);
}

TEST(Visibility, AnOccluderInFrontIsCountedAndOnlyThere) {
  // A card 50 mm in front of the object over the left 20 of 45 columns.
  const cv::Mat r = RenderDepth(Cube(0.025), At(0.700), TestK(), kW, kH);
  cv::Mat d = ToDepthU16(r, 0.001);
  for (int y = 0; y < kH; ++y)
    for (int x = 402; x < 422; ++x)
      if (d.at<uint16_t>(y, x)) d.at<uint16_t>(y, x) -= 50;
  cv::Mat labels;
  const VisibilityCounts c = MeasureVisibility(r, d, 0.001, 0.020, 0, &labels);
  EXPECT_EQ(c.occluded, 20 * 45);
  EXPECT_EQ(c.consistent, 25 * 45);
  EXPECT_NEAR(c.visible_fraction(), 25.0 / 45.0, 1e-12);
  EXPECT_EQ(labels.at<uint8_t>(240, 410), static_cast<uint8_t>(VisLabel::kOccluded));
  EXPECT_EQ(labels.at<uint8_t>(240, 430), static_cast<uint8_t>(VisLabel::kConsistent));
  EXPECT_EQ(labels.at<uint8_t>(240, 300), static_cast<uint8_t>(VisLabel::kNone));
}

TEST(Visibility, NearerThanTheToleranceIsNotOcclusion) {
  // Depth noise and ground-truth error are millimetres; the tolerance absorbs them.
  const cv::Mat r = RenderDepth(Cube(0.025), At(0.700), TestK(), kW, kH);
  cv::Mat d = ToDepthU16(r, 0.001);
  cv::subtract(d, cv::Scalar(15), d, d > 0);
  const VisibilityCounts c = MeasureVisibility(r, d, 0.001, 0.020, 0);
  EXPECT_EQ(c.occluded, 0);
  EXPECT_DOUBLE_EQ(c.visible_fraction(), 1.0);
}

TEST(Visibility, MissingDepthIsUnknownNotOccluded) {
  const cv::Mat r = RenderDepth(Cube(0.025), At(0.700), TestK(), kW, kH);
  cv::Mat d = ToDepthU16(r, 0.001);
  d.rowRange(218, 228).setTo(0);  // 10 of 45 rows lost
  const VisibilityCounts c = MeasureVisibility(r, d, 0.001, 0.020, 0);
  EXPECT_EQ(c.invalid, 10 * 45);
  EXPECT_EQ(c.measured(), 35 * 45);
  EXPECT_DOUBLE_EQ(c.visible_fraction(), 1.0) << "a hole is excluded, not scored as hidden";
  EXPECT_NEAR(c.invalid_fraction(), 10.0 / 45.0, 1e-12);

  const VisibilityCounts none = MeasureVisibility(r, cv::Mat::zeros(r.size(), CV_16UC1), 0.001,
                                                  0.020, 0);
  EXPECT_TRUE(std::isnan(none.visible_fraction())) << "no depth at all is unknown, not 0%";
}

TEST(Visibility, FartherThanTheModelIsVisibleButReported) {
  // The model lands on the table beyond it: a ground-truth error, not an
  // occluder, so it must not lower visibility -- but it must be visible.
  const cv::Mat r = RenderDepth(Cube(0.025), At(0.700), TestK(), kW, kH);
  cv::Mat d = ToDepthU16(r, 0.001);
  cv::add(d, cv::Scalar(60), d, d > 0);
  const VisibilityCounts c = MeasureVisibility(r, d, 0.001, 0.020, 0);
  EXPECT_EQ(c.behind, c.silhouette);
  EXPECT_DOUBLE_EQ(c.visible_fraction(), 1.0);
  EXPECT_DOUBLE_EQ(c.behind_fraction(), 1.0);
}

TEST(Visibility, ErosionShrinksTheSilhouetteByTheRadius) {
  const cv::Mat r = RenderDepth(Cube(0.025), At(0.700), TestK(), kW, kH);
  const VisibilityCounts c = MeasureVisibility(r, ToDepthU16(r, 0.001), 0.001, 0.020, 2);
  EXPECT_EQ(c.silhouette, 41 * 41);
}

TEST(Visibility, RefusesMismatchedInputs) {
  const cv::Mat r = RenderDepth(Cube(0.025), At(0.700), TestK(), kW, kH);
  EXPECT_THROW(MeasureVisibility(r, cv::Mat::zeros(10, 10, CV_16UC1), 0.001, 0.02, 0),
               std::invalid_argument);
  EXPECT_THROW(MeasureVisibility(r, cv::Mat::zeros(r.size(), CV_32FC1), 0.001, 0.02, 0),
               std::invalid_argument);
  EXPECT_THROW(MeasureVisibility(r, cv::Mat::zeros(r.size(), CV_16UC1), 0.001, 0.0, 0),
               std::invalid_argument);
}

TEST(Visibility, ObjLoaderHandlesEveryFaceSpellingAndFansPolygons) {
  const std::string path = WriteTemp(
      "# comment\n"
      "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n"
      "vt 0 0\nvn 0 0 1\n"
      "f 1 2 3 4\n"               // quad -> 2 triangles
      "f 1/1 2/1 3/1\n"
      "f 1//1 3//1 4//1\n"
      "f -4/1/1 -3/1/1 -2/1/1\n");  // relative: vertices 1, 2, 3
  const TriMesh m = LoadObjMesh(path);
  ASSERT_EQ(m.vertices.size(), 4u);
  ASSERT_EQ(m.triangles.size(), 5u);
  EXPECT_EQ(m.triangles[1], (std::array<int, 3>{0, 2, 3}));
  EXPECT_EQ(m.triangles[4], (std::array<int, 3>{0, 1, 2}));
}

TEST(Visibility, ObjLoaderRefusesBadIndices) {
  EXPECT_THROW(LoadObjMesh(WriteTemp("v 0 0 0\nv 1 0 0\nv 1 1 0\nf 1 2 9\n")), std::runtime_error);
  EXPECT_THROW(LoadObjMesh(WriteTemp("v 0 0 0\nv 1 0 0\nv 1 1 0\nf 0 1 2\n")), std::runtime_error);
  EXPECT_THROW(LoadObjMesh(WriteTemp("v 0 0 0\n")), std::runtime_error);
  EXPECT_THROW(LoadObjMesh("/nonexistent/m.obj"), std::runtime_error);
}

TEST(Visibility, LoadsAndRendersTheShippedMeshes) {
  for (const char* name : {"red_cube", "bowl", "mustard_bottle"}) {
    const TriMesh m =
        LoadObjMesh(std::string(THE_EYES_REPO_ROOT) + "/assets/meshes/" + name + ".obj");
    EXPECT_GT(m.triangles.size(), 0u) << name;
    const cv::Mat r = RenderDepth(m, At(0.700, 10.0, 25.0), TestK(), kW, kH);
    EXPECT_GT(cv::countNonZero(r), 500) << name;
  }
}
