// The depth evidence decides which view fusion believes, so it is checked on
// scenes that can be written down: a cube on a table seen by two cameras placed
// like the rig's, with the depth images rendered from the true geometry.
#include "the_eyes/runtime/view_evidence.hpp"

#include <cmath>

#include <gtest/gtest.h>

using namespace the_eyes;
using namespace the_eyes::runtime;

namespace {

constexpr int kW = 848, kH = 480;
constexpr double kCubeHalf = 0.025;

Eigen::Matrix3d TestK() {
  Eigen::Matrix3d K;
  K << 600, 0, 424, 0, 600, 240, 0, 0, 1;
  return K;
}

TriMesh Box(double hx, double hy, double hz) {
  TriMesh m;
  for (int i = 0; i < 8; ++i) {
    m.vertices.emplace_back((i & 1) ? hx : -hx, (i & 2) ? hy : -hy, (i & 4) ? hz : -hz);
  }
  const int quads[6][4] = {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 4, 5, 1},
                           {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}};
  for (const auto& q : quads) {
    m.triangles.push_back({q[0], q[1], q[2]});
    m.triangles.push_back({q[0], q[2], q[3]});
  }
  return m;
}

// A camera at `eye` looking at `target`, image y pointing down (OpenCV).
Eigen::Isometry3d LookAt(const Eigen::Vector3d& eye, const Eigen::Vector3d& target) {
  const Eigen::Vector3d z = (target - eye).normalized();
  const Eigen::Vector3d x = z.cross(Eigen::Vector3d::UnitZ()).normalized();
  const Eigen::Vector3d y = z.cross(x);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear().col(0) = x;
  T.linear().col(1) = y;
  T.linear().col(2) = z;
  T.translation() = eye;
  return T;
}

// The rig of 2026-10-07, rounded: both cameras 0.4 m up, in front of the table.
fusion::ExtrinsicsSet Rig() {
  fusion::ExtrinsicsSet ex;
  ex.world_frame = "table_origin";
  const std::pair<const char*, Eigen::Vector3d> cams[] = {
      {"cam_front", {-0.41, -0.37, 0.40}}, {"cam_over", {0.32, -0.43, 0.40}}};
  for (const auto& [id, eye] : cams) {
    CameraExtrinsics c;
    c.camera_id = id;
    c.serial = id;
    c.T_world_cam = LookAt(eye, Eigen::Vector3d(0.0, 0.0, 0.03));
    c.reprojection_rmse_px = 0.3;
    ex.cameras.emplace(id, c);
  }
  return ex;
}

Eigen::Isometry3d At(const Eigen::Vector3d& t) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = t;
  return p;
}

struct Thing {
  TriMesh mesh;
  Eigen::Isometry3d pose_w;
};

// What a depth camera would measure of these things, in millimetre units.
cv::Mat DepthOf(const std::vector<Thing>& things, const Eigen::Isometry3d& T_world_cam) {
  cv::Mat nearest(kH, kW, CV_32FC1, cv::Scalar(0.0f));
  for (const Thing& t : things) {
    const cv::Mat r = RenderDepth(t.mesh, T_world_cam.inverse() * t.pose_w, TestK(), kW, kH);
    for (int v = 0; v < kH; ++v) {
      for (int u = 0; u < kW; ++u) {
        const float z = r.at<float>(v, u);
        float& n = nearest.at<float>(v, u);
        if (z > 0.0f && (n == 0.0f || z < n)) n = z;
      }
    }
  }
  cv::Mat u16;
  nearest.convertTo(u16, CV_16UC1, 1000.0);
  return u16;
}

const Eigen::Vector3d kTruth(0.0, 0.0, kCubeHalf);

Thing Table() { return {Box(0.59, 0.375, 0.0115), At({0.0, 0.155, -0.0115})}; }
Thing Cube(const Eigen::Vector3d& centre = kTruth) { return {Box(kCubeHalf, kCubeHalf, kCubeHalf), At(centre)}; }

std::map<std::string, EvidenceView> Views(const fusion::ExtrinsicsSet& ex,
                                          const std::map<std::string, std::vector<Thing>>& scene) {
  std::map<std::string, EvidenceView> views;
  for (const auto& [id, things] : scene) {
    EvidenceView v;
    v.K = TestK();
    v.depth_u16 = DepthOf(things, ex.at(id).T_world_cam);
    v.depth_scale_m = 0.001;
    views.emplace(id, std::move(v));
  }
  return views;
}

PoseEstimate Estimate(const fusion::ExtrinsicsSet& ex, const std::string& cam,
                      const Eigen::Vector3d& centre_w, const std::string& cls = "red_cube") {
  PoseEstimate e;
  e.camera_id = cam;
  e.cls = cls;
  e.pose_cam = ex.at(cam).T_world_cam.inverse() * At(centre_w);
  e.score = 0.9;
  return e;
}

ViewEvidence Evidence(const fusion::ExtrinsicsSet& ex, bool enabled = true) {
  fusion::ViewEvidenceConfig cfg;
  cfg.enabled = enabled;
  return ViewEvidence(cfg, ex, {{"red_cube", Box(kCubeHalf, kCubeHalf, kCubeHalf)}});
}

// A point `metres` nearer `cam` than the true cube, along that camera's line of
// sight to it: where a pose lands when the depth it was fitted to belongs to
// something standing in front of the object.
Eigen::Vector3d TowardCamera(const fusion::ExtrinsicsSet& ex, const std::string& cam, double metres) {
  const Eigen::Vector3d eye = ex.at(cam).T_world_cam.translation();
  return kTruth + (eye - kTruth).normalized() * metres;
}

}  // namespace

TEST(ViewEvidence, ACorrectPoseIsSupportedByItsOwnViewAndNotSeenThrough) {
  const fusion::ExtrinsicsSet ex = Rig();
  const std::vector<Thing> world = {Table(), Cube()};
  const auto views = Views(ex, {{"cam_front", world}, {"cam_over", world}});
  std::vector<PoseEstimate> est = {Estimate(ex, "cam_front", kTruth), Estimate(ex, "cam_over", kTruth)};
  Evidence(ex).Attach(views, &est);
  for (const PoseEstimate& e : est) {
    EXPECT_TRUE(has_view_evidence(e)) << e.camera_id;
    EXPECT_GT(e.depth_support, 0.97) << e.camera_id;
    EXPECT_LT(e.seen_through, 0.02) << e.camera_id;
  }
}

TEST(ViewEvidence, APoseAtTheOccludersRangeIsSeenThroughByTheOtherCamera) {
  // The recorded failure: the half-hidden view puts the object 12 cm nearer
  // itself, in mid-air. Its own depth there is the table and the true object,
  // both beyond the claimed surface; and the other camera looks straight
  // through the claimed place at the table behind it.
  const fusion::ExtrinsicsSet ex = Rig();
  const std::vector<Thing> world = {Table(), Cube()};
  const auto views = Views(ex, {{"cam_front", world}, {"cam_over", world}});
  std::vector<PoseEstimate> est = {Estimate(ex, "cam_front", kTruth),
                                   Estimate(ex, "cam_over", TowardCamera(ex, "cam_over", 0.12))};
  Evidence(ex).Attach(views, &est);
  EXPECT_GT(est[0].depth_support, 0.97);
  EXPECT_LT(est[0].seen_through, 0.02);
  EXPECT_LT(est[1].depth_support, 0.10);
  EXPECT_GT(est[1].seen_through, 0.90);
  // Which is the comparison fusion makes.
  EXPECT_GT(est[0].depth_support * (1.0 - est[0].seen_through),
            est[1].depth_support * (1.0 - est[1].seen_through));
}

TEST(ViewEvidence, AnOccluderInFrontIsNotFreeSpace) {
  // cam_over cannot see the cube: a card stands between them. It measures the
  // card, NEARER than the cube, over the cube's whole silhouette -- which says
  // nothing about what is behind the card. cam_front's correct pose must not
  // be marked as seen through.
  const fusion::ExtrinsicsSet ex = Rig();
  const Thing cube = Cube();
  Thing card{Box(0.15, 0.002, 0.15), Eigen::Isometry3d::Identity()};
  const Eigen::Vector3d eye = ex.at("cam_over").T_world_cam.translation();
  const Eigen::Vector3d toward = (eye - kTruth).normalized();
  // Facing cam_over squarely, 20 cm from the cube.
  card.pose_w = LookAt(kTruth + toward * 0.20, eye);
  card.pose_w.linear() = card.pose_w.linear() *
                         Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto views = Views(ex, {{"cam_front", {Table(), cube}}, {"cam_over", {Table(), cube, card}}});

  std::vector<PoseEstimate> est = {Estimate(ex, "cam_front", kTruth)};
  Evidence(ex).Attach(views, &est);
  EXPECT_GT(est[0].depth_support, 0.97);
  EXPECT_LT(est[0].seen_through, 0.02) << "a surface in front is occlusion, not refutation";
}

TEST(ViewEvidence, NoDepthIsNoOpinion) {
  // The other camera has no depth at all where the object would be (the stereo
  // shadow beside an occluder, a blown-out surface). That refutes nothing, and
  // in the own view it supports nothing.
  const fusion::ExtrinsicsSet ex = Rig();
  const std::vector<Thing> world = {Table(), Cube()};
  auto views = Views(ex, {{"cam_front", world}, {"cam_over", world}});
  views["cam_over"].depth_u16.setTo(0);
  std::vector<PoseEstimate> est = {Estimate(ex, "cam_front", kTruth), Estimate(ex, "cam_over", kTruth)};
  Evidence(ex).Attach(views, &est);
  EXPECT_DOUBLE_EQ(est[0].seen_through, 0.0);
  EXPECT_GT(est[0].depth_support, 0.97);
  EXPECT_DOUBLE_EQ(est[1].depth_support, 0.0);
  EXPECT_LT(est[1].seen_through, 0.02);
}

TEST(ViewEvidence, OneCameraCannotContradictItself) {
  // With no second view present seen_through stays "not supplied", and an
  // estimate with half the evidence counts as having none: fusion falls back
  // to the score rather than trusting a veto nobody could have cast.
  const fusion::ExtrinsicsSet ex = Rig();
  const auto views = Views(ex, {{"cam_front", {Table(), Cube()}}});
  std::vector<PoseEstimate> est = {Estimate(ex, "cam_front", kTruth)};
  Evidence(ex).Attach(views, &est);
  EXPECT_GT(est[0].depth_support, 0.97);
  EXPECT_LT(est[0].seen_through, 0.0);
  EXPECT_FALSE(has_view_evidence(est[0]));
}

TEST(ViewEvidence, WhatItCannotMeasureItLeavesAlone) {
  const fusion::ExtrinsicsSet ex = Rig();
  const std::vector<Thing> world = {Table(), Cube()};
  const auto views = Views(ex, {{"cam_front", world}, {"cam_over", world}});

  // A class with no mesh, a camera with no frame, a camera nobody calibrated.
  std::vector<PoseEstimate> est = {Estimate(ex, "cam_front", kTruth, "unknown_object"),
                                   Estimate(ex, "cam_front", kTruth)};
  est[1].camera_id = "cam_side";
  // And a pose so far away its silhouette is a few pixels.
  est.push_back(Estimate(ex, "cam_front", kTruth + Eigen::Vector3d(4.0, 6.0, 0.0)));
  Evidence(ex).Attach(views, &est);
  for (const PoseEstimate& e : est) {
    EXPECT_LT(e.depth_support, 0.0) << e.cls << " " << e.camera_id;
    EXPECT_LT(e.seen_through, 0.0);
  }

  // Disabled, or default-constructed: nothing is measured.
  std::vector<PoseEstimate> off = {Estimate(ex, "cam_front", kTruth)};
  Evidence(ex, false).Attach(views, &off);
  EXPECT_FALSE(has_view_evidence(off[0]));
  ViewEvidence().Attach(views, &off);
  EXPECT_FALSE(has_view_evidence(off[0]));
  EXPECT_FALSE(ViewEvidence().enabled());
}

TEST(ViewEvidence, TheDepthScaleIsTheCorrectedOne) {
  // cam_front's depth reads about 1% short on the rig, and the pipeline hands
  // this module the corrected scale. With raw depth 8% short the correct pose
  // loses its support; with the scale that undoes it, it is back.
  const fusion::ExtrinsicsSet ex = Rig();
  const std::vector<Thing> world = {Table(), Cube()};
  auto views = Views(ex, {{"cam_front", world}, {"cam_over", world}});
  cv::Mat shortened;
  views["cam_front"].depth_u16.convertTo(shortened, CV_16UC1, 1.0 / 1.08);
  views["cam_front"].depth_u16 = shortened;

  std::vector<PoseEstimate> raw = {Estimate(ex, "cam_front", kTruth)};
  Evidence(ex).Attach(views, &raw);
  EXPECT_LT(raw[0].depth_support, 0.2);

  views["cam_front"].depth_scale_m = 0.001 * 1.08;
  std::vector<PoseEstimate> corrected = {Estimate(ex, "cam_front", kTruth)};
  Evidence(ex).Attach(views, &corrected);
  EXPECT_GT(corrected[0].depth_support, 0.95);
}
