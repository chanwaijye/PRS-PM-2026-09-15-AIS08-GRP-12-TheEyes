// The mock-mode acceptance test: synthetic observations from known ground
// truth, and the solver must recover what went in. This validates the solver
// independently of detector quality, lens distortion, and lighting -- none of
// which exist yet (AGENTS.md §2).
#include "the_eyes/calibration/solver.hpp"

#include <cmath>

#include <gtest/gtest.h>

#include "the_eyes/calibration/synthetic.hpp"

namespace the_eyes::calibration {
namespace {

BoardSpec Board() {
  BoardSpec s;
  s.squares_x = 8;
  s.squares_y = 6;
  s.square_length_m = 0.030;
  s.marker_length_m = 0.022;
  return s;
}

SyntheticCamera Camera(const std::string& id = "cam_front") {
  SyntheticCamera c;
  c.camera_id = id;
  c.image_size = cv::Size(1280, 720);
  c.fx = 900.0;
  c.fy = 900.0;
  c.cx = 640.0;
  c.cy = 360.0;
  return c;
}

std::vector<BoardObservation> SyntheticViews(const SyntheticCamera& camera,
                                             const BoardSpec& board, int count,
                                             const SyntheticNoise& noise = {}) {
  std::vector<BoardObservation> out;
  const auto poses = SampleBoardPoses(board, count);
  for (size_t i = 0; i < poses.size(); ++i) {
    BoardObservation observation =
        ProjectBoard(camera, board, poses[i], static_cast<uint64_t>(i) * 33'000'000ULL, noise);
    if (observation.size() >= 12) out.push_back(std::move(observation));
  }
  return out;
}

TEST(Solver, RecoversKnownIntrinsicsFromNoiselessViews) {
  const BoardSpec board = Board();
  const SyntheticCamera camera = Camera();
  const IntrinsicResult result = CalibrateIntrinsics(SyntheticViews(camera, board, 40), board);

  const Eigen::Matrix3d K = result.KEigen();
  EXPECT_NEAR(K(0, 0), camera.fx, 1.0);
  EXPECT_NEAR(K(1, 1), camera.fy, 1.0);
  EXPECT_NEAR(K(0, 2), camera.cx, 2.0);
  EXPECT_NEAR(K(1, 2), camera.cy, 2.0);
  // Noiseless input must land far inside the 0.5 px acceptance threshold.
  EXPECT_LT(result.reprojection_rmse_px, 0.05);
  EXPECT_GE(result.views_used, 12);
  EXPECT_EQ(result.per_view_rmse_px.size(), static_cast<size_t>(result.views_used));
}

TEST(Solver, RecoversIntrinsicsUnderRealisticCornerNoise) {
  const BoardSpec board = Board();
  const SyntheticCamera camera = Camera();
  SyntheticNoise noise;
  noise.pixel_sigma = 0.2;  // typical ChArUco corner localisation
  noise.seed = 99;

  const IntrinsicResult result =
      CalibrateIntrinsics(SyntheticViews(camera, board, 45, noise), board);
  const Eigen::Matrix3d K = result.KEigen();
  EXPECT_NEAR(K(0, 0), camera.fx, 15.0);
  EXPECT_NEAR(K(1, 1), camera.fy, 15.0);
  // The acceptance criterion itself: RMSE below 0.5 px.
  EXPECT_LT(result.reprojection_rmse_px, 0.5);
}

TEST(Solver, RecoversLensDistortion) {
  const BoardSpec board = Board();
  SyntheticCamera camera = Camera();
  camera.distortion = {-0.12, 0.05, 0.0, 0.0, 0.0};

  const IntrinsicResult result = CalibrateIntrinsics(SyntheticViews(camera, board, 45), board);
  EXPECT_NEAR(result.distortion.at<double>(0, 0), -0.12, 0.02);
  EXPECT_NEAR(result.distortion.at<double>(0, 1), 0.05, 0.05);
  EXPECT_LT(result.reprojection_rmse_px, 0.1);
}

TEST(Solver, RefusesTooFewViews) {
  // Intrinsics from too few views fit noise: the RMSE comes out low precisely
  // because the solution is under-constrained, which is the most misleading
  // failure this module can produce.
  const BoardSpec board = Board();
  const auto views = SyntheticViews(Camera(), board, 5);
  EXPECT_THROW(CalibrateIntrinsics(views, board, 12), std::runtime_error);
  EXPECT_NO_THROW(CalibrateIntrinsics(views, board, 3));
}

TEST(Solver, RejectsMixedImageSizes) {
  const BoardSpec board = Board();
  auto views = SyntheticViews(Camera(), board, 20);
  views[3].image_size = cv::Size(640, 480);
  EXPECT_THROW(CalibrateIntrinsics(views, board, 12), std::runtime_error);
}

TEST(Solver, RecoversAKnownBoardPose) {
  const BoardSpec board = Board();
  const SyntheticCamera camera = Camera();
  const IntrinsicResult intrinsics = CalibrateIntrinsics(SyntheticViews(camera, board, 40), board);

  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.linear() = Eigen::AngleAxisd(0.25, Eigen::Vector3d(0.3, 1.0, 0.2).normalized())
                       .toRotationMatrix();
  truth.translation() = Eigen::Vector3d(-0.09, -0.06, 0.55);

  const BoardObservation observation = ProjectBoard(camera, board, truth);
  const PoseResult pose = EstimateBoardPose(observation, intrinsics, board);

  EXPECT_LT((pose.T_cam_board.translation() - truth.translation()).norm(), 1e-3);
  const Eigen::AngleAxisd delta(pose.T_cam_board.linear().transpose() * truth.linear());
  EXPECT_LT(std::abs(delta.angle()), 0.01);
  EXPECT_LT(pose.reprojection_rmse_px, 0.1);
}

TEST(Solver, RefusesTooFewCornersForPnp) {
  const BoardSpec board = Board();
  const IntrinsicResult intrinsics = CalibrateIntrinsics(SyntheticViews(Camera(), board, 40), board);

  BoardObservation sparse;
  sparse.camera_id = "cam_front";
  sparse.image_size = cv::Size(1280, 720);
  sparse.corner_ids = {0, 1, 2};
  sparse.image_points = {{100, 100}, {150, 100}, {200, 100}};
  EXPECT_THROW(EstimateBoardPose(sparse, intrinsics, board), std::runtime_error);
}

TEST(Solver, WorldRegistrationInvertsTheBoardPose) {
  Eigen::Isometry3d T_cam_board = Eigen::Isometry3d::Identity();
  T_cam_board.linear() =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  T_cam_board.translation() = Eigen::Vector3d(0.02, -0.03, 0.62);

  // Board at the world origin: the camera pose is exactly the inverse.
  const Eigen::Isometry3d T_world_cam =
      RegisterToWorld(T_cam_board, Eigen::Isometry3d::Identity());
  EXPECT_TRUE((T_world_cam * T_cam_board).isApprox(Eigen::Isometry3d::Identity(), 1e-12));

  // Board offset from the world origin: the offset carries through.
  Eigen::Isometry3d T_world_board = Eigen::Isometry3d::Identity();
  T_world_board.translation() = Eigen::Vector3d(0.10, 0.20, 0.0);
  const Eigen::Isometry3d shifted = RegisterToWorld(T_cam_board, T_world_board);
  EXPECT_LT(((shifted * T_cam_board).translation() - T_world_board.translation()).norm(), 1e-12);
}

TEST(Solver, ReprojectionRmseIsZeroForTheTruePose) {
  const BoardSpec board = Board();
  const SyntheticCamera camera = Camera();
  const IntrinsicResult intrinsics = CalibrateIntrinsics(SyntheticViews(camera, board, 40), board);

  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.translation() = Eigen::Vector3d(-0.09, -0.06, 0.5);
  const BoardObservation observation = ProjectBoard(camera, board, truth);

  EXPECT_LT(ReprojectionRmse(observation, intrinsics, board, truth), 0.2);
  // A deliberately wrong pose must score much worse -- otherwise the metric
  // is not measuring anything.
  Eigen::Isometry3d wrong = truth;
  wrong.translation().x() += 0.05;
  EXPECT_GT(ReprojectionRmse(observation, intrinsics, board, wrong), 10.0);
}


// The 2026-09-20 failure, pinned: a single mislocalised corner is enough to
// fail the acceptance gate on a pose that is actually correct. Marker corner
// refinement was snapping one corner of 49 by ~4.6 px, which lifted the RMSE
// from 0.27 to 0.75 px against a 0.50 px gate while moving the recovered pose
// by 0.09 mm. The RMSE alone cannot tell that story; the pair can.
TEST(Solver, OneBadCornerInflatesRmseWhileThePoseSurvives) {
  const BoardSpec board = Board();
  const SyntheticCamera camera = Camera();
  const IntrinsicResult intrinsics = CalibrateIntrinsics(SyntheticViews(camera, board, 40), board);

  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.linear() =
      Eigen::AngleAxisd(0.30, Eigen::Vector3d(0.2, 1.0, 0.1).normalized()).toRotationMatrix();
  truth.translation() = Eigen::Vector3d(-0.08, -0.05, 0.58);

  const BoardObservation clean = ProjectBoard(camera, board, truth);
  ASSERT_GE(clean.size(), 30u);  // the synthetic board is 8x6 -> 7x5 = 35 corners
  const PoseResult before = EstimateBoardPose(clean, intrinsics, board);

  // Displace exactly one corner, the way a bad sub-pixel refinement does.
  BoardObservation nudged = clean;
  nudged.image_points[nudged.image_points.size() / 2].x += 4.6f;
  const PoseResult after = EstimateBoardPose(nudged, intrinsics, board);

  // The RMSE degrades sharply...
  EXPECT_GT(after.reprojection_rmse_px, 3.0 * before.reprojection_rmse_px);
  // ...the worst-corner figure names the cause, sitting far above the RMSE...
  EXPECT_GT(after.max_residual_px, 4.0 * after.reprojection_rmse_px);
  // ...and the pose barely moves, which is the whole point: the calibration
  // was never bad, one corner was.
  EXPECT_LT((after.T_cam_board.translation() - before.T_cam_board.translation()).norm(), 1e-3);
}

// On a clean fit the worst corner stays close to the RMSE, so the ratio above
// is diagnostic rather than a property of every fit.
TEST(Solver, WorstCornerTracksTheRmseOnACleanFit) {
  const BoardSpec board = Board();
  const SyntheticCamera camera = Camera();
  const IntrinsicResult intrinsics = CalibrateIntrinsics(SyntheticViews(camera, board, 40), board);

  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.translation() = Eigen::Vector3d(-0.07, -0.05, 0.52);
  const PoseResult pose = EstimateBoardPose(ProjectBoard(camera, board, truth), intrinsics, board);

  EXPECT_GE(pose.max_residual_px, pose.reprojection_rmse_px);
  EXPECT_LT(pose.max_residual_px, 4.0 * std::max(pose.reprojection_rmse_px, 1e-6));
}

}  // namespace
}  // namespace the_eyes::calibration
