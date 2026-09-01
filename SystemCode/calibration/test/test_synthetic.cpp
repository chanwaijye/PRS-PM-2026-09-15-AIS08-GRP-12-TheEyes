#include "the_eyes/calibration/synthetic.hpp"

#include <gtest/gtest.h>

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

SyntheticCamera Camera() {
  SyntheticCamera c;
  c.camera_id = "cam_front";
  return c;
}

// Board centred in front of the camera at `distance`, facing it.
Eigen::Isometry3d FacingPose(const BoardSpec& board, double distance) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(-board.corners_x() * board.square_length_m / 2.0,
                                       -board.corners_y() * board.square_length_m / 2.0,
                                       distance);
  return pose;
}

TEST(Synthetic, ProjectsEveryCornerWhenTheBoardIsFullyInFrame) {
  const BoardSpec board = Board();
  const BoardObservation observation = ProjectBoard(Camera(), board, FacingPose(board, 0.6));

  EXPECT_EQ(static_cast<int>(observation.size()), board.corner_count());
  EXPECT_TRUE(observation.valid());
  EXPECT_EQ(observation.camera_id, "cam_front");
  EXPECT_EQ(observation.image_size, cv::Size(1280, 720));
}

TEST(Synthetic, DropsCornersThatFallOutsideTheSensor) {
  const BoardSpec board = Board();
  // Very close: the board overflows the frame, as it would in a real capture.
  const BoardObservation observation = ProjectBoard(Camera(), board, FacingPose(board, 0.10));
  EXPECT_GT(observation.size(), 0u);
  EXPECT_LT(static_cast<int>(observation.size()), board.corner_count());

  for (const cv::Point2f& p : observation.image_points) {
    EXPECT_GE(p.x, 0.0f);
    EXPECT_GE(p.y, 0.0f);
    EXPECT_LT(p.x, 1280.0f);
    EXPECT_LT(p.y, 720.0f);
  }
}

TEST(Synthetic, DropsCornersBehindTheCamera) {
  // A corner behind the camera projects to a finite point that is pure
  // nonsense, so depth must be checked rather than trusting image bounds.
  const BoardSpec board = Board();
  Eigen::Isometry3d behind = Eigen::Isometry3d::Identity();
  behind.translation() = Eigen::Vector3d(0.0, 0.0, -0.5);
  EXPECT_EQ(ProjectBoard(Camera(), board, behind).size(), 0u);
}

TEST(Synthetic, CornerIdsAndPointsStayAligned) {
  // The classic ChArUco bug is pairing a partial corner list positionally with
  // a full 3D grid. Ids must travel with the points.
  const BoardSpec board = Board();
  const BoardObservation observation = ProjectBoard(Camera(), board, FacingPose(board, 0.15));
  ASSERT_EQ(observation.corner_ids.size(), observation.image_points.size());
  for (const int id : observation.corner_ids) {
    EXPECT_GE(id, 0);
    EXPECT_LT(id, board.corner_count());
  }
  // Ids are strictly increasing, matching ChArUco's own ordering.
  for (size_t i = 1; i < observation.corner_ids.size(); ++i) {
    EXPECT_LT(observation.corner_ids[i - 1], observation.corner_ids[i]);
  }
}

TEST(Synthetic, NoiseAndDropoutAreReproducibleFromTheSeed) {
  const BoardSpec board = Board();
  SyntheticNoise noise;
  noise.pixel_sigma = 0.4;
  noise.dropout_probability = 0.2;
  noise.seed = 4242;

  const BoardObservation a = ProjectBoard(Camera(), board, FacingPose(board, 0.6), 7, noise);
  const BoardObservation b = ProjectBoard(Camera(), board, FacingPose(board, 0.6), 7, noise);

  ASSERT_EQ(a.corner_ids, b.corner_ids);
  for (size_t i = 0; i < a.image_points.size(); ++i) {
    EXPECT_FLOAT_EQ(a.image_points[i].x, b.image_points[i].x);
    EXPECT_FLOAT_EQ(a.image_points[i].y, b.image_points[i].y);
  }
  // Dropout actually removed corners.
  const BoardObservation clean = ProjectBoard(Camera(), board, FacingPose(board, 0.6));
  EXPECT_LT(a.size(), clean.size());
}

TEST(Synthetic, SampledPosesVaryInDistanceAndTilt) {
  // Views that are all fronto-parallel at one distance leave focal length and
  // board distance nearly degenerate, and calibration converges confidently to
  // the wrong answer. The sampler has to actually spread them.
  const BoardSpec board = Board();
  const auto poses = SampleBoardPoses(board, 30);
  ASSERT_EQ(poses.size(), 30u);

  double min_z = 1e9;
  double max_z = -1e9;
  double max_tilt = 0.0;
  for (const Eigen::Isometry3d& pose : poses) {
    const double z = pose.translation().z();
    min_z = std::min(min_z, z);
    max_z = std::max(max_z, z);
    const Eigen::Vector3d normal = pose.linear() * Eigen::Vector3d::UnitZ();
    max_tilt = std::max(max_tilt, std::acos(std::min(1.0, std::abs(normal.z()))));
  }
  EXPECT_GT(max_z - min_z, 0.2) << "distances barely vary";
  EXPECT_GT(max_tilt, 0.3) << "no meaningful tilt variation";
}

}  // namespace
}  // namespace the_eyes::calibration
