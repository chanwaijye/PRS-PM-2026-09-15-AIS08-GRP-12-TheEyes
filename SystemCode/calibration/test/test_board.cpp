#include "the_eyes/calibration/board.hpp"

#include <gtest/gtest.h>

namespace the_eyes::calibration {
namespace {

BoardSpec Spec() {
  BoardSpec s;
  s.squares_x = 8;
  s.squares_y = 6;
  s.square_length_m = 0.030;
  s.marker_length_m = 0.022;
  return s;
}

TEST(BoardSpec, InteriorCornerCountMatchesTheChessboard) {
  const BoardSpec s = Spec();
  EXPECT_EQ(s.corners_x(), 7);
  EXPECT_EQ(s.corners_y(), 5);
  EXPECT_EQ(s.corner_count(), 35);
}

TEST(BoardSpec, CornerPositionsFormAUniformGridOffsetByOneSquare) {
  const BoardSpec s = Spec();
  // Corner 0 is one square in from the board edge, not at the edge itself.
  EXPECT_LT((s.CornerPosition(0) - Eigen::Vector3d(0.030, 0.030, 0.0)).norm(), 1e-12);
  // Row-major: corner 1 steps in +x.
  EXPECT_LT((s.CornerPosition(1) - Eigen::Vector3d(0.060, 0.030, 0.0)).norm(), 1e-12);
  // Corner 7 wraps to the next row.
  EXPECT_LT((s.CornerPosition(7) - Eigen::Vector3d(0.030, 0.060, 0.0)).norm(), 1e-12);
  // All corners are coplanar at z = 0.
  for (int i = 0; i < s.corner_count(); ++i) EXPECT_DOUBLE_EQ(s.CornerPosition(i).z(), 0.0);
}

TEST(BoardSpec, AllCornerPositionsAgreesWithTheIndexedAccessor) {
  const BoardSpec s = Spec();
  const auto all = s.AllCornerPositions();
  ASSERT_EQ(static_cast<int>(all.size()), s.corner_count());
  for (int i = 0; i < s.corner_count(); ++i) {
    const Eigen::Vector3d p = s.CornerPosition(i);
    EXPECT_NEAR(all[static_cast<size_t>(i)].x, p.x(), 1e-6);
    EXPECT_NEAR(all[static_cast<size_t>(i)].y, p.y(), 1e-6);
  }
}

TEST(BoardSpec, OutOfRangeCornerThrows) {
  const BoardSpec s = Spec();
  EXPECT_THROW(s.CornerPosition(-1), std::out_of_range);
  EXPECT_THROW(s.CornerPosition(s.corner_count()), std::out_of_range);
}

TEST(BoardSpec, RejectsAMarkerNotSmallerThanItsSquare) {
  // Equal lengths leave no quiet zone and marker detection degrades in a way
  // that reads as poor lighting rather than a board-spec error.
  BoardSpec s = Spec();
  s.marker_length_m = s.square_length_m;
  EXPECT_THROW(s.Validate(), std::runtime_error);
  s.marker_length_m = s.square_length_m + 0.001;
  EXPECT_THROW(s.Validate(), std::runtime_error);
}

TEST(BoardSpec, RejectsDegenerateGeometry) {
  BoardSpec s = Spec();
  s.squares_x = 2;
  EXPECT_THROW(s.Validate(), std::runtime_error);

  s = Spec();
  s.square_length_m = 0.0;
  EXPECT_THROW(s.Validate(), std::runtime_error);

  s = Spec();
  s.dictionary = "5X5_250";  // missing the DICT_ prefix
  EXPECT_THROW(s.Validate(), std::runtime_error);
}

TEST(BoardSpec, LoadsTheShippedConfig) {
  // The physical board ordered 2026-08-26: a 324 x 324 mm panel carrying 8 x 8
  // squares at 38 mm = 304 x 304 mm of squares, so a ~10 mm border each side.
  //
  // [VERIFY] on arrival with calipers. This test failing after the board
  // arrives is the intended behaviour, not a bug: it means the config and the
  // physical board disagree, and both must be corrected before any calibration
  // is trusted.
  const BoardSpec s = LoadBoardSpec(std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml");
  EXPECT_EQ(s.squares_x, 8);
  EXPECT_EQ(s.squares_y, 8);
  EXPECT_NEAR(s.square_length_m, 0.038, 1e-12);
  EXPECT_NEAR(s.marker_length_m, 0.0285, 1e-12);
  EXPECT_LT(s.marker_length_m, s.square_length_m);
  EXPECT_EQ(s.corner_count(), 49);
  EXPECT_NO_THROW(s.Validate());
}

TEST(BoardSpec, ShippedBoardMarkersFitTheConfiguredDictionary) {
  // A ChArUco board needs squares_x * squares_y / 2 markers. Overrunning the
  // dictionary is a load-time failure that reads as a corrupt board.
  const BoardSpec s = LoadBoardSpec(std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml");
  const int markers_needed = (s.squares_x * s.squares_y) / 2;
  EXPECT_EQ(markers_needed, 32);
  EXPECT_EQ(s.dictionary, "DICT_5X5_250");
  EXPECT_LE(markers_needed, 250);
}

TEST(BoardSpec, ShippedBoardSpansTheExpectedPhysicalSize) {
  // Guards a typo in squares_x/y or square_length_m against the board actually
  // bought -- nominally 304 x 304 mm of squares within a 324 x 324 mm panel.
  const BoardSpec s = LoadBoardSpec(std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml");
  EXPECT_NEAR(s.squares_x * s.square_length_m, 0.304, 1e-9);
  EXPECT_NEAR(s.squares_y * s.square_length_m, 0.304, 1e-9);
  // The printed squares must fit inside the 324 mm panel that was ordered.
  EXPECT_LE(s.squares_x * s.square_length_m, 0.324);
  EXPECT_LE(s.squares_y * s.square_length_m, 0.324);
  // Furthest interior corner sits one square in from each far edge.
  const Eigen::Vector3d far = s.CornerPosition(s.corner_count() - 1);
  EXPECT_NEAR(far.x(), 0.304 - 0.038, 1e-9);
  EXPECT_NEAR(far.y(), 0.304 - 0.038, 1e-9);
}

TEST(BoardSpec, MissingFileThrows) {
  EXPECT_THROW(LoadBoardSpec("/nonexistent/calibration.yaml"), std::runtime_error);
}


// --- decode reach -----------------------------------------------------------
//
// The number that decides whether a cheap board is usable at all.

TEST(BoardSpec, MarkerModulesCountsTheQuietBorder) {
  BoardSpec board;
  board.dictionary = "DICT_4X4_50";
  EXPECT_EQ(board.marker_modules(), 6);
  board.dictionary = "DICT_5X5_250";
  EXPECT_EQ(board.marker_modules(), 7);
  board.dictionary = "DICT_6X6_1000";
  EXPECT_EQ(board.marker_modules(), 8);
}

TEST(BoardSpec, MarkerModulesRejectsUnparseableDictionaries) {
  BoardSpec board;
  board.dictionary = "DICT_ARUCO_ORIGINAL";
  EXPECT_THROW(board.marker_modules(), std::runtime_error);
}

// The board ordered 2026-08-26 against the MEASURED factory colour fx from
// calibration_backup/ (worst unit). This is the test that says the purchase
// works: it fails at the streaming resolution and passes at capture resolution.
TEST(BoardSpec, OrderedBoardNeedsFullResolutionCapture) {
  BoardSpec board;
  board.squares_x = board.squares_y = 8;
  board.square_length_m = 0.038;
  board.marker_length_m = 0.0285;
  board.dictionary = "DICT_5X5_250";

  // The far corner of this board on the table, seen obliquely at 35 deg from
  // 0.70 m, sits 0.884 m away -- not 0.70.
  const double kObliqueFarCorner = 0.884;
  EXPECT_LT(board.MaxDecodeDistance(603.43), kObliqueFarCorner);   // 848x480
  EXPECT_GT(board.MaxDecodeDistance(905.15), kObliqueFarCorner);   // 1280x720
  EXPECT_GT(board.MaxDecodeDistance(1357.72), kObliqueFarCorner);  // 1920x1080

  // And with margin, not marginally, at the resolution we will actually use.
  EXPECT_GT(board.MaxDecodeDistance(1357.72), 2.0 * kObliqueFarCorner);
}

TEST(BoardSpec, DecodeReachScalesWithFocalLengthAndMarkerSize) {
  BoardSpec board;
  board.marker_length_m = 0.0285;
  board.dictionary = "DICT_5X5_250";
  EXPECT_NEAR(board.MaxDecodeDistance(1200.0), 2.0 * board.MaxDecodeDistance(600.0), 1e-9);

  BoardSpec bigger = board;
  bigger.marker_length_m = 0.057;
  EXPECT_NEAR(bigger.MaxDecodeDistance(600.0), 2.0 * board.MaxDecodeDistance(600.0), 1e-9);

  // A denser dictionary packs more modules into the same marker, so it decodes
  // no further despite being the same physical size.
  BoardSpec denser = board;
  denser.dictionary = "DICT_6X6_250";
  EXPECT_LT(denser.MaxDecodeDistance(600.0), board.MaxDecodeDistance(600.0));
}

TEST(BoardSpec, DecodeReachRejectsNonsenseInputs) {
  BoardSpec board;
  EXPECT_THROW(board.MaxDecodeDistance(0.0), std::runtime_error);
  EXPECT_THROW(board.MaxDecodeDistance(-600.0), std::runtime_error);
  EXPECT_THROW(board.MaxDecodeDistance(600.0, 0.0), std::runtime_error);
}

TEST(BoardSpec, ConfiguredBoardDecodesAtTheCaptureResolutionItDocuments) {
  const BoardSpec board =
      LoadBoardSpec(std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml");
  // Whatever board is configured, it must reach the oblique far corner at
  // 1920x1080 -- otherwise calibration on this rig cannot work at all.
  EXPECT_GT(board.MaxDecodeDistance(1357.72), 0.884)
      << "the configured board cannot decode at 1920x1080 on this rig";
}

}  // namespace
}  // namespace the_eyes::calibration
