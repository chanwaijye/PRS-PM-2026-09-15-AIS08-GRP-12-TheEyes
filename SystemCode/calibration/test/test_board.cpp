#include "the_eyes/calibration/board.hpp"
#include "the_eyes/calibration/report.hpp"

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
  // The physical board, MEASURED 2026-09-05: a 324 x 324 mm panel carrying 8 x 8
  // squares. Callipers spanning 4 squares read 152.1 mm, so the square is
  // 152.1 / 4 = 38.025 mm -- +0.066% against the 38 mm nominal, and now the
  // value the whole system is scaled by.
  //
  // These numbers are pinned deliberately. This test failing means the config
  // and the physical board disagree, which must be resolved before any
  // calibration is trusted -- it failing when the board was measured was the
  // intended behaviour, not a bug.
  const BoardSpec s = LoadBoardSpec(std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml");
  EXPECT_EQ(s.squares_x, 8);
  EXPECT_EQ(s.squares_y, 8);
  EXPECT_NEAR(s.square_length_m, 0.038025, 1e-12);
  // Derived from the measured square under uniform print scale, not measured.
  EXPECT_NEAR(s.marker_length_m, 0.028519, 1e-12);
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
  // bought: 8 x 8 squares within a 324 x 324 mm panel, measured at 38.025 mm per
  // square (2026-09-05), so 304.2 x 304.2 mm of squares.
  //
  // The geometric relations below are derived from the spec rather than
  // hardcoded. An earlier version pinned 0.304 and 0.038 literally and failed
  // the moment the board was measured -- for a 0.2 mm difference that changed
  // nothing it was actually testing. What it exists to catch is a wrong square
  // COUNT or a decimal-place typo, both of which these still catch.
  const BoardSpec s = LoadBoardSpec(std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml");
  const double span_x = s.squares_x * s.square_length_m;
  const double span_y = s.squares_y * s.square_length_m;

  // Sanity band on the measured square, wide enough for the 1-2% a printed
  // board drifts and narrow enough to catch a decimal-place error.
  EXPECT_GT(s.square_length_m, 0.037);
  EXPECT_LT(s.square_length_m, 0.039);
  // The printed squares must fit inside the 324 mm panel that was ordered.
  EXPECT_LE(span_x, 0.324);
  EXPECT_LE(span_y, 0.324);
  EXPECT_NEAR(span_x, span_y, 1e-12);  // square board
  // Furthest interior corner sits one square in from each far edge.
  const Eigen::Vector3d far = s.CornerPosition(s.corner_count() - 1);
  EXPECT_NEAR(far.x(), span_x - s.square_length_m, 1e-9);
  EXPECT_NEAR(far.y(), span_y - s.square_length_m, 1e-9);
}

TEST(BoardSpec, WorldPlacementTracksTheMeasuredSquare) {
  // T_world_board's x/y translation is +/-(squares * square_length)/2, so it
  // MUST move whenever square_length_m does. Editing one without the other is a
  // silent, self-consistent error: every reprojection residual stays small while
  // the world origin sits in the wrong place.
  const BoardSpec s = LoadBoardSpec(std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml");
  const WorldConfig w = LoadWorldConfig(std::string(THE_EYES_CONFIG_DIR) + "/calibration.yaml");
  const double half_x = s.squares_x * s.square_length_m / 2.0;
  const double half_y = s.squares_y * s.square_length_m / 2.0;
  EXPECT_NEAR(w.T_world_board.translation().x(), -half_x, 1e-9);
  EXPECT_NEAR(w.T_world_board.translation().y(), +half_y, 1e-9);
  // z is the measured board thickness: the pattern is on the TOP face and world
  // z = 0 is the tabletop. Zero here would put the world plane on the printed
  // surface and bias every absolute height handed to PM1.
  EXPECT_GT(w.T_world_board.translation().z(), 0.0);
  EXPECT_LT(w.T_world_board.translation().z(), 0.020);
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
