// ChArUco board geometry.
//
// Pure geometry, no OpenCV aruco dependency: the 3D position of every
// chessboard corner follows from the board spec alone. Keeping it separate is
// what lets the whole solver be exercised on synthetic observations, with no
// images and no aruco module (AGENTS.md §2 mock mode).
#ifndef THE_EYES_CALIBRATION_BOARD_HPP
#define THE_EYES_CALIBRATION_BOARD_HPP

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace the_eyes::calibration {

struct BoardSpec {
  int squares_x = 8;
  int squares_y = 6;
  double square_length_m = 0.030;
  double marker_length_m = 0.022;
  std::string dictionary = "DICT_5X5_250";

  // Interior chessboard corners -- the ones ChArUco actually reports.
  int corners_x() const { return squares_x - 1; }
  int corners_y() const { return squares_y - 1; }
  int corner_count() const { return corners_x() * corners_y(); }

  // Board-frame position of a corner, in metres.
  //
  // The origin is the OUTER corner of the square grid, one square out from the
  // first interior corner -- CornerPosition(0) is (square, square, 0), not the
  // origin. And +z points INTO the board, away from a camera viewing the
  // printed face: ChArUco numbers corners with +y running DOWN the printed
  // image, so x cross y goes through the panel. VERIFIED against OpenCV 4.6
  // estimatePoseCharucoBoard, which returns identity for a fronto-parallel
  // board.
  //
  // Both halves matter for world.T_world_board in calibration.yaml: a board
  // lying pattern-up on the table has +z DOWN in world terms, so registering it
  // with an identity pose mirrors the rig and puts every camera below the
  // tabletop. Reprojection RMSE and the round trip both stay happy.
  Eigen::Vector3d CornerPosition(int corner_id) const;

  std::vector<cv::Point3f> AllCornerPositions() const;

  // Modules across one marker, including its one-module quiet border:
  // DICT_4X4 -> 6, DICT_5X5 -> 7, DICT_6X6 -> 8. Throws if the dictionary name
  // does not carry a parseable NxN.
  int marker_modules() const;

  // Largest distance at which this board's markers still decode, in metres.
  //
  // A marker below ~3 px per module cannot be read, and this is the single
  // most common way a calibration capture silently produces nothing: the board
  // is in frame, well lit, and perfectly in focus, and zero corners come back.
  //
  //   max distance = fx * marker_length / (min_px_per_module * marker_modules)
  //
  // Note what this depends on: fx, which scales with CAPTURE RESOLUTION. The
  // same board that fails at the 848x480 streaming resolution decodes easily at
  // 1920x1080, and the recovered camera POSE is resolution-independent -- which
  // is what makes a small, cheap board usable. See `calibrate run --factory`.
  double MaxDecodeDistance(double fx_px, double min_px_per_module = 3.0) const;

  // Throws std::runtime_error describing what is wrong. A board that is
  // geometrically impossible produces a calibration that converges to nonsense
  // rather than failing, so this is checked at load.
  void Validate() const;
};

BoardSpec LoadBoardSpec(const std::string& yaml_path);

}  // namespace the_eyes::calibration

#endif  // THE_EYES_CALIBRATION_BOARD_HPP
