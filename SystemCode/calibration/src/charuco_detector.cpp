#include "the_eyes/calibration/charuco_detector.hpp"

#include "aruco_dictionary.hpp"

#include <map>
#include <stdexcept>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace the_eyes::calibration {
namespace {

// DictionaryId now lives in aruco_dictionary.hpp so the tray detector shares it.

}  // namespace

struct CharucoDetector::Impl {
  BoardSpec spec;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::CharucoBoard> board;
  cv::Ptr<cv::aruco::DetectorParameters> parameters;
};

CharucoDetector::CharucoDetector(const BoardSpec& board) : impl_(std::make_unique<Impl>()) {
  board.Validate();
  impl_->spec = board;
  impl_->dictionary = cv::aruco::getPredefinedDictionary(DictionaryId(board.dictionary));
  impl_->board = cv::aruco::CharucoBoard::create(
      board.squares_x, board.squares_y, static_cast<float>(board.square_length_m),
      static_cast<float>(board.marker_length_m), impl_->dictionary);

  impl_->parameters = cv::aruco::DetectorParameters::create();
  // Marker corner refinement is deliberately OFF here, which reverses an
  // earlier decision. The claim it rested on -- that without it corner
  // localisation is integer-ish and the RMSE floor sits above the 0.5 px gate
  // -- is false for ChArUco: interpolateCornersCharuco runs its own
  // cornerSubPix on the interpolated chessboard corners, so the marker-level
  // pass is redundant. It is also actively harmful on an oblique view.
  //
  // MEASURED over 21 registration frames per camera (2026-09-20):
  //
  //                   median RMSE   max     over the 0.50 px gate
  //   cam_front  on        0.460   0.910          10 / 21
  //   cam_front  off       0.354   0.543           2 / 21
  //   cam_over   on        0.342   0.450           0 / 21
  //   cam_over   off       0.341   0.421           0 / 21
  //
  // The mechanism: refinement moves 48 of 49 corners by a median 0.005 px --
  // nothing -- and then snaps ONE corner 4.6 px onto a neighbouring feature.
  // A single such outlier in a 49-point fit lifts the RMSE to ~0.70 px while
  // moving the recovered pose by 0.09 mm, so it fails the gate on a
  // calibration that is actually fine. It bites cam_front and not cam_over
  // because cam_front's view is more oblique: at the board's far edge the
  // 5 px refinement window spans more than one feature.
  //
  // NOT the same call as marker_detector.cpp, which keeps refinement on: there
  // the marker corners go straight to pose with no charuco interpolation to
  // re-refine them, so the refinement is load-bearing rather than redundant.
  impl_->parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
}

CharucoDetector::~CharucoDetector() = default;

BoardObservation CharucoDetector::Detect(const cv::Mat& image, const std::string& camera_id,
                                         uint64_t stamp_ns) const {
  BoardObservation observation;
  observation.camera_id = camera_id;
  observation.stamp_ns = stamp_ns;
  observation.image_size = image.size();
  if (image.empty()) return observation;

  cv::Mat grey;
  if (image.channels() == 3) {
    cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
  } else {
    grey = image;
  }

  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(grey, impl_->dictionary, marker_corners, marker_ids,
                           impl_->parameters);
  if (marker_ids.empty()) return observation;  // board not in frame: normal, not an error

  // Chessboard corners are interpolated from the surrounding markers, which is
  // what makes them sub-pixel accurate and identifiable in a partial view.
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;
  cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, grey, impl_->board,
                                       charuco_corners, charuco_ids);

  observation.corner_ids = std::move(charuco_ids);
  observation.image_points = std::move(charuco_corners);
  return observation;
}

void CharucoDetector::WriteBoardImage(const std::string& out_path, int pixels_per_square) const {
  const cv::Size size(impl_->spec.squares_x * pixels_per_square,
                      impl_->spec.squares_y * pixels_per_square);
  cv::Mat image;
  impl_->board->draw(size, image, /*marginSize=*/pixels_per_square / 2, /*borderBits=*/1);
  if (!cv::imwrite(out_path, image)) {
    throw std::runtime_error("calibration: cannot write board image to '" + out_path + "'");
  }
}

}  // namespace the_eyes::calibration
