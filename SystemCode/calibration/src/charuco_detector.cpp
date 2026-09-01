#include "the_eyes/calibration/charuco_detector.hpp"

#include <map>
#include <stdexcept>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace the_eyes::calibration {
namespace {

// NOTE: this is the OpenCV 4.6 aruco API (cv::aruco::CharucoBoard::create,
// interpolateCornersCharuco). OpenCV 4.7 moved ChArUco into objdetect with a
// different interface, so code copied from current tutorials will not compile
// here. See SystemCode/calibration/README.md.
int DictionaryId(const std::string& name) {
  static const std::map<std::string, int> kDictionaries = {
      {"DICT_4X4_50", cv::aruco::DICT_4X4_50},     {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
      {"DICT_4X4_250", cv::aruco::DICT_4X4_250},   {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
      {"DICT_5X5_100", cv::aruco::DICT_5X5_100},   {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
      {"DICT_6X6_50", cv::aruco::DICT_6X6_50},     {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
      {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
      {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11},
  };
  const auto it = kDictionaries.find(name);
  if (it == kDictionaries.end()) {
    throw std::runtime_error("calibration: unknown aruco dictionary '" + name + "'");
  }
  return it->second;
}

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
  // Sub-pixel marker corner refinement. Without it corner localisation is
  // integer-ish and the reprojection RMSE floor sits well above the 0.5 px
  // acceptance threshold no matter how good the capture is.
  impl_->parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
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
