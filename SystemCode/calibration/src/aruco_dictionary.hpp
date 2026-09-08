// Dictionary-name -> opencv_aruco id. Private to the calibration module.
//
// Shared by charuco_detector (the calibration board) and marker_detector (the
// ground-truth trays) so the two cannot drift: config/calibration.yaml and
// config/marker_trays.yaml both name a dictionary as a string, and a name this
// file accepts in one place must mean the same thing in the other.
#ifndef THE_EYES_CALIBRATION_ARUCO_DICTIONARY_HPP
#define THE_EYES_CALIBRATION_ARUCO_DICTIONARY_HPP

#include <map>
#include <stdexcept>
#include <string>

#include <opencv2/aruco.hpp>

namespace the_eyes::calibration {

// NOTE: OpenCV 4.6 aruco API. 4.7 moved these into objdetect with a different
// interface, so code copied from current tutorials will not compile here.
inline int DictionaryId(const std::string& name) {
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

}  // namespace the_eyes::calibration

#endif  // THE_EYES_CALIBRATION_ARUCO_DICTIONARY_HPP
