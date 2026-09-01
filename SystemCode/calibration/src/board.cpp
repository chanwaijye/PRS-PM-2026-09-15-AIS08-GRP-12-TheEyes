#include "the_eyes/calibration/board.hpp"

#include <sstream>
#include <string>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::calibration {

Eigen::Vector3d BoardSpec::CornerPosition(int corner_id) const {
  if (corner_id < 0 || corner_id >= corner_count()) {
    std::ostringstream os;
    os << "BoardSpec: corner id " << corner_id << " outside [0, " << corner_count() << ")";
    throw std::out_of_range(os.str());
  }
  // ChArUco numbers interior corners row-major from the board origin. The
  // first interior corner sits one square in from the board edge, which is why
  // both indices are offset by one square length.
  const int col = corner_id % corners_x();
  const int row = corner_id / corners_x();
  return Eigen::Vector3d((col + 1) * square_length_m, (row + 1) * square_length_m, 0.0);
}

std::vector<cv::Point3f> BoardSpec::AllCornerPositions() const {
  std::vector<cv::Point3f> out;
  out.reserve(static_cast<size_t>(corner_count()));
  for (int i = 0; i < corner_count(); ++i) {
    const Eigen::Vector3d p = CornerPosition(i);
    out.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
                     static_cast<float>(p.z()));
  }
  return out;
}

int BoardSpec::marker_modules() const {
  // "DICT_5X5_250" -> 5 bits across -> 7 modules with the quiet border.
  const size_t start = dictionary.find('_');
  const size_t x = dictionary.find('X', start == std::string::npos ? 0 : start);
  if (start == std::string::npos || x == std::string::npos || x <= start + 1) {
    throw std::runtime_error("calibration.yaml: cannot read a marker size out of dictionary '" +
                             dictionary + "'; expected a name like DICT_5X5_250");
  }
  const int bits = std::stoi(dictionary.substr(start + 1, x - start - 1));
  if (bits < 3 || bits > 7) {
    throw std::runtime_error("calibration.yaml: dictionary '" + dictionary +
                             "' reports an implausible marker size of " + std::to_string(bits) +
                             " bits");
  }
  return bits + 2;
}

double BoardSpec::MaxDecodeDistance(double fx_px, double min_px_per_module) const {
  if (!(fx_px > 0.0) || !(min_px_per_module > 0.0)) {
    throw std::runtime_error("BoardSpec::MaxDecodeDistance: fx and min_px_per_module "
                             "must be positive");
  }
  return fx_px * marker_length_m / (min_px_per_module * marker_modules());
}

void BoardSpec::Validate() const {
  if (squares_x < 3 || squares_y < 3) {
    throw std::runtime_error("calibration.yaml: board needs at least 3x3 squares");
  }
  if (!(square_length_m > 0.0)) {
    throw std::runtime_error("calibration.yaml: board.square_length_m must be positive");
  }
  if (!(marker_length_m > 0.0)) {
    throw std::runtime_error("calibration.yaml: board.marker_length_m must be positive");
  }
  // The marker sits inside the white square. Equal lengths leave no quiet zone
  // and the detector's marker segmentation becomes unreliable in a way that
  // looks like poor lighting rather than a board-spec error.
  if (marker_length_m >= square_length_m) {
    std::ostringstream os;
    os << "calibration.yaml: board.marker_length_m (" << marker_length_m
       << ") must be strictly less than board.square_length_m (" << square_length_m << ")";
    throw std::runtime_error(os.str());
  }
  if (dictionary.rfind("DICT_", 0) != 0) {
    throw std::runtime_error("calibration.yaml: board.dictionary must be a cv::aruco "
                             "predefined name, e.g. DICT_5X5_250");
  }
}

BoardSpec LoadBoardSpec(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("calibration.yaml: cannot parse '" + yaml_path + "': " + e.what());
  }
  const YAML::Node node = root["board"];
  if (!node || !node.IsMap()) {
    throw std::runtime_error("calibration.yaml: missing section 'board'");
  }

  BoardSpec spec;
  const auto require = [&node](const char* key) {
    if (!node[key]) {
      throw std::runtime_error(std::string("calibration.yaml: missing 'board.") + key + "'");
    }
    return node[key];
  };
  spec.squares_x = require("squares_x").as<int>();
  spec.squares_y = require("squares_y").as<int>();
  spec.square_length_m = require("square_length_m").as<double>();
  spec.marker_length_m = require("marker_length_m").as<double>();
  spec.dictionary = require("dictionary").as<std::string>();
  spec.Validate();
  return spec;
}

}  // namespace the_eyes::calibration
