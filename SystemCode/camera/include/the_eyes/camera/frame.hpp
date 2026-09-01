// One camera's aligned RGB-D frame.
#ifndef THE_EYES_CAMERA_FRAME_HPP
#define THE_EYES_CAMERA_FRAME_HPP

#include <cstdint>
#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace the_eyes::camera {

struct CameraFrame {
  std::string camera_id;      // "cam_front" | "cam_over"
  std::string serial;         // the physical unit this came from
  uint64_t stamp_ns = 0;      // SOURCE capture time, not receive time
  uint64_t frame_number = 0;
  cv::Mat rgb;                // CV_8UC3, RGB order
  cv::Mat depth;              // CV_16UC1, aligned to colour
  double depth_scale_m = 0.001;
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();  // colour intrinsics

  bool valid() const;
};

}  // namespace the_eyes::camera

#endif  // THE_EYES_CAMERA_FRAME_HPP
