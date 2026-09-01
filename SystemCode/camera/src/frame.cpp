#include "the_eyes/camera/frame.hpp"

namespace the_eyes::camera {

bool CameraFrame::valid() const {
  if (rgb.empty() || depth.empty()) return false;
  if (rgb.type() != CV_8UC3 || depth.type() != CV_16UC1) return false;
  // Depth aligned to colour shares the colour pixel grid (INTERFACES.md §1).
  // Differing sizes mean alignment was never applied.
  if (rgb.size() != depth.size()) return false;
  return depth_scale_m > 0.0 && !serial.empty();
}

}  // namespace the_eyes::camera
