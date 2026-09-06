#include "the_eyes/runtime/frame_pair.hpp"

namespace the_eyes::runtime {

pose::Frame ToPoseFrame(const camera::CameraFrame& frame) {
  pose::Frame out;
  out.camera_id = frame.camera_id;
  out.stamp_ns = frame.stamp_ns;
  out.rgb = frame.rgb;
  out.depth = frame.depth;
  out.depth_scale_m = frame.depth_scale_m;
  out.K = frame.K;
  return out;
}

detector::DetectorInput ToDetectorInput(const camera::CameraFrame& frame) {
  detector::DetectorInput out;
  out.camera_id = frame.camera_id;
  out.stamp_ns = frame.stamp_ns;
  out.rgb = frame.rgb;
  out.depth = frame.depth;
  out.depth_scale_m = frame.depth_scale_m;
  return out;
}

}  // namespace the_eyes::runtime
