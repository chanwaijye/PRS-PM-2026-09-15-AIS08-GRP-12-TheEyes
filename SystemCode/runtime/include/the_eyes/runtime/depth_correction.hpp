// Per-camera depth scale correction, fitted against the bare table.
//
// The extrinsics come from the COLOUR image of a board and are good to a tenth
// of a millimetre between the two cameras. The DEPTH each camera reports is a
// separate measurement with its own factory calibration, and nothing had ever
// checked one against the other. MEASURED on the empty desk, 2026-10-04: through
// the calibrated extrinsics cam_front's depth put the tabletop 4.2 mm ABOVE
// world z = 0 -- its depth reads 1.0% short, 5.7 mm at 0.5 m rising to 8.9 mm
// at 0.8 m -- and every pose FoundationPose built on that depth sat 7-8 mm too
// close to the camera. One multiplier, fitted on half the frames and checked on
// the other half, brings the table to +0.2 mm and the error to within about a
// millimetre at every range.
//
// It is a SCALE and nothing more, on purpose. cam_over's error the same day
// was not a scale (near zero at 0.5 m, +6.5 mm past 0.8 m, and no affine or
// tilt model fitted the held-out frames), so it is left alone rather than
// corrected with a number that moves its error around. `FitIsWorthApplying`
// is that judgement written down.
//
// Applied as a multiplier on the frame's depth unit, never on the pixels, so a
// recording is always the raw sensor and the correction can be refitted later.
#ifndef THE_EYES_RUNTIME_DEPTH_CORRECTION_HPP
#define THE_EYES_RUNTIME_DEPTH_CORRECTION_HPP

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "the_eyes/camera/frame.hpp"

namespace the_eyes::runtime {

struct DepthCorrection {
  std::map<std::string, double> scale;  // camera_id -> multiplier on metric depth

  // 1.0 for a camera the file does not mention.
  double For(const std::string& camera_id) const;
  // "cam_front x1.0102, cam_over x1.0000", or "none".
  std::string Describe() const;
};

// A missing file is the identity -- the correction is optional. A file that
// exists but is malformed, or gives a scale outside (0.9, 1.1), throws: a depth
// camera that needs more than 10% is broken, not miscalibrated.
DepthCorrection LoadDepthCorrection(const std::string& yaml_path);

// Depth samples of one frame against the world plane z = 0: for every `stride`
// pixel with depth, where its ray meets the plane (through T_world_cam), kept
// when that point lies within +-half_x, +-half_y of the origin and the
// measurement is within max_dev_m of the plane. The last gate is what keeps an
// object left on the desk out of the fit.
struct PlaneSamples {
  std::vector<double> z_measured;  // camera-z the sensor reported, metres
  std::vector<double> z_plane;     // camera-z at which that ray meets the table
};
void CollectTablePlaneSamples(const camera::CameraFrame& frame,
                              const Eigen::Isometry3d& T_world_cam, double half_x_m,
                              double half_y_m, double max_dev_m, int stride, PlaneSamples* out);

// Least-squares multiplier s minimising sum (s * z_measured - z_plane)^2.
// Throws on empty or mismatched input.
double FitDepthScale(const PlaneSamples& samples);

// The error that matters is systematic and range-dependent, so it is judged in
// range bins: the median of (scale * z_measured - z_plane) in each of `bins`
// equal-count bins of z_plane, in metres.
std::vector<double> BinnedDepthError(const PlaneSamples& samples, double scale, int bins = 5);

// Whether a fitted scale earns its place: on HELD-OUT samples the rms of the
// binned errors must fall to a quarter or less. A true scale error leaves
// almost nothing behind (cam_front: 6.7 -> 0.7 mm, a tenth). An error of some
// other shape is merely re-centred by the best multiplier -- a ramp that is
// zero near the camera and 7 mm at the far edge keeps about 45% of its rms,
// traded from the far range into the near one -- and that is a number, not a
// correction.
bool FitIsWorthApplying(const PlaneSamples& held_out, double scale);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_DEPTH_CORRECTION_HPP
