// Device discovery and selection BY SERIAL NUMBER.
//
// CLAUDE.md constraint 4 is absolute: never select a camera by /dev/video* or
// enumeration order. USB enumeration is not stable across reboots or replugs,
// and an index-based mapping silently swaps the two views -- producing fused
// poses that are wrong in a way nothing downstream can detect, because every
// module stays internally consistent.
//
// There is deliberately NO index-based accessor in this API. The constraint is
// enforced by the interface, not by remembering to follow it.
#ifndef THE_EYES_CAMERA_DEVICE_REGISTRY_HPP
#define THE_EYES_CAMERA_DEVICE_REGISTRY_HPP

#include <string>
#include <vector>

#include <Eigen/Core>

#include "the_eyes/camera/config.hpp"

namespace the_eyes::camera {

struct DeviceInfo {
  std::string serial;
  std::string name;
  std::string firmware_version;
  std::string usb_type;  // "3.2", "2.1", ...

  bool usb3() const { return !usb_type.empty() && usb_type[0] == '3'; }
};

// Enumerates attached RealSense devices. Returns empty when librealsense was
// not compiled in, or when nothing is attached.
std::vector<DeviceInfo> EnumerateDevices();

struct RegistryCheck {
  std::vector<DeviceInfo> attached;
  std::vector<std::string> missing_serials;   // configured but not attached
  std::vector<std::string> extra_serials;     // attached but not configured
  std::vector<std::string> warnings;
  // Conditions that make the configured stream profile impossible, as opposed
  // to merely worth knowing about.
  std::vector<std::string> blocking;
  bool ok = false;
  std::string Format() const;
};

// Confirms every configured serial is attached AND on a link that can carry the
// configured streams. Extra devices are reported but are not an error -- a third
// RealSense on the bench is allowed, it just is not part of this rig.
RegistryCheck CheckDevices(const CameraAgentConfig& config);

// The device's own factory colour intrinsics at a given resolution.
//
// Intrinsics are per-resolution, so the width/height must match the images the
// caller will actually use them on. The camera POSE derived from them is not
// resolution-dependent, however: the projection centre is physically fixed
// (measured half-FOV 34.93 deg at 848x480 vs 35.10 deg at 1920x1080 -- the same
// optical frame, differently sampled). That is what makes it valid to register
// extrinsics from high-resolution captures and then stream at 848x480.
//
// Throws std::runtime_error if the serial is not attached or the profile does
// not exist. `distortion_out` receives the reported coefficients, which are
// zero on D400 colour because that stream comes out rectified.
Eigen::Matrix3d FactoryColorIntrinsics(const std::string& serial, int width, int height,
                                       std::vector<double>* distortion_out = nullptr);

}  // namespace the_eyes::camera

#endif  // THE_EYES_CAMERA_DEVICE_REGISTRY_HPP
