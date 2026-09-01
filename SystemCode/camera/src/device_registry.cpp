#include "the_eyes/camera/device_registry.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

#ifdef THE_EYES_WITH_REALSENSE
#include <librealsense2/rs.hpp>
#endif

namespace the_eyes::camera {

std::vector<DeviceInfo> EnumerateDevices() {
  std::vector<DeviceInfo> out;
#ifdef THE_EYES_WITH_REALSENSE
  rs2::context context;
  for (const rs2::device& device : context.query_devices()) {
    DeviceInfo info;
    const auto get = [&device](rs2_camera_info key) -> std::string {
      return device.supports(key) ? device.get_info(key) : std::string();
    };
    info.serial = get(RS2_CAMERA_INFO_SERIAL_NUMBER);
    info.name = get(RS2_CAMERA_INFO_NAME);
    info.firmware_version = get(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    info.usb_type = get(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
    out.push_back(std::move(info));
  }
  // Sorted by serial: enumeration order is exactly what must never matter here,
  // so it is discarded rather than passed on.
  std::sort(out.begin(), out.end(),
            [](const DeviceInfo& a, const DeviceInfo& b) { return a.serial < b.serial; });
#endif
  return out;
}

RegistryCheck CheckDevices(const CameraAgentConfig& config) {
  RegistryCheck check;
  check.attached = EnumerateDevices();

  std::vector<std::string> attached_serials;
  for (const DeviceInfo& device : check.attached) attached_serials.push_back(device.serial);

  for (const auto& [camera_id, entry] : config.cameras) {
    if (std::find(attached_serials.begin(), attached_serials.end(), entry.serial) ==
        attached_serials.end()) {
      check.missing_serials.push_back(entry.serial + " (" + camera_id + ")");
    }
  }
  for (const DeviceInfo& device : check.attached) {
    bool configured = false;
    for (const auto& [camera_id, entry] : config.cameras) {
      if (entry.serial == device.serial) configured = true;
    }
    // Not an error: a third RealSense on the bench is allowed, it is simply
    // not part of this rig.
    if (!configured) check.extra_serials.push_back(device.serial);
  }

  // USB 2 is a BLOCKER, not a warning: the configured dual-stream profile
  // cannot be resolved on that bandwidth, and librealsense reports it as
  // "Couldn't resolve requests" or "failed to set power state" -- neither of
  // which points at the cable. Naming it here saves that debugging session.
  for (const DeviceInfo& device : check.attached) {
    bool configured = false;
    for (const auto& [camera_id, entry] : config.cameras) {
      if (entry.serial == device.serial) configured = true;
    }
    if (!configured || device.usb_type.empty() || device.usb3()) continue;

    std::ostringstream os;
    os << device.serial << " is enumerated on USB " << device.usb_type << ", but "
       << config.streams.color_width << "x" << config.streams.color_height << "@"
       << config.streams.fps << " depth + colour needs USB 3.x. The D435i drops to USB 2 "
       << "silently on a marginal cable or port -- try a different USB 3 port and a known-good "
       << "cable, then re-run. This is a physical fix, not a config change.";
    check.blocking.push_back(os.str());
  }

  // Mixed firmware across the two units can change timestamping and sync
  // behaviour, which is precisely what the skew check measures.
  if (check.attached.size() >= 2) {
    const std::string& first = check.attached.front().firmware_version;
    for (const DeviceInfo& device : check.attached) {
      if (device.firmware_version != first) {
        check.warnings.push_back(
            "attached units run different firmware (" + first + " vs " +
            device.firmware_version +
            ") -- timestamping and inter-camera sync behaviour can differ between versions");
        break;
      }
    }
  }

  check.ok = check.missing_serials.empty() && check.blocking.empty();
  return check;
}

Eigen::Matrix3d FactoryColorIntrinsics(const std::string& serial, int width, int height,
                                       std::vector<double>* distortion_out) {
#ifdef THE_EYES_WITH_REALSENSE
  rs2::context context;
  for (const rs2::device& device : context.query_devices()) {
    if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) continue;
    if (std::string(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) != serial) continue;

    for (const rs2::sensor& sensor : device.query_sensors()) {
      for (const rs2::stream_profile& profile : sensor.get_stream_profiles()) {
        const auto video = profile.as<rs2::video_stream_profile>();
        if (!video) continue;
        if (video.stream_type() != RS2_STREAM_COLOR) continue;
        if (video.width() != width || video.height() != height) continue;

        const rs2_intrinsics intrinsics = video.get_intrinsics();
        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        K(0, 0) = intrinsics.fx;
        K(1, 1) = intrinsics.fy;
        K(0, 2) = intrinsics.ppx;
        K(1, 2) = intrinsics.ppy;
        if (distortion_out != nullptr) {
          distortion_out->assign(std::begin(intrinsics.coeffs), std::end(intrinsics.coeffs));
        }
        return K;
      }
    }
    std::ostringstream os;
    os << "camera " << serial << " has no colour profile at " << width << "x" << height;
    throw std::runtime_error(os.str());
  }
  throw std::runtime_error("camera " + serial + " is not attached");
#else
  (void)serial; (void)width; (void)height; (void)distortion_out;
  throw std::runtime_error("built without librealsense: factory intrinsics unavailable");
#endif
}

std::string RegistryCheck::Format() const {
  std::ostringstream os;
  os << "Attached RealSense devices (" << attached.size() << ")\n";
  for (const DeviceInfo& device : attached) {
    os << "  " << device.serial << "  " << device.name << "  fw " << device.firmware_version
       << "  USB " << (device.usb_type.empty() ? "?" : device.usb_type) << "\n";
  }
  if (!missing_serials.empty()) {
    os << "MISSING (configured but not attached):\n";
    for (const std::string& serial : missing_serials) os << "  " << serial << "\n";
  }
  if (!extra_serials.empty()) {
    os << "not part of this rig:\n";
    for (const std::string& serial : extra_serials) os << "  " << serial << "\n";
  }
  for (const std::string& warning : warnings) os << "WARNING: " << warning << "\n";
  for (const std::string& issue : blocking) os << "BLOCKING: " << issue << "\n";
  if (ok) {
    os << "RESULT: all configured cameras present and on adequate links\n";
  } else if (!missing_serials.empty()) {
    os << "RESULT: FAIL -- configured cameras are missing\n";
  } else {
    os << "RESULT: FAIL -- a configured camera cannot support the requested streams\n";
  }
  return os.str();
}

}  // namespace the_eyes::camera
