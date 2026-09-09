#include "the_eyes/camera/realsense_camera.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include <librealsense2/rs.hpp>
#include <opencv2/imgproc.hpp>

namespace the_eyes::camera {
namespace {

std::string DomainName(rs2_timestamp_domain domain) {
  switch (domain) {
    case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK: return "Hardware Clock";
    case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME: return "System Time";
    case RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME: return "Global Time";
    default: return "unknown";
  }
}

}  // namespace

struct RealSenseCamera::Impl {
  CameraEntry entry;
  StreamConfig streams;
  rs2::pipeline pipeline;
  rs2::align align{RS2_STREAM_COLOR};
  double depth_scale = 0.001;
  int actual_sync_mode = 0;
  std::string timestamp_domain = "unknown";
  // What the colour sensor actually reports after the requested controls were
  // applied, so a recorded session says what it was shot at.
  bool actual_auto_exposure = true;
  int actual_exposure = 0;
  int actual_gain = 0;
  bool actual_auto_white_balance = true;
  int actual_white_balance_k = 0;
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  std::vector<std::string> warnings;
};

RealSenseCamera::RealSenseCamera(const CameraEntry& entry, const StreamConfig& streams)
    : impl_(std::make_unique<Impl>()) {
  impl_->entry = entry;
  impl_->streams = streams;

  // inter_cam_sync_mode MUST be set before the pipeline starts. librealsense
  // rejects it while streaming ("Cannot change Inter-camera HW synchronization
  // mode while streaming!"), so the sensor is reached through the context
  // first. Setting it after pipeline.start() silently leaves the cameras
  // free-running -- which is R4b's mitigation quietly not applied.
  // Applied unconditionally, INCLUDING mode 0. The device retains this setting
  // in non-volatile memory across power cycles, so "0 = default" has to be
  // actively written to undo a previous master/slave configuration -- skipping
  // the write when 0 is requested leaves a slave camera silently waiting for a
  // trigger that will never come.
  {
    rs2::context context;
    bool found = false;
    for (const rs2::device& device : context.query_devices()) {
      if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) continue;
      if (std::string(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) != entry.serial) continue;
      found = true;
      try {
        rs2::depth_sensor sensor = device.first<rs2::depth_sensor>();
        if (!sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
          impl_->warnings.push_back("device does not support inter_cam_sync_mode; "
                                    "R4b mitigation is unavailable on this unit");
          break;
        }
        sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE,
                          static_cast<float>(entry.inter_cam_sync_mode));
      } catch (const rs2::error& e) {
        impl_->warnings.push_back(std::string("could not set inter_cam_sync_mode: ") + e.what());
      }
      break;
    }
    if (!found) {
      impl_->warnings.push_back("serial " + entry.serial +
                                " not found when setting inter_cam_sync_mode");
    }
  }

  rs2::config config;
  // THE constraint: bind this pipeline to one physical unit by serial. Nothing
  // in this class ever looks at enumeration order.
  config.enable_device(entry.serial);
  if (!streams.color_only) {
    config.enable_stream(RS2_STREAM_DEPTH, streams.depth_width, streams.depth_height,
                         RS2_FORMAT_Z16, streams.fps);
  }
  config.enable_stream(RS2_STREAM_COLOR, streams.color_width, streams.color_height,
                       RS2_FORMAT_RGB8, streams.fps);

  rs2::pipeline_profile profile;
  try {
    profile = impl_->pipeline.start(config);
  } catch (const rs2::error& e) {
    std::ostringstream os;
    os << "camera '" << entry.camera_id << "' (serial " << entry.serial
       << "): cannot start pipeline at colour " << streams.color_width << "x"
       << streams.color_height;
    if (!streams.color_only) {
      os << " + depth " << streams.depth_width << "x" << streams.depth_height;
    }
    os << " @" << streams.fps << ": " << e.what();
    // "Couldn't resolve requests" is almost always bandwidth, not a bad profile.
    // Say so, because the fix is a cable or a port rather than a config change.
    try {
      rs2::context context;
      for (const rs2::device& device : context.query_devices()) {
        if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) continue;
        if (std::string(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) != entry.serial) continue;
        if (device.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
          const std::string usb = device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
          os << "\n  device is enumerated on USB " << usb;
          if (!usb.empty() && usb[0] != '3') {
            os << " -- this profile needs USB 3.x. Try a different port or a "
                  "USB 3 cable; the D435i falls back to USB 2 silently on a "
                  "marginal link.";
          }
        }
        break;
      }
    } catch (...) {
      // Diagnostics are best-effort; the original failure is what matters.
    }
    throw std::runtime_error(os.str());
  }

  const rs2::device device = profile.get_device();
  if (device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
    const std::string opened = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    if (opened != entry.serial) {
      // Should be impossible given enable_device, which is exactly why it is
      // worth asserting: silently opening the wrong unit is the failure mode
      // the serial rule exists to prevent.
      throw std::runtime_error("camera '" + entry.camera_id + "': asked for serial " +
                               entry.serial + " but opened " + opened);
    }
  }

  rs2::depth_sensor depth_sensor = device.first<rs2::depth_sensor>();
  impl_->depth_scale = depth_sensor.get_depth_scale();

  // Read back what the device actually reports, rather than assuming the set
  // took. A unit with no sync cable can accept the option and do nothing, and
  // R4b's mitigation depends on it genuinely taking effect.
  if (depth_sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
    impl_->actual_sync_mode =
        static_cast<int>(depth_sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE));
    if (impl_->actual_sync_mode != entry.inter_cam_sync_mode) {
      std::ostringstream os;
      os << "requested inter_cam_sync_mode " << entry.inter_cam_sync_mode << " but device "
         << "reports " << impl_->actual_sync_mode
         << " -- hardware sync needs a physical cable between the units' sync connectors";
      impl_->warnings.push_back(os.str());
    }
  }

  // Colour controls. Unlike inter_cam_sync_mode these are settable while
  // streaming, so they go after start() where the colour sensor is reachable
  // from the profile.
  //
  // ORDER MATTERS: auto must be switched off BEFORE the manual value is
  // written, or the control loop overwrites it on the next frame and the
  // capture silently runs on auto anyway.
  try {
    rs2::sensor colour = profile.get_device().first<rs2::color_sensor>();
    const ColorControl& c = streams.color;

    auto set_checked = [&](rs2_option opt, double value, const char* name) {
      if (!colour.supports(opt)) {
        impl_->warnings.push_back(std::string("colour sensor does not support ") + name);
        return;
      }
      const rs2::option_range r = colour.get_option_range(opt);
      if (value < r.min || value > r.max) {
        std::ostringstream os;
        os << name << " " << value << " is outside the device range [" << r.min << ", " << r.max
           << "] (device units)";
        impl_->warnings.push_back(os.str());
        return;
      }
      colour.set_option(opt, static_cast<float>(value));
    };

    // The colour sensor's exposure option is in DEVICE UNITS, range [1, 10000]
    // on this rig, NOT the microseconds the depth sensor uses. Writing
    // microseconds through asked for 20000, which was out of range and was
    // refused with a warning rather than silently applied -- which is why the
    // read-back exists.
    set_checked(RS2_OPTION_ENABLE_AUTO_EXPOSURE, c.auto_exposure ? 1.0 : 0.0,
                "enable_auto_exposure");
    if (c.manual_exposure()) {
      set_checked(RS2_OPTION_EXPOSURE, c.exposure, "exposure");
      set_checked(RS2_OPTION_GAIN, c.gain, "gain");
    }
    set_checked(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, c.auto_white_balance ? 1.0 : 0.0,
                "enable_auto_white_balance");
    if (c.manual_white_balance()) {
      set_checked(RS2_OPTION_WHITE_BALANCE, c.white_balance_k, "white_balance");
    }

    // Read back rather than assume, for the same reason the sync mode is read
    // back: a device can accept an option and not apply it.
    auto get_opt = [&](rs2_option opt) -> double {
      return colour.supports(opt) ? colour.get_option(opt) : 0.0;
    };
    impl_->actual_auto_exposure = get_opt(RS2_OPTION_ENABLE_AUTO_EXPOSURE) != 0.0;
    impl_->actual_exposure = static_cast<int>(get_opt(RS2_OPTION_EXPOSURE));
    impl_->actual_gain = static_cast<int>(get_opt(RS2_OPTION_GAIN));
    impl_->actual_auto_white_balance = get_opt(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) != 0.0;
    impl_->actual_white_balance_k = static_cast<int>(get_opt(RS2_OPTION_WHITE_BALANCE));

    if (impl_->actual_auto_exposure != c.auto_exposure) {
      impl_->warnings.push_back("requested auto_exposure " +
                                std::string(c.auto_exposure ? "on" : "off") +
                                " but the device reports the opposite");
    }
    if (c.manual_exposure() && impl_->actual_exposure != c.exposure) {
      std::ostringstream os;
      os << "requested exposure " << c.exposure << " but device reports "
         << impl_->actual_exposure;
      impl_->warnings.push_back(os.str());
    }
  } catch (const rs2::error& e) {
    impl_->warnings.push_back(std::string("could not apply colour controls: ") + e.what());
  }

  const auto colour_profile =
      profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  const rs2_intrinsics intrinsics = colour_profile.get_intrinsics();
  impl_->K << intrinsics.fx, 0.0, intrinsics.ppx, 0.0, intrinsics.fy, intrinsics.ppy, 0.0, 0.0,
      1.0;
}

RealSenseCamera::~RealSenseCamera() {
  try {
    impl_->pipeline.stop();
  } catch (...) {
    // Nothing useful to do while unwinding.
  }
}

const std::string& RealSenseCamera::camera_id() const { return impl_->entry.camera_id; }
const std::string& RealSenseCamera::serial() const { return impl_->entry.serial; }
int RealSenseCamera::actual_sync_mode() const { return impl_->actual_sync_mode; }

bool RealSenseCamera::actual_auto_exposure() const { return impl_->actual_auto_exposure; }
int RealSenseCamera::actual_exposure() const { return impl_->actual_exposure; }
int RealSenseCamera::actual_gain() const { return impl_->actual_gain; }
bool RealSenseCamera::actual_auto_white_balance() const {
  return impl_->actual_auto_white_balance;
}
int RealSenseCamera::actual_white_balance_k() const { return impl_->actual_white_balance_k; }
double RealSenseCamera::depth_scale_m() const { return impl_->depth_scale; }
std::string RealSenseCamera::timestamp_domain() const { return impl_->timestamp_domain; }
const std::vector<std::string>& RealSenseCamera::warnings() const { return impl_->warnings; }

CameraFrame RealSenseCamera::NextFrame(int timeout_ms) {
  rs2::frameset frames;
  if (!impl_->pipeline.try_wait_for_frames(&frames, static_cast<unsigned int>(timeout_ms))) {
    throw std::runtime_error("camera '" + impl_->entry.camera_id + "': timed out after " +
                             std::to_string(timeout_ms) + " ms waiting for frames");
  }

  // Align depth INTO the colour frame (INTERFACES.md §1). FoundationPose needs
  // both on one pixel grid; without this they are separate optical frames.
  if (impl_->streams.align_depth_to_color && !impl_->streams.color_only) {
    frames = impl_->align.process(frames);
  }

  const rs2::video_frame colour = frames.get_color_frame();
  if (!colour) {
    throw std::runtime_error("camera '" + impl_->entry.camera_id + "': incomplete frameset");
  }
  rs2::depth_frame depth = frames.get_depth_frame();
  if (!depth && !impl_->streams.color_only) {
    throw std::runtime_error("camera '" + impl_->entry.camera_id + "': incomplete frameset");
  }

  CameraFrame frame;
  frame.camera_id = impl_->entry.camera_id;
  frame.serial = impl_->entry.serial;
  // Capture time from the device, not receive time (INTERFACES.md §2).
  frame.stamp_ns = static_cast<uint64_t>(colour.get_timestamp() * 1e6);
  frame.frame_number = colour.get_frame_number();
  frame.depth_scale_m = impl_->depth_scale;
  frame.K = impl_->K;
  impl_->timestamp_domain = DomainName(colour.get_frame_timestamp_domain());

  // Copy out of librealsense's buffers: the frames are recycled as soon as the
  // frameset goes out of scope, and a cv::Mat wrapping them would dangle.
  frame.rgb = cv::Mat(colour.get_height(), colour.get_width(), CV_8UC3,
                      const_cast<void*>(colour.get_data()))
                  .clone();
  // Left empty in colour-only mode; see StreamConfig::color_only.
  if (depth) {
    frame.depth = cv::Mat(depth.get_height(), depth.get_width(), CV_16UC1,
                          const_cast<void*>(depth.get_data()))
                      .clone();
  }
  return frame;
}

}  // namespace the_eyes::camera
