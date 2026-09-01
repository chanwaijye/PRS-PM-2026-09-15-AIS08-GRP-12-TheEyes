// Back up (and optionally restore) each camera's on-device calibration.
//
// WHY THIS MATTERS. The calibration table is written per unit at the factory and
// is not reproducible without Intel's rig. It can be overwritten by an on-chip
// self-calibration run, a firmware mishap, or a bad write -- and nothing
// downstream would report it. Every depth measurement would simply be wrong,
// self-consistently.
//
// The device does keep a separate factory copy, recoverable with
// reset_to_factory_calibration(), so total loss is unlikely. What this backup
// protects is the CURRENT table -- including any self-calibration result you
// want to keep, which a factory reset would discard.
//
// Usage:
//   backup_calibration <out_dir>                       back up every attached unit
//   backup_calibration --restore <serial> <table.bin>  write a table back (see below)
//
// Writes per unit:
//   <serial>_calibration_table.bin   raw 512 bytes, the restorable artefact
//   <serial>_calibration.yaml        human-readable intrinsics, extrinsics, metadata
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

namespace {

std::string Info(const rs2::device& device, rs2_camera_info key) {
  return device.supports(key) ? device.get_info(key) : std::string("(unavailable)");
}

// FNV-1a. Not cryptographic -- it exists so a restore can notice a truncated or
// corrupted file before writing it to EEPROM, which is the operation you cannot
// take back.
std::string Fingerprint(const std::vector<uint8_t>& bytes) {
  uint64_t hash = 1469598103934665603ULL;
  for (const uint8_t b : bytes) {
    hash ^= b;
    hash *= 1099511628211ULL;
  }
  std::ostringstream os;
  os << std::hex << std::setw(16) << std::setfill('0') << hash;
  return os.str();
}

std::string StreamName(rs2_stream stream, int index) {
  std::ostringstream os;
  os << rs2_stream_to_string(stream);
  if (index != 0) os << "_" << index;
  return os.str();
}

void DumpIntrinsics(std::ostream& out, const rs2::device& device) {
  out << "  # Every colour/depth/IR profile the device reports, with its factory\n"
      << "  # intrinsics. Not directly restorable -- this is the human-readable\n"
      << "  # record, and what lets you spot a table that has drifted.\n";
  out << "  profiles:\n";
  // Per-device, NOT static: a static here is shared across every device in the
  // enumeration, so the second camera's profiles all look "already seen" and its
  // record comes out empty -- a silent, plausible-looking backup of nothing.
  std::vector<std::string> seen;
  for (const rs2::sensor& sensor : device.query_sensors()) {
    for (const rs2::stream_profile& profile : sensor.get_stream_profiles()) {
      const auto video = profile.as<rs2::video_stream_profile>();
      if (!video) continue;
      // One entry per (stream, resolution); frame rate does not change intrinsics.
      const std::string key = StreamName(video.stream_type(), video.stream_index()) + "_" +
                              std::to_string(video.width()) + "x" +
                              std::to_string(video.height());
      if (std::find(seen.begin(), seen.end(), key) != seen.end()) continue;
      seen.push_back(key);

      // Not every advertised profile exposes intrinsics without being started
      // (some auxiliary streams throw). Skip those rather than abandoning the
      // whole backup -- the restorable table is already safely written.
      rs2_intrinsics k{};
      try {
        k = video.get_intrinsics();
      } catch (const rs2::error&) {
        continue;
      }
      out << "    - stream: " << StreamName(video.stream_type(), video.stream_index()) << "\n"
          << "      format: " << rs2_format_to_string(video.format()) << "\n"
          << "      width: " << k.width << "\n"
          << "      height: " << k.height << "\n"
          << std::setprecision(10)
          << "      fx: " << k.fx << "\n"
          << "      fy: " << k.fy << "\n"
          << "      cx: " << k.ppx << "\n"
          << "      cy: " << k.ppy << "\n"
          << "      model: " << rs2_distortion_to_string(k.model) << "\n"
          << "      coeffs: [";
      for (int i = 0; i < 5; ++i) out << (i ? ", " : "") << k.coeffs[i];
      out << "]\n";
    }
  }
}

void DumpExtrinsics(std::ostream& out, const rs2::device& device) {
  out << "  # Inter-stream extrinsics. depth->colour is what `rs2::align` uses,\n"
      << "  # and the pipeline relies on it rather than calibrating it locally.\n";
  out << "  extrinsics:\n";

  std::vector<rs2::stream_profile> anchors;
  for (const rs2::sensor& sensor : device.query_sensors()) {
    for (const rs2::stream_profile& profile : sensor.get_stream_profiles()) {
      const auto video = profile.as<rs2::video_stream_profile>();
      if (!video) continue;
      const bool have = std::any_of(anchors.begin(), anchors.end(),
                                    [&](const rs2::stream_profile& p) {
                                      return p.stream_type() == video.stream_type() &&
                                             p.stream_index() == video.stream_index();
                                    });
      if (!have) anchors.push_back(profile);
    }
  }

  for (const rs2::stream_profile& from : anchors) {
    for (const rs2::stream_profile& to : anchors) {
      if (from.stream_type() == to.stream_type() && from.stream_index() == to.stream_index()) {
        continue;
      }
      try {
        const rs2_extrinsics e = from.get_extrinsics_to(to);
        out << "    - from: " << StreamName(from.stream_type(), from.stream_index()) << "\n"
            << "      to: " << StreamName(to.stream_type(), to.stream_index()) << "\n"
            << std::setprecision(10) << "      rotation: [";
        for (int i = 0; i < 9; ++i) out << (i ? ", " : "") << e.rotation[i];
        out << "]\n      translation: [";
        for (int i = 0; i < 3; ++i) out << (i ? ", " : "") << e.translation[i];
        out << "]\n";
      } catch (const rs2::error&) {
        // Not every stream pair has a defined transform; skip quietly.
      }
    }
  }
}

int BackupAll(const std::string& out_dir) {
  std::filesystem::create_directories(out_dir);
  rs2::context context;
  const rs2::device_list devices = context.query_devices();
  if (devices.size() == 0) {
    std::cerr << "no RealSense devices attached\n";
    return EXIT_FAILURE;
  }

  int failures = 0;
  for (const rs2::device& device : devices) {
    const std::string serial = Info(device, RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::cout << "=== " << serial << " (" << Info(device, RS2_CAMERA_INFO_NAME) << ") ===\n";

    std::vector<uint8_t> table;
    try {
      auto calibrated = device.as<rs2::auto_calibrated_device>();
      if (!calibrated) throw std::runtime_error("device is not auto-calibratable");
      table = calibrated.get_calibration_table();
    } catch (const std::exception& e) {
      std::cerr << "  FAILED to read calibration table: " << e.what() << "\n";
      ++failures;
      continue;
    }

    const std::string bin_path = out_dir + "/" + serial + "_calibration_table.bin";
    std::ofstream bin(bin_path, std::ios::binary);
    bin.write(reinterpret_cast<const char*>(table.data()),
              static_cast<std::streamsize>(table.size()));
    bin.close();
    const std::string fingerprint = Fingerprint(table);
    std::cout << "  calibration table : " << table.size() << " bytes, fnv1a "
              << fingerprint << "\n  -> " << bin_path << "\n";

    const std::string yaml_path = out_dir + "/" + serial + "_calibration.yaml";
    std::ofstream yaml(yaml_path);
    yaml << "# On-device calibration backup -- " << serial << "\n"
         << "# Generated by SystemCode/camera/tools/backup_calibration.cpp\n"
         << "#\n"
         << "# The RESTORABLE artefact is the .bin beside this file. This YAML is the\n"
         << "# readable record: intrinsics and inter-stream extrinsics as the device\n"
         << "# reports them, for spotting drift and for documenting the rig.\n"
         << "device:\n"
         << "  serial: \"" << serial << "\"\n"
         << "  name: \"" << Info(device, RS2_CAMERA_INFO_NAME) << "\"\n"
         << "  firmware: \"" << Info(device, RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\"\n"
         << "  product_line: \"" << Info(device, RS2_CAMERA_INFO_PRODUCT_LINE) << "\"\n"
         << "  usb_type: \"" << Info(device, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << "\"\n"
         << "calibration_table:\n"
         << "  file: \"" << serial << "_calibration_table.bin\"\n"
         << "  bytes: " << table.size() << "\n"
         << "  fnv1a: \"" << fingerprint << "\"\n";
    DumpIntrinsics(yaml, device);
    DumpExtrinsics(yaml, device);
    yaml.close();
    std::cout << "  -> " << yaml_path << "\n\n";
  }
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

int Restore(const std::string& serial, const std::string& table_path, bool confirmed) {
  std::ifstream in(table_path, std::ios::binary);
  if (!in) {
    std::cerr << "cannot read " << table_path << "\n";
    return EXIT_FAILURE;
  }
  const std::vector<uint8_t> table((std::istreambuf_iterator<char>(in)),
                                   std::istreambuf_iterator<char>());
  std::cout << "table: " << table.size() << " bytes, fnv1a " << Fingerprint(table) << "\n";
  if (table.empty()) {
    std::cerr << "refusing to write an empty table\n";
    return EXIT_FAILURE;
  }

  if (!confirmed) {
    std::cerr << "\nThis writes to the camera's EEPROM and cannot be undone by this tool.\n"
              << "Before doing that, consider the safer option: the device keeps its own\n"
              << "factory copy, and librealsense can restore it with\n"
              << "reset_to_factory_calibration() -- try `rs-enumerate-devices` tooling or\n"
              << "the Intel calibration utility first.\n"
              << "\nRe-run with --yes-write-eeprom if you are sure.\n";
    return EXIT_FAILURE;
  }

  rs2::context context;
  for (const rs2::device& device : context.query_devices()) {
    if (Info(device, RS2_CAMERA_INFO_SERIAL_NUMBER) != serial) continue;
    auto calibrated = device.as<rs2::auto_calibrated_device>();
    if (!calibrated) {
      std::cerr << "device is not auto-calibratable\n";
      return EXIT_FAILURE;
    }
    calibrated.set_calibration_table(table);
    calibrated.write_calibration();
    std::cout << "restored and written to EEPROM on " << serial << "\n";
    return EXIT_SUCCESS;
  }
  std::cerr << "serial " << serial << " is not attached\n";
  return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage:\n"
              << "  backup_calibration <out_dir>\n"
              << "  backup_calibration --restore <serial> <table.bin> [--yes-write-eeprom]\n";
    return EXIT_FAILURE;
  }
  try {
    if (std::string(argv[1]) == "--restore") {
      if (argc < 4) {
        std::cerr << "--restore needs <serial> <table.bin>\n";
        return EXIT_FAILURE;
      }
      bool confirmed = false;
      for (int i = 4; i < argc; ++i) {
        if (std::string(argv[i]) == "--yes-write-eeprom") confirmed = true;
      }
      return Restore(argv[2], argv[3], confirmed);
    }
    return BackupAll(argv[1]);
  } catch (const std::exception& e) {
    std::cerr << "backup_calibration: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
