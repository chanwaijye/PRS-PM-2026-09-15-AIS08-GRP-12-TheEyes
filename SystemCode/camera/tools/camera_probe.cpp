// camera_agent bring-up probe.
//
// Checks the acceptance criteria in AGENTS.md §1 that can be checked
// without a measured target:
//   - both configured serials attached, and opened BY SERIAL
//   - both streaming aligned depth + colour simultaneously at the target rate
//   - cross-camera timestamp skew within one frame period
//   - inter_cam_sync_mode actually taking effect on the device
//
// Usage: camera_probe <cameras.yaml> [frames]
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include "the_eyes/camera/device_registry.hpp"
#include "the_eyes/camera/realsense_camera.hpp"
#include "the_eyes/camera/sync.hpp"

int main(int argc, char** argv) {
  using namespace the_eyes::camera;

  const std::string config_path = (argc > 1) ? argv[1] : "config/cameras.yaml";
  const int frames = (argc > 2) ? std::atoi(argv[2]) : 60;

  try {
    const CameraAgentConfig config = LoadCameraAgentConfig(config_path);

    std::cout << "=== device discovery (by serial, never by index) ===\n";
    const RegistryCheck check = CheckDevices(config);
    std::cout << check.Format() << "\n";
    if (!check.ok) return EXIT_FAILURE;

    std::cout << "=== opening cameras ===\n";
    std::map<std::string, std::unique_ptr<RealSenseCamera>> cameras;
    for (const auto& [camera_id, entry] : config.cameras) {
      cameras[camera_id] = std::make_unique<RealSenseCamera>(entry, config.streams);
      const RealSenseCamera& camera = *cameras[camera_id];
      std::cout << "  " << camera_id << " -> serial " << camera.serial()
                << ", depth scale " << camera.depth_scale_m() << " m/unit"
                << ", sync mode " << camera.actual_sync_mode()
                << " (requested " << entry.inter_cam_sync_mode << ")\n";
      for (const std::string& warning : camera.warnings()) {
        std::cout << "    WARNING: " << warning << "\n";
      }
    }

    // Discard a warm-up burst before measuring. The two pipelines do not start
    // at the same instant, so the first frames pair across a large gap and a
    // max-skew taken from them measures start-up ordering, not synchronisation.
    constexpr int kWarmupFrames = 15;
    std::cout << "\n=== warm-up (" << kWarmupFrames << " frames, discarded) ===\n";
    for (int i = 0; i < kWarmupFrames; ++i) {
      for (auto& [camera_id, camera] : cameras) camera->NextFrame();
    }

    std::cout << "\n=== streaming " << frames << " frames from each camera ===\n";
    std::map<std::string, std::vector<uint64_t>> stamps;
    std::map<std::string, int> received;
    const auto start = std::chrono::steady_clock::now();

    for (int i = 0; i < frames; ++i) {
      for (auto& [camera_id, camera] : cameras) {
        const CameraFrame frame = camera->NextFrame();
        if (!frame.valid()) {
          std::cerr << camera_id << ": invalid frame at " << i << "\n";
          return EXIT_FAILURE;
        }
        stamps[camera_id].push_back(frame.stamp_ns);
        ++received[camera_id];
        if (i == 0) {
          std::cout << "  " << camera_id << ": colour " << frame.rgb.cols << "x"
                    << frame.rgb.rows << ", depth " << frame.depth.cols << "x"
                    << frame.depth.rows << " (aligned), domain "
                    << camera->timestamp_domain() << "\n";
        }
      }
    }
    const double elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    std::cout << "\n=== throughput ===\n";
    bool rate_ok = true;
    for (const auto& [camera_id, count] : received) {
      const double fps = count / elapsed_s;
      // The acceptance criterion is both cameras simultaneously at >= 15 fps.
      const bool ok = fps >= 15.0;
      rate_ok = rate_ok && ok;
      std::cout << "  " << std::left << std::setw(12) << camera_id << std::fixed
                << std::setprecision(1) << fps << " fps  " << (ok ? "OK" : "BELOW 15 fps")
                << "\n";
    }

    std::cout << "\n=== cross-camera timestamp skew ===\n";
    if (cameras.size() >= 2) {
      auto it = stamps.begin();
      const std::string& a = it->first;
      const std::vector<uint64_t>& stamps_a = it->second;
      ++it;
      const std::string& b = it->first;
      const std::vector<uint64_t>& stamps_b = it->second;

      const SyncReport report =
          AnalyseSkew(PairNearest(stamps_a, stamps_b), a, b, config.sync.max_skew_ms);
      std::cout << "  " << report.camera_a << " vs " << report.camera_b << ", " << report.samples
                << " samples\n"
                << std::setprecision(2)
                << "  mean |skew|   : " << report.mean_skew_ms << " ms\n"
                << "  p95 |skew|    : " << report.p95_skew_ms << " ms\n"
                << "  max |skew|    : " << report.max_skew_ms << " ms\n"
                << "  mean signed   : " << report.mean_signed_skew_ms << " ms  (fixed offset)\n"
                << "  JITTER (sigma): " << report.jitter_ms << " ms  <-- the sync metric\n"
                << "  outliers      : " << report.outliers << " / " << report.samples << "\n"
                << "  verdict       : " << report.verdict << "\n";
      if (!report.within_threshold || !rate_ok) return EXIT_FAILURE;
    }

    std::cout << "\nRESULT: PASS\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "camera_probe: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
