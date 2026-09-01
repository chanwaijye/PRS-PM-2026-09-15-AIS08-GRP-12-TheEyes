// Depth-noise characterisation -- a DELIVERABLE, not a debugging step.
//
// AGENTS.md §1: flat wall at 0.4 / 0.6 / 0.8 / 1.0 m, plane-fit RMS per
// distance. The result is the measured input to the evaluation error budget: it
// is what says whether a fused-pose error is sensor noise or an algorithm
// problem.
//
// Point the camera squarely at a flat, textured, matte wall and run once per
// distance:
//   depth_characterise <cameras.yaml> <camera_id> --distance 0.4
//
// Or measure whatever is in front of the camera as a smoke test:
//   depth_characterise <cameras.yaml> <camera_id> --scene
#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <vector>
#include <iostream>
#include <string>

#include "the_eyes/camera/depth_noise.hpp"
#include "the_eyes/camera/realsense_camera.hpp"

int main(int argc, char** argv) {
  using namespace the_eyes::camera;

  if (argc < 3) {
    std::cerr << "usage: depth_characterise <cameras.yaml> <camera_id> "
                 "[--distance <m> | --scene] [--roi <fraction>]\n"
                 "\n  --roi  override the ROI fraction. Use it to keep the PHYSICAL patch\n"
                 "         size constant across distances, so RMS is comparable:\n"
                 "           0.4 m -> --roi 0.50    0.7 m -> --roi 0.29    1.0 m -> --roi 0.20\n";
    return EXIT_FAILURE;
  }
  const std::string config_path = argv[1];
  const std::string camera_id = argv[2];
  double nominal = 0.0;
  bool scene_mode = false;
  double roi_override = 0.0;
  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--distance" && i + 1 < argc) nominal = std::atof(argv[++i]);
    if (arg == "--roi" && i + 1 < argc) roi_override = std::atof(argv[++i]);
    if (arg == "--scene") scene_mode = true;
  }
  if (nominal <= 0.0 && !scene_mode) {
    std::cerr << "give --distance <m> for a measured wall, or --scene for a smoke test\n";
    return EXIT_FAILURE;
  }

  try {
    CameraAgentConfig config = LoadCameraAgentConfig(config_path);
    if (roi_override > 0.0) config.noise.roi_fraction = roi_override;
    const CameraEntry& entry = config.at(camera_id);

    // A FIXED pixel ROI covers a physical area that grows linearly with
    // distance -- 28x16 cm at 0.4 m but 70x40 cm at 1.0 m for roi 0.5. If the
    // flat surface is smaller than that, the far measurements silently include
    // edges and background, and the RMS reads as sensor degradation when it is
    // really a shrinking target. Hold the patch constant with --roi.
    std::cout << "ROI fraction " << config.noise.roi_fraction;
    if (nominal > 0.0) {
      std::cout << "  (~" << static_cast<int>(848 * config.noise.roi_fraction * nominal / 605.0
                                              * 100)
                << " x "
                << static_cast<int>(480 * config.noise.roi_fraction * nominal / 605.0 * 100)
                << " cm of wall at " << nominal << " m)";
    }
    std::cout << "\n";
    RealSenseCamera camera(entry, config.streams);

    DepthNoiseReport report;
    report.camera_id = camera_id;
    report.serial = camera.serial();

    // Auto-exposure and the depth filters need a few frames to settle; measuring
    // immediately reports the transient, not the sensor.
    for (int i = 0; i < 15; ++i) camera.NextFrame();

    double rms_sum = 0.0;
    double distance_sum = 0.0;
    double median_sum = 0.0;
    double valid_sum = 0.0;
    int points_last = 0;
    int used = 0;

    for (int i = 0; i < config.noise.frames_per_distance; ++i) {
      const CameraFrame frame = camera.NextFrame();
      const auto points =
          DepthToPoints(frame.depth, frame.K, frame.depth_scale_m, config.noise.roi_fraction);
      if (points.size() < 100) continue;

      const PlaneFit fit = FitPlane(points);
      if (!fit.valid) continue;

      const int roi_pixels =
          static_cast<int>(frame.depth.cols * config.noise.roi_fraction) *
          static_cast<int>(frame.depth.rows * config.noise.roi_fraction);
      // Median point depth, independent of the plane fit. When the scene is not
      // a wall the two disagree, and that disagreement is the signal.
      std::vector<double> depths;
      depths.reserve(points.size());
      for (const Eigen::Vector3d& q : points) depths.push_back(q.z());
      std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
      median_sum += depths[depths.size() / 2];

      rms_sum += fit.rms_error_m;
      distance_sum += std::abs(fit.offset_m);
      valid_sum += static_cast<double>(points.size()) / std::max(1, roi_pixels);
      points_last = fit.points_used;
      ++used;
    }

    if (used == 0) {
      std::cerr << "no usable frames -- is anything in the depth range?\n";
      return EXIT_FAILURE;
    }

    DepthNoiseMeasurement measurement;
    measurement.nominal_distance_m = scene_mode ? median_sum / used : nominal;
    measurement.measured_distance_m = distance_sum / used;
    measurement.median_depth_m = median_sum / used;
    measurement.rms_error_m = rms_sum / used;
    measurement.valid_pixel_fraction = valid_sum / used;
    measurement.frames = used;
    measurement.points_per_frame = points_last;
    report.measurements.push_back(measurement);

    std::cout << report.Format();
    if (scene_mode) {
      std::cout << "\nSCENE MODE: this measured whatever was in front of the camera, which is\n"
                << "almost certainly not a flat wall. The RMS below is an upper bound on\n"
                << "sensor noise, not a characterisation. Use --distance against a measured\n"
                << "flat wall for the reportable figure.\n";
    }
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "depth_characterise: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
