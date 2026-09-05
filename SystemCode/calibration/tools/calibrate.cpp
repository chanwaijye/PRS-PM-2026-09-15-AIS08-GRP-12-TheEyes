// calib_agent CLI.
//
//   calibrate board  <out.png>                    render the board for printing
//   calibrate capture <cameras.yaml> <cfg> <out_dir>   shoot the calibration frames
//   calibrate mock   <out.yaml>                   full run on synthetic data
//   calibrate run    <cfg> <front_dir> <front_serial> <over_dir> <over_serial> <out.yaml>
//   calibrate reregister <cfg> <prev.yaml> <out.yaml> <id>:<serial>:<image> ...
//   calibrate reregister-mock                     re-registration on synthetic poses
//
// `mock` and `reregister-mock` are the ones that run without hardware: they
// exercise the entire pipeline -- intrinsics, registration, round trip, report,
// writer -- with no cameras.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "the_eyes/calibration/report.hpp"
#include "the_eyes/calibration/reregister.hpp"
#include "the_eyes/calibration/solver.hpp"
#include "the_eyes/calibration/synthetic.hpp"

#ifdef THE_EYES_WITH_ARUCO
#include "the_eyes/calibration/charuco_detector.hpp"
#include "the_eyes/camera/device_registry.hpp"
#include "the_eyes/fusion/extrinsics.hpp"
#endif

// `capture` is the one subcommand that needs a live camera as well as aruco.
#if defined(THE_EYES_WITH_ARUCO) && defined(THE_EYES_WITH_REALSENSE)
#define THE_EYES_CALIBRATE_CAN_CAPTURE 1
#include "the_eyes/camera/realsense_camera.hpp"
#endif

namespace {

using namespace the_eyes::calibration;

int RunMock(const std::string& out_path) {
  BoardSpec board;
  board.squares_x = 8;
  board.squares_y = 6;
  board.square_length_m = 0.030;
  board.marker_length_m = 0.022;

  CalibrationReport report;
  report.world_frame = "table_origin";
  report.thresholds = QualityThresholds{};

  // Two viewpoints on a board lying at the world origin, roughly matching the
  // intended rig: one across the table, one more overhead.
  struct Spec { const char* id; const char* serial; Eigen::Isometry3d T_cam_board; };
  // The board lies face-up on the table, so a camera looking at its front face
  // must be rotated ~180 deg about x relative to the board frame. Without that
  // flip the maths stays self-consistent but places both cameras BELOW the
  // table -- a good reminder that "the numbers are consistent" and "the setup
  // is physical" are different claims.
  const Eigen::Matrix3d face_up =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

  Eigen::Isometry3d front = Eigen::Isometry3d::Identity();
  front.linear() = Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitX()).toRotationMatrix() * face_up;
  front.translation() = Eigen::Vector3d(-0.10, -0.08, 0.60);
  Eigen::Isometry3d over = Eigen::Isometry3d::Identity();
  over.linear() = Eigen::AngleAxisd(-0.20, Eigen::Vector3d::UnitY()).toRotationMatrix() * face_up;
  over.translation() = Eigen::Vector3d(-0.09, -0.07, 0.75);

  const Spec specs[] = {{"cam_front", "SYNTHETIC-FRONT", front},
                        {"cam_over", "SYNTHETIC-OVER", over}};

  std::vector<BoardObservation> registrations;
  for (const Spec& spec : specs) {
    SyntheticCamera camera;
    camera.camera_id = spec.id;

    std::vector<BoardObservation> views;
    for (const Eigen::Isometry3d& pose : SampleBoardPoses(board, 40)) {
      BoardObservation observation = ProjectBoard(camera, board, pose);
      if (observation.size() >= 12) views.push_back(std::move(observation));
    }

    CameraCalibration calibration;
    calibration.camera_id = spec.id;
    calibration.serial = spec.serial;
    calibration.intrinsics = CalibrateIntrinsics(views, board);

    BoardObservation registration = ProjectBoard(camera, board, spec.T_cam_board);
    const PoseResult pose = EstimateBoardPose(registration, calibration.intrinsics, board);
    calibration.registration_rmse_px = pose.reprojection_rmse_px;
    calibration.T_world_cam = RegisterToWorld(pose.T_cam_board, Eigen::Isometry3d::Identity());

    registrations.push_back(std::move(registration));
    report.cameras.push_back(std::move(calibration));
  }

  report.round_trip =
      ComputeRoundTrip(registrations[0], report.cameras[0], registrations[1], report.cameras[1]);
  Evaluate(&report);

  std::cout << report.Format();
  std::cout << "\nSYNTHETIC INPUT -- these are not calibration results for the real rig.\n";
  WriteExtrinsicsYaml(report, out_path);
  std::cout << "wrote " << out_path << "\n";
  return report.passed() ? EXIT_SUCCESS : EXIT_FAILURE;
}


// Re-registration on synthetic poses: the table is nudged, and in the second
// scenario a tripod is bumped as well. No cameras, no images.
int RunReRegisterMock() {
  const std::vector<std::string> ids{"cam_front", "cam_over"};
  auto rig_camera = [](double azimuth_deg) {
    const double d = 0.70, elevation = 35.0 * M_PI / 180.0;
    const double a = azimuth_deg * M_PI / 180.0, r = d * std::cos(elevation);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(r * std::sin(a), -r * std::cos(a), d * std::sin(elevation));
    const Eigen::Vector3d forward = -T.translation().normalized();
    const Eigen::Vector3d right = forward.cross(Eigen::Vector3d::UnitZ()).normalized();
    Eigen::Matrix3d R;
    R.col(0) = right;
    R.col(1) = forward.cross(right);
    R.col(2) = forward;
    T.linear() = R;
    return T;
  };
  const std::vector<Eigen::Isometry3d> before{rig_camera(-40.0), rig_camera(40.0)};

  Eigen::Isometry3d table = Eigen::Isometry3d::Identity();
  table.translation() = Eigen::Vector3d(0.011, -0.007, 0.0);
  std::vector<Eigen::Isometry3d> moved;
  for (const Eigen::Isometry3d& T : before) moved.push_back(table.inverse() * T);

  std::cout << "--- scenario 1: the desk was nudged, the tripods were not ---\n";
  std::cout << CheckReRegistration(ids, before, moved, RigidityThresholds{}).Format();

  std::vector<Eigen::Isometry3d> bumped = moved;
  bumped[1].linear() =
      Eigen::AngleAxisd(0.6 * M_PI / 180.0, Eigen::Vector3d::UnitX()).matrix() * bumped[1].linear();
  std::cout << "\n--- scenario 2: the desk was nudged AND cam_over was bumped ---\n";
  const ReRegistrationReport report = CheckReRegistration(ids, before, bumped,
                                                          RigidityThresholds{});
  std::cout << report.Format();
  std::cout << "\nSYNTHETIC INPUT -- not a result for the real rig.\n";
  return EXIT_SUCCESS;
}

#ifdef THE_EYES_WITH_ARUCO
std::vector<BoardObservation> DetectDirectory(const CharucoDetector& detector,
                                              const std::string& dir,
                                              const std::string& camera_id) {
  std::vector<std::string> paths;
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    const std::string ext = entry.path().extension().string();
    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") paths.push_back(entry.path().string());
  }
  std::sort(paths.begin(), paths.end());

  std::vector<BoardObservation> observations;
  for (size_t i = 0; i < paths.size(); ++i) {
    const cv::Mat image = cv::imread(paths[i], cv::IMREAD_COLOR);
    if (image.empty()) continue;
    BoardObservation observation = detector.Detect(image, camera_id, i);
    std::cout << "  " << paths[i] << ": " << observation.size() << " corners\n";
    if (observation.size() >= 12) observations.push_back(std::move(observation));
  }
  return observations;
}


// Turns "no usable views" into a message that names the cause.
//
// A board whose markers cannot decode at the capture resolution produces zero
// corners while looking perfectly fine to a human: in frame, lit, in focus.
// The fix is always the same -- capture at a higher resolution -- so say so
// before the detection loop rather than after it fails.
void ReportDecodeReach(const BoardSpec& board, double fx_px, int width, int height) {
  const double reach = board.MaxDecodeDistance(fx_px);
  std::cout << "  board: " << board.squares_x << "x" << board.squares_y << " @ "
            << board.square_length_m * 1000.0 << " mm, " << board.dictionary << " ("
            << board.marker_modules() << " modules/marker)\n";
  std::cout << "  markers decode out to " << reach << " m at " << width << "x" << height
            << " (fx " << fx_px << " px)\n";
  if (reach < 1.0) {
    std::cout << "  NOTE: that is short. The oblique far corner of a board on the table\n"
                 "        sits ~0.87 m from the camera on this rig, so capture the\n"
                 "        calibration frames at 1920x1080 -- the recovered pose is\n"
                 "        resolution-independent and stays valid for 848x480 streaming.\n";
  }
}

// Size of the first image in a capture directory, for the up-front decode
// advisory. Nullopt if the directory holds none -- the detection loop reports
// that case with a better message than this could.
std::optional<cv::Size> FirstImageSize(const std::string& dir) {
  std::vector<std::string> paths;
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    const std::string ext = entry.path().extension().string();
    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") paths.push_back(entry.path().string());
  }
  if (paths.empty()) return std::nullopt;
  std::sort(paths.begin(), paths.end());
  const cv::Mat image = cv::imread(paths.front(), cv::IMREAD_COLOR);
  if (image.empty()) return std::nullopt;
  return image.size();
}

#ifdef THE_EYES_CALIBRATE_CAN_CAPTURE

struct CaptureOptions {
  // 1920x1080, not the 848x480 the pipeline streams. A DICT_5X5 marker needs
  // ~3 px/module to decode and this rig's board sits 0.88 m away at its far
  // corner, which is 2.8 px/module at 848x480 -- zero corners from images that
  // look perfectly good. See ReportDecodeReach. The recovered POSE is
  // resolution-independent, so these captures stay valid for 848x480 streaming.
  int width = 1920;
  int height = 1080;
  int fps = 30;
  int auto_count = 0;
  double interval_s = 2.0;
  bool registration = false;
  std::string prefix = "view";
};

struct CaptureCamera {
  std::string camera_id;
  std::string serial;
  std::filesystem::path dir;
  std::unique_ptr<the_eyes::camera::RealSenseCamera> camera;
};

// One frame from every camera, as close together as the pipelines allow.
//
// The pipeline keeps the most recent frame, but "most recent" can still be one
// captured while the operator was reaching for the keyboard, so a short burst
// is discarded first. That drain is INTERLEAVED across cameras and the kept
// frames are taken back to back: draining one camera fully and then the other
// separates the two saved frames by the whole drain -- ~200 ms at 30 fps, which
// then shows up as a capture skew six frame periods wide that is an artefact of
// this loop rather than anything about the rig.
std::vector<the_eyes::camera::CameraFrame> GrabPair(std::vector<CaptureCamera>& cameras,
                                                    int discard = 5) {
  for (int i = 0; i < discard; ++i) {
    for (CaptureCamera& entry : cameras) entry.camera->NextFrame();
  }
  std::vector<the_eyes::camera::CameraFrame> frames;
  frames.reserve(cameras.size());
  for (CaptureCamera& entry : cameras) frames.push_back(entry.camera->NextFrame());
  return frames;
}

// Laplacian variance over the board's bounding box. A RELATIVE figure, not a
// calibrated one -- it is only compared against the best frame of the same
// session, on the same board, at the same resolution.
//
// Motion blur is the silent failure of a handheld sweep: a blurred board still
// decodes, still reports a healthy corner count, and still contributes badly
// localised points that raise the reprojection RMSE with no indication of
// which frames caused it.
double BoardSharpness(const cv::Mat& bgr, const BoardObservation& observation) {
  if (observation.image_points.empty()) return 0.0;
  cv::Rect box = cv::boundingRect(observation.image_points);
  box &= cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (box.width < 16 || box.height < 16) return 0.0;
  cv::Mat grey, laplacian;
  cv::cvtColor(bgr(box), grey, cv::COLOR_BGR2GRAY);
  cv::Laplacian(grey, laplacian, CV_64F);
  cv::Scalar mean, stddev;
  cv::meanStdDev(laplacian, mean, stddev);
  return stddev[0] * stddev[0];
}

// Highest existing <prefix>NN in a capture directory, plus one. Re-running the
// tool against a directory that already holds a sweep must extend it, not
// overwrite it -- a clobbered capture set is not recoverable without re-setting
// the board.
int NextIndex(const std::filesystem::path& dir, const std::string& prefix) {
  int next = 0;
  if (!std::filesystem::exists(dir)) return next;
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    const std::string stem = entry.path().stem().string();
    if (stem.size() <= prefix.size() || stem.compare(0, prefix.size(), prefix) != 0) continue;
    try {
      next = std::max(next, std::stoi(stem.substr(prefix.size())) + 1);
    } catch (const std::exception&) {
      // Not a numbered capture. zz_register.png lands here, which is correct:
      // it must not influence the sweep numbering.
    }
  }
  return next;
}

std::string IndexedName(const std::string& prefix, int index) {
  std::ostringstream os;
  os << prefix << std::setfill('0') << std::setw(2) << index << ".png";
  return os.str();
}

// One shot from every camera, saved and immediately detected.
//
// Detecting here rather than at `run` time is the entire point of the tool: a
// capture set that yields no corners looks identical to a good one until an
// hour later, by which time the board, the lighting and the operator have all
// moved.
bool CapturePair(std::vector<CaptureCamera>& cameras, const CharucoDetector& detector,
                 const std::string& filename, const QualityThresholds& quality,
                 std::map<std::string, double>* best_sharpness) {
  const std::vector<the_eyes::camera::CameraFrame> frames = GrabPair(cameras);

  std::vector<BoardObservation> observations;
  bool all_usable = true;

  for (size_t i = 0; i < cameras.size(); ++i) {
    // librealsense hands back RS2_FORMAT_RGB8; cv::imwrite expects BGR. Writing
    // the buffer straight out swaps the red and blue channels -- harmless to
    // greyscale corner detection, and quietly wrong for anything that later
    // looks at these files.
    cv::Mat bgr;
    cv::cvtColor(frames[i].rgb, bgr, cv::COLOR_RGB2BGR);

    const std::filesystem::path path = cameras[i].dir / filename;
    // PNG, not JPEG. JPEG's 8x8 block artefacts move sub-pixel corners, and
    // sub-pixel corners are the whole product of this capture.
    if (!cv::imwrite(path.string(), bgr, {cv::IMWRITE_PNG_COMPRESSION, 3})) {
      throw std::runtime_error("cannot write '" + path.string() + "'");
    }

    BoardObservation observation = detector.Detect(bgr, cameras[i].camera_id, frames[i].stamp_ns);
    const double sharpness = BoardSharpness(bgr, observation);
    double& best = (*best_sharpness)[cameras[i].camera_id];
    best = std::max(best, sharpness);

    std::cout << "  " << std::left << std::setw(10) << cameras[i].camera_id << std::right
              << std::setw(3) << observation.size() << " corners"
              << "  sharpness " << std::fixed << std::setprecision(0) << std::setw(6)
              << sharpness;
    if (observation.size() < static_cast<size_t>(quality.min_shared_corners)) {
      std::cout << "  TOO FEW (need " << quality.min_shared_corners << ")";
      all_usable = false;
    } else if (best > 0.0 && sharpness < 0.5 * best) {
      // Half the best frame of this session is a large drop for a static board
      // under fixed lighting; the usual cause is the board still moving.
      std::cout << "  BLURRED vs this session's best (" << best << ")";
      all_usable = false;
    }
    std::cout << "  -> " << path.string() << "\n";
    observations.push_back(std::move(observation));
  }

  if (cameras.size() >= 2) {
    const std::vector<int> shared = SharedCorners(observations[0], observations[1]);
    const double skew_ms = (static_cast<double>(frames[0].stamp_ns) -
                            static_cast<double>(frames[1].stamp_ns)) / 1e6;
    std::cout << "  shared " << shared.size() << " corners, capture skew " << std::setprecision(1)
              << skew_ms << " ms";
    if (static_cast<int>(shared.size()) < quality.min_shared_corners) {
      std::cout << "  <-- below min_shared_corners " << quality.min_shared_corners;
      all_usable = false;
    }
    std::cout << "\n";
  }
  return all_usable;
}

// Shoots the calibration frames both cameras need, at a resolution the board
// can actually be decoded at.
//
// Two kinds of capture, and they are not interchangeable:
//   the SWEEP        board handheld at varied distance and tilt, each camera
//                    independent. Insurance -- with --factory the solver never
//                    reads these, but they are the only fallback if the
//                    factory intrinsics turn out not to be good enough.
//   the REGISTRATION board at its measured world position, both cameras
//                    simultaneously, board untouched between the two. This is
//                    the shot every camera pose is derived from.
//
// `run` takes the alphabetically LAST image in each directory as the
// registration view, so it is written as zz_register.png.
// Live aiming view. Positioning tripods by eye and only finding out at `run`
// time that the two cameras never shared enough of the board is the expensive
// failure this exists to prevent -- by then the board, the lighting and the
// operator have all moved.
//
// It draws the SAME numbers `capture` gates on, live: per-camera corner count,
// board sharpness, and the shared-corner count across the pair. Aim until the
// banner is green, then shoot.
//
// Deliberately NOT a capture path -- it writes nothing. Same colour-only 1080p
// profile as `capture`, so what you aim at is what you will shoot.
int RunAim(const std::string& cameras_path, const std::string& config_path) {
  const BoardSpec board = LoadBoardSpec(config_path);
  const QualityThresholds quality = LoadQualityThresholds(config_path);
  const CharucoDetector detector(board);

  the_eyes::camera::CameraAgentConfig config =
      the_eyes::camera::LoadCameraAgentConfig(cameras_path);
  if (config.cameras.size() != 2) {
    throw std::runtime_error("aim expects exactly two cameras in " + cameras_path);
  }
  config.streams.color_width = 1920;
  config.streams.color_height = 1080;
  config.streams.fps = 30;
  config.streams.color_only = true;   // same hardware limit as `capture`

  std::vector<CaptureCamera> cameras;
  for (const auto& entry : config.cameras) {
    CaptureCamera cam;
    cam.camera_id = entry.first;
    cam.serial = entry.second.serial;
    cam.camera =
        std::make_unique<the_eyes::camera::RealSenseCamera>(entry.second, config.streams);
    std::cout << "opened " << cam.camera_id << " serial " << cam.camera->serial() << "\n";
    cameras.push_back(std::move(cam));
  }

  // Same reason as `capture`: auto-exposure needs frames to settle, and aiming
  // against the transient shows corner counts the settled stream will not.
  std::cout << "warming up...\n";
  for (int i = 0; i < 15; ++i) {
    for (CaptureCamera& entry : cameras) entry.camera->NextFrame();
  }

  std::cout << "\naim: live view at 1920x1080, colour only. Nothing is written.\n"
            << "  move the tripods until BOTH corner counts and the shared count are green\n"
            << "  q or ESC to quit\n\n";

  const std::string window = "calibrate aim -- shared corners is the number that matters";
  cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
  const int kPanelW = 860;
  std::map<std::string, double> best_sharpness;

  for (;;) {
    // discard=0: latency matters more than pair simultaneity here, because
    // nothing is being measured off these frames.
    std::vector<the_eyes::camera::CameraFrame> frames = GrabPair(cameras, 0);
    std::vector<BoardObservation> observations;
    std::vector<cv::Mat> panels;

    for (size_t i = 0; i < cameras.size(); ++i) {
      cv::Mat bgr;
      cv::cvtColor(frames[i].rgb, bgr, cv::COLOR_RGB2BGR);
      BoardObservation obs = detector.Detect(bgr, cameras[i].camera_id, frames[i].stamp_ns);
      const double sharp = BoardSharpness(bgr, obs);
      double& best = best_sharpness[cameras[i].camera_id];
      best = std::max(best, sharp);

      for (const cv::Point2f& pt : obs.image_points) {
        cv::circle(bgr, pt, 7, cv::Scalar(0, 220, 255), 2);
      }
      if (!obs.image_points.empty()) {
        cv::rectangle(bgr, cv::boundingRect(obs.image_points), cv::Scalar(0, 220, 255), 2);
      }

      cv::Mat panel;
      const double k = static_cast<double>(kPanelW) / bgr.cols;
      cv::resize(bgr, panel, cv::Size(), k, k, cv::INTER_AREA);

      const bool corners_ok = static_cast<int>(obs.size()) >= quality.min_shared_corners;
      // Half this session's best is the same blur heuristic `capture` gates on.
      const bool sharp_ok = best <= 0.0 || sharp >= 0.5 * best;
      const cv::Scalar colour = corners_ok && sharp_ok ? cv::Scalar(80, 220, 80)
                                                       : cv::Scalar(60, 60, 235);
      cv::rectangle(panel, cv::Rect(0, 0, panel.cols, 64), cv::Scalar(30, 30, 30), cv::FILLED);
      std::ostringstream line;
      line << cameras[i].camera_id << "   " << obs.size() << " corners";
      if (!corners_ok) line << " (need " << quality.min_shared_corners << ")";
      line << "   sharp " << std::fixed << std::setprecision(0) << sharp;
      if (!sharp_ok) line << " BLURRED";
      cv::putText(panel, line.str(), cv::Point(12, 42), cv::FONT_HERSHEY_SIMPLEX, 0.85,
                  colour, 2);
      cv::rectangle(panel, cv::Rect(0, 0, panel.cols, panel.rows), colour, 3);
      panels.push_back(panel);
      observations.push_back(std::move(obs));
    }

    cv::Mat view;
    cv::hconcat(panels, view);

    const std::vector<int> shared = SharedCorners(observations[0], observations[1]);
    const bool shared_ok = static_cast<int>(shared.size()) >= quality.min_shared_corners;
    const cv::Scalar banner = shared_ok ? cv::Scalar(80, 220, 80) : cv::Scalar(60, 60, 235);
    cv::Mat strip(70, view.cols, view.type(), cv::Scalar(30, 30, 30));
    std::ostringstream msg;
    msg << (shared_ok ? "READY   " : "NOT READY   ") << "shared corners " << shared.size()
        << " / " << quality.min_shared_corners << " needed";
    cv::putText(strip, msg.str(), cv::Point(16, 48), cv::FONT_HERSHEY_SIMPLEX, 1.0, banner, 2);
    cv::vconcat(view, strip, view);

    cv::imshow(window, view);
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
  }
  cv::destroyAllWindows();
  return EXIT_SUCCESS;
}

int RunCapture(const std::string& cameras_path, const std::string& config_path,
               const std::string& out_dir, const CaptureOptions& options) {
  const BoardSpec board = LoadBoardSpec(config_path);
  const QualityThresholds quality = LoadQualityThresholds(config_path);
  const CharucoDetector detector(board);

  the_eyes::camera::CameraAgentConfig config =
      the_eyes::camera::LoadCameraAgentConfig(cameras_path);
  if (config.cameras.size() != 2) {
    throw std::runtime_error("capture expects exactly two cameras in " + cameras_path +
                             " -- the registration view has to be shot by both at once, and "
                             "one camera has no baseline to check against.");
  }
  // What the pipeline normally streams, kept for the message below before it is
  // overwritten -- the point of this tool is that the two differ.
  const int stream_width = config.streams.color_width;
  const int stream_height = config.streams.color_height;

  config.streams.color_width = options.width;
  config.streams.color_height = options.height;
  config.streams.fps = options.fps;
  // Colour only, and this is not an optimisation. MEASURED on this rig: two
  // D435i units cannot both run 1920x1080 colour AND 848x480 depth at once --
  // the second pipeline.start() blocks forever, at 30 fps and at 15. Colour
  // only, both start in ~0.2 s. ChArUco corners come from the colour sensor, so
  // depth was never going to be read here anyway.
  config.streams.color_only = true;

  std::cout << "capturing at " << options.width << "x" << options.height << " @" << options.fps
            << " (pipeline streams " << stream_width << "x" << stream_height
            << " -- the recovered pose is the same either way)\n\n";

  // Before opening anything: if the board cannot be decoded at this resolution,
  // every frame below is wasted and nothing in the output would say so.
  for (const auto& entry : config.cameras) {
    std::vector<double> ignored;
    const Eigen::Matrix3d K = the_eyes::camera::FactoryColorIntrinsics(
        entry.second.serial, options.width, options.height, &ignored);
    std::cout << entry.first << " (" << entry.second.serial << "):\n";
    ReportDecodeReach(board, K(0, 0), options.width, options.height);
  }
  std::cout << "\n";

  std::vector<CaptureCamera> cameras;
  for (const auto& entry : config.cameras) {
    CaptureCamera capture;
    capture.camera_id = entry.first;
    capture.serial = entry.second.serial;
    capture.dir = std::filesystem::path(out_dir) / entry.first;
    std::filesystem::create_directories(capture.dir);
    capture.camera =
        std::make_unique<the_eyes::camera::RealSenseCamera>(entry.second, config.streams);
    std::cout << "opened " << capture.camera_id << " serial " << capture.camera->serial()
              << ", sync mode " << capture.camera->actual_sync_mode() << " -> " << capture.dir
              << "\n";
    for (const std::string& warning : capture.camera->warnings()) {
      std::cout << "  WARNING: " << warning << "\n";
    }
    cameras.push_back(std::move(capture));
  }

  // Auto-exposure and the colour pipeline need frames to settle. Calibrating
  // off the transient gives darker, noisier corners for no reason.
  // Said plainly because the rig IS hardware-synced and this mode is not: the
  // sync trigger drives the depth sensor, which is closed here. The two colour
  // streams are free-running, and the per-pair skew printed below is real. It
  // does not matter for a board that is not moving -- which is the registration
  // shot, the only one where the two cameras have to agree.
  std::cout << "\ncolour-only: the depth sensor is closed, so the hardware sync trigger is\n"
               "not driving these frames. Hold the board still for the registration shot.\n";
  std::cout << "\nwarming up...\n";
  for (int i = 0; i < 15; ++i) {
    for (CaptureCamera& entry : cameras) entry.camera->NextFrame();
  }

  std::map<std::string, double> best_sharpness;
  int saved = 0;

  if (options.registration) {
    std::cout << "\n=== registration view ===\n"
              << "Board at its measured world position, flat, pattern up, both cameras\n"
              << "seeing it. Do not touch it between now and the end of this shot.\n"
              << "Press ENTER when the board is set: ";
    std::string line;
    std::getline(std::cin, line);
    const bool ok = CapturePair(cameras, detector, "zz_register.png", quality, &best_sharpness);
    saved = 1;
    if (!ok) {
      std::cerr << "\nthe registration view is not usable. It is the shot every camera pose "
                   "is derived from -- reshoot it before running `calibrate run`.\n";
    }
  } else if (options.auto_count > 0) {
    std::cout << "\n=== sweep: " << options.auto_count << " frames every " << options.interval_s
              << " s ===\nMove the board between shots: vary distance, tilt, and where it "
                 "sits in the frame.\n";
    const int first = NextIndex(cameras.front().dir, options.prefix);
    for (int i = 0; i < options.auto_count; ++i) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(options.interval_s * 1000)));
      std::cout << "[" << (i + 1) << "/" << options.auto_count << "]\n";
      CapturePair(cameras, detector, IndexedName(options.prefix, first + i), quality,
                  &best_sharpness);
      ++saved;
    }
  } else {
    std::cout << "\n=== sweep: ENTER to shoot, q then ENTER to finish ===\n";
    int index = NextIndex(cameras.front().dir, options.prefix);
    while (true) {
      std::cout << "[" << IndexedName(options.prefix, index) << "] ";
      std::cout.flush();
      std::string line;
      if (!std::getline(std::cin, line)) break;
      if (line == "q" || line == "quit") break;
      CapturePair(cameras, detector, IndexedName(options.prefix, index), quality,
                  &best_sharpness);
      ++index;
      ++saved;
    }
  }

  std::cout << "\nsaved " << saved << " frame" << (saved == 1 ? "" : "s") << " per camera into "
            << out_dir << "\n";
  if (!options.registration) {
    std::cout << "The registration view is a SEPARATE shot -- rerun with --register once the "
                 "board is at its measured world position.\n";
  }
  std::cout << "\nthen:\n"
            << "  calibrate run " << config_path << " \\\n"
            << "      " << cameras[0].dir << " " << cameras[0].serial << " \\\n"
            << "      " << cameras[1].dir << " " << cameras[1].serial << " \\\n"
            << "      config/extrinsics.yaml --factory\n";
  return EXIT_SUCCESS;
}

#endif  // THE_EYES_CALIBRATE_CAN_CAPTURE

// `use_factory` skips the intrinsic sweep and reads the device's own
// calibration at the resolution of the captured images.
//
// This is what lets a small, cheap board work. Corner detection at 848x480 is
// resolution-starved -- a 300 mm board with 30 mm squares stops decoding past
// 0.63 m -- but captures taken at 1920x1080 decode out to 1.43 m. The camera
// POSE recovered from those captures is resolution-independent (same projection
// centre, differently sampled), so it stays valid when the pipeline streams at
// 848x480.
int RunReal(const std::string& config_path, const std::string& front_dir,
            const std::string& front_serial, const std::string& over_dir,
            const std::string& over_serial, const std::string& out_path,
            bool use_factory) {
  const BoardSpec board = LoadBoardSpec(config_path);
  const WorldConfig world = LoadWorldConfig(config_path);

  CalibrationReport report;
  report.world_frame = world.frame;
  report.thresholds = LoadQualityThresholds(config_path);

  const CharucoDetector detector(board);
  struct Input { const char* id; std::string dir; std::string serial; };
  const Input inputs[] = {{"cam_front", front_dir, front_serial},
                          {"cam_over", over_dir, over_serial}};

  // Peek one capture to learn the resolution and report the decode reach BEFORE
  // any detection runs. A board that cannot decode at this resolution yields
  // zero corners from images that look perfectly good, and the per-file "0
  // corners" lines below give no hint why.
  if (const auto first = FirstImageSize(front_dir)) {
    std::vector<double> ignored;
    const Eigen::Matrix3d K = the_eyes::camera::FactoryColorIntrinsics(
        front_serial, first->width, first->height, &ignored);
    ReportDecodeReach(board, K(0, 0), first->width, first->height);
    std::cout << "\n";
  }

  std::vector<BoardObservation> registrations;
  for (const Input& input : inputs) {
    std::cout << "detecting " << input.id << " in " << input.dir << "\n";
    const std::vector<BoardObservation> views =
        DetectDirectory(detector, input.dir, input.id);
    if (views.empty()) throw std::runtime_error("no usable views for " + std::string(input.id));

    CameraCalibration calibration;
    calibration.camera_id = input.id;
    calibration.serial = input.serial;
    if (use_factory) {
      const cv::Size size = views.front().image_size;
      std::vector<double> distortion;
      const Eigen::Matrix3d K = the_eyes::camera::FactoryColorIntrinsics(
          input.serial, size.width, size.height, &distortion);
      calibration.intrinsics = FactoryIntrinsics(K, size, distortion);
      std::cout << "  using FACTORY intrinsics at " << size.width << "x" << size.height
                << ": fx=" << K(0, 0) << " fy=" << K(1, 1) << " cx=" << K(0, 2)
                << " cy=" << K(1, 2) << "\n";
    } else {
      calibration.intrinsics =
          CalibrateIntrinsics(views, board, report.thresholds.min_views_for_intrinsics);
    }

    // The LAST capture is the registration view, by convention: both cameras
    // photograph the board at its measured world position as the final step,
    // after the free-form intrinsic sweep.
    const BoardObservation& registration = views.back();
    const PoseResult pose = EstimateBoardPose(registration, calibration.intrinsics, board);
    calibration.registration_rmse_px = pose.reprojection_rmse_px;
    calibration.T_world_cam = RegisterToWorld(pose.T_cam_board, world.T_world_board);

    registrations.push_back(registration);
    report.cameras.push_back(std::move(calibration));
  }

  report.round_trip =
      ComputeRoundTrip(registrations[0], report.cameras[0], registrations[1], report.cameras[1]);
  Evaluate(&report);
  std::cout << "\n" << report.Format();
  WriteExtrinsicsYaml(report, out_path);
  std::cout << "wrote " << out_path << "\n";
  return report.passed() ? EXIT_SUCCESS : EXIT_FAILURE;
}

// Session re-registration against a previous calibration.
//
// One board view per camera. Intrinsics come from the device, so this does NOT
// re-solve them -- which is also the caveat: if the previous calibration solved
// its own intrinsics rather than using --factory, the baseline comparison is
// between two different K matrices and can false-alarm. Calibrate with
// --factory if you intend to re-register.
int RunReRegister(const std::string& config_path, const std::string& previous_path,
                  const std::string& out_path,
                  const std::vector<std::string>& camera_specs) {
  const BoardSpec board = LoadBoardSpec(config_path);
  const WorldConfig world = LoadWorldConfig(config_path);
  const RigidityThresholds rigidity = LoadRigidityThresholds(config_path);

  // Read the previous extrinsics through fusion's own loader, so a file this
  // tool accepts is provably a file fusion will accept.
  const the_eyes::fusion::ExtrinsicsSet previous =
      the_eyes::fusion::LoadExtrinsics(previous_path);

  CalibrationReport report;
  report.world_frame = world.frame;
  report.thresholds = LoadQualityThresholds(config_path);

  const CharucoDetector detector(board);
  std::vector<std::string> ids;
  std::vector<Eigen::Isometry3d> before, after;
  std::vector<BoardObservation> registrations;

  for (const std::string& spec : camera_specs) {
    const size_t first = spec.find(':');
    const size_t second = spec.find(':', first + 1);
    if (first == std::string::npos || second == std::string::npos) {
      throw std::runtime_error("expected <camera_id>:<serial>:<image>, got '" + spec + "'");
    }
    const std::string id = spec.substr(0, first);
    const std::string serial = spec.substr(first + 1, second - first - 1);
    const std::string image_path = spec.substr(second + 1);

    const auto it = previous.cameras.find(id);
    if (it == previous.cameras.end()) {
      throw std::runtime_error("camera '" + id + "' is not in " + previous_path +
                               " -- re-registration updates an existing calibration, it "
                               "cannot introduce a new camera. Run a full calibration.");
    }
    // CLAUDE.md constraint 4: cameras are addressed by serial. A silent serial
    // swap would re-register the wrong camera's pose onto this id.
    if (it->second.serial != serial) {
      throw std::runtime_error("camera '" + id + "' is serial " + it->second.serial + " in " +
                               previous_path + " but " + serial +
                               " was supplied -- the cameras have been swapped.");
    }

    const cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) throw std::runtime_error("cannot read '" + image_path + "'");

    CameraCalibration calibration;
    calibration.camera_id = id;
    calibration.serial = serial;
    std::vector<double> distortion;
    const Eigen::Matrix3d K = the_eyes::camera::FactoryColorIntrinsics(
        serial, image.cols, image.rows, &distortion);
    calibration.intrinsics = FactoryIntrinsics(K, image.size(), distortion);

    // Before detecting, not after: if the capture resolution cannot resolve the
    // markers the detection returns nothing, and the reason is not visible in
    // the image.
    std::cout << id << ":\n";
    ReportDecodeReach(board, K(0, 0), image.cols, image.rows);

    const BoardObservation observation = detector.Detect(image, id, 0);
    std::cout << "  " << observation.size() << " corners in " << image_path << "\n";
    if (static_cast<int>(observation.size()) < report.thresholds.min_shared_corners) {
      throw std::runtime_error(id + ": only " + std::to_string(observation.size()) +
                               " corners -- the board must be fully visible for "
                               "re-registration, there is no second view to fall back on. "
                               "If the count is zero, re-read the decode reach above: the "
                               "usual cause is capturing below 1920x1080.");
    }

    const PoseResult pose = EstimateBoardPose(observation, calibration.intrinsics, board);
    calibration.registration_rmse_px = pose.reprojection_rmse_px;
    calibration.T_world_cam = RegisterToWorld(pose.T_cam_board, world.T_world_board);

    ids.push_back(id);
    before.push_back(it->second.T_world_cam);
    after.push_back(calibration.T_world_cam);
    registrations.push_back(observation);
    report.cameras.push_back(std::move(calibration));
  }

  const ReRegistrationReport rigid = CheckReRegistration(ids, before, after, rigidity);
  std::cout << "\n" << rigid.Format();

  if (registrations.size() >= 2) {
    report.round_trip = ComputeRoundTrip(registrations[0], report.cameras[0],
                                         registrations[1], report.cameras[1]);
  }
  Evaluate(&report);
  std::cout << "\n" << report.Format();

  if (!rigid.passed()) {
    std::cerr << "\nrefusing to write: the rig is not intact, so these poses are not a "
                 "re-registration of the same rig. Run a full calibration.\n";
    return EXIT_FAILURE;
  }
  if (!report.passed()) return EXIT_FAILURE;

  WriteExtrinsicsYaml(report, out_path);
  std::cout << "wrote " << out_path << "\n";
  return EXIT_SUCCESS;
}
#endif

}  // namespace

int main(int argc, char** argv) {
  // reregister-mock is the one subcommand that takes no arguments.
  const bool bare_command = argc == 2 && std::string(argv[1]) == "reregister-mock";
  if (argc < 3 && !bare_command) {
    std::cerr << "usage:\n"
              << "  calibrate board <out.png>\n"
              << "  calibrate aim     <cameras.yaml> <cfg>\n"
              << "  calibrate capture <cameras.yaml> <cfg> <out_dir> [options]\n"
              << "  calibrate mock  <out.yaml>\n"
              << "  calibrate run   <cfg> <front_dir> <front_serial> <over_dir> "
                 "<over_serial> <out.yaml> [--factory]\n"
              << "  calibrate reregister <cfg> <prev.yaml> <out.yaml> "
                 "<id>:<serial>:<image> ...\n"
              << "  calibrate reregister-mock\n"
              << "\ncapture options:\n"
                 "  --register        shoot ONE pair as zz_register.png, the view every\n"
                 "                    camera pose is derived from. Board at its measured\n"
                 "                    world position, both cameras, board untouched.\n"
                 "  --auto <n>        shoot n sweep frames unattended\n"
                 "  --interval <s>    seconds between --auto frames (default 2)\n"
                 "  --prefix <name>   sweep filename prefix (default 'view'). Must sort\n"
                 "                    BEFORE zz_register.png.\n"
                 "  --width/--height/--fps   capture profile (default 1920x1080 @30)\n"
              << "\n  --factory  use the device's own intrinsics instead of solving them.\n"
                 "             Lets you capture at 1920x1080 for corner detection while the\n"
                 "             pipeline streams 848x480 -- the recovered pose is the same.\n";
    return EXIT_FAILURE;
  }
  const std::string command = argv[1];
  try {
    if (command == "mock") return RunMock(argv[2]);
    if (command == "reregister-mock") return RunReRegisterMock();
#ifdef THE_EYES_WITH_ARUCO
    if (command == "board") {
      BoardSpec board;
      const CharucoDetector detector(board);
      detector.WriteBoardImage(argv[2]);
      std::cout << "wrote " << argv[2] << "\n"
                << "Print at 100% scale, then MEASURE a square with callipers. Printer\n"
                << "scaling is the classic silent failure: every reprojection error stays\n"
                << "small while every distance is wrong by a constant factor.\n";
      return EXIT_SUCCESS;
    }
#ifdef THE_EYES_CALIBRATE_CAN_CAPTURE
    if (command == "aim") {
      if (argc < 4) {
        std::cerr << "aim needs: <cameras.yaml> <calibration.yaml>\n";
        return EXIT_FAILURE;
      }
      return RunAim(argv[2], argv[3]);
    }
    if (command == "capture") {
      if (argc < 5) {
        std::cerr << "capture needs: <cameras.yaml> <calibration.yaml> <out_dir>\n";
        return EXIT_FAILURE;
      }
      CaptureOptions options;
      for (int i = 5; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--register") options.registration = true;
        else if (arg == "--auto" && i + 1 < argc) options.auto_count = std::atoi(argv[++i]);
        else if (arg == "--interval" && i + 1 < argc) options.interval_s = std::atof(argv[++i]);
        else if (arg == "--prefix" && i + 1 < argc) options.prefix = argv[++i];
        else if (arg == "--width" && i + 1 < argc) options.width = std::atoi(argv[++i]);
        else if (arg == "--height" && i + 1 < argc) options.height = std::atoi(argv[++i]);
        else if (arg == "--fps" && i + 1 < argc) options.fps = std::atoi(argv[++i]);
        else if (arg == "--auto" || arg == "--interval" || arg == "--prefix" ||
                 arg == "--width" || arg == "--height" || arg == "--fps") {
          std::cerr << arg << " needs a value\n";
          return EXIT_FAILURE;
        } else {
          std::cerr << "unknown capture option '" << arg << "'\n";
          return EXIT_FAILURE;
        }
      }
      if (options.registration && options.auto_count > 0) {
        std::cerr << "--register and --auto are different shots: the registration view is a "
                     "single deliberate frame with the board at its measured position, not "
                     "one of a timed sweep. Run them separately.\n";
        return EXIT_FAILURE;
      }
      return RunCapture(argv[2], argv[3], argv[4], options);
    }
#else
    if (command == "capture" || command == "aim") {
      std::cerr << "this build has no librealsense (or no opencv_aruco), so " << command
                << " is unavailable.\n";
      return EXIT_FAILURE;
    }
#endif
    if (command == "run") {
      if (argc < 8) {
        std::cerr << "run needs: <cfg> <front_dir> <front_serial> <over_dir> <over_serial> "
                     "<out.yaml> [--factory]\n";
        return EXIT_FAILURE;
      }
      bool use_factory = false;
      for (int i = 8; i < argc; ++i) {
        if (std::string(argv[i]) == "--factory") use_factory = true;
      }
      return RunReal(argv[2], argv[3], argv[4], argv[5], argv[6], argv[7], use_factory);
    }
    if (command == "reregister") {
      if (argc < 7) {
        std::cerr << "reregister needs: <cfg> <prev.yaml> <out.yaml> then at least two "
                     "<camera_id>:<serial>:<image> arguments. Two cameras is the minimum: "
                     "one camera has no baseline to check the rig against.\n";
        return EXIT_FAILURE;
      }
      std::vector<std::string> specs(argv + 5, argv + argc);
      return RunReRegister(argv[2], argv[3], argv[4], specs);
    }
#else
    if (command == "board" || command == "run" || command == "reregister" ||
        command == "capture" || command == "aim") {
      std::cerr << "this build has no opencv_aruco, so ChArUco detection is unavailable. "
                   "Only 'mock' works.\n";
      return EXIT_FAILURE;
    }
#endif
    std::cerr << "unknown command '" << command << "'\n";
    return EXIT_FAILURE;
  } catch (const std::exception& e) {
    std::cerr << "calibrate: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
