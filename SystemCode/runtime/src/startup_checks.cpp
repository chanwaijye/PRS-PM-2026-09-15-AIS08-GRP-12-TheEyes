#include "the_eyes/runtime/startup_checks.hpp"

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <set>
#include <sstream>
#include <stdexcept>

#include "the_eyes/runtime/backends.hpp"
#include "the_eyes/runtime/pipeline.hpp"
#include "the_eyes/runtime/fixed_box_detector.hpp"
#if THE_EYES_WITH_REALSENSE
#include "the_eyes/camera/device_registry.hpp"
#endif

namespace the_eyes::runtime {
namespace {

template <typename F>
void Load(const std::string& what, F&& f) {
  try {
    f();
  } catch (const std::exception& e) {
    throw std::runtime_error(what + ": " + e.what());
  }
}

}  // namespace

std::string StartupReport::Format() const {
  std::ostringstream os;
  for (const std::string& s : info) os << "  " << s << "\n";
  for (const std::string& s : warnings) os << "  WARNING: " << s << "\n";
  for (const std::string& s : failures) os << "  FAIL: " << s << "\n";
  os << (ok() ? "  startup checks: OK" : "  startup checks: FAILED") << " (" << failures.size()
     << " failure(s), " << warnings.size() << " warning(s))\n";
  return os.str();
}

LoadedConfigs LoadAllConfigs(const std::string& config_dir, const std::string& repo_root) {
  LoadedConfigs c;
  c.config_dir = config_dir;
  c.repo_root = repo_root;
  const auto path = [&](const char* name) { return config_dir + "/" + name; };
  Load(path("runtime.yaml"), [&] { c.runtime = LoadRuntimeConfig(path("runtime.yaml")); });
  Load(path("cameras.yaml"), [&] { c.cameras = camera::LoadCameraAgentConfig(path("cameras.yaml")); });
  Load(path("extrinsics.yaml"), [&] { c.extrinsics = fusion::LoadExtrinsics(path("extrinsics.yaml")); });
  Load(path("fusion.yaml"), [&] { c.fusion = fusion::LoadFusionConfig(path("fusion.yaml")); });
  Load(path("predicates.yaml"),
       [&] { c.predicates = scenegraph::LoadPredicateConfig(path("predicates.yaml")); });
  Load(path("scene.yaml"), [&] { c.scene = LoadStaticScene(path("scene.yaml")); });
  Load(path("pose.yaml"), [&] {
    c.pose_agent = pose::LoadPoseAgentConfig(path("pose.yaml"));
    c.pose_backend = LoadPoseBackendConfig(path("pose.yaml"));
  });
  Load(path("meshes.yaml"), [&] {
    const bool require_files = c.pose_backend.backend == "foundationpose";
    c.meshes = pose::LoadMeshRegistry(path("meshes.yaml"), require_files, repo_root);
  });
  // The objects' symmetry groups live in meshes.yaml and fusion needs them.
  c.fusion.symmetry = SymmetriesFrom(c.meshes);
  Load(path("detector.yaml"), [&] { c.detector = detector::LoadDetectorConfig(path("detector.yaml")); });
  // Optional: a rig that has not been tared runs on the factory depth.
  Load(path("depth_correction.yaml"),
       [&] { c.depth_correction = LoadDepthCorrection(path("depth_correction.yaml")); });
  return c;
}

void CheckConfigCrossRefs(const LoadedConfigs& c, StartupReport* r) {
  // Camera ids: cameras.yaml is the authority; every other file must agree.
  std::vector<std::string> camera_ids;
  for (const auto& [id, entry] : c.cameras.cameras) camera_ids.push_back(id);

  for (const std::string& id : camera_ids) {
    auto ex = c.extrinsics.cameras.find(id);
    if (ex == c.extrinsics.cameras.end()) {
      r->Fail("camera '" + id + "' is in cameras.yaml but has no entry in extrinsics.yaml");
      continue;
    }
    const std::string& serial = c.cameras.at(id).serial;
    if (ex->second.serial != serial) {
      r->Fail("camera '" + id + "' serial " + serial + " in cameras.yaml but " + ex->second.serial +
              " in extrinsics.yaml -- the extrinsics were measured on a different unit");
    }
  }
  for (const auto& [id, ex] : c.extrinsics.cameras) {
    if (!c.cameras.cameras.count(id)) {
      r->Warn("extrinsics.yaml has camera '" + id + "' that cameras.yaml does not list");
    }
  }

  if (c.fusion.view_names != camera_ids) {
    r->Fail("fusion.yaml views.names do not match cameras.yaml camera ids in order -- "
            "visible_views bits would be mislabelled");
  }
  if (c.predicates.view_names != camera_ids) {
    r->Fail("predicates.yaml views.names do not match cameras.yaml camera ids in order");
  }
  if (c.scene.world_frame != c.extrinsics.world_frame) {
    r->Fail("scene.yaml world_frame '" + c.scene.world_frame + "' but extrinsics.yaml world_frame '" +
            c.extrinsics.world_frame + "'");
  }
  for (const StaticObject& s : c.scene.objects) {
    if (c.meshes.Contains(s.cls)) {
      r->Fail("scene.yaml static object '" + s.id + "' has class '" + s.cls +
              "' which is also a detected class -- fused ids would collide");
    }
  }
  for (const std::string& cls : c.detector.model.class_names) {
    if (!c.meshes.Contains(cls)) {
      r->Warn("detector class '" + cls + "' has no mesh in meshes.yaml; its detections will be "
              "dropped by pose_agent");
    }
  }
  if (c.fusion.calibration.max_reprojection_rmse_px > 0.0) {
    try {
      fusion::AssertCalibrationQuality(c.extrinsics, c.fusion.calibration.max_reprojection_rmse_px);
      std::ostringstream os;
      os << "extrinsics pass the reprojection gate (";
      for (const auto& [id, ex] : c.extrinsics.cameras) {
        os << id << " " << ex.reprojection_rmse_px << " px ";
      }
      os << "<= " << c.fusion.calibration.max_reprojection_rmse_px << " px)";
      r->Info(os.str());
    } catch (const std::exception& e) {
      r->Fail(e.what());
    }
  }
  if (c.pose_backend.request_timeout_ms < 10000) {
    r->Warn("pose.yaml estimator.request_timeout_ms = " +
            std::to_string(c.pose_backend.request_timeout_ms) +
            " ms; FoundationPose registration measured up to 4.3 s, so a busy server can hit this");
  }
  r->Info("configs: " + std::to_string(camera_ids.size()) + " cameras, " +
          std::to_string(c.meshes.size()) + " mesh classes, " +
          std::to_string(c.scene.objects.size()) + " static objects, world frame '" +
          c.extrinsics.world_frame + "'");
}

void CheckDetectorBackend(const LoadedConfigs& c, StartupReport* r) {
  const DetectorBackendConfig& d = c.runtime.detector;
  if (d.backend == "onnx") {
#if !THE_EYES_WITH_ONNXRUNTIME
    r->Fail("detector backend 'onnx' but ONNX Runtime was not compiled in "
            "(set THE_EYES_ONNXRUNTIME_ROOT at configure time)");
#endif
    const std::string model = ResolveAgainst(c.repo_root, c.detector.model.onnx_path);
    if (!std::filesystem::exists(model)) {
      r->Fail("ONNX model not found: " + model +
              "\n        regenerate: python training/detector/export_onnx.py "
              "--weights runs/detector/rgbd_50k_v2/weights/best.pt --imgsz 640 "
              "--out assets/models/yolov8_the_eyes.onnx");
    } else {
      r->Info("detector: onnx model " + model + ", provider '" + c.detector.inference.provider +
              "'" + (c.detector.inference.allow_cpu_fallback ? " (cpu fallback allowed)" : ""));
    }
  } else if (d.backend == "mock") {
    const std::string fixture = ResolveAgainst(c.repo_root, d.mock_fixture);
    if (d.mock_fixture.empty() || !std::filesystem::exists(fixture)) {
      r->Fail("detector backend 'mock' needs an existing fixture; got '" + d.mock_fixture + "'");
    } else {
      r->Info("detector: mock fixture " + fixture);
    }
  } else if (d.backend == "fixed") {
    try {
      const FixedBoxes boxes = ParseFixedBoxes(d.fixed_boxes);
      std::ostringstream os;
      os << "detector: FIXED boxes, not detections --";
      for (const auto& [cam, list] : boxes) {
        for (const Detection& b : list) {
          os << " " << cam << ":" << b.cls << "@" << b.bbox.x << "," << b.bbox.y << "," << b.bbox.width
             << "x" << b.bbox.height;
        }
        if (!c.cameras.cameras.count(cam)) r->Warn("fixed box for unknown camera '" + cam + "'");
      }
      r->Info(os.str());
      r->Warn("fixed-box detector: nothing from this run is a detection result");
    } catch (const std::exception& e) {
      r->Fail(e.what());
    }
  } else {
    r->Fail("unknown detector backend '" + d.backend + "'");
  }
}

std::string FoundationPoseServerCommand(const std::string& socket_path,
                                        const std::string& repo_root) {
  return "python " + repo_root + "/SystemCode/pose/server/foundationpose_server.py --socket " +
         socket_path + " --asset-root " + repo_root + " --foundationpose-root ~/src/FoundationPose";
}

void CheckPoseBackend(const LoadedConfigs& c, StartupReport* r) {
  const PoseBackendConfig& p = c.pose_backend;
  if (p.backend == "mock") {
    r->Info("pose: mock identity estimator (poses are NOT measurements)");
    return;
  }
  if (p.backend != "foundationpose") {
    r->Fail("unknown pose backend '" + p.backend + "'");
    return;
  }
  const std::string hint = "\n        start it with:\n        " +
                           FoundationPoseServerCommand(p.socket_path, c.repo_root);
  struct stat st {};
  if (::stat(p.socket_path.c_str(), &st) != 0 || !S_ISSOCK(st.st_mode)) {
    r->Fail("FoundationPose server socket not present at " + p.socket_path + hint);
    return;
  }
  const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    r->Fail(std::string("cannot create a unix socket: ") + std::strerror(errno));
    return;
  }
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, p.socket_path.c_str(), sizeof(addr.sun_path) - 1);
  const bool connected = ::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
  ::close(fd);
  if (!connected) {
    r->Fail("FoundationPose server socket exists but refuses connections (stale file?) at " +
            p.socket_path + hint);
    return;
  }
  r->Info("pose: FoundationPose server reachable at " + p.socket_path + ", timeout " +
          std::to_string(p.request_timeout_ms) + " ms");
}

void CheckDevices(const LoadedConfigs& c, StartupReport* r) {
#if THE_EYES_WITH_REALSENSE
  const camera::RegistryCheck check = camera::CheckDevices(c.cameras);
  std::istringstream lines(check.Format());
  std::string line;
  while (std::getline(lines, line)) {
    if (!line.empty()) r->Info("devices: " + line);
  }
  for (const std::string& w : check.warnings) r->Warn("devices: " + w);
  for (const std::string& b : check.blocking) r->Fail("devices: " + b);
  for (const std::string& s : check.missing_serials) {
    r->Fail("devices: configured serial " + s + " is not attached");
  }
  if (!check.ok && check.blocking.empty() && check.missing_serials.empty()) {
    r->Fail("devices: registry check failed");
  }
#else
  (void)c;
  r->Fail("live capture needs librealsense, which was not found at configure time");
#endif
}

std::string GitRevision(const std::string& repo_root) {
  const std::string cmd = "git -C '" + repo_root + "' rev-parse --short HEAD 2>/dev/null";
  FILE* pipe = ::popen(cmd.c_str(), "r");
  if (pipe == nullptr) return "";
  char buf[128] = {0};
  std::string out;
  if (std::fgets(buf, sizeof(buf), pipe) != nullptr) out = buf;
  ::pclose(pipe);
  while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
  return out;
}

}  // namespace the_eyes::runtime
