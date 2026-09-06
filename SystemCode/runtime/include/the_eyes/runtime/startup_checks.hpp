// Everything that must be true before the first frame is processed, checked
// up front and reported together. Each failure names the file or command
// that fixes it. A live demo has no time to discover a swapped serial or a
// dead pose server one frame at a time.
#ifndef THE_EYES_RUNTIME_STARTUP_CHECKS_HPP
#define THE_EYES_RUNTIME_STARTUP_CHECKS_HPP

#include <string>
#include <vector>

#include "the_eyes/camera/config.hpp"
#include "the_eyes/detector/detector.hpp"
#include "the_eyes/fusion/config.hpp"
#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/pose/mesh_registry.hpp"
#include "the_eyes/pose/pose_agent.hpp"
#include "the_eyes/runtime/runtime_config.hpp"
#include "the_eyes/runtime/static_scene.hpp"
#include "the_eyes/scenegraph/predicates.hpp"

namespace the_eyes::runtime {

struct StartupReport {
  std::vector<std::string> info;
  std::vector<std::string> warnings;
  std::vector<std::string> failures;
  bool ok() const { return failures.empty(); }
  void Info(std::string s) { info.push_back(std::move(s)); }
  void Warn(std::string s) { warnings.push_back(std::move(s)); }
  void Fail(std::string s) { failures.push_back(std::move(s)); }
  std::string Format() const;
};

// Every config the runner consumes, loaded through the owning module's own
// loader so their validation runs unchanged.
struct LoadedConfigs {
  std::string config_dir;
  std::string repo_root;
  camera::CameraAgentConfig cameras;
  fusion::ExtrinsicsSet extrinsics;
  fusion::FusionConfig fusion;
  scenegraph::PredicateConfig predicates;
  StaticScene scene;
  pose::MeshRegistry meshes;
  pose::PoseAgentConfig pose_agent;
  PoseBackendConfig pose_backend;
  detector::DetectorConfig detector;
  RuntimeConfig runtime;
};

// Throws std::runtime_error naming the file that failed. Mesh files are
// required on disk only when the FoundationPose backend will load them.
LoadedConfigs LoadAllConfigs(const std::string& config_dir, const std::string& repo_root);

// The cross-file invariants no single loader can see.
void CheckConfigCrossRefs(const LoadedConfigs& c, StartupReport* report);
void CheckDetectorBackend(const LoadedConfigs& c, StartupReport* report);
// Exists AND accepts a connection; otherwise the failure carries the exact
// command to start the server.
void CheckPoseBackend(const LoadedConfigs& c, StartupReport* report);
std::string FoundationPoseServerCommand(const std::string& socket_path,
                                        const std::string& repo_root);
// Devices attached by serial (camera::CheckDevices). Without librealsense it
// records a failure saying so.
void CheckDevices(const LoadedConfigs& c, StartupReport* report);

// "git rev-parse --short HEAD" of the repo, or "" if unavailable.
std::string GitRevision(const std::string& repo_root);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_STARTUP_CHECKS_HPP
