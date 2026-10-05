// Re-runs the occlusion ablation from a metrics log.
//
//   ablation_from_log <metrics.sqlite> [--run N] [--config-dir DIR] [--repo-root DIR]
//
// --repo-root is where relative mesh paths resolve (default: the parent of the
// config directory). It matters when the config directory is a per-scene copy
// that does not sit beside assets/.
//
// Reads the per-view PoseEstimates the runner logged (`estimates`) and the
// `ground_truth` rows for the same run, rebuilds fusion::AblationFrames and
// runs fusion's eight-arm harness on them. Ground truth is supplied by a
// separate tool (the ArUco-tray path); until it has been, this prints what
// the log holds and stops -- it never scores against invented poses
// (CLAUDE.md constraint 5).
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "the_eyes/fusion/config.hpp"
#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/fusion/metrics.hpp"
#include "the_eyes/pose/mesh_registry.hpp"
#include "the_eyes/runtime/backends.hpp"
#include "the_eyes/runtime/metrics_log.hpp"
#include "the_eyes/runtime/pipeline.hpp"

namespace {

using namespace the_eyes;
using namespace the_eyes::runtime;

// Vertices of a Wavefront OBJ, decimated to at most max_points by stride so
// ADD-S (O(m^2)) stays tractable per frame. Deterministic.
fusion::ModelPoints LoadObjVertices(const std::string& path, size_t max_points) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open mesh " + path);
  fusion::ModelPoints all;
  std::string line;
  while (std::getline(f, line)) {
    if (line.size() < 2 || line[0] != 'v' || line[1] != ' ') continue;
    std::istringstream is(line.substr(2));
    double x, y, z;
    if (is >> x >> y >> z) all.emplace_back(x, y, z);
  }
  if (all.empty()) throw std::runtime_error("no vertices in " + path);
  if (all.size() <= max_points) return all;
  fusion::ModelPoints out;
  const size_t stride = (all.size() + max_points - 1) / max_points;
  for (size_t i = 0; i < all.size(); i += stride) out.push_back(all[i]);
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: ablation_from_log <metrics.sqlite> [--run N] [--config-dir DIR] "
                 "[--repo-root DIR]\n";
    return EXIT_FAILURE;
  }
  const std::string db = argv[1];
  int64_t run_id = -1;
  std::string config_dir = "config";
  std::string repo_root_arg;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--run" && i + 1 < argc) run_id = std::stoll(argv[++i]);
    else if (a == "--config-dir" && i + 1 < argc) config_dir = argv[++i];
    else if (a == "--repo-root" && i + 1 < argc) repo_root_arg = argv[++i];
    else {
      std::cerr << "ablation_from_log: unknown argument " << a << "\n";
      return EXIT_FAILURE;
    }
  }
  try {
    if (run_id < 0) run_id = LatestRunId(db);
    if (run_id < 0) {
      std::cerr << "ablation_from_log: " << db << " holds no runs\n";
      return EXIT_FAILURE;
    }
    const std::vector<fusion::AblationFrame> frames = LoadAblationFrames(db, run_id);
    size_t estimates = 0, gt = 0;
    for (const fusion::AblationFrame& f : frames) {
      estimates += f.estimates.size();
      gt += f.ground_truth.size();
    }
    std::cout << "run " << run_id << ": " << frames.size() << " frames, " << estimates
              << " per-view estimates, " << gt << " ground-truth poses\n";
    if (gt == 0) {
      std::cout << "No ground truth in this run's `ground_truth` table, so there is nothing to score "
                   "against.\nThe estimates are logged; add ground truth (the ArUco-tray tool) and "
                   "re-run. No numbers were produced.\n";
      return EXIT_SUCCESS;
    }

    const std::string repo_root =
        repo_root_arg.empty() ? std::filesystem::absolute(config_dir).parent_path().string()
                              : std::filesystem::absolute(repo_root_arg).string();
    fusion::FusionConfig cfg = fusion::LoadFusionConfig(config_dir + "/fusion.yaml");
    const fusion::ExtrinsicsSet ex = fusion::LoadExtrinsics(config_dir + "/extrinsics.yaml");
    const pose::MeshRegistry meshes = pose::LoadMeshRegistry(config_dir + "/meshes.yaml", true, repo_root);
    cfg.symmetry = SymmetriesFrom(meshes);
    fusion::ModelRegistry models;
    for (const std::string& cls : meshes.classes()) {
      const MeshEntry* e = meshes.Find(cls);
      models[cls] = fusion::ModelEntry{LoadObjVertices(ResolveAgainst(repo_root, e->mesh_path), 2000),
                                       e->diameter_m};
    }
    const fusion::AblationResult result =
        fusion::RunOcclusionAblation(frames, ex, cfg, ExtentsFrom(meshes), models);
    std::cout << "--- occlusion ablation from logged estimates + ground truth (run " << run_id
              << ") ---\n"
              << result.Format() << "\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "ablation_from_log: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
