#include "the_eyes/runtime/view_evidence.hpp"

#include <algorithm>
#include <stdexcept>

#include "the_eyes/runtime/backends.hpp"

namespace the_eyes::runtime {

std::map<std::string, TriMesh> LoadEvidenceMeshes(const pose::MeshRegistry& registry,
                                                  const std::string& repo_root) {
  std::map<std::string, TriMesh> out;
  for (const std::string& cls : registry.classes()) {
    const MeshEntry* e = registry.Find(cls);
    if (e == nullptr || e->mesh_path.empty()) continue;
    out.emplace(cls, LoadObjMesh(ResolveAgainst(repo_root, e->mesh_path)));
  }
  return out;
}

ViewEvidence::ViewEvidence(fusion::ViewEvidenceConfig cfg, fusion::ExtrinsicsSet extrinsics,
                           std::map<std::string, TriMesh> meshes)
    : cfg_(cfg), extrinsics_(std::move(extrinsics)), meshes_(std::move(meshes)) {}

void ViewEvidence::Attach(const std::map<std::string, EvidenceView>& views,
                          std::vector<PoseEstimate>* estimates) const {
  if (!cfg_.enabled || estimates == nullptr) return;
  for (PoseEstimate& e : *estimates) {
    const auto mesh = meshes_.find(e.cls);
    const auto own = views.find(e.camera_id);
    const auto own_ex = extrinsics_.cameras.find(e.camera_id);
    if (mesh == meshes_.end() || own == views.end() || own_ex == extrinsics_.cameras.end() ||
        own->second.depth_u16.empty()) {
      continue;
    }

    const EvidenceView& v = own->second;
    const cv::Mat rendered =
        RenderDepth(mesh->second, e.pose_cam, v.K, v.depth_u16.cols, v.depth_u16.rows);
    const VisibilityCounts c =
        MeasureVisibility(rendered, v.depth_u16, v.depth_scale_m, cfg_.agree_tol_m, kErodePx);
    if (c.silhouette < kMinSilhouettePx) continue;
    e.depth_support = static_cast<double>(c.consistent) / static_cast<double>(c.silhouette);

    const Eigen::Isometry3d T_world_obj = own_ex->second.T_world_cam * e.pose_cam;
    bool another_view = false;
    double seen = 0.0;
    for (const auto& [camera_id, other] : views) {
      if (camera_id == e.camera_id || other.depth_u16.empty()) continue;
      const auto ex = extrinsics_.cameras.find(camera_id);
      if (ex == extrinsics_.cameras.end()) continue;
      another_view = true;
      const cv::Mat r = RenderDepth(mesh->second, ex->second.T_world_cam.inverse() * T_world_obj,
                                    other.K, other.depth_u16.cols, other.depth_u16.rows);
      const VisibilityCounts o =
          MeasureVisibility(r, other.depth_u16, other.depth_scale_m, cfg_.through_tol_m, kErodePx);
      // Outside this camera's image, or a sliver of it: no opinion.
      if (o.silhouette < kMinSilhouettePx) continue;
      seen = std::max(seen, static_cast<double>(o.behind) / static_cast<double>(o.silhouette));
    }
    if (another_view) e.seen_through = seen;
  }
}

}  // namespace the_eyes::runtime
