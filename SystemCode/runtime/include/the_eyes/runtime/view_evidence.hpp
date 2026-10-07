// What the depth images say about each per-view pose, measured before fusion.
//
// PoseEstimate::score is the estimator's opinion of its own answer, and on the
// scored recordings it does not rank the views (types.hpp has the numbers). So
// each estimate is checked against the two things that were actually measured:
// the object is rendered at the estimated pose into its own view and into
// every other view, and each silhouette is compared with that camera's depth.
//
//   depth_support  own view: share of the silhouette where the measured depth
//                  agrees with the pose, within agree_tol_m
//   seen_through   other views: the largest share of the silhouette where the
//                  measured surface lies more than through_tol_m BEYOND the
//                  object -- a camera looking through the place the object is
//                  claimed to occupy
//
// A surface NEARER than the object is an occluder and is evidence of nothing;
// no depth at a pixel is evidence of nothing. That asymmetry is the whole
// point: a camera that cannot see an object has no opinion about it, and a
// camera that sees past where it should be has a decisive one.
//
// MEASURED on the 16 scored scenes of 2026-10-05..07, every estimate logged:
// the 10,843 within 30 mm of the truth are seen through by 0.13 at most and
// supported by 0.33 at least; the poses the gate was getting wrong -- the
// half-hidden bottle and cube at the occluder's range, the cube fitted to the
// occluder's print -- are seen through by 0.46 at least.
//
// Fusion decides what to do with the two numbers (fusion.yaml view_evidence);
// this file only measures them. It reuses the renderer the visibility tool
// uses on ground truth, on purpose: one rasteriser, one pixel convention.
#ifndef THE_EYES_RUNTIME_VIEW_EVIDENCE_HPP
#define THE_EYES_RUNTIME_VIEW_EVIDENCE_HPP

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "the_eyes/fusion/config.hpp"
#include "the_eyes/fusion/extrinsics.hpp"
#include "the_eyes/pose/mesh_registry.hpp"
#include "the_eyes/runtime/visibility.hpp"
#include "the_eyes/types.hpp"

namespace the_eyes::runtime {

// One camera's measurement for the current pair.
struct EvidenceView {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();  // colour intrinsics
  cv::Mat depth_u16;                                // aligned to colour
  double depth_scale_m = 0.001;                     // depth correction applied
};

// Every registered class's mesh, loaded once. Throws naming the file.
std::map<std::string, TriMesh> LoadEvidenceMeshes(const pose::MeshRegistry& registry,
                                                  const std::string& repo_root);

class ViewEvidence {
 public:
  ViewEvidence() = default;  // measures nothing; estimates keep "not supplied"
  ViewEvidence(fusion::ViewEvidenceConfig cfg, fusion::ExtrinsicsSet extrinsics,
               std::map<std::string, TriMesh> meshes);

  bool enabled() const { return cfg_.enabled; }

  // Fills depth_support and seen_through on every estimate it can measure: its
  // class has a mesh, its camera is calibrated and has a frame in `views`, and
  // its silhouette there is large enough to mean something. seen_through is
  // 0 when at least one other view is present and none sees through it, and
  // stays negative when there is no other view at all -- one camera cannot
  // contradict itself. Estimates it cannot measure are left untouched.
  void Attach(const std::map<std::string, EvidenceView>& views,
              std::vector<PoseEstimate>* estimates) const;

  // A silhouette smaller than this (px, after erosion) says nothing either way.
  static constexpr int kMinSilhouettePx = 60;
  // The rim of a silhouette is where a millimetre of pose error lands model
  // pixels on the background; it is shaved off before counting (visibility.hpp).
  static constexpr int kErodePx = 1;

 private:
  fusion::ViewEvidenceConfig cfg_;
  fusion::ExtrinsicsSet extrinsics_;
  std::map<std::string, TriMesh> meshes_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_VIEW_EVIDENCE_HPP
