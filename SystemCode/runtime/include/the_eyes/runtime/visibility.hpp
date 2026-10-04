// Occlusion severity from ground truth: how much of an object's projected
// silhouette a camera actually sees.
//
// The proposal (7.1) fixes the occlusion tiers this way -- "a target counts as
// occluded from a view when under 25% of its projected silhouette is visible,
// computed from the ground-truth pose and mesh" -- and until this file nothing
// computed it, so a T4 scene was T4 because someone said so. The mesh is
// rendered at its ground-truth pose into each view, and every pixel of that
// silhouette is classified against the depth the camera measured there:
//
//   occluded    measured surface NEARER than the model by more than tol
//   consistent  within tol of the model -- the camera sees the object here
//   behind      measured surface FARTHER than the model by more than tol
//   invalid     no depth at this pixel
//
//   visible fraction = (silhouette - occluded) / silhouette
//   depth fraction   = (consistent + behind)   / silhouette
//
// `behind` counts as visible because an occluder can only ever be in front.
// It is reported separately because on an unoccluded scene it measures how
// well the ground truth lands on the object: model pixels that fall on the
// table beyond it are ground-truth or depth error, not occlusion.
//
// `invalid` counts as visible too, and that is a measurement, not a
// convenience. Active stereo leaves a band with no depth beside every
// foreground edge, on the surface BEHIND it -- so a half-hidden object's
// visible half is exactly where its depth goes missing. On the first real
// half-occluded cube (2026-10-04) 45% of the silhouette had no depth and was
// plainly in view in the colour image; excluding it scored the cube 2% visible.
// An occluder, being the foreground, keeps its depth (0.0% missing on the
// fully hidden cube the same day). The depth fraction says how much of what is
// visible an RGB-D estimator can actually use, and the two are reported side
// by side because the gap between them is itself the difficulty of the scene.
//
// Two biases, both measured by a T1 scene rather than corrected here. The
// tray's socket wall hides the bottom 6-12 mm of each object from an oblique
// view, which reads as occlusion; and the rim of the silhouette is where a
// millimetre of ground-truth error lands model pixels on the background, which
// reads as `behind`. The second is why the silhouette is eroded first.
//
// No OpenCV beyond cv::Mat, no hardware: testable against a synthetic cube.
#ifndef THE_EYES_RUNTIME_VISIBILITY_HPP
#define THE_EYES_RUNTIME_VISIBILITY_HPP

#include <array>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace the_eyes::runtime {

struct TriMesh {
  std::vector<Eigen::Vector3d> vertices;
  std::vector<std::array<int, 3>> triangles;  // 0-based vertex indices
};

// Wavefront OBJ: `v` and `f` lines only. Faces may be `a`, `a/b`, `a//c` or
// `a/b/c`, negative (relative) indices are resolved, and polygons are fanned
// into triangles. Throws on a missing file, no faces, or an index out of range.
TriMesh LoadObjMesh(const std::string& path);

// Z-buffer render of the mesh at T_cam_object: CV_32FC1, metres along the
// optical axis, 0 where the mesh does not cover. Pinhole K with no distortion,
// as the pipeline itself treats the colour stream. Pixel (u, v) is sampled at
// its centre in K's convention, i.e. at integer coordinates.
cv::Mat RenderDepth(const TriMesh& mesh, const Eigen::Isometry3d& T_cam_object,
                    const Eigen::Matrix3d& K, int width, int height);

// Per-pixel class, for overlays.
enum class VisLabel : uint8_t { kNone = 0, kConsistent, kBehind, kOccluded, kInvalid };

struct VisibilityCounts {
  int silhouette = 0;  // after erosion
  int invalid = 0;
  int occluded = 0;
  int consistent = 0;
  int behind = 0;

  int measured() const { return silhouette - invalid; }
  // Share of the silhouette not hidden by anything nearer. NaN with no silhouette.
  double visible_fraction() const;
  // Share of the silhouette that is visible AND has depth. NaN likewise.
  double depth_fraction() const;
  double behind_fraction() const;   // of measured; NaN when nothing was measured
  double invalid_fraction() const;  // of the silhouette; NaN when there is none
};

// Classifies the silhouette of `rendered_m` (RenderDepth's output) against a
// CV_16UC1 depth image in device units. `erode_px` shrinks the silhouette
// first. When `labels` is given it receives a CV_8UC1 VisLabel image.
VisibilityCounts MeasureVisibility(const cv::Mat& rendered_m, const cv::Mat& depth_u16,
                                   double depth_scale_m, double tol_m, int erode_px,
                                   cv::Mat* labels = nullptr);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_VISIBILITY_HPP
