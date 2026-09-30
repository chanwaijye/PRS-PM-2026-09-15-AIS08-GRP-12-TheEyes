// ArUco ground-truth trays: the geometry half, with no OpenCV in it.
//
// `config/marker_trays.yaml` describes where each marker sits on each tray and
// where the object sits relative to the tray. Evaluation needs
//
//     T_cam_object = T_cam_marker . T_marker_tray . T_tray_object
//
// of which only the first term is measured from an image. Everything here is
// the part that is exact by construction, so it is kept away from the detector
// for the same reason `calibration/charuco_detector.hpp` is isolated: the
// arithmetic that can be silently wrong should be testable without images,
// without opencv_contrib and without hardware.
//
// Ground truth is the yardstick every ADD-S number is measured against. If this
// file is wrong, the ablation is wrong in a way that looks entirely healthy.
#ifndef THE_EYES_RUNTIME_MARKER_TRAYS_HPP
#define THE_EYES_RUNTIME_MARKER_TRAYS_HPP

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace the_eyes::runtime {

// One marker's placement on its tray. Translation only: every marker lies in
// the tray's top plane and the object is seated upright, so the marker, tray
// and object frames share one orientation (config/marker_trays.yaml).
struct TrayMarker {
  int id = -1;
  Eigen::Vector3d t_tray_marker = Eigen::Vector3d::Zero();
};

struct Tray {
  std::string cls;                                          // key into config/meshes.yaml
  Eigen::Vector3d t_tray_object = Eigen::Vector3d::Zero();  // object AABB centre in tray frame
  std::vector<TrayMarker> markers;
};

struct TrayConfig {
  std::string dictionary;        // e.g. "DICT_5X5_250"; must match what was printed
  double marker_length_m = 0.0;  // scales every recovered translation, so it is fatal if wrong
  std::map<std::string, Tray> trays;  // by class

  // Marker id -> class. Built at load time because a detection arrives as an
  // id and has to be attributed to exactly one object.
  std::map<int, std::string> cls_of_marker;
};

// Throws std::runtime_error on a missing file, a malformed value, or a marker
// id that appears on more than one tray.
//
// The duplicate-id check is the important one. A repeated id is not a typo the
// pipeline can absorb: the same observation would be attributed to two
// different objects, and both would get plausible-looking ground truth. The
// generator cannot repeat an id, but a hand-edit can, and the file says "do not
// hand-edit" precisely because someone will.
TrayConfig LoadTrayConfig(const std::string& yaml_path);

// T_marker_object for one marker on one tray -- pure translation,
// t_tray_object - t_tray_marker. Throws if the id is not on this tray.
Eigen::Isometry3d MarkerToObject(const Tray& tray, int marker_id);

// A mean pose plus how far the samples scattered around it.
//
// The scatter is not diagnostic decoration. It is ground truth's own noise
// floor: an ADD-S smaller than this spread is measuring the yardstick, not the
// estimator, and the report has to be able to say so.
struct PoseAverage {
  Eigen::Isometry3d mean = Eigen::Isometry3d::Identity();
  int n = 0;
  double translation_rms_m = 0.0;
  double translation_max_m = 0.0;
  double rotation_rms_deg = 0.0;
  double rotation_max_deg = 0.0;
};

// Rotations are averaged by Markley's method -- the principal eigenvector of
// sum(q q^T) -- rather than by averaging quaternion components, which is only
// valid for tightly clustered samples and fails silently when they are not.
// Returns n == 0 for an empty input; callers must check before using `mean`.
PoseAverage AveragePoses(const std::vector<Eigen::Isometry3d>& poses);

// Which of ONE object's pose samples agree with the rest.
//
// AveragePoses trusts every sample, and on the per-marker path one marker that
// resolved to the wrong planar solution is a sample tens of degrees off. The
// ambiguity gate does not stop all of them: on the board's 28 mm markers flips
// of ~120 deg survived it (docs/CAPTURE_PROTOCOL.md 4.2f). Averaged in, such a
// sample drags the mean while the pooled spread merely reads large, which a
// human has to notice. So every sample is judged against the MEDOID -- the
// sample whose summed distance to all the others is smallest, which stays
// inside the majority cluster as long as there is one -- and a sample further
// than either limit from it is not averaged.
//
// Distance for choosing the medoid is rotation/max_rotation_deg +
// translation/max_translation_m, so neither unit dominates.
struct OutlierScreen {
  std::vector<bool> keep;  // parallel to the input
  int medoid = -1;         // index of the reference sample; -1 for an empty input
  int rejected = 0;
};

OutlierScreen ScreenPoseOutliers(const std::vector<Eigen::Isometry3d>& poses,
                                 double max_rotation_deg, double max_translation_m);

// Shortest-arc angle between two rotations, in degrees.
double RotationAngleDeg(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_MARKER_TRAYS_HPP
