#include "the_eyes/fusion/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace the_eyes::fusion {
namespace {

std::vector<Eigen::Vector3d> TransformAll(const Eigen::Isometry3d& t, const ModelPoints& model) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(model.size());
  for (const Eigen::Vector3d& p : model) out.push_back(t * p);
  return out;
}

}  // namespace

double Add(const Eigen::Isometry3d& estimated, const Eigen::Isometry3d& ground_truth,
           const ModelPoints& model) {
  if (model.empty()) throw std::invalid_argument("Add: empty model point set");
  double sum = 0.0;
  for (const Eigen::Vector3d& p : model) {
    sum += (estimated * p - ground_truth * p).norm();
  }
  return sum / static_cast<double>(model.size());
}

double AddS(const Eigen::Isometry3d& estimated, const Eigen::Isometry3d& ground_truth,
            const ModelPoints& model) {
  if (model.empty()) throw std::invalid_argument("AddS: empty model point set");
  const std::vector<Eigen::Vector3d> gt = TransformAll(ground_truth, model);

  double sum = 0.0;
  for (const Eigen::Vector3d& p : model) {
    const Eigen::Vector3d e = estimated * p;
    double nearest = std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d& g : gt) nearest = std::min(nearest, (e - g).squaredNorm());
    sum += std::sqrt(nearest);
  }
  return sum / static_cast<double>(model.size());
}

ModelPoints BoxModelPoints(const Eigen::Vector3d& half_extents, int samples_per_axis) {
  const int n = std::max(2, samples_per_axis);
  ModelPoints out;
  out.reserve(static_cast<size_t>(n) * n * 6);
  const auto coord = [n](int i, double half) {
    return -half + 2.0 * half * static_cast<double>(i) / static_cast<double>(n - 1);
  };
  // Sample the six faces rather than the solid interior: ADD-S is a surface
  // metric, and interior points would dilute it toward the centroid distance.
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double a = coord(i, half_extents.x());
      const double b = coord(j, half_extents.y());
      const double c = coord(i, half_extents.z());
      const double d = coord(j, half_extents.z());
      out.emplace_back(a, b, half_extents.z());
      out.emplace_back(a, b, -half_extents.z());
      out.emplace_back(a, half_extents.y(), d);
      out.emplace_back(a, -half_extents.y(), d);
      out.emplace_back(half_extents.x(), b, c);
      out.emplace_back(-half_extents.x(), b, c);
    }
  }
  return out;
}

double BoxDiameter(const Eigen::Vector3d& half_extents) {
  return 2.0 * half_extents.norm();
}

std::string AblationResult::Format() const {
  std::ostringstream os;
  os << std::left << std::setw(15) << "arm" << std::setw(7) << "views" << std::setw(7)
     << "policy" << std::setw(7) << "filter" << std::right << std::setw(6) << "gt"
     << std::setw(9) << "matched" << std::setw(8) << "missed" << std::setw(12) << "ADD-S mm"
     << std::setw(9) << "/diam" << std::setw(13) << "recall@0.1d" << "\n";
  os << std::string(93, '-') << "\n";
  bool previous_filter = false;
  for (size_t i = 0; i < rows.size(); ++i) {
    const AblationRow& r = rows[i];
    // A blank line where the filter setting changes, so the two tiers read as
    // the two halves of the design rather than as one ranked list.
    if (i > 0 && r.temporal_filter != previous_filter) os << "\n";
    previous_filter = r.temporal_filter;
    os << std::left << std::setw(15) << r.label << std::setw(7) << r.views.size()
       << std::setw(7) << (r.mode == FusionMode::kNaive ? "naive" : (r.by_score ? "score" : "gated"))
       << std::setw(7)
       << (r.temporal_filter ? "ekf" : "raw") << std::right << std::setw(6)
       << r.ground_truth_objects << std::setw(9) << r.matched << std::setw(8) << r.missed
       << std::setw(12) << std::fixed << std::setprecision(2) << (r.mean_add_s_m * 1000.0)
       << std::setw(9) << std::setprecision(4) << r.mean_add_s_fraction << std::setw(13)
       << std::setprecision(4) << r.recall_at_0p1_diameter << "\n";
  }

  // Per object, when there is more than one: the pooled rows above cannot say
  // which object a scene fails on. Indented, so the pooled table still reads
  // (and greps) as it did.
  size_t classes = 0;
  for (const AblationRow& r : rows) classes = std::max(classes, r.by_class.size());
  if (classes > 1) {
    os << "\nper object:\n";
    os << "  " << std::left << std::setw(15) << "arm" << std::setw(16) << "class" << std::right
       << std::setw(6) << "gt" << std::setw(9) << "matched" << std::setw(8) << "missed"
       << std::setw(12) << "ADD-S mm" << std::setw(13) << "recall@0.1d" << "\n";
    for (const AblationRow& r : rows) {
      for (const auto& [cls, c] : r.by_class) {
        os << "  " << std::left << std::setw(15) << r.label << std::setw(16) << cls << std::right
           << std::setw(6) << c.ground_truth_objects << std::setw(9) << c.matched << std::setw(8)
           << c.missed << std::setw(12) << std::fixed << std::setprecision(2)
           << (c.mean_add_s_m * 1000.0) << std::setw(13) << std::setprecision(4)
           << c.recall_at_0p1_diameter << "\n";
      }
    }
  }
  return os.str();
}

AblationResult RunOcclusionAblation(const std::vector<AblationFrame>& frames,
                                    const ExtrinsicsSet& extrinsics, const FusionConfig& cfg,
                                    const ExtentRegistry& extents, const ModelRegistry& models,
                                    double match_max_distance_m) {
  // Configurations: the raw tier first (each single view, naive, gated), then
  // the same four with the EKF. Every row is produced by filtering the SAME
  // estimate stream through the SAME association, so a difference between two
  // rows is attributable to the one thing that differs between them.
  //
  // Ordering is deliberate for the report table. Within a tier the single
  // views set the floor, `naive` shows what a second camera alone buys, and the
  // policy row sits last so the eye lands on the policy-minus-naive gap -- the
  // only gap this module can take credit for. The two tiers then show whether
  // that gap survives the filter, which on a static scene does most of the
  // error reduction on its own.
  //
  // A single view runs the gated policy: with one estimate per object the gate
  // has nothing to compare and the weighting has one weight, so the two
  // policies are identical there and the label omits it.
  //
  // When the configuration uses depth evidence (view_evidence.enabled) each
  // tier gains one row, `gate-score`: both views, the gate as it was before the
  // evidence -- a conflict settled by the estimator's score, no veto. It is to
  // `gated` what `naive` is to the gate: the same input with exactly one thing
  // taken away. The single views never use the evidence: half of it comes from
  // the other camera.
  struct Configuration {
    std::string label;
    std::vector<std::string> allowed_views;
    FusionMode mode;
    bool temporal_filter;
    bool view_evidence;
  };
  const bool evidence = cfg.view_evidence.enabled;
  std::vector<Configuration> configurations;
  for (const bool filter : {false, true}) {
    const std::string suffix = filter ? "+ekf" : "";
    for (const std::string& view : cfg.view_names) {
      configurations.push_back({view + suffix, {view}, FusionMode::kGated, filter, false});
    }
    configurations.push_back({"naive" + suffix, cfg.view_names, FusionMode::kNaive, filter, false});
    if (evidence) {
      configurations.push_back(
          {"gate-score" + suffix, cfg.view_names, FusionMode::kGated, filter, false});
    }
    configurations.push_back(
        {filter ? "fused" : "gated", cfg.view_names, FusionMode::kGated, filter, evidence});
  }

  AblationResult result;
  for (const auto& [label, allowed_views, mode, temporal_filter, view_evidence] : configurations) {
    FusionConfig arm_cfg = cfg;
    arm_cfg.mode = mode;
    arm_cfg.temporal_filter = temporal_filter;
    arm_cfg.use_view_evidence = view_evidence;
    Fusion fusion(arm_cfg, extrinsics, extents);

    AblationRow row;
    row.label = label;
    row.views = allowed_views;
    row.mode = mode;
    row.temporal_filter = temporal_filter;
    row.by_score = evidence && !view_evidence && mode == FusionMode::kGated &&
                   allowed_views.size() > 1;
    double add_s_sum = 0.0;
    double fraction_sum = 0.0;
    int below_threshold = 0;
    std::map<std::string, double> class_add_s_sum;
    std::map<std::string, int> class_below_threshold;

    for (const AblationFrame& frame : frames) {
      std::vector<PoseEstimate> filtered;
      for (const PoseEstimate& e : frame.estimates) {
        if (std::find(allowed_views.begin(), allowed_views.end(), e.camera_id) !=
            allowed_views.end()) {
          filtered.push_back(e);
        }
      }
      const SceneState scene = fusion.Process(filtered, frame.stamp_ns);

      for (const GroundTruthObject& gt : frame.ground_truth) {
        ++row.ground_truth_objects;
        AblationClassStats& per_class = row.by_class[gt.cls];
        ++per_class.ground_truth_objects;

        const auto model = models.find(gt.cls);
        if (model == models.end()) {
          throw std::runtime_error("RunOcclusionAblation: no model registered for class '" +
                                   gt.cls + "'; ADD-S cannot be normalised without it");
        }
        if (!(model->second.diameter_m > 0.0)) {
          throw std::runtime_error("RunOcclusionAblation: class '" + gt.cls +
                                   "' has diameter_m <= 0 (INTERFACES.md §7 requires it)");
        }

        // Match by nearest centroid within a gate. Class must agree, so a
        // confident estimate of the wrong class counts as a miss rather than
        // silently scoring against the wrong ground truth.
        const ObjectState* best = nullptr;
        double best_distance = match_max_distance_m;
        for (const ObjectState& o : scene.objects) {
          if (o.cls != gt.cls) continue;
          const double d = (o.pose_w.translation() - gt.pose_w.translation()).norm();
          if (d <= best_distance) {
            best_distance = d;
            best = &o;
          }
        }
        if (best == nullptr) {
          ++row.missed;
          ++per_class.missed;
          continue;
        }

        ++row.matched;
        ++per_class.matched;
        const double add_s = AddS(best->pose_w, gt.pose_w, model->second.points);
        const double fraction = add_s / model->second.diameter_m;
        add_s_sum += add_s;
        fraction_sum += fraction;
        class_add_s_sum[gt.cls] += add_s;
        if (fraction < 0.1) {
          ++below_threshold;
          ++class_below_threshold[gt.cls];
        }
      }
    }
    for (auto& [cls, stats] : row.by_class) {
      if (stats.matched > 0) stats.mean_add_s_m = class_add_s_sum[cls] / stats.matched;
      if (stats.ground_truth_objects > 0) {
        stats.recall_at_0p1_diameter =
            static_cast<double>(class_below_threshold[cls]) / stats.ground_truth_objects;
      }
    }

    if (row.matched > 0) {
      row.mean_add_s_m = add_s_sum / row.matched;
      row.mean_add_s_fraction = fraction_sum / row.matched;
    }
    if (row.ground_truth_objects > 0) {
      // Denominator is all GT instances: a view that misses the object entirely
      // must not be rewarded for the few it happens to see.
      row.recall_at_0p1_diameter =
          static_cast<double>(below_threshold) / row.ground_truth_objects;
    }
    result.rows.push_back(std::move(row));
  }
  return result;
}

}  // namespace the_eyes::fusion
