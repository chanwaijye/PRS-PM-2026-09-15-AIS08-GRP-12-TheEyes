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
  os << std::left << std::setw(12) << "view" << std::right << std::setw(8) << "gt"
     << std::setw(9) << "matched" << std::setw(8) << "missed" << std::setw(14) << "ADD-S (mm)"
     << std::setw(14) << "ADD-S (/diam)" << std::setw(14) << "recall@0.1d" << "\n";
  os << std::string(79, '-') << "\n";
  for (const AblationRow& r : rows) {
    os << std::left << std::setw(12) << r.label << std::right << std::setw(8)
       << r.ground_truth_objects << std::setw(9) << r.matched << std::setw(8) << r.missed
       << std::setw(14) << std::fixed << std::setprecision(2) << (r.mean_add_s_m * 1000.0)
       << std::setw(14) << std::setprecision(4) << r.mean_add_s_fraction << std::setw(14)
       << std::setprecision(4) << r.recall_at_0p1_diameter << "\n";
  }
  return os.str();
}

AblationResult RunOcclusionAblation(const std::vector<AblationFrame>& frames,
                                    const ExtrinsicsSet& extrinsics, const FusionConfig& cfg,
                                    const ExtentRegistry& extents, const ModelRegistry& models,
                                    double match_max_distance_m) {
  // Configurations: each single view alone, then all views fused. Single-view
  // rows are produced by filtering the SAME estimate stream, so any difference
  // between rows is attributable to fusion and nothing else.
  std::vector<std::pair<std::string, std::vector<std::string>>> configurations;
  for (const std::string& view : cfg.view_names) {
    configurations.emplace_back(view, std::vector<std::string>{view});
  }
  configurations.emplace_back("fused", cfg.view_names);

  AblationResult result;
  for (const auto& [label, allowed_views] : configurations) {
    Fusion fusion(cfg, extrinsics, extents);

    AblationRow row;
    row.label = label;
    double add_s_sum = 0.0;
    double fraction_sum = 0.0;
    int below_threshold = 0;

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
          continue;
        }

        ++row.matched;
        const double add_s = AddS(best->pose_w, gt.pose_w, model->second.points);
        const double fraction = add_s / model->second.diameter_m;
        add_s_sum += add_s;
        fraction_sum += fraction;
        if (fraction < 0.1) ++below_threshold;
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
