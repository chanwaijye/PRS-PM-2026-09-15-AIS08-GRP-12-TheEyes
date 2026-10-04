#include "the_eyes/fusion/fusion.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "the_eyes/fusion/quaternion_average.hpp"
#include "the_eyes/fusion/so3.hpp"
#include "the_eyes/fusion/symmetry.hpp"

namespace the_eyes::fusion {
namespace {

// One per-view estimate, already lifted into the world frame.
struct WorldEstimate {
  std::string camera_id;
  std::string cls;
  Eigen::Isometry3d pose_w = Eigen::Isometry3d::Identity();
  double score = 0.0;
  bool symmetric = false;
  // The class's declared symmetry group, kNone when there is none on record.
  SymmetryGroup group = SymmetryGroup::kNone;
  Eigen::Vector3d extent = Eigen::Vector3d::Zero();
  uint32_t view_bit = 0;
};

SymmetryGroup GroupOf(const FusionConfig& cfg, const std::string& cls) {
  const auto it = cfg.symmetry.find(cls);
  return it == cfg.symmetry.end() ? SymmetryGroup::kNone : it->second;
}

// Deterministic ordering: fusion output must not depend on the order the pose
// estimators happened to publish in.
bool LessDeterministic(const WorldEstimate& a, const WorldEstimate& b) {
  if (a.cls != b.cls) return a.cls < b.cls;
  if (a.camera_id != b.camera_id) return a.camera_id < b.camera_id;
  const Eigen::Vector3d& pa = a.pose_w.translation();
  const Eigen::Vector3d& pb = b.pose_w.translation();
  if (pa.x() != pb.x()) return pa.x() < pb.x();
  if (pa.y() != pb.y()) return pa.y() < pb.y();
  return pa.z() < pb.z();
}

Eigen::Vector3d UnweightedCentroid(const std::vector<const WorldEstimate*>& group) {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const WorldEstimate* e : group) sum += e->pose_w.translation();
  return sum / static_cast<double>(group.size());
}

Eigen::Vector3d WeightedCentroid(const std::vector<const WorldEstimate*>& group) {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  double weight = 0.0;
  for (const WorldEstimate* e : group) {
    sum += e->score * e->pose_w.translation();
    weight += e->score;
  }
  if (weight <= 0.0) {
    // All-zero scores: fall back to the unweighted mean rather than dividing by
    // zero. A detector emitting score 0 is a bug upstream, not a reason to NaN.
    return UnweightedCentroid(group);
  }
  return sum / weight;
}

Eigen::Vector3d FuseTranslation(const std::vector<const WorldEstimate*>& group, FusionMode mode) {
  return (mode == FusionMode::kNaive) ? UnweightedCentroid(group) : WeightedCentroid(group);
}

Eigen::Quaterniond FuseOrientation(const std::vector<const WorldEstimate*>& group,
                                   FusionMode mode) {
  // Markley averaging itself is used by BOTH modes, deliberately. The naive
  // baseline is meant to isolate what this module's policy buys, and swapping
  // in coefficient-averaging here would not measure that: at the angular
  // separations two calibrated cameras of the same static object produce, the
  // two averages agree to well inside the per-view noise. Putting it in the
  // baseline would manufacture a difference rather than measure one.
  //
  // What kNaive DOES drop is the symmetry special-case and the score weighting
  // below -- those are policy, and policy is what the ablation is comparing.
  if (mode == FusionMode::kNaive) {
    if (group.size() == 1) {
      return Eigen::Quaterniond(group.front()->pose_w.rotation()).normalized();
    }
    std::vector<Eigen::Quaterniond> quats;
    quats.reserve(group.size());
    for (const WorldEstimate* e : group) {
      quats.emplace_back(Eigen::Quaterniond(e->pose_w.rotation()).normalized());
    }
    return WeightedQuaternionAverage(quats, std::vector<double>(quats.size(), 1.0));
  }

  // Symmetric objects: averaging is not merely inaccurate, it is wrong. Two
  // views of a symmetric object legitimately report orientations differing by a
  // symmetry-group element, and the Markley mean of those lands on a rotation
  // that neither view saw and that no symmetry element explains.
  const WorldEstimate* best = *std::max_element(
      group.begin(), group.end(),
      [](const WorldEstimate* a, const WorldEstimate* b) { return a->score < b->score; });
  const Eigen::Quaterniond best_q = Eigen::Quaterniond(best->pose_w.rotation()).normalized();
  if (group.size() == 1) return best_q;

  // When the class's group is declared the difference can be REMOVED rather
  // than avoided: map every view's orientation to the member of its symmetry
  // class nearest the best-scoring view's, and average those. The views then
  // contribute to the orientation the way they already do to the translation.
  const SymmetryGroup sym = best->group;
  if (sym != SymmetryGroup::kNone) {
    std::vector<Eigen::Quaterniond> quats;
    std::vector<double> weights;
    for (const WorldEstimate* e : group) {
      quats.push_back(NearestEquivalent(Eigen::Quaterniond(e->pose_w.rotation()), best_q, sym));
      weights.push_back(std::max(e->score, 1e-9));
    }
    return WeightedQuaternionAverage(quats, weights);
  }

  // Flagged symmetric with no group on record: nothing says which orientations
  // are the same, so take the highest-scoring view's and combine nothing.
  const bool symmetric =
      std::any_of(group.begin(), group.end(), [](const WorldEstimate* e) { return e->symmetric; });
  if (symmetric) return best_q;

  std::vector<Eigen::Quaterniond> quats;
  std::vector<double> weights;
  quats.reserve(group.size());
  weights.reserve(group.size());
  for (const WorldEstimate* e : group) {
    quats.emplace_back(Eigen::Quaterniond(e->pose_w.rotation()).normalized());
    weights.push_back(std::max(e->score, 1e-9));
  }
  return WeightedQuaternionAverage(quats, weights);
}

// Drops views that disagree with the consensus beyond the configured gate.
// Returns the survivors; sets *rejected_bits and *two_view_conflict.
std::vector<const WorldEstimate*> ApplyDisagreementGate(
    const std::vector<const WorldEstimate*>& group, const DisagreementGateConfig& gate,
    uint32_t* rejected_bits, bool* two_view_conflict) {
  *rejected_bits = 0;
  *two_view_conflict = false;

  // All return paths build this explicitly. Returning a braced single-element
  // list here instead makes GCC 13 mis-infer the vector length when it inlines
  // through FuseFrame and emit a spurious -Warray-bounds.
  std::vector<const WorldEstimate*> survivors;
  if (group.size() < 2) {
    survivors = group;
    return survivors;
  }

  if (group.size() == 2) {
    // With two cameras there is no majority to appeal to: the consensus sits
    // exactly between them, so both views are equally far from it and a
    // residual test would reject both or neither. Compare them to each other
    // instead, and when they conflict keep the higher-scoring one -- recording
    // that we guessed, rather than reporting a consensus that does not exist.
    const double dt = (group[0]->pose_w.translation() - group[1]->pose_w.translation()).norm();
    // Measured with the class's symmetry removed when it is declared: 88 deg
    // between two views of a cube is agreement, 40 deg is not, and only the
    // group can tell them apart.
    const SymmetryGroup sym = group[0]->group;
    const double dr = AngularDistanceModulo(Eigen::Quaterniond(group[0]->pose_w.rotation()),
                                            Eigen::Quaterniond(group[1]->pose_w.rotation()), sym);
    const bool translation_conflict = dt > gate.max_translation_m;
    // Flagged symmetric with NO group on record: orientation disagreement is
    // expected and cannot be interpreted, so it must not trip the gate.
    const bool uninterpretable =
        sym == SymmetryGroup::kNone && (group[0]->symmetric || group[1]->symmetric);
    const bool rotation_conflict = !uninterpretable && dr > gate.max_rotation_rad;
    if (!translation_conflict && !rotation_conflict) {
      survivors = group;
      return survivors;
    }

    *two_view_conflict = true;
    const size_t keep = (group[0]->score >= group[1]->score) ? 0 : 1;
    *rejected_bits = group[1 - keep]->view_bit;
    survivors.push_back(group[keep]);
    return survivors;
  }

  // Three or more views: there is a majority, so measure each view against the
  // consensus and drop the outliers.
  const Eigen::Vector3d consensus_t = WeightedCentroid(group);
  const Eigen::Quaterniond consensus_r = FuseOrientation(group, FusionMode::kGated);

  for (const WorldEstimate* e : group) {
    const double dt = (e->pose_w.translation() - consensus_t).norm();
    const double dr = AngularDistanceModulo(Eigen::Quaterniond(e->pose_w.rotation()),
                                            consensus_r, e->group);
    const bool uninterpretable = e->group == SymmetryGroup::kNone && e->symmetric;
    const bool rotation_conflict = !uninterpretable && dr > gate.max_rotation_rad;
    if (dt > gate.max_translation_m || rotation_conflict) {
      *rejected_bits |= e->view_bit;
    } else {
      survivors.push_back(e);
    }
  }
  if (survivors.empty()) {
    // Every view disagreed with the consensus, which means there is no
    // consensus. Keep the best-scoring one rather than emitting nothing.
    const WorldEstimate* best = *std::max_element(
        group.begin(), group.end(),
        [](const WorldEstimate* a, const WorldEstimate* b) { return a->score < b->score; });
    *rejected_bits &= ~best->view_bit;
    survivors.push_back(best);
    return survivors;
  }
  return survivors;
}

}  // namespace

std::vector<FusedObservation> FuseFrame(const std::vector<PoseEstimate>& estimates,
                                        const ExtrinsicsSet& extrinsics,
                                        const FusionConfig& cfg,
                                        const ExtentRegistry& extents,
                                        FusionDiagnostics* diagnostics) {
  // --- Lift every estimate into the world frame ------------------------------
  std::vector<WorldEstimate> world;
  world.reserve(estimates.size());
  for (const PoseEstimate& est : estimates) {
    const auto it = extrinsics.cameras.find(est.camera_id);
    if (it == extrinsics.cameras.end()) {
      if (diagnostics != nullptr) {
        diagnostics->warnings.push_back("dropped estimate from uncalibrated camera '" +
                                        est.camera_id + "'");
      }
      continue;
    }
    const uint32_t bit = ExtrinsicsSet::ViewBitFor(est.camera_id, cfg.view_names);
    if (bit == 0u) {
      if (diagnostics != nullptr) {
        diagnostics->warnings.push_back("dropped estimate from camera '" + est.camera_id +
                                        "': not present in fusion.yaml views.names, so it "
                                        "has no visible_views bit");
      }
      continue;
    }

    WorldEstimate w;
    w.camera_id = est.camera_id;
    w.cls = est.cls;
    w.pose_w = ToWorld(est, it->second);
    w.score = est.score;
    w.symmetric = est.symmetric;
    w.group = GroupOf(cfg, est.cls);
    w.view_bit = bit;
    w.extent = est.extent;
    if (w.extent.isZero()) {
      const auto reg = extents.find(est.cls);
      if (reg != extents.end()) w.extent = reg->second;
    }
    world.push_back(std::move(w));
  }

  std::sort(world.begin(), world.end(), LessDeterministic);

  // --- Associate across views ------------------------------------------------
  // Greedy nearest-neighbour within a class. A cluster holds at most one
  // estimate per camera: two detections from the same view are, by definition,
  // two different object instances.
  std::vector<std::vector<const WorldEstimate*>> clusters;
  for (const WorldEstimate& e : world) {
    int best = -1;
    double best_distance = cfg.association.max_distance_m;
    for (size_t c = 0; c < clusters.size(); ++c) {
      if (clusters[c].front()->cls != e.cls) continue;
      const bool camera_taken =
          std::any_of(clusters[c].begin(), clusters[c].end(),
                      [&e](const WorldEstimate* m) { return m->camera_id == e.camera_id; });
      if (camera_taken) continue;
      const double d = (WeightedCentroid(clusters[c]) - e.pose_w.translation()).norm();
      if (d <= best_distance) {
        best_distance = d;
        best = static_cast<int>(c);
      }
    }
    if (best >= 0) {
      clusters[static_cast<size_t>(best)].push_back(&e);
    } else {
      clusters.push_back({&e});
    }
  }

  // --- Gate and fuse ---------------------------------------------------------
  std::vector<FusedObservation> out;
  out.reserve(clusters.size());
  for (const std::vector<const WorldEstimate*>& cluster : clusters) {
    uint32_t rejected_bits = 0;
    bool two_view_conflict = false;
    // kNaive believes every view. Skipping the gate is the point of the
    // baseline, not an oversight: an outlier view that the gate would have
    // dropped is exactly what the naive arm is supposed to be damaged by.
    const std::vector<const WorldEstimate*> survivors =
        (cfg.mode == FusionMode::kNaive)
            ? cluster
            : ApplyDisagreementGate(cluster, cfg.disagreement_gate, &rejected_bits,
                                    &two_view_conflict);

    FusedObservation obs;
    obs.cls = survivors.front()->cls;
    obs.pose_w.translation() = FuseTranslation(survivors, cfg.mode);
    obs.pose_w.linear() = FuseOrientation(survivors, cfg.mode).toRotationMatrix();
    obs.rejected_views = rejected_bits;
    obs.two_view_conflict = two_view_conflict;

    double score_sum = 0.0;
    double weight_sum = 0.0;
    for (const WorldEstimate* e : survivors) {
      obs.visible_views |= e->view_bit;
      obs.symmetric = obs.symmetric || e->symmetric;
      if (!e->extent.isZero() && obs.extent.isZero()) obs.extent = e->extent;
      score_sum += e->score * e->score;
      weight_sum += e->score;
    }
    // Score-weighted mean of the surviving view scores. Deliberately NOT a
    // noisy-OR: agreement between views is already reported through
    // visible_views, and folding it into confidence too would double-count it.
    // kNaive takes the plain mean, matching its unweighted pose.
    if (cfg.mode == FusionMode::kNaive) {
      obs.confidence = weight_sum / static_cast<double>(survivors.size());
    } else {
      obs.confidence = (weight_sum > 0.0) ? score_sum / weight_sum : 0.0;
    }
    if (two_view_conflict) {
      obs.confidence *= cfg.disagreement_gate.two_view_conflict_confidence_penalty;
    }
    obs.confidence = std::min(1.0, std::max(0.0, obs.confidence));

    if (diagnostics != nullptr) {
      if (two_view_conflict) ++diagnostics->two_view_conflicts;
      for (uint32_t b = 0; b < 32; ++b) {
        if (rejected_bits & (1u << b)) ++diagnostics->views_rejected;
      }
    }
    out.push_back(std::move(obs));
  }
  return out;
}

Fusion::Fusion(FusionConfig cfg, ExtrinsicsSet extrinsics, ExtentRegistry extents)
    : cfg_(std::move(cfg)), extrinsics_(std::move(extrinsics)), extents_(std::move(extents)) {
  AssertCalibrationQuality(extrinsics_, cfg_.calibration.max_reprojection_rmse_px);
}

std::string Fusion::AllocateId(const std::string& cls) {
  const int n = ++class_counters_[cls];
  std::ostringstream os;
  os << cls << "_" << std::setw(2) << std::setfill('0') << n;
  return os.str();
}

SceneState Fusion::Process(const std::vector<PoseEstimate>& estimates, uint64_t stamp_ns) {
  diagnostics_ = FusionDiagnostics{};

  const double dt_s =
      (last_stamp_ns_ == 0 || stamp_ns <= last_stamp_ns_)
          ? 0.0
          : static_cast<double>(stamp_ns - last_stamp_ns_) * 1e-9;

  for (Track& t : tracks_) t.ekf->Predict(dt_s);

  const std::vector<FusedObservation> observations =
      FuseFrame(estimates, extrinsics_, cfg_, extents_, &diagnostics_);

  // --- Associate observations to existing tracks -----------------------------
  std::vector<bool> track_used(tracks_.size(), false);
  std::vector<std::string> emitted_ids(observations.size());

  for (size_t i = 0; i < observations.size(); ++i) {
    const FusedObservation& obs = observations[i];
    int best = -1;
    double best_distance = cfg_.tracking.track_match_max_distance_m;
    for (size_t t = 0; t < tracks_.size(); ++t) {
      if (track_used[t] || tracks_[t].cls != obs.cls) continue;
      const double d = (tracks_[t].ekf->position() - obs.pose_w.translation()).norm();
      if (d <= best_distance) {
        best_distance = d;
        best = static_cast<int>(t);
      }
    }

    if (best < 0) {
      Track t;
      t.id = AllocateId(obs.cls);
      t.cls = obs.cls;
      t.ekf = std::make_unique<ObjectEkf>(cfg_.ekf, obs.pose_w);
      t.last_seen_ns = stamp_ns;
      t.extent = obs.extent;
      emitted_ids[i] = t.id;
      tracks_.push_back(std::move(t));
      track_used.push_back(true);
      ++diagnostics_.tracks_created;
    } else {
      Track& t = tracks_[static_cast<size_t>(best)];
      // The filter's orientation innovation is the rotation from its state to
      // the measurement, and it has to be a small one for the linearisation to
      // mean anything. A symmetric object's fused orientation can legitimately
      // jump by a group element between frames -- whenever the best-scoring
      // view changes -- so the measurement is first moved to the equivalent
      // orientation nearest the track. Same pose, no jump. kNaive is defined as
      // having no symmetry handling, and that includes this.
      Eigen::Isometry3d measured = obs.pose_w;
      const SymmetryGroup sym = GroupOf(cfg_, obs.cls);
      if (cfg_.mode != FusionMode::kNaive && sym != SymmetryGroup::kNone) {
        measured.linear() = NearestEquivalent(Eigen::Quaterniond(obs.pose_w.rotation()),
                                              t.ekf->orientation(), sym)
                                .toRotationMatrix();
      }
      t.ekf->Update(measured, obs.confidence);
      t.last_seen_ns = stamp_ns;
      if (!obs.extent.isZero()) t.extent = obs.extent;
      track_used[static_cast<size_t>(best)] = true;
      emitted_ids[i] = t.id;
    }
  }

  // --- Emit ------------------------------------------------------------------
  SceneState scene;
  scene.stamp_ns = stamp_ns;
  scene.objects.reserve(observations.size());
  for (size_t i = 0; i < observations.size(); ++i) {
    const FusedObservation& obs = observations[i];
    const auto it = std::find_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track& t) { return t.id == emitted_ids[i]; });
    if (it == tracks_.end()) continue;

    ObjectState o;
    o.id = it->id;
    o.cls = obs.cls;
    o.confidence = obs.confidence;
    o.visible_views = obs.visible_views;
    o.extent = it->extent;
    o.stamp_ns = stamp_ns;
    // relations stays empty: it is the scenegraph module's field to fill.

    if (!cfg_.temporal_filter) {
      // No temporal filter: the raw per-frame fuse IS the answer. The
      // uncertainty fields stay at their "not supplied" sentinel: the EKF ran
      // (track association reads its position, and that must stay identical
      // across ablation arms so they compare fusion policy and not association)
      // but this arm does not use its output, so reporting its covariance would
      // credit it with a filter it is defined not to have. Independent of
      // `mode` on purpose -- see FusionConfig::temporal_filter.
      o.pose_w = obs.pose_w;
    } else {
      o.pose_w = it->ekf->pose();  // temporally smoothed, not the raw fuse
      // The filter's own covariance, which nothing consumed before this. See
      // ObjectState in types.hpp for why this is not the same thing as
      // `confidence` and why a consumer must check has_uncertainty() first.
      o.position_sigma_m = it->ekf->position_sigma_m();
      o.orientation_sigma_rad = it->ekf->orientation_sigma_rad();
      o.velocity_w = it->ekf->velocity();
    }

    if (o.extent.isZero()) {
      diagnostics_.warnings.push_back(
          "object '" + o.id + "' (class '" + o.cls +
          "') has zero extent: no per-estimate extent and no entry in the extent "
          "registry. Downstream predicates over it will be degenerate.");
    }
    if (is_invalid(o)) continue;  // INTERFACES.md §5: never emit visible_views == 0
    scene.objects.push_back(std::move(o));
  }

  // Deterministic output order regardless of association order.
  std::sort(scene.objects.begin(), scene.objects.end(),
            [](const ObjectState& a, const ObjectState& b) { return a.id < b.id; });

  // --- Retire stale tracks ---------------------------------------------------
  // A track that missed this frame is kept, not emitted: its visible_views would
  // be 0. Holding it lets a briefly-occluded object keep its id on reappearance.
  const auto stale = [&](const Track& t) {
    if (t.last_seen_ns == stamp_ns) return false;
    if (stamp_ns <= t.last_seen_ns) return false;
    const double age_s = static_cast<double>(stamp_ns - t.last_seen_ns) * 1e-9;
    return age_s > cfg_.ekf.max_coast_s;
  };
  const size_t before = tracks_.size();
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), stale), tracks_.end());
  diagnostics_.tracks_retired = static_cast<int>(before - tracks_.size());

  last_stamp_ns_ = stamp_ns;
  return scene;
}

}  // namespace the_eyes::fusion
