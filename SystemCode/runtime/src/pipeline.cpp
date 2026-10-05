#include "the_eyes/runtime/pipeline.hpp"

#include <sstream>
#include <stdexcept>

#include "the_eyes/pose/foundationpose_client.hpp"
#include "the_eyes/runtime/clock.hpp"
#include "the_eyes/scenegraph/pddl.hpp"
#if THE_EYES_WITH_ONNXRUNTIME
#include "the_eyes/detector/onnx_detector.hpp"
#endif

namespace the_eyes::runtime {

fusion::ExtentRegistry ExtentsFrom(const pose::MeshRegistry& registry) {
  fusion::ExtentRegistry out;
  for (const std::string& cls : registry.classes()) {
    if (const MeshEntry* e = registry.Find(cls)) out[cls] = e->extent;
  }
  return out;
}

std::map<std::string, SymmetryGroup> SymmetriesFrom(const pose::MeshRegistry& registry) {
  std::map<std::string, SymmetryGroup> out;
  for (const std::string& cls : registry.classes()) {
    const MeshEntry* e = registry.Find(cls);
    if (e != nullptr && e->symmetry_group != SymmetryGroup::kNone) out[cls] = e->symmetry_group;
  }
  return out;
}

Pipeline::Pipeline(DetectorHandle detector, PoseHandle pose,
                   std::map<std::string, std::unique_ptr<pose::PoseAgent>> agents,
                   std::unique_ptr<fusion::Fusion> fusion, scenegraph::PredicateConfig predicates,
                   StaticScene statics, std::vector<std::string> view_names)
    : detector_(std::move(detector)),
      pose_(std::move(pose)),
      agents_(std::move(agents)),
      fusion_(std::move(fusion)),
      predicates_(std::move(predicates)),
      statics_(std::move(statics)),
      view_names_(std::move(view_names)) {
  if (!detector_.detector) throw std::invalid_argument("Pipeline: null detector");
  if (!fusion_) throw std::invalid_argument("Pipeline: null fusion");
  if (view_names_.empty()) throw std::invalid_argument("Pipeline: no views");
  for (const std::string& v : view_names_) {
    if (!agents_.count(v)) throw std::invalid_argument("Pipeline: no pose agent for view '" + v + "'");
  }
#if THE_EYES_WITH_ONNXRUNTIME
  // Construction-time warnings (provider fallback) are the caller's to report
  // once; only warnings raised while processing belong to a pair.
  if (detector_.onnx) detector_warnings_seen_ = detector_.onnx->warnings().size();
#endif
}

PipelineResult Pipeline::Process(const FramePair& pair) {
  PipelineResult r;
  const Clock::time_point start = Clock::now();
  r.pair_index = pair.pair_index;
  r.stamp_ns = pair.stamp_ns;
  r.skew_ms = pair.skew_ms;
  r.frames_dropped_since_last = pair.frames_dropped_since_last;

  std::vector<PoseEstimate> all_estimates;
  for (const std::string& view : view_names_) {
    auto it = pair.frames.find(view);
    if (it == pair.frames.end()) {
      r.warnings.emplace_back("pairer", "pair " + std::to_string(pair.pair_index) +
                                            " has no frame for '" + view + "'");
      continue;
    }
    const camera::CameraFrame& frame = it->second;
    r.view_stamps[view] = {frame.stamp_ns, frame.frame_number};

    // INTERFACES.md §2: a frame without aligned depth is skipped, never
    // zero-filled -- the 4-channel network would read zeros as "at zero range".
    if (!frame.valid()) {
      r.warnings.emplace_back("camera", view + ": frame " + std::to_string(frame.frame_number) +
                                            " is not a valid RGB-D frame; skipped");
      continue;
    }

    // The colour image the networks see. `frame.rgb` itself is left alone: the
    // same buffer may be on its way to the recorder.
    const double gain = BrightnessGain(frame.rgb, brightness_);
    const cv::Mat rgb_normalised = Brighten(frame.rgb, gain);
    r.brightness_gain[view] = gain;

    DetectionSet dets;
    {
      double ms = 0.0;
      {
        ScopedTimer t(&ms);
        try {
          detector::DetectorInput in = ToDetectorInput(frame);
          in.rgb = rgb_normalised;
          in.depth_scale_m *= depth_correction_.For(view);
          dets = detector_.detector->Detect(in);
        } catch (const std::exception& e) {
          r.warnings.emplace_back("detector", view + ": " + e.what());
          dets.camera_id = view;
          dets.stamp_ns = frame.stamp_ns;
        }
      }
      r.timings.detect_ms[view] = ms;
      latency_["detect:" + view].Record(ms);
    }
#if THE_EYES_WITH_ONNXRUNTIME
    if (detector_.onnx) {
      const auto& w = detector_.onnx->warnings();
      for (size_t i = detector_warnings_seen_; i < w.size(); ++i) r.warnings.emplace_back("detector", w[i]);
      detector_warnings_seen_ = w.size();
    }
#endif
    r.detections[view] = dets;

    {
      pose::PoseAgent& agent = *agents_.at(view);
      double ms = 0.0;
      std::vector<PoseEstimate> est;
      {
        ScopedTimer t(&ms);
        try {
          pose::Frame pf = ToPoseFrame(frame);
          if (brightness_.apply_to_pose) pf.rgb = rgb_normalised;
          pf.depth_scale_m *= depth_correction_.For(view);
          est = agent.Process(dets, pf);
        } catch (const std::exception& e) {
          r.warnings.emplace_back("pose", view + ": " + e.what());
        }
      }
      r.timings.pose_ms[view] = ms;
      latency_["pose:" + view].Record(ms);
      r.pose_diag[view] = agent.diagnostics();
      for (const std::string& w : agent.diagnostics().warnings) r.warnings.emplace_back("pose", view + ": " + w);
      all_estimates.insert(all_estimates.end(), est.begin(), est.end());
      r.estimates[view] = std::move(est);
    }
  }
  if (pose_.client) {
    for (const std::string& w : pose_.client->warnings()) r.warnings.emplace_back("pose", w);
    pose_.client->ClearWarnings();
  }

  {
    double ms = 0.0;
    {
      ScopedTimer t(&ms);
      r.scene = fusion_->Process(all_estimates, pair.stamp_ns);
    }
    r.timings.fusion_ms = ms;
    latency_["fusion"].Record(ms);
    r.fusion_diag = fusion_->diagnostics();
    for (const std::string& w : r.fusion_diag.warnings) r.warnings.emplace_back("fusion", w);
  }

  {
    double ms = 0.0;
    {
      ScopedTimer t(&ms);
      statics_.Inject(r.scene, view_names_);
      for (const StaticObject& s : statics_.objects) r.static_ids.push_back(s.id);
      // Annotate reports the unsupported objects under fusion's own ids;
      // EmitInit's copy is after PDDL renaming, and ids are the join key.
      r.unsupported_ids = scenegraph::Annotate(r.scene, predicates_).unsupported;
      r.support_checked = predicates_.require_support;
      r.pddl = scenegraph::EmitInit(r.scene, predicates_);
    }
    r.timings.scenegraph_ms = ms;
    latency_["scenegraph"].Record(ms);
    for (const std::string& w : r.pddl.warnings) r.warnings.emplace_back("scenegraph", w);
  }
  r.timings.total_ms = ElapsedMs(start);
  latency_["total"].Record(r.timings.total_ms);
  return r;
}

std::string Pipeline::Summary() const {
  std::ostringstream os;
  for (const std::string& v : view_names_) {
    auto d = latency_.find("detect:" + v);
    if (d != latency_.end()) os << d->second.Format("detector " + v + " (" + detector_.label + ")") << "\n";
    auto p = latency_.find("pose:" + v);
    if (p != latency_.end()) os << p->second.Format("pose_agent " + v + " (" + pose_.label + ")") << "\n";
  }
  for (const char* stage : {"fusion", "scenegraph", "total"}) {
    auto it = latency_.find(stage);
    if (it != latency_.end()) os << it->second.Format(stage) << "\n";
  }
  return os.str();
}

}  // namespace the_eyes::runtime
