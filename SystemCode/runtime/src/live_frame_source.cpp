#include "the_eyes/runtime/live_frame_source.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace the_eyes::runtime {

LiveFrameSource::LiveFrameSource(const camera::CameraAgentConfig& cameras, PairerConfig pairer,
                                 LiveConfig live, std::ostream& log)
    : camera_ids_(pairer.camera_ids), streams_(cameras.streams), live_(live), pairer_(pairer) {
  if (camera_ids_.size() < 2) {
    throw std::runtime_error("live: the runner needs two cameras; pairer has " +
                             std::to_string(camera_ids_.size()));
  }

  log << "=== opening cameras (by serial) ===\n";
  for (const std::string& id : camera_ids_) {
    const camera::CameraEntry& entry = cameras.at(id);   // throws if unconfigured
    cameras_[id] = std::make_unique<camera::RealSenseCamera>(entry, cameras.streams);
    const camera::RealSenseCamera& cam = *cameras_[id];
    log << "  " << id << " -> serial " << cam.serial() << ", depth scale " << cam.depth_scale_m()
        << " m/unit, sync mode " << cam.actual_sync_mode() << " (requested "
        << entry.inter_cam_sync_mode << ")\n";
    for (const std::string& w : cam.warnings()) log << "    WARNING: " << w << "\n";
    if (cam.actual_sync_mode() != entry.inter_cam_sync_mode) {
      throw std::runtime_error("live: camera '" + id + "' reports inter_cam_sync_mode " +
                               std::to_string(cam.actual_sync_mode()) + " but cameras.yaml asks for " +
                               std::to_string(entry.inter_cam_sync_mode) +
                               " -- the R4b hardware-sync mitigation is not in effect");
    }
    CameraMeta m;
    m.serial = cam.serial();
    m.depth_scale_m = cam.depth_scale_m();
    m.sync_mode_requested = entry.inter_cam_sync_mode;
    m.sync_mode_actual = cam.actual_sync_mode();
    meta_[id] = m;
  }

  for (const std::string& id : camera_ids_) grabbers_.emplace_back([this, id] { GrabLoop(id); });

  // Startup skew assertion over real pairs (after the grabbers' warm-up).
  log << "=== warm-up (" << live_.warmup_frames << " frames per camera, discarded) and sync check ("
      << cameras.sync.startup_check_frames << " pairs) ===\n";
  std::vector<camera::SkewSample> samples;
  const std::string& a = camera_ids_[0];
  const std::string& b = camera_ids_[1];
  for (int i = 0; i < cameras.sync.startup_check_frames; ++i) {
    std::optional<FramePair> pair;
    try {
      pair = pairer_.WaitPair(5000);
    } catch (const std::exception& e) {
      stop_.store(true);
      for (std::thread& t : grabbers_) t.join();
      throw std::runtime_error(std::string("live: ") + e.what());
    }
    if (!pair) {
      stop_.store(true);
      for (std::thread& t : grabbers_) t.join();
      throw std::runtime_error("live: no synchronised pair within 5 s during the startup check -- "
                               "a camera is not delivering frames");
    }
    samples.push_back({pair->frames.at(a).stamp_ns, pair->frames.at(b).stamp_ns});
  }
  startup_sync_ = camera::AnalyseSkew(samples, a, b, cameras.sync.max_skew_ms);
  log << std::fixed << std::setprecision(2) << "  " << a << " vs " << b << ", " << startup_sync_.samples
      << " pairs: mean |skew| " << startup_sync_.mean_skew_ms << " ms, p95 " << startup_sync_.p95_skew_ms
      << " ms, max " << startup_sync_.max_skew_ms << " ms, fixed offset "
      << startup_sync_.mean_signed_skew_ms << " ms, JITTER " << startup_sync_.jitter_ms
      << " ms sigma, outliers " << startup_sync_.outliers << "\n  verdict: " << startup_sync_.verdict
      << "\n";
  if (!startup_sync_.within_threshold) {
    stop_.store(true);
    for (std::thread& t : grabbers_) t.join();
    throw std::runtime_error("live: startup skew check failed: " + startup_sync_.verdict);
  }

  std::lock_guard<std::mutex> lock(meta_mutex_);
  const std::string domain = meta_.at(a).timestamp_domain;
  for (const std::string& id : camera_ids_) {
    if (meta_.at(id).timestamp_domain != domain) {
      stop_.store(true);
      throw std::runtime_error("live: timestamp domains differ (" + a + " '" + domain + "', " + id +
                               " '" + meta_.at(id).timestamp_domain +
                               "'); cross-camera stamps are not comparable");
    }
  }
  log << "  timestamp domain: " << domain << "\n";
}

LiveFrameSource::~LiveFrameSource() {
  stop_.store(true);
  for (std::thread& t : grabbers_) {
    if (t.joinable()) t.join();
  }
}

void LiveFrameSource::GrabLoop(const std::string& camera_id) {
  camera::RealSenseCamera& cam = *cameras_.at(camera_id);
  int seen = 0;
  while (!stop_.load()) {
    camera::CameraFrame frame;
    try {
      frame = cam.NextFrame(5000);
    } catch (const std::exception& e) {
      pairer_.Fail(camera_id, e.what());
      return;
    }
    if (seen == 0) {
      std::lock_guard<std::mutex> lock(meta_mutex_);
      meta_[camera_id].K = frame.K;
      meta_[camera_id].timestamp_domain = cam.timestamp_domain();
    }
    if (++seen <= live_.warmup_frames) continue;
    pairer_.Push(std::move(frame));
  }
}

std::optional<FramePair> LiveFrameSource::NextPair(int timeout_ms) {
  return pairer_.WaitPair(timeout_ms);
}

std::string LiveFrameSource::describe() const {
  std::ostringstream os;
  os << "live";
  for (const std::string& id : camera_ids_) os << " " << id << "=" << cameras_.at(id)->serial();
  os << " @ " << streams_.color_width << "x" << streams_.color_height << " " << streams_.fps << " fps";
  return os.str();
}

}  // namespace the_eyes::runtime
