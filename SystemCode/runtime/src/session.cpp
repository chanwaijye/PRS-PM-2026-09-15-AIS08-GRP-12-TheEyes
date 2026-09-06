#include "the_eyes/runtime/session.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace the_eyes::runtime {
namespace {

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string field;
  std::istringstream is(line);
  while (std::getline(is, field, ',')) out.push_back(field);
  if (!line.empty() && line.back() == ',') out.emplace_back();
  return out;
}

}  // namespace

void WriteSessionMeta(const SessionMeta& meta, const std::filesystem::path& yaml_path) {
  YAML::Emitter out;
  out.SetDoublePrecision(17);
  out << YAML::BeginMap;
  out << YAML::Key << "format_version" << YAML::Value << meta.format_version;
  out << YAML::Key << "session_id" << YAML::Value << meta.session_id;
  out << YAML::Key << "created_utc" << YAML::Value << meta.created_utc;
  out << YAML::Key << "source" << YAML::Value << meta.source;
  out << YAML::Key << "camera_ids" << YAML::Value << YAML::Flow << meta.camera_ids;
  out << YAML::Key << "max_skew_ms" << YAML::Value << meta.max_skew_ms;
  out << YAML::Key << "streams" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "width" << YAML::Value << meta.width;
  out << YAML::Key << "height" << YAML::Value << meta.height;
  out << YAML::Key << "fps" << YAML::Value << meta.fps;
  out << YAML::Key << "align_depth_to_color" << YAML::Value << meta.align_depth_to_color;
  out << YAML::EndMap;
  out << YAML::Key << "cameras" << YAML::Value << YAML::BeginMap;
  for (const auto& [id, cam] : meta.cameras) {
    out << YAML::Key << id << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "serial" << YAML::Value << YAML::DoubleQuoted << cam.serial;
    out << YAML::Key << "depth_scale_m" << YAML::Value << cam.depth_scale_m;
    std::vector<double> k(9);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) k[static_cast<size_t>(r * 3 + c)] = cam.K(r, c);
    out << YAML::Key << "K" << YAML::Value << YAML::Flow << k;
    out << YAML::Key << "inter_cam_sync_mode_requested" << YAML::Value << cam.sync_mode_requested;
    out << YAML::Key << "inter_cam_sync_mode_actual" << YAML::Value << cam.sync_mode_actual;
    out << YAML::Key << "timestamp_domain" << YAML::Value << cam.timestamp_domain;
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
  if (meta.startup_sync) {
    const camera::SyncReport& s = *meta.startup_sync;
    out << YAML::Key << "startup_sync" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "camera_a" << YAML::Value << s.camera_a;
    out << YAML::Key << "camera_b" << YAML::Value << s.camera_b;
    out << YAML::Key << "samples" << YAML::Value << s.samples;
    out << YAML::Key << "mean_skew_ms" << YAML::Value << s.mean_skew_ms;
    out << YAML::Key << "max_skew_ms" << YAML::Value << s.max_skew_ms;
    out << YAML::Key << "p95_skew_ms" << YAML::Value << s.p95_skew_ms;
    out << YAML::Key << "mean_signed_skew_ms" << YAML::Value << s.mean_signed_skew_ms;
    out << YAML::Key << "jitter_ms" << YAML::Value << s.jitter_ms;
    out << YAML::Key << "outliers" << YAML::Value << s.outliers;
    out << YAML::Key << "within_threshold" << YAML::Value << s.within_threshold;
    out << YAML::Key << "verdict" << YAML::Value << s.verdict;
    out << YAML::EndMap;
  }
  out << YAML::Key << "pairs_written" << YAML::Value << meta.pairs_written;
  out << YAML::Key << "pairs_dropped" << YAML::Value << meta.pairs_dropped;
  out << YAML::EndMap;

  std::ofstream f(yaml_path);
  if (!f) throw std::runtime_error("session: cannot write '" + yaml_path.string() + "'");
  f << "# Recorded by the_eyes_run (SystemCode/runtime). Do not hand-edit.\n" << out.c_str()
    << "\n";
}

SessionMeta ReadSessionMeta(const std::filesystem::path& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("session: cannot parse '" + yaml_path.string() + "': " + e.what());
  }
  SessionMeta meta;
  if (!root["format_version"]) throw std::runtime_error("session.yaml: missing format_version");
  meta.format_version = root["format_version"].as<int>();
  if (meta.format_version != kSessionFormatVersion) {
    throw std::runtime_error("session.yaml: format_version " +
                             std::to_string(meta.format_version) + " but this reader knows " +
                             std::to_string(kSessionFormatVersion));
  }
  auto str = [&](const char* key) -> std::string {
    return root[key] ? root[key].as<std::string>() : std::string();
  };
  meta.session_id = str("session_id");
  meta.created_utc = str("created_utc");
  meta.source = str("source");
  if (!root["camera_ids"] || !root["camera_ids"].IsSequence() || root["camera_ids"].size() == 0) {
    throw std::runtime_error("session.yaml: camera_ids must be a non-empty sequence");
  }
  meta.camera_ids = root["camera_ids"].as<std::vector<std::string>>();
  meta.max_skew_ms = root["max_skew_ms"] ? root["max_skew_ms"].as<double>() : 33.0;
  if (root["streams"]) {
    const YAML::Node s = root["streams"];
    meta.width = s["width"].as<int>(0);
    meta.height = s["height"].as<int>(0);
    meta.fps = s["fps"].as<int>(0);
    meta.align_depth_to_color = s["align_depth_to_color"].as<bool>(true);
  }
  const YAML::Node cams = root["cameras"];
  if (!cams || !cams.IsMap()) throw std::runtime_error("session.yaml: missing 'cameras' map");
  for (const std::string& id : meta.camera_ids) {
    const YAML::Node c = cams[id];
    if (!c || !c.IsMap()) throw std::runtime_error("session.yaml: no cameras entry for '" + id + "'");
    if (!c["serial"] || !c["K"] || !c["depth_scale_m"]) {
      throw std::runtime_error("session.yaml: camera '" + id +
                               "' needs serial, K and depth_scale_m");
    }
    CameraMeta cam;
    cam.serial = c["serial"].as<std::string>();
    cam.depth_scale_m = c["depth_scale_m"].as<double>();
    const auto k = c["K"].as<std::vector<double>>();
    if (k.size() != 9) throw std::runtime_error("session.yaml: camera '" + id + "' K must have 9 entries");
    for (int r = 0; r < 3; ++r)
      for (int col = 0; col < 3; ++col) cam.K(r, col) = k[static_cast<size_t>(r * 3 + col)];
    cam.sync_mode_requested = c["inter_cam_sync_mode_requested"].as<int>(0);
    cam.sync_mode_actual = c["inter_cam_sync_mode_actual"].as<int>(0);
    cam.timestamp_domain = c["timestamp_domain"].as<std::string>("");
    meta.cameras.emplace(id, cam);
  }
  if (root["startup_sync"]) {
    const YAML::Node s = root["startup_sync"];
    camera::SyncReport r;
    r.camera_a = s["camera_a"].as<std::string>("");
    r.camera_b = s["camera_b"].as<std::string>("");
    r.samples = s["samples"].as<int>(0);
    r.mean_skew_ms = s["mean_skew_ms"].as<double>(0.0);
    r.max_skew_ms = s["max_skew_ms"].as<double>(0.0);
    r.p95_skew_ms = s["p95_skew_ms"].as<double>(0.0);
    r.mean_signed_skew_ms = s["mean_signed_skew_ms"].as<double>(0.0);
    r.jitter_ms = s["jitter_ms"].as<double>(0.0);
    r.outliers = s["outliers"].as<int>(0);
    r.within_threshold = s["within_threshold"].as<bool>(false);
    r.verdict = s["verdict"].as<std::string>("");
    meta.startup_sync = r;
  }
  meta.pairs_written = root["pairs_written"].as<uint64_t>(0);
  meta.pairs_dropped = root["pairs_dropped"].as<uint64_t>(0);
  return meta;
}

std::string IndexHeader(const std::vector<std::string>& camera_ids) {
  std::string h = "pair_index,stamp_ns,skew_ms";
  for (const std::string& id : camera_ids) h += "," + id + "_stamp_ns," + id + "_frame_number";
  h += ",frames_dropped_since_last";
  return h;
}

std::string FormatIndexRow(const IndexRow& row, const std::vector<std::string>& camera_ids) {
  std::ostringstream os;
  os << row.pair_index << ',' << row.stamp_ns << ',' << std::setprecision(6) << std::fixed
     << row.skew_ms;
  for (const std::string& id : camera_ids) {
    const auto& [stamp, number] = row.per_camera.at(id);
    os << ',' << stamp << ',' << number;
  }
  os << ',' << row.frames_dropped_since_last;
  return os.str();
}

std::vector<IndexRow> ReadIndex(const std::filesystem::path& csv_path,
                                const std::vector<std::string>& camera_ids) {
  std::ifstream f(csv_path);
  if (!f) throw std::runtime_error("session: cannot open index '" + csv_path.string() + "'");
  std::string line;
  if (!std::getline(f, line)) throw std::runtime_error("session: empty index '" + csv_path.string() + "'");
  const std::vector<std::string> header = SplitCsv(line);
  std::map<std::string, size_t> col;
  for (size_t i = 0; i < header.size(); ++i) col[header[i]] = i;
  auto need = [&](const std::string& name) -> size_t {
    auto it = col.find(name);
    if (it == col.end()) throw std::runtime_error("session: index has no column '" + name + "'");
    return it->second;
  };
  const size_t c_index = need("pair_index");
  const size_t c_stamp = need("stamp_ns");
  const size_t c_skew = need("skew_ms");
  const size_t c_drop = need("frames_dropped_since_last");
  std::map<std::string, std::pair<size_t, size_t>> c_cam;
  for (const std::string& id : camera_ids) {
    c_cam[id] = {need(id + "_stamp_ns"), need(id + "_frame_number")};
  }

  std::vector<IndexRow> rows;
  size_t line_no = 1;
  while (std::getline(f, line)) {
    ++line_no;
    if (line.empty()) continue;
    const std::vector<std::string> fields = SplitCsv(line);
    if (fields.size() < header.size()) {
      throw std::runtime_error("session: index line " + std::to_string(line_no) + " is truncated");
    }
    IndexRow row;
    row.pair_index = std::stoull(fields[c_index]);
    row.stamp_ns = std::stoull(fields[c_stamp]);
    row.skew_ms = std::stod(fields[c_skew]);
    row.frames_dropped_since_last = std::stoull(fields[c_drop]);
    for (const auto& [id, cols] : c_cam) {
      row.per_camera[id] = {std::stoull(fields[cols.first]), std::stoull(fields[cols.second])};
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

std::string FrameFileStem(uint64_t pair_index) {
  std::ostringstream os;
  os << std::setw(6) << std::setfill('0') << pair_index;
  return os.str();
}

std::filesystem::path RgbPath(const std::filesystem::path& session_dir,
                              const std::string& camera_id, uint64_t pair_index) {
  return session_dir / camera_id / (FrameFileStem(pair_index) + ".rgb.png");
}

std::filesystem::path DepthPath(const std::filesystem::path& session_dir,
                                const std::string& camera_id, uint64_t pair_index) {
  return session_dir / camera_id / (FrameFileStem(pair_index) + ".depth.png");
}

}  // namespace the_eyes::runtime
