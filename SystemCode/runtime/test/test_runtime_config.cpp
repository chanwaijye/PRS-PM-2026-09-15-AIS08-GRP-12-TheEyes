#include <cstdlib>
#include <fstream>

#include <gtest/gtest.h>

#include "the_eyes/runtime/runtime_config.hpp"

namespace {

using namespace the_eyes::runtime;

std::string WriteTemp(const std::string& name, const std::string& body) {
  char tmpl[] = "/tmp/the_eyes_rtcfg_XXXXXX";
  const char* d = mkdtemp(tmpl);
  if (d == nullptr) throw std::runtime_error("mkdtemp failed");
  const std::string path = std::string(d) + "/" + name;
  std::ofstream(path) << body;
  return path;
}

const char* kValid =
    "detector: {backend: mock, mock_fixture: f.yaml}\n"
    "pairing: {ring_depth: 3}\n"
    "live: {warmup_frames: 5, stall_timeout_ms: 500, max_consecutive_stalls: 2}\n"
    "recorder: {queue_capacity: 8, png_compression: 3, writer_threads: 1}\n"
    "output: {captures_dir: c, runs_dir: r, print_every: 7}\n";

}  // namespace

TEST(RuntimeConfig, LoadsTheShippedFile) {
  const RuntimeConfig cfg = LoadRuntimeConfig(std::string(THE_EYES_CONFIG_DIR) + "/runtime.yaml");
  EXPECT_EQ(cfg.detector.backend, "onnx");
  EXPECT_EQ(cfg.pairing.ring_depth, 4u);
  EXPECT_EQ(cfg.live.warmup_frames, 15);
  EXPECT_EQ(cfg.recorder.writer_threads, 2);
  EXPECT_EQ(cfg.output.print_every, 30);
}

TEST(RuntimeConfig, ParsesEveryKeyAndRejectsBadRanges) {
  const RuntimeConfig cfg = LoadRuntimeConfig(WriteTemp("ok.yaml", kValid));
  EXPECT_EQ(cfg.detector.backend, "mock");
  EXPECT_EQ(cfg.detector.mock_fixture, "f.yaml");
  EXPECT_EQ(cfg.pairing.ring_depth, 3u);
  EXPECT_EQ(cfg.live.stall_timeout_ms, 500);
  EXPECT_EQ(cfg.live.max_consecutive_stalls, 2);
  EXPECT_EQ(cfg.recorder.queue_capacity, 8u);
  EXPECT_EQ(cfg.recorder.png_compression, 3);
  EXPECT_EQ(cfg.output.captures_dir, "c");
  EXPECT_EQ(cfg.output.print_every, 7);

  std::string bad = kValid;
  bad.replace(bad.find("ring_depth: 3"), 13, "ring_depth: 0");
  EXPECT_THROW(LoadRuntimeConfig(WriteTemp("b1.yaml", bad)), std::runtime_error);
  bad = kValid;
  bad.replace(bad.find("queue_capacity: 8"), 17, "queue_capacity: 0");
  EXPECT_THROW(LoadRuntimeConfig(WriteTemp("b2.yaml", bad)), std::runtime_error);
  bad = kValid;
  bad.replace(bad.find("backend: mock"), 13, "backend: nope");
  EXPECT_THROW(LoadRuntimeConfig(WriteTemp("b3.yaml", bad)), std::runtime_error);
  EXPECT_THROW(LoadRuntimeConfig(WriteTemp("b4.yaml", "detector: {backend: onnx}\n")),
               std::runtime_error);
}

TEST(RuntimeConfig, ReadsThePoseBackendFromPoseYaml) {
  const PoseBackendConfig cfg = LoadPoseBackendConfig(std::string(THE_EYES_CONFIG_DIR) + "/pose.yaml");
  EXPECT_TRUE(cfg.backend == "mock" || cfg.backend == "foundationpose");
  EXPECT_EQ(cfg.socket_path, "/tmp/the_eyes_foundationpose.sock");
  EXPECT_GE(cfg.request_timeout_ms, 1);

  EXPECT_THROW(LoadPoseBackendConfig(WriteTemp("p1.yaml", "estimator: {backend: magic}\n")),
               std::runtime_error);
  EXPECT_THROW(LoadPoseBackendConfig(WriteTemp("p2.yaml", "roi: {}\n")), std::runtime_error);
  const PoseBackendConfig custom = LoadPoseBackendConfig(WriteTemp(
      "p3.yaml", "estimator: {backend: foundationpose, socket_path: /x.sock, request_timeout_ms: 9}\n"));
  EXPECT_EQ(custom.socket_path, "/x.sock");
  EXPECT_EQ(custom.request_timeout_ms, 9);
}
