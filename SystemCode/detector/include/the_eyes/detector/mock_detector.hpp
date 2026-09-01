// Fixture-driven detection backend.
//
// AGENTS.md §3: "fixed bbox list from a JSON fixture -- lets pose_agent
// develop without a trained model." YAML rather than JSON so both this and
// every other config in the repo share one parser; YAML is a JSON superset, so
// a JSON fixture loads unchanged.
#ifndef THE_EYES_DETECTOR_MOCK_DETECTOR_HPP
#define THE_EYES_DETECTOR_MOCK_DETECTOR_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "the_eyes/detector/detector.hpp"
#include "the_eyes/detector/tracker.hpp"

namespace the_eyes::detector {

// One fixture entry: what the detector should report for a given camera/frame.
struct FixtureFrame {
  std::string camera_id;
  uint64_t stamp_ns = 0;
  std::vector<Detection> detections;  // track_id is assigned by the tracker
};

// Fixture file shape:
//
//   frames:
//     - camera_id: cam_front
//       stamp_ns: 1000000000
//       detections:
//         - cls: mustard_bottle
//           score: 0.93
//           bbox: [x, y, w, h]
//
// Throws std::runtime_error on a missing file or a malformed entry.
std::vector<FixtureFrame> LoadDetectionFixture(const std::string& yaml_path);

class MockDetector : public Detector {
 public:
  MockDetector(std::vector<FixtureFrame> frames, TrackingConfig tracking);

  // Returns an empty set for an unmatched (camera_id, stamp_ns) rather than
  // inventing detections -- a fixture gap should look like "nothing detected",
  // which is a real case the pipeline must handle, not like fabricated data.
  DetectionSet Detect(const DetectorInput& input) override;
  std::string name() const override { return "mock:fixture"; }

 private:
  std::vector<FixtureFrame> frames_;
  // One tracker per camera: track ids are stable WITHIN a camera only (§2).
  std::map<std::string, IouTracker> trackers_;
  TrackingConfig tracking_;
};

}  // namespace the_eyes::detector

#endif  // THE_EYES_DETECTOR_MOCK_DETECTOR_HPP
