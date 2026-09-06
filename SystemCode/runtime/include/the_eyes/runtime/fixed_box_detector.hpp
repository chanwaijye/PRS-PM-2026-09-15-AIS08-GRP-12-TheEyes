// A detector that returns the same hand-placed boxes on every frame.
//
// For exercising the pose, fusion and scene-graph chain on a real object the
// trained detector cannot see -- an off-palette test print, say -- or for
// isolating FoundationPose from detector flicker. Boxes go through the same
// IoU tracker as real detections so track ids are stable and FoundationPose
// registers once per object, not once per frame.
//
// Spec, one entry per camera separated by ';':
//   cam_front=red_cube,300,200,120,120;cam_over=red_cube,400,180,100,100
// Box is x,y,w,h in colour-frame pixels. A camera with no entry gets no
// detections. Never a measurement of anything: the log labels the backend
// "fixed:..." so nothing downstream can mistake it for detection output.
#ifndef THE_EYES_RUNTIME_FIXED_BOX_DETECTOR_HPP
#define THE_EYES_RUNTIME_FIXED_BOX_DETECTOR_HPP

#include <map>
#include <string>
#include <vector>

#include "the_eyes/detector/detector.hpp"
#include "the_eyes/detector/tracker.hpp"

namespace the_eyes::runtime {

using FixedBoxes = std::map<std::string, std::vector<Detection>>;   // camera_id -> boxes

// Throws std::runtime_error on a malformed spec, a non-positive box, or a
// duplicate camera.
FixedBoxes ParseFixedBoxes(const std::string& spec);

class FixedBoxDetector : public detector::Detector {
 public:
  FixedBoxDetector(FixedBoxes boxes, detector::TrackingConfig tracking);
  DetectionSet Detect(const detector::DetectorInput& input) override;
  std::string name() const override { return "fixed:boxes"; }
  const FixedBoxes& boxes() const { return boxes_; }

 private:
  FixedBoxes boxes_;
  detector::TrackingConfig tracking_;
  std::map<std::string, detector::IouTracker> trackers_;
};

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_FIXED_BOX_DETECTOR_HPP
