// Brightness normalisation of the colour image, ahead of the detector.
//
// The detector was trained on synthetic frames whose mean is about 200 and
// whose darkest one per cent is 132 (docs/STATUS.md, "Sim-to-real"). The rig
// cannot expose into that range at 30 fps, and what it does deliver depends on
// the light in the room. MEASURED on 2026-10-05, the object unoccluded:
//
//   frame mean (cam_over)   85    60    47    28    23    37 (gain 128)
//   found on                45/45 300/300 300/300 0/300 0/300 7/300
//
// So below a frame mean of about 40 a view goes blind to an object it can see
// perfectly well. Camera gain does not fix it -- at 128 the same view found the
// bottle on 45/45 pairs and then 7/300 a minute later, 2 grey levels darker,
// and in brighter light the same gain washes the picture out. But the
// information is in the recording: the gain-64 frames multiplied by 2, 3 or 4
// before the detector gave 60/60 in both views at scores of 0.89-0.95.
//
// So the pipeline scales each frame so its mean luma reaches a target, by a
// gain clamped to [1, max_gain]. It never darkens (a washed-out frame has
// already lost what it clipped), and past max_gain it stops rather than
// amplify noise into a confident guess. The gain is per frame and per view,
// because the two cameras see different amounts of the room's light.
#ifndef THE_EYES_RUNTIME_BRIGHTNESS_HPP
#define THE_EYES_RUNTIME_BRIGHTNESS_HPP

#include <opencv2/core.hpp>

namespace the_eyes::runtime {

struct BrightnessConfig {
  bool enabled = false;
  double target_mean = 100.0;  // luma, 0-255
  double max_gain = 4.0;
  // Whether the pose estimator gets the brightened image too, or the raw one.
  bool apply_to_pose = false;
};

// Mean luma of a CV_8UC3 RGB image, 0-255. 0 for an empty image.
double MeanLuma(const cv::Mat& rgb);

// clamp(target_mean / MeanLuma(rgb), 1, max_gain). 1.0 when disabled, and for
// an empty or completely black image, where there is nothing to scale.
double BrightnessGain(const cv::Mat& rgb, const BrightnessConfig& cfg);

// rgb * gain, saturating at 255, as a NEW image: the caller's pixels are never
// touched, because the same buffer may be queued for the recorder, and a
// recording is the raw sensor. gain == 1 returns the input header unchanged.
cv::Mat Brighten(const cv::Mat& rgb, double gain);

}  // namespace the_eyes::runtime

#endif  // THE_EYES_RUNTIME_BRIGHTNESS_HPP
