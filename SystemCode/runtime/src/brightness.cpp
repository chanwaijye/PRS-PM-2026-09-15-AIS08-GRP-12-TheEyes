#include "the_eyes/runtime/brightness.hpp"

#include <algorithm>
#include <stdexcept>

namespace the_eyes::runtime {

double MeanLuma(const cv::Mat& rgb) {
  if (rgb.empty()) return 0.0;
  if (rgb.type() != CV_8UC3) throw std::invalid_argument("MeanLuma: want a CV_8UC3 RGB image");
  // Rec. 601 weights, on RGB order. A plain channel mean would let a red or a
  // blue object swing the gain the way no change of light does.
  const cv::Scalar m = cv::mean(rgb);
  return 0.299 * m[0] + 0.587 * m[1] + 0.114 * m[2];
}

double BrightnessGain(const cv::Mat& rgb, const BrightnessConfig& cfg) {
  if (!cfg.enabled) return 1.0;
  const double mean = MeanLuma(rgb);
  if (mean <= 0.0) return 1.0;
  return std::min(cfg.max_gain, std::max(1.0, cfg.target_mean / mean));
}

cv::Mat Brighten(const cv::Mat& rgb, double gain) {
  if (gain == 1.0 || rgb.empty()) return rgb;
  cv::Mat out;
  rgb.convertTo(out, -1, gain, 0.0);  // saturate_cast per channel
  return out;
}

}  // namespace the_eyes::runtime
