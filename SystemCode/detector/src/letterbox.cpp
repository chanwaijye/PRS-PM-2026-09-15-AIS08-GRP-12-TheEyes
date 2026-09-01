#include "the_eyes/detector/letterbox.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace the_eyes::detector {

cv::Rect2i LetterboxTransform::ToSource(const cv::Rect2f& network_box) const {
  if (scale <= 0.0) return cv::Rect2i();

  // Undo the padding first, then the scale -- the reverse order of Letterbox().
  const double x0 = (network_box.x - pad_x) / scale;
  const double y0 = (network_box.y - pad_y) / scale;
  const double x1 = (network_box.x + network_box.width - pad_x) / scale;
  const double y1 = (network_box.y + network_box.height - pad_y) / scale;

  const int cx0 = std::clamp(static_cast<int>(std::lround(x0)), 0, source_size.width);
  const int cy0 = std::clamp(static_cast<int>(std::lround(y0)), 0, source_size.height);
  const int cx1 = std::clamp(static_cast<int>(std::lround(x1)), 0, source_size.width);
  const int cy1 = std::clamp(static_cast<int>(std::lround(y1)), 0, source_size.height);

  if (cx1 <= cx0 || cy1 <= cy0) return cv::Rect2i();
  return cv::Rect2i(cx0, cy0, cx1 - cx0, cy1 - cy0);
}

LetterboxTransform Letterbox(const cv::Mat& src, cv::Mat* dst, int network_size,
                             const cv::Scalar& pad_value) {
  if (src.empty()) throw std::invalid_argument("Letterbox: empty source image");
  if (network_size <= 0) throw std::invalid_argument("Letterbox: network_size must be positive");
  if (dst == nullptr) throw std::invalid_argument("Letterbox: null destination");

  LetterboxTransform transform;
  transform.source_size = src.size();
  transform.network_size = cv::Size(network_size, network_size);

  // One scale for both axes -- that is what preserves aspect ratio.
  transform.scale = std::min(static_cast<double>(network_size) / src.cols,
                             static_cast<double>(network_size) / src.rows);

  const int scaled_w = std::max(1, static_cast<int>(std::lround(src.cols * transform.scale)));
  const int scaled_h = std::max(1, static_cast<int>(std::lround(src.rows * transform.scale)));

  cv::Mat resized;
  // INTER_LINEAR down and up: matches what ultralytics does at export time, and
  // a mismatch here shows up as a small systematic score difference against the
  // Python reference.
  cv::resize(src, resized, cv::Size(scaled_w, scaled_h), 0, 0, cv::INTER_LINEAR);

  // Centre the image; split odd padding so the left/top gets the smaller half,
  // matching ultralytics.
  const int total_pad_x = network_size - scaled_w;
  const int total_pad_y = network_size - scaled_h;
  transform.pad_x = total_pad_x / 2;
  transform.pad_y = total_pad_y / 2;

  cv::copyMakeBorder(resized, *dst, transform.pad_y, total_pad_y - transform.pad_y,
                     transform.pad_x, total_pad_x - transform.pad_x, cv::BORDER_CONSTANT,
                     pad_value);
  return transform;
}

}  // namespace the_eyes::detector
