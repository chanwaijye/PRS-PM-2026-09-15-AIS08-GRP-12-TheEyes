#include "the_eyes/pose/roi.hpp"

#include <algorithm>
#include <cmath>

namespace the_eyes::pose {

cv::Rect2i ComputeRoi(const cv::Rect2i& bbox, int image_width, int image_height,
                      const RoiConfig& cfg) {
  const cv::Rect2i empty;
  if (image_width <= 0 || image_height <= 0) return empty;
  if (bbox.width <= 0 || bbox.height <= 0) return empty;

  const int margin_x = static_cast<int>(std::lround(bbox.width * cfg.margin_ratio));
  const int margin_y = static_cast<int>(std::lround(bbox.height * cfg.margin_ratio));

  // Expand first, then clamp. Clamping first would let a box touching the image
  // edge grow back over the edge.
  int x0 = bbox.x - margin_x;
  int y0 = bbox.y - margin_y;
  int x1 = bbox.x + bbox.width + margin_x;
  int y1 = bbox.y + bbox.height + margin_y;

  x0 = std::max(0, x0);
  y0 = std::max(0, y0);
  x1 = std::min(image_width, x1);
  y1 = std::min(image_height, y1);

  if (x1 <= x0 || y1 <= y0) return empty;  // bbox lies entirely off-frame

  const cv::Rect2i roi(x0, y0, x1 - x0, y1 - y0);
  if (roi.width < cfg.min_size_px || roi.height < cfg.min_size_px) return empty;
  return roi;
}

}  // namespace the_eyes::pose
