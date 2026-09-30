#include "the_eyes/runtime/visibility.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace the_eyes::runtime {
namespace {

// Twice the signed area of (a, b, p): positive when p is left of a->b.
double Edge(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& p) {
  return (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x());
}

}  // namespace

TriMesh LoadObjMesh(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open mesh " + path);
  TriMesh m;
  std::string line;
  int line_no = 0;
  while (std::getline(f, line)) {
    ++line_no;
    if (line.size() < 2 || line[1] != ' ') continue;
    if (line[0] == 'v') {
      std::istringstream is(line.substr(2));
      double x, y, z;
      if (!(is >> x >> y >> z)) {
        throw std::runtime_error(path + ":" + std::to_string(line_no) + ": malformed vertex");
      }
      m.vertices.emplace_back(x, y, z);
    } else if (line[0] == 'f') {
      std::istringstream is(line.substr(2));
      std::vector<int> idx;
      std::string tok;
      while (is >> tok) {
        // Only the position index matters: `a`, `a/b`, `a//c`, `a/b/c`.
        const int raw = std::stoi(tok.substr(0, tok.find('/')));
        if (raw == 0) {
          throw std::runtime_error(path + ":" + std::to_string(line_no) +
                                   ": OBJ indices are 1-based; 0 is not valid");
        }
        // Negative indices count back from the last vertex defined so far.
        idx.push_back(raw > 0 ? raw - 1 : static_cast<int>(m.vertices.size()) + raw);
      }
      if (idx.size() < 3) {
        throw std::runtime_error(path + ":" + std::to_string(line_no) + ": face with < 3 vertices");
      }
      for (size_t i = 1; i + 1 < idx.size(); ++i) m.triangles.push_back({idx[0], idx[i], idx[i + 1]});
    }
  }
  if (m.triangles.empty()) throw std::runtime_error("no faces in " + path);
  const int n = static_cast<int>(m.vertices.size());
  for (const auto& t : m.triangles) {
    for (int i : t) {
      if (i < 0 || i >= n) throw std::runtime_error(path + ": face index out of range");
    }
  }
  return m;
}

cv::Mat RenderDepth(const TriMesh& mesh, const Eigen::Isometry3d& T_cam_object,
                    const Eigen::Matrix3d& K, int width, int height) {
  cv::Mat out(height, width, CV_32FC1, cv::Scalar(0.0f));
  // Objects sit 0.5-1 m from the cameras, so a triangle that reaches the near
  // plane is not a case this rig produces; it is skipped rather than clipped.
  constexpr double kNear = 0.01;

  std::vector<Eigen::Vector3d> pc(mesh.vertices.size());
  std::vector<Eigen::Vector2d> uv(mesh.vertices.size());
  for (size_t i = 0; i < mesh.vertices.size(); ++i) {
    pc[i] = T_cam_object * mesh.vertices[i];
    if (pc[i].z() > kNear) uv[i] = (K * pc[i]).hnormalized();
  }

  for (const auto& t : mesh.triangles) {
    const double z0 = pc[t[0]].z(), z1 = pc[t[1]].z(), z2 = pc[t[2]].z();
    if (z0 <= kNear || z1 <= kNear || z2 <= kNear) continue;
    const Eigen::Vector2d& p0 = uv[t[0]];
    const Eigen::Vector2d& p1 = uv[t[1]];
    const Eigen::Vector2d& p2 = uv[t[2]];
    const double area = Edge(p0, p1, p2);
    if (std::abs(area) < 1e-12) continue;  // edge-on

    const int x_lo = std::max(0, static_cast<int>(std::ceil(std::min({p0.x(), p1.x(), p2.x()}))));
    const int x_hi =
        std::min(width - 1, static_cast<int>(std::floor(std::max({p0.x(), p1.x(), p2.x()}))));
    const int y_lo = std::max(0, static_cast<int>(std::ceil(std::min({p0.y(), p1.y(), p2.y()}))));
    const int y_hi =
        std::min(height - 1, static_cast<int>(std::floor(std::max({p0.y(), p1.y(), p2.y()}))));

    for (int y = y_lo; y <= y_hi; ++y) {
      float* row = out.ptr<float>(y);
      for (int x = x_lo; x <= x_hi; ++x) {
        const Eigen::Vector2d p(x, y);
        // Dividing by the signed area makes the test winding-independent.
        const double w0 = Edge(p1, p2, p) / area;
        const double w1 = Edge(p2, p0, p) / area;
        const double w2 = 1.0 - w0 - w1;
        // A hair of tolerance so a pixel centre exactly on a shared edge is
        // not dropped by both of the triangles that share it.
        constexpr double kEps = -1e-9;
        if (w0 < kEps || w1 < kEps || w2 < kEps) continue;
        // 1/z is linear in screen space; z is not.
        const double z = 1.0 / (w0 / z0 + w1 / z1 + w2 / z2);
        float& d = row[x];
        if (d == 0.0f || z < d) d = static_cast<float>(z);
      }
    }
  }
  return out;
}

double VisibilityCounts::visible_fraction() const {
  const int m = measured();
  return m > 0 ? static_cast<double>(consistent + behind) / m
               : std::numeric_limits<double>::quiet_NaN();
}

double VisibilityCounts::behind_fraction() const {
  const int m = measured();
  return m > 0 ? static_cast<double>(behind) / m : std::numeric_limits<double>::quiet_NaN();
}

double VisibilityCounts::invalid_fraction() const {
  return silhouette > 0 ? static_cast<double>(invalid) / silhouette
                        : std::numeric_limits<double>::quiet_NaN();
}

VisibilityCounts MeasureVisibility(const cv::Mat& rendered_m, const cv::Mat& depth_u16,
                                   double depth_scale_m, double tol_m, int erode_px,
                                   cv::Mat* labels) {
  if (rendered_m.type() != CV_32FC1 || depth_u16.type() != CV_16UC1) {
    throw std::invalid_argument("MeasureVisibility: want CV_32FC1 render and CV_16UC1 depth");
  }
  if (rendered_m.size() != depth_u16.size()) {
    throw std::invalid_argument("MeasureVisibility: render and depth differ in size");
  }
  if (tol_m <= 0.0 || depth_scale_m <= 0.0 || erode_px < 0) {
    throw std::invalid_argument("MeasureVisibility: tol and depth scale must be positive");
  }

  cv::Mat mask = rendered_m > 0.0f;
  if (erode_px > 0) {
    const cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                cv::Size(2 * erode_px + 1, 2 * erode_px + 1));
    cv::erode(mask, mask, k);
  }
  if (labels) *labels = cv::Mat::zeros(rendered_m.size(), CV_8UC1);

  VisibilityCounts c;
  for (int y = 0; y < mask.rows; ++y) {
    const uint8_t* m = mask.ptr<uint8_t>(y);
    const float* z = rendered_m.ptr<float>(y);
    const uint16_t* d = depth_u16.ptr<uint16_t>(y);
    for (int x = 0; x < mask.cols; ++x) {
      if (!m[x]) continue;
      ++c.silhouette;
      VisLabel label;
      if (d[x] == 0) {
        ++c.invalid;
        label = VisLabel::kInvalid;
      } else {
        const double measured = d[x] * depth_scale_m;
        if (measured < z[x] - tol_m) {
          ++c.occluded;
          label = VisLabel::kOccluded;
        } else if (measured > z[x] + tol_m) {
          ++c.behind;
          label = VisLabel::kBehind;
        } else {
          ++c.consistent;
          label = VisLabel::kConsistent;
        }
      }
      if (labels) labels->at<uint8_t>(y, x) = static_cast<uint8_t>(label);
    }
  }
  return c;
}

}  // namespace the_eyes::runtime
