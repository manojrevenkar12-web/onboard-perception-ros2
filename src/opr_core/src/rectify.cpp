#include "opr_core/rectify.hpp"

#include <cmath>
#include <stdexcept>

namespace opr::core {

void distort_point(const CameraModel& m, double xu, double yu, double& xd, double& yd) {
  // Operation order is part of the contract: it matches visionbox 2.3 bit for bit.
  const double x = (xu - m.cx) / m.fx;
  const double y = (yu - m.cy) / m.fy;
  const double r2 = x * x + y * y;
  const double radial = 1.0 + r2 * (m.k1 + r2 * (m.k2 + r2 * m.k3));
  const double dx = 2.0 * m.p1 * x * y + m.p2 * (r2 + 2.0 * x * x);
  const double dy = m.p1 * (r2 + 2.0 * y * y) + 2.0 * m.p2 * x * y;
  xd = (x * radial + dx) * m.fx + m.cx;
  yd = (y * radial + dy) * m.fy + m.cy;
}

Rectifier::Rectifier(const CameraModel& model) : model_(model) {
  if (model.width < 2 || model.height < 2) throw std::invalid_argument("image too small to rectify");
  const std::size_t n = static_cast<std::size_t>(model.width) * model.height;
  offset_.assign(n, -1);
  wx_.assign(n, 0);
  wy_.assign(n, 0);
  row_.assign(n, 0);
  for (int v = 0; v < model.height; ++v) {
    for (int u = 0; u < model.width; ++u) {
      double xd = 0.0;
      double yd = 0.0;
      distort_point(model, u, v, xd, yd);
      const double fx0 = std::floor(xd);
      const double fy0 = std::floor(yd);
      const int x0 = static_cast<int>(fx0);
      const int y0 = static_cast<int>(fy0);
      if (x0 < 0 || y0 < 0 || x0 + 1 >= model.width || y0 + 1 >= model.height) continue;
      const std::size_t i = static_cast<std::size_t>(v) * model.width + u;
      offset_[i] = x0;
      row_[i] = y0;
      wx_[i] = static_cast<std::uint16_t>((xd - fx0) * 256.0 + 0.5);
      wy_[i] = static_cast<std::uint16_t>((yd - fy0) * 256.0 + 0.5);
    }
  }
}

void Rectifier::apply(ConstImageView src, ImageView dst) const {
  if (src.width != model_.width || src.height != model_.height || dst.width != model_.width ||
      dst.height != model_.height) {
    throw std::invalid_argument("image size does not match the camera model");
  }
  std::size_t i = 0;
  for (int v = 0; v < model_.height; ++v) {
    std::uint8_t* out = dst.row(v);
    for (int u = 0; u < model_.width; ++u, ++i) {
      if (offset_[i] < 0) {
        out[u] = 0;
        continue;
      }
      const std::uint32_t wx = wx_[i];
      const std::uint32_t wy = wy_[i];
      const std::uint8_t* p = src.row(row_[i]) + offset_[i];
      const std::uint8_t* q = p + src.step;
      const std::uint32_t top = p[0] * (256 - wx) + p[1] * wx;
      const std::uint32_t bot = q[0] * (256 - wx) + q[1] * wx;
      out[u] = static_cast<std::uint8_t>((top * (256 - wy) + bot * wy + 32768) >> 16);
    }
  }
}

}  // namespace opr::core
