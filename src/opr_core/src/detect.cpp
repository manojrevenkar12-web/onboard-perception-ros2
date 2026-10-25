#include "opr_core/detect.hpp"

namespace opr::core {

void Detector::resize(int width, int height) {
  if (width / 2 == hw_ && height / 2 == hh_) return;
  hw_ = width / 2;
  hh_ = height / 2;
  const std::size_t n = static_cast<std::size_t>(hw_) * hh_;
  half_.assign(n, 0);
  integral_.assign(static_cast<std::size_t>(hw_ + 1) * (hh_ + 1), 0);
  contrast_.assign(n, 0);
  labels_.assign(n, -1);
}

int Detector::local_mean(int x, int y) const {
  const int r = params_.bg_radius;
  const int iw = hw_ + 1;
  const int x0 = x - r < 0 ? 0 : x - r;
  const int y0 = y - r < 0 ? 0 : y - r;
  const int x1 = x + r >= hw_ ? hw_ - 1 : x + r;
  const int y1 = y + r >= hh_ ? hh_ - 1 : y + r;
  auto at = [&](int yy, int xx) { return integral_[static_cast<std::size_t>(yy) * iw + xx]; };
  const std::uint32_t sum = at(y1 + 1, x1 + 1) - at(y0, x1 + 1) - at(y1 + 1, x0) + at(y0, x0);
  const auto count = static_cast<std::uint32_t>((x1 - x0 + 1) * (y1 - y0 + 1));
  return static_cast<int>((sum + count / 2) / count);
}

std::int32_t Detector::root(std::int32_t a) {
  while (parent_[a] != a) {
    parent_[a] = parent_[parent_[a]];
    a = parent_[a];
  }
  return a;
}

std::vector<Detection> Detector::detect(ConstImageView img) {
  resize(img.width, img.height);
  // 1. Downscale 2x2 with rounding.
  for (int y = 0; y < hh_; ++y) {
    const std::uint8_t* a = img.row(2 * y);
    const std::uint8_t* b = img.row(2 * y + 1);
    std::uint8_t* out = &half_[static_cast<std::size_t>(y) * hw_];
    for (int x = 0; x < hw_; ++x) {
      out[x] = static_cast<std::uint8_t>((a[2 * x] + a[2 * x + 1] + b[2 * x] + b[2 * x + 1] + 2) >> 2);
    }
  }
  // 2. Integral image with a leading zero row and column.
  const int iw = hw_ + 1;
  for (int x = 0; x < iw; ++x) integral_[x] = 0;
  for (int y = 1; y <= hh_; ++y) {
    std::uint32_t run = 0;
    integral_[static_cast<std::size_t>(y) * iw] = 0;
    for (int x = 1; x <= hw_; ++x) {
      run += half_[static_cast<std::size_t>(y - 1) * hw_ + (x - 1)];
      integral_[static_cast<std::size_t>(y) * iw + x] = integral_[static_cast<std::size_t>(y - 1) * iw + x] + run;
    }
  }
  // 3. Threshold against the local mean and label with union-find (two passes, 8-connected).
  parent_.clear();
  static constexpr int kNx[4] = {-1, -1, 0, 1};
  static constexpr int kNy[4] = {0, -1, -1, -1};
  for (int y = 0; y < hh_; ++y) {
    for (int x = 0; x < hw_; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * hw_ + x;
      const int c = static_cast<int>(half_[i]) - local_mean(x, y);
      contrast_[i] = c;
      if (c < params_.threshold) {
        labels_[i] = -1;
        continue;
      }
      std::int32_t best = -1;
      for (int k = 0; k < 4; ++k) {
        const int xx = x + kNx[k];
        const int yy = y + kNy[k];
        if (xx < 0 || yy < 0 || xx >= hw_) continue;
        std::int32_t l = labels_[static_cast<std::size_t>(yy) * hw_ + xx];
        if (l < 0) continue;
        l = root(l);
        if (best < 0) {
          best = l;
        } else if (l != best) {
          if (l < best) {
            parent_[best] = l;
            best = l;
          } else {
            parent_[l] = best;
          }
        }
      }
      if (best < 0) {
        best = static_cast<std::int32_t>(parent_.size());
        parent_.push_back(best);
      }
      labels_[i] = best;
    }
  }
  // 4. Accumulate contrast-weighted moments per component, in order of first appearance.
  struct Moments {
    std::int64_t w = 0, wx = 0, wy = 0;
    int area = 0;
  };
  std::vector<Moments> acc(parent_.size());
  std::vector<std::int32_t> order;
  std::vector<char> seen(parent_.size(), 0);
  for (int y = 0; y < hh_; ++y) {
    for (int x = 0; x < hw_; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * hw_ + x;
      if (labels_[i] < 0) continue;
      const std::int32_t r = root(labels_[i]);
      if (!seen[r]) {
        seen[r] = 1;
        order.push_back(r);
      }
      const std::int64_t w = contrast_[i];
      acc[r].w += w;
      acc[r].wx += w * x;
      acc[r].wy += w * y;
      acc[r].area++;
    }
  }
  std::vector<Detection> out;
  for (const std::int32_t r : order) {
    const Moments& m = acc[r];
    if (m.area < params_.min_area || m.area > params_.max_area) continue;
    Detection d;
    // Half-resolution pixel i covers full-resolution pixels 2i and 2i+1: centre 2i + 0.5.
    d.x = static_cast<double>(m.wx) / static_cast<double>(m.w) * 2.0 + 0.5;
    d.y = static_cast<double>(m.wy) / static_cast<double>(m.w) * 2.0 + 0.5;
    d.strength = static_cast<double>(m.w) / static_cast<double>(m.area);
    d.area = m.area;
    out.push_back(d);
  }
  return out;
}

}  // namespace opr::core
