#include "opr_sim/scene.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace opr::sim {

namespace {

constexpr std::uint64_t kGolden = 0x9E3779B97F4A7C15ULL;

std::uint64_t mix(std::uint64_t z) {
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

/// Lattice value in [0, 1) for integer coordinates; independent of call order.
double lattice(std::int64_t ix, std::int64_t iy, std::uint64_t seed) {
  const std::uint64_t h = mix(seed ^ mix(static_cast<std::uint64_t>(ix) * 0x632BE59BD9B4E019ULL ^
                                         static_cast<std::uint64_t>(iy) * kGolden));
  return static_cast<double>(h >> 11) * (1.0 / 9007199254740992.0);
}

int clamp_u8(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }

}  // namespace

std::uint64_t Rng::next() {
  state_ += kGolden;
  return mix(state_);
}

int Rng::uniform(int lo, int hi) {
  const std::uint64_t span = static_cast<std::uint64_t>(hi - lo) + 1U;
  return lo + static_cast<int>(next() % span);
}

double Rng::unit() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }

std::vector<std::string> scenario_names() {
  return {"crossing", "fast", "clutter", "exposure", "occlusion", "swarm", "edge"};
}

void distort(const Calibration& c, double xu, double yu, double& xd, double& yd) {
  const double x = (xu - c.cx) / c.fx;
  const double y = (yu - c.cy) / c.fy;
  const double r2 = x * x + y * y;
  const double radial = 1.0 + r2 * (c.k1 + r2 * (c.k2 + r2 * c.k3));
  const double dx = 2.0 * c.p1 * x * y + c.p2 * (r2 + 2.0 * x * x);
  const double dy = c.p1 * (r2 + 2.0 * y * y) + 2.0 * c.p2 * x * y;
  xd = (x * radial + dx) * c.fx + c.cx;
  yd = (y * radial + dy) * c.fy + c.cy;
}

void undistort(const Calibration& c, double xd, double yd, double& xu, double& yu) {
  const double x0 = (xd - c.cx) / c.fx;
  const double y0 = (yd - c.cy) / c.fy;
  double x = x0;
  double y = y0;
  for (int i = 0; i < 20; ++i) {
    const double r2 = x * x + y * y;
    const double radial = 1.0 + r2 * (c.k1 + r2 * (c.k2 + r2 * c.k3));
    const double dx = 2.0 * c.p1 * x * y + c.p2 * (r2 + 2.0 * x * x);
    const double dy = c.p1 * (r2 + 2.0 * y * y) + 2.0 * c.p2 * x * y;
    x = (x0 - dx) / radial;
    y = (y0 - dy) / radial;
  }
  xu = x * c.fx + c.cx;
  yu = y * c.fy + c.cy;
}

Scene::Scene(const std::string& scenario, std::uint64_t seed, Calibration calib)
    : scenario_(scenario), seed_(seed), calib_(calib) {
  Rng rng(seed ^ 0xC0FFEEULL);
  const double w = calib_.width;
  const double h = calib_.height;
  auto add = [&](double x, double y, double vx, double vy, double radius, int contrast) {
    Target t;
    t.id = static_cast<std::uint32_t>(start_.size() + 1);
    t.x = x;
    t.y = y;
    t.vx = vx;
    t.vy = vy;
    t.radius = radius;
    t.contrast = contrast;
    start_.push_back(t);
  };
  // Positions are jittered by the seed so each seed is a different flight of the same kind.
  auto jitter = [&](double span) { return (rng.unit() - 0.5) * span; };

  if (scenario == "crossing") {
    frame_count_ = 150;
    add(0.2 * w + jitter(20), 0.45 * h + jitter(20), 120.0, 18.0, 6.0, 90);
    add(0.8 * w + jitter(20), 0.55 * h + jitter(20), -120.0, -18.0, 6.5, 85);
  } else if (scenario == "fast") {
    frame_count_ = 120;
    add(0.1 * w + jitter(10), 0.3 * h + jitter(30), 420.0, 95.0, 7.0, 100);
    add(0.5 * w + jitter(40), 0.8 * h + jitter(20), -60.0, -40.0, 6.0, 80);
  } else if (scenario == "clutter") {
    frame_count_ = 150;
    flashes_per_frame_ = 3;
    sensor_noise_ = 6;
    add(0.3 * w + jitter(40), 0.3 * h + jitter(40), 50.0, 30.0, 6.0, 85);
    add(0.6 * w + jitter(40), 0.6 * h + jitter(40), -40.0, 25.0, 6.0, 85);
    add(0.5 * w + jitter(40), 0.2 * h + jitter(30), 10.0, 60.0, 7.0, 95);
  } else if (scenario == "exposure") {
    frame_count_ = 150;
    exposure_ramp_ = true;
    add(0.25 * w + jitter(30), 0.5 * h + jitter(30), 60.0, -10.0, 6.0, 95);
    add(0.6 * w + jitter(30), 0.35 * h + jitter(30), -30.0, 45.0, 6.0, 90);
    add(0.7 * w + jitter(30), 0.75 * h + jitter(20), -50.0, -25.0, 7.0, 100);
  } else if (scenario == "occlusion") {
    frame_count_ = 150;
    add(0.2 * w + jitter(20), 0.4 * h + jitter(20), 80.0, 10.0, 6.0, 90);
    add(0.7 * w + jitter(20), 0.6 * h + jitter(20), -60.0, -20.0, 6.0, 90);
    occlusions_.push_back({1, 40, 47});  // longer than the tracker's patience: a new identity
    occlusions_.push_back({2, 70, 72});  // short: the track coasts and keeps its identity
  } else if (scenario == "swarm") {
    frame_count_ = 150;
    for (int i = 0; i < 8; ++i) {
      const double x = (0.15 + 0.1 * i) * w + jitter(10);
      const double y = (i % 2 == 0 ? 0.3 : 0.7) * h + jitter(30);
      add(x, y, 25.0 + jitter(40), (i % 2 == 0 ? 30.0 : -30.0) + jitter(20), 5.5, 85);
    }
  } else if (scenario == "edge") {
    frame_count_ = 120;
    add(0.85 * w + jitter(10), 0.5 * h + jitter(20), 140.0, 0.0, 6.0, 90);
    add(0.5 * w + jitter(30), 0.08 * h, 0.0, -50.0, 6.0, 90);
    add(0.4 * w + jitter(30), 0.6 * h + jitter(30), 20.0, 20.0, 6.0, 90);
  } else {
    throw std::invalid_argument("unknown scenario: " + scenario);
  }

  const std::size_t n = static_cast<std::size_t>(calib_.width) * calib_.height;
  ideal_x_.resize(n);
  ideal_y_.resize(n);
  for (int v = 0; v < calib_.height; ++v) {
    for (int u = 0; u < calib_.width; ++u) {
      const std::size_t i = static_cast<std::size_t>(v) * calib_.width + u;
      undistort(calib_, u, v, ideal_x_[i], ideal_y_[i]);
    }
  }
  background_.resize(n);
  for (std::size_t i = 0; i < n; ++i) background_[i] = background(ideal_x_[i], ideal_y_[i]);
}

std::vector<Target> Scene::targets_at(std::uint32_t sequence) const {
  const double t = sequence / fps_;
  std::vector<Target> out = start_;
  for (auto& target : out) {
    target.x = target.x + target.vx * t;
    target.y = target.y + target.vy * t;
    for (const auto& e : occlusions_) {
      if (e.target_id == target.id && sequence >= e.from && sequence <= e.to) target.visible = false;
    }
  }
  return out;
}

int Scene::background(double x, double y) const {
  const double gx = x / 24.0;
  const double gy = y / 24.0;
  const double fx0 = std::floor(gx);
  const double fy0 = std::floor(gy);
  const auto ix = static_cast<std::int64_t>(fx0);
  const auto iy = static_cast<std::int64_t>(fy0);
  const double ax = gx - fx0;
  const double ay = gy - fy0;
  const double a = lattice(ix, iy, seed_);
  const double b = lattice(ix + 1, iy, seed_);
  const double c = lattice(ix, iy + 1, seed_);
  const double d = lattice(ix + 1, iy + 1, seed_);
  const double noise = (a + (b - a) * ax) + ((c + (d - c) * ax) - (a + (b - a) * ax)) * ay;
  const double v = 50.0 + x * 40.0 / calib_.width + y * 30.0 / calib_.height + noise * 16.0;
  return static_cast<int>(v + 0.5);
}

int Scene::gain_permille(std::uint32_t sequence) const {
  if (!exposure_ramp_) return 1000;
  // Down from 100% to 55% over the first half, back up over the second.
  const int half = frame_count_ / 2;
  const int s = static_cast<int>(sequence);
  const int dist = s <= half ? s : frame_count_ - s;
  return 1000 - (450 * dist) / half;
}

bool Scene::next(Frame& out) {
  const auto seq = static_cast<std::uint32_t>(next_sequence_);
  if (next_sequence_ >= frame_count_) return false;
  render(seq, out);
  ++next_sequence_;
  return true;
}

void Scene::render(std::uint32_t sequence, Frame& out) const {
  const int w = calib_.width;
  const int h = calib_.height;
  out.sequence = sequence;
  // 30 fps nominal with a little deterministic jitter, as a real sensor clock has. Microsecond
  // resolution, like the struct timeval in a V4L2 buffer, so stamps survive the driver unchanged.
  Rng rng(seed_ ^ (static_cast<std::uint64_t>(sequence) * kGolden));
  const std::uint64_t period_ns = 33'333'000ULL;
  out.stamp_ns = 1'000'000'000ULL + sequence * period_ns + static_cast<std::uint64_t>(rng.uniform(0, 400)) * 1000ULL;
  out.truth = targets_at(sequence);
  out.flashes.clear();

  struct Flash {
    double x, y, r;
  };
  std::vector<Flash> flashes;
  for (int i = 0; i < flashes_per_frame_; ++i) {
    Flash f{rng.unit() * w, rng.unit() * h, 4.0 + rng.unit() * 2.0};
    flashes.push_back(f);
    out.flashes.push_back(static_cast<std::uint32_t>(i));
  }
  const int gain = gain_permille(sequence);

  // Light added by targets and flashes, accumulated only inside each one's footprint.
  std::vector<double> add(static_cast<std::size_t>(w) * h, 0.0);
  for (const auto& t : out.truth) {
    if (!t.visible) continue;
    const double outer = t.radius + 0.5;
    // Footprint in the distorted image: distort a ring around the target, then pad.
    double umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
    for (int k = 0; k < 8; ++k) {
      static const double kx[8] = {1, 0.7071, 0, -0.7071, -1, -0.7071, 0, 0.7071};
      static const double ky[8] = {0, 0.7071, 1, 0.7071, 0, -0.7071, -1, -0.7071};
      double xd, yd;
      distort(calib_, t.x + kx[k] * outer, t.y + ky[k] * outer, xd, yd);
      umin = std::min(umin, xd);
      umax = std::max(umax, xd);
      vmin = std::min(vmin, yd);
      vmax = std::max(vmax, yd);
    }
    const int u0 = std::max(0, static_cast<int>(std::floor(umin)) - 3);
    const int u1 = std::min(w - 1, static_cast<int>(std::floor(umax)) + 3);
    const int v0 = std::max(0, static_cast<int>(std::floor(vmin)) - 3);
    const int v1 = std::min(h - 1, static_cast<int>(std::floor(vmax)) + 3);
    for (int v = v0; v <= v1; ++v) {
      for (int u = u0; u <= u1; ++u) {
        const std::size_t i = static_cast<std::size_t>(v) * w + u;
        const double dx = ideal_x_[i] - t.x;
        const double dy = ideal_y_[i] - t.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 >= outer * outer) continue;
        const double d = std::sqrt(d2);
        const double cover = d <= t.radius - 0.5 ? 1.0 : (outer - d);
        add[i] += t.contrast * cover;
      }
    }
  }
  for (const auto& f : flashes) {
    const int u0 = std::max(0, static_cast<int>(std::floor(f.x - f.r)));
    const int u1 = std::min(w - 1, static_cast<int>(std::floor(f.x + f.r)) + 1);
    const int v0 = std::max(0, static_cast<int>(std::floor(f.y - f.r)));
    const int v1 = std::min(h - 1, static_cast<int>(std::floor(f.y + f.r)) + 1);
    for (int v = v0; v <= v1; ++v) {
      for (int u = u0; u <= u1; ++u) {
        const double dx = u - f.x;
        const double dy = v - f.y;
        if (dx * dx + dy * dy <= f.r * f.r) add[static_cast<std::size_t>(v) * w + u] += 80.0;
      }
    }
  }

  out.yuyv.assign(static_cast<std::size_t>(w) * h * 2, 128);
  for (std::size_t i = 0; i < add.size(); ++i) {
    const int base = background_[i] + static_cast<int>(add[i] + 0.5);
    const int lit = (base * gain) / 1000;
    const int noisy = lit + rng.uniform(-sensor_noise_, sensor_noise_);
    out.yuyv[i * 2] = static_cast<std::uint8_t>(clamp_u8(noisy));
  }
}

}  // namespace opr::sim
