#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "opr_sim/scene.hpp"

using opr::sim::Calibration;
using opr::sim::Frame;
using opr::sim::Scene;

namespace {
std::uint64_t fnv1a(const std::vector<std::uint8_t>& bytes) {
  std::uint64_t h = 1469598103934665603ULL;
  for (auto b : bytes) {
    h ^= b;
    h *= 1099511628211ULL;
  }
  return h;
}
}  // namespace

TEST(Scene, EveryScenarioRendersItsFrames) {
  for (const auto& name : opr::sim::scenario_names()) {
    Scene scene(name, 7);
    Frame f;
    int n = 0;
    while (scene.next(f)) ++n;
    EXPECT_EQ(n, scene.frame_count()) << name;
    EXPECT_EQ(f.yuyv.size(), 640u * 480u * 2u) << name;
  }
}

TEST(Scene, UnknownScenarioIsRejected) { EXPECT_THROW(Scene("hover", 1), std::invalid_argument); }

TEST(Scene, SameSeedSameBytes) {
  Scene a("clutter", 42), b("clutter", 42), c("clutter", 43);
  Frame fa, fb, fc;
  a.render(17, fa);
  b.render(17, fb);
  c.render(17, fc);
  EXPECT_EQ(fa.yuyv, fb.yuyv);
  EXPECT_EQ(fa.stamp_ns, fb.stamp_ns);
  EXPECT_NE(fa.yuyv, fc.yuyv);
}

TEST(Scene, RenderIsRandomAccess) {
  Scene a("crossing", 3), b("crossing", 3);
  Frame f;
  for (int i = 0; i < 10; ++i) a.next(f);
  Frame g;
  b.render(9, g);
  EXPECT_EQ(f.yuyv, g.yuyv);
}

TEST(Scene, StampsIncreaseAtAboutThirtyHertz) {
  Scene s("fast", 5);
  Frame prev, cur;
  s.render(0, prev);
  for (std::uint32_t i = 1; i < 60; ++i) {
    s.render(i, cur);
    const auto dt = static_cast<double>(cur.stamp_ns - prev.stamp_ns);
    EXPECT_GT(dt, 32.9e6);
    EXPECT_LT(dt, 33.8e6);
    prev = cur;
  }
}

TEST(Scene, OcclusionHidesTheTarget) {
  Scene s("occlusion", 1);
  Frame f;
  s.render(43, f);
  EXPECT_FALSE(f.truth[0].visible);
  s.render(48, f);
  EXPECT_TRUE(f.truth[0].visible);
}

TEST(Scene, TargetIsBrighterThanItsSurroundings) {
  Scene s("crossing", 11);
  Frame f;
  s.render(0, f);
  const auto& t = f.truth[0];
  double xd, yd;
  opr::sim::distort(s.calibration(), t.x, t.y, xd, yd);
  const int u = static_cast<int>(xd + 0.5), v = static_cast<int>(yd + 0.5);
  const int centre = f.yuyv[(static_cast<std::size_t>(v) * 640 + u) * 2];
  const int away = f.yuyv[(static_cast<std::size_t>(v) * 640 + u + 25) * 2];
  EXPECT_GT(centre - away, 60);
}

TEST(Distortion, UndistortInvertsDistort) {
  Calibration c;
  for (double x = 0; x < 640; x += 37) {
    for (double y = 0; y < 480; y += 29) {
      double xd, yd, xu, yu;
      opr::sim::distort(c, x, y, xd, yd);
      opr::sim::undistort(c, xd, yd, xu, yu);
      EXPECT_NEAR(xu, x, 1e-6);
      EXPECT_NEAR(yu, y, 1e-6);
    }
  }
}

// Pinned fingerprint: the simulator uses only correctly rounded operations, so this value must be
// the same with any compiler and CPU. If it changes, every recorded output in the repository is
// stale; regenerate them on purpose rather than updating this number to make the test pass.
TEST(Scene, FingerprintIsStableAcrossMachines) {
  Scene s("swarm", 2026);
  Frame f;
  s.render(75, f);
  EXPECT_EQ(fnv1a(f.yuyv), 5537529423413159159ULL);
}
