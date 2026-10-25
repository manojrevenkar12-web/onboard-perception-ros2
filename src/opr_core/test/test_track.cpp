#include <gtest/gtest.h>

#include <cmath>

#include "opr_core/track.hpp"
#include "opr_sim/scene.hpp"
#include "opr_core/pipeline.hpp"

using opr::core::Detection;
using opr::core::Tracker;

namespace {
constexpr std::uint64_t kPeriod = 33'333'333;
Detection det(double x, double y) {
  Detection d;
  d.x = x;
  d.y = y;
  d.strength = 50;
  d.area = 20;
  return d;
}
}  // namespace

TEST(Track, ConfirmsAfterThreeHitsAndLearnsVelocity) {
  Tracker t;
  for (int i = 0; i < 30; ++i) {
    t.step(1'000'000'000ULL + i * kPeriod, {det(100 + 3.0 * i, 200 - 1.5 * i)});
    if (i < 2) {
      EXPECT_TRUE(t.confirmed().empty());
    }
  }
  const auto c = t.confirmed();
  ASSERT_EQ(c.size(), 1u);
  EXPECT_NEAR(c[0].x[2], 90.0, 2.0);   // 3 px per frame at 30 Hz
  EXPECT_NEAR(c[0].x[3], -45.0, 2.0);
}

TEST(Track, TentativeTracksDieOnTheirFirstMiss) {
  Tracker t;
  t.step(1'000'000'000ULL, {det(50, 50)});
  const auto r = t.step(1'000'000'000ULL + kPeriod, {});
  EXPECT_EQ(r.dropped, 1);
  EXPECT_TRUE(t.all().empty());
}

TEST(Track, ConfirmedTracksCoastThroughShortGaps) {
  Tracker t;
  std::uint64_t s = 1'000'000'000ULL;
  for (int i = 0; i < 10; ++i, s += kPeriod) t.step(s, {det(100 + 2.0 * i, 100)});
  for (int i = 0; i < 5; ++i, s += kPeriod) t.step(s, {});
  ASSERT_EQ(t.confirmed().size(), 1u);
  EXPECT_EQ(t.confirmed()[0].misses, 5);
  t.step(s, {});
  EXPECT_TRUE(t.confirmed().empty()) << "a sixth miss ends the track";
}

TEST(Track, ReacquiresAfterCoastingWithTheSameId) {
  Tracker t;
  std::uint64_t s = 1'000'000'000ULL;
  int i = 0;
  for (; i < 10; ++i, s += kPeriod) t.step(s, {det(100 + 2.0 * i, 100)});
  for (int k = 0; k < 3; ++k, ++i, s += kPeriod) t.step(s, {});
  t.step(s, {det(100 + 2.0 * i, 100)});
  ASSERT_EQ(t.confirmed().size(), 1u);
  EXPECT_EQ(t.confirmed()[0].id, 1u);
  EXPECT_EQ(t.confirmed()[0].misses, 0);
}

TEST(Track, FarDetectionsAreGatedIntoNewTracks) {
  Tracker t;
  std::uint64_t s = 1'000'000'000ULL;
  for (int i = 0; i < 10; ++i, s += kPeriod) t.step(s, {det(100, 100)});
  const auto r = t.step(s, {det(300, 300)});
  EXPECT_EQ(r.assigned, 0);
  EXPECT_EQ(r.born, 1);
}

TEST(Track, AStampThatGoesBackwardsIsIgnored) {
  Tracker t;
  t.step(2'000'000'000ULL, {det(10, 10)});
  const auto r = t.step(1'000'000'000ULL, {det(10, 10)});
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(t.all()[0].hits, 1);
}

TEST(Track, DroppedFramesArePredictedAcross) {
  // Frames 10..14 lost in transport: the next frame arrives 6 periods later and must still
  // associate because the prediction covers the real elapsed time.
  Tracker t;
  std::uint64_t s = 1'000'000'000ULL;
  int i = 0;
  for (; i < 10; ++i, s += kPeriod) t.step(s, {det(100 + 4.0 * i, 100)});
  i += 5;
  s += 5 * kPeriod;
  const auto r = t.step(s, {det(100 + 4.0 * i, 100)});
  EXPECT_EQ(r.assigned, 1);
}

TEST(Track, CrossingTargetsKeepTheirIdentities) {
  // End-to-end on the simulator: two targets cross in the middle of the frame. While they overlap
  // the detector sees one merged blob, which can briefly start a third track (a known limit of
  // visionbox 2.3, kept for equivalence; docs/MIGRATION.md). What must hold is that after the
  // crossing each target is still reported under its original identity.
  for (int seed = 1; seed <= 5; ++seed) {
    opr::sim::Scene scene("crossing", static_cast<std::uint64_t>(seed));
    opr::core::Pipeline p({}, {}, {});
    opr::sim::Frame f;
    while (scene.next(f)) {
      p.process({f.yuyv.data(), 640, 480, 1280}, f.sequence, f.stamp_ns);
      if (f.sequence != 100) continue;
      const auto confirmed = p.tracker().confirmed();
      for (const auto& target : f.truth) {
        const opr::core::Track* nearest = nullptr;
        double best = 1e9;
        for (const auto& t : confirmed) {
          const double d = std::hypot(t.x[0] - target.x, t.x[1] - target.y);
          if (d < best) {
            best = d;
            nearest = &t;
          }
        }
        ASSERT_NE(nearest, nullptr) << "seed " << seed;
        EXPECT_EQ(nearest->id, target.id) << "seed " << seed;
        EXPECT_LT(best, 1.0) << "seed " << seed;
      }
    }
  }
}
