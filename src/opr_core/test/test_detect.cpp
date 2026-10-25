#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "opr_core/detect.hpp"

using opr::core::Detection;
using opr::core::Detector;

namespace {
struct Canvas {
  std::vector<std::uint8_t> px = std::vector<std::uint8_t>(640 * 480, 60);
  void disc(double cx, double cy, double r, int add) {
    for (int y = 0; y < 480; ++y) {
      for (int x = 0; x < 640; ++x) {
        const double d = std::hypot(x - cx, y - cy);
        if (d <= r) px[y * 640 + x] = static_cast<std::uint8_t>(std::min(255, px[y * 640 + x] + add));
      }
    }
  }
  opr::core::ConstImageView view() const { return {px.data(), 640, 480, 640}; }
};
}  // namespace

TEST(Detect, FindsADiscAtItsCentre) {
  Canvas c;
  c.disc(200.5, 150.5, 6, 90);
  Detector d;
  const auto out = d.detect(c.view());
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].x, 200.5, 0.3);
  EXPECT_NEAR(out[0].y, 150.5, 0.3);
  EXPECT_GT(out[0].strength, 18);
}

TEST(Detect, IgnoresAFlatImage) {
  Canvas c;
  Detector d;
  EXPECT_TRUE(d.detect(c.view()).empty());
}

TEST(Detect, IgnoresSpecksBelowTheMinimumArea) {
  Canvas c;
  c.disc(300, 300, 1.2, 120);
  Detector d;
  EXPECT_TRUE(d.detect(c.view()).empty());
}

TEST(Detect, IgnoresFaintBlobs) {
  Canvas c;
  c.disc(300, 300, 6, 12);
  Detector d;
  EXPECT_TRUE(d.detect(c.view()).empty());
}

TEST(Detect, SeparatesTwoTargetsAndOrdersThemByRaster) {
  Canvas c;
  c.disc(400, 100, 6, 90);
  c.disc(100, 300, 6, 90);
  Detector d;
  const auto out = d.detect(c.view());
  ASSERT_EQ(out.size(), 2u);
  EXPECT_LT(out[0].y, out[1].y);
}

TEST(Detect, MergesAUShapeIntoOneComponent) {
  // The two arms of a U meet only at the bottom: labelling must union them.
  Canvas c;
  for (int y = 200; y < 230; ++y) {
    for (int x = 300; x < 304; ++x) c.px[y * 640 + x] = 200;
    for (int x = 320; x < 324; ++x) c.px[y * 640 + x] = 200;
  }
  for (int y = 230; y < 234; ++y) {
    for (int x = 300; x < 324; ++x) c.px[y * 640 + x] = 200;
  }
  Detector d;
  const auto out = d.detect(c.view());
  ASSERT_EQ(out.size(), 1u);
}

TEST(Detect, LargeAreaIsRejected) {
  Canvas c;
  c.disc(320, 240, 60, 100);  // a lit building, not a target
  Detector d;
  for (const auto& det : d.detect(c.view())) EXPECT_LE(det.area, d.params().max_area);
}

TEST(Detect, WorksOnAPaddedRowStep) {
  std::vector<std::uint8_t> padded(704 * 480, 60);
  for (int y = 0; y < 480; ++y) {
    for (int x = 0; x < 640; ++x) {
      if (std::hypot(x - 250.0, y - 260.0) <= 6) padded[y * 704 + x] = 150;
    }
  }
  Detector d;
  const auto out = d.detect({padded.data(), 640, 480, 704});
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].x, 250.5, 0.6);
}
