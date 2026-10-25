#include <gtest/gtest.h>

#include <vector>

#include "opr_core/rectify.hpp"
#include "opr_sim/scene.hpp"

using opr::core::CameraModel;
using opr::core::Rectifier;

TEST(Rectify, WithoutDistortionCopiesTheImage) {
  CameraModel m;
  m.k1 = m.k2 = m.k3 = m.p1 = m.p2 = 0.0;
  Rectifier r(m);
  std::vector<std::uint8_t> src(640 * 480), dst(640 * 480, 7);
  for (std::size_t i = 0; i < src.size(); ++i) src[i] = static_cast<std::uint8_t>(i * 31 % 251);
  r.apply({src.data(), 640, 480, 640}, {dst.data(), 640, 480, 640});
  for (int y = 0; y < 479; ++y) {
    for (int x = 0; x < 639; ++x) ASSERT_EQ(dst[y * 640 + x], src[y * 640 + x]) << x << "," << y;
  }
  // The last row and column have no right/bottom neighbour to interpolate with.
  EXPECT_EQ(dst[479 * 640 + 10], 0);
  EXPECT_EQ(dst[10 * 640 + 639], 0);
}

TEST(Rectify, AgreesWithTheSimulatorsLens) {
  // The simulator distorts with its own code; the rectifier must invert the same model.
  opr::sim::Calibration c;
  CameraModel m;
  for (double x = 20; x < 620; x += 50) {
    for (double y = 20; y < 460; y += 40) {
      double sx, sy, cx, cy;
      opr::sim::distort(c, x, y, sx, sy);
      opr::core::distort_point(m, x, y, cx, cy);
      EXPECT_DOUBLE_EQ(sx, cx);
      EXPECT_DOUBLE_EQ(sy, cy);
    }
  }
}

TEST(Rectify, StraightensABentLine) {
  // A horizontal line in the world is curved by barrel distortion; after rectification the row
  // with the maximum along the line must be the same at the centre and near the edge.
  opr::sim::Calibration c;
  CameraModel m;
  std::vector<std::uint8_t> src(640 * 480, 20), dst(640 * 480);
  const double world_y = 80.0;
  for (int u = 0; u < 640; ++u) {
    double xd, yd;
    opr::sim::distort(c, u, world_y, xd, yd);
    const int x = static_cast<int>(xd + 0.5), y = static_cast<int>(yd + 0.5);
    if (x >= 0 && x < 640 && y >= 1 && y < 479) {
      src[(y - 1) * 640 + x] = 200;
      src[y * 640 + x] = 250;
      src[(y + 1) * 640 + x] = 200;
    }
  }
  Rectifier r(m);
  r.apply({src.data(), 640, 480, 640}, {dst.data(), 640, 480, 640});
  auto peak_row = [&](int u) {
    int best = 0, row = -1;
    for (int y = 0; y < 480; ++y) {
      if (dst[y * 640 + u] > best) {
        best = dst[y * 640 + u];
        row = y;
      }
    }
    return row;
  };
  EXPECT_NEAR(peak_row(320), world_y, 1.0);
  EXPECT_NEAR(peak_row(120), world_y, 1.0);
  EXPECT_NEAR(peak_row(520), world_y, 1.0);
}

TEST(Rectify, HonoursRowStep) {
  CameraModel m;
  Rectifier r(m);
  std::vector<std::uint8_t> dense(640 * 480, 90), padded(704 * 480, 90), a(640 * 480), b(704 * 480, 1);
  for (int y = 0; y < 480; ++y) {
    for (int x = 0; x < 640; ++x) {
      const auto v = static_cast<std::uint8_t>((x * 7 + y * 3) & 0xff);
      dense[y * 640 + x] = v;
      padded[y * 704 + x] = v;
    }
  }
  r.apply({dense.data(), 640, 480, 640}, {a.data(), 640, 480, 640});
  r.apply({padded.data(), 640, 480, 704}, {b.data(), 640, 480, 704});
  for (int y = 0; y < 480; ++y) {
    for (int x = 0; x < 640; ++x) ASSERT_EQ(a[y * 640 + x], b[y * 704 + x]);
  }
}

TEST(Rectify, RejectsTheWrongSize) {
  Rectifier r(CameraModel{});
  std::vector<std::uint8_t> img(320 * 240);
  EXPECT_THROW(r.apply({img.data(), 320, 240, 320}, {img.data(), 320, 240, 320}), std::invalid_argument);
}
