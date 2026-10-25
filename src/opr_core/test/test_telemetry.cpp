#include <gtest/gtest.h>

#include "opr_core/telemetry.hpp"

namespace tel = opr::core::telemetry;

TEST(Crc16, MatchesTheCcittFalseCheckValue) {
  const std::string s = "123456789";
  EXPECT_EQ(tel::crc16(reinterpret_cast<const std::uint8_t*>(s.data()), s.size()), 0x29B1);
}

TEST(Telemetry, RoundTrips) {
  tel::Packet p;
  p.sequence = 4242;
  p.stamp_ns = 1'234'567'890'123ULL;
  p.tracks.push_back({7, 100.5f, 200.25f, -3.0f, 4.5f, 12, false});
  p.tracks.push_back({9, 1.0f, 2.0f, 3.0f, 4.0f, 255, true});
  const auto bytes = tel::encode(p);
  EXPECT_EQ(bytes.size(), 20u + 2u * 22u + 2u);
  std::string err;
  const auto back = tel::decode(bytes, &err);
  ASSERT_TRUE(back) << err;
  EXPECT_EQ(back->sequence, 4242u);
  EXPECT_EQ(back->stamp_ns, p.stamp_ns);
  ASSERT_EQ(back->tracks.size(), 2u);
  EXPECT_EQ(back->tracks[0].id, 7u);
  EXPECT_FLOAT_EQ(back->tracks[0].y, 200.25f);
  EXPECT_TRUE(back->tracks[1].coasting);
  EXPECT_EQ(back->tracks[1].hits, 255);
}

TEST(Telemetry, EncodesTrackerStateLikeTheLegacyProgram) {
  opr::core::Track t;
  t.id = 3;
  t.x[0] = 10.0;
  t.x[1] = 20.0;
  t.x[2] = 1.5;
  t.x[3] = -2.5;
  t.hits = 400;
  t.misses = 2;
  const auto p = tel::decode(tel::encode(5, 99, {t}));
  ASSERT_TRUE(p);
  EXPECT_EQ(p->tracks[0].hits, 255);  // capped
  EXPECT_TRUE(p->tracks[0].coasting);
}

TEST(Telemetry, RejectsDamagedDatagrams) {
  tel::Packet p;
  p.tracks.push_back({1, 1, 2, 3, 4, 5, false});
  const auto good = tel::encode(p);
  std::string err;

  auto flipped = good;
  flipped[25] ^= 0x01;
  EXPECT_FALSE(tel::decode(flipped, &err));
  EXPECT_EQ(err, "bad crc");

  auto shorter = good;
  shorter.pop_back();
  EXPECT_FALSE(tel::decode(shorter, &err));
  EXPECT_EQ(err, "length does not match track count");

  auto magic = good;
  magic[0] = 'X';
  EXPECT_FALSE(tel::decode(magic, &err));
  EXPECT_EQ(err, "bad magic");

  auto version = good;
  version[4] = 2;
  EXPECT_FALSE(tel::decode(version, &err));
  EXPECT_EQ(err, "unsupported version");

  EXPECT_FALSE(tel::decode({1, 2, 3}, &err));
  EXPECT_EQ(err, "too short");
}

TEST(Hex, RoundTripsAndRejectsGarbage) {
  const std::vector<std::uint8_t> b = {0x00, 0x7f, 0xab, 0xff};
  EXPECT_EQ(tel::to_hex(b), "007fabff");
  EXPECT_EQ(tel::from_hex("007FABff"), b);
  EXPECT_THROW(tel::from_hex("abc"), std::invalid_argument);
  EXPECT_THROW(tel::from_hex("zz"), std::invalid_argument);
}
