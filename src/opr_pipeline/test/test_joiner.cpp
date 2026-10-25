#include <gtest/gtest.h>

#include "opr_pipeline/joiner.hpp"

using opr::core::telemetry::Packet;
using opr::pipeline::Joiner;

namespace {
Packet at(std::uint64_t stamp) {
  Packet p;
  p.stamp_ns = stamp;
  p.sequence = 999999;  // must be overwritten
  return p;
}
constexpr std::uint64_t kI = 33'333'333;
}  // namespace

TEST(Joiner, UsesTheCamerasSequenceWhenFrameInfoCameFirst) {
  Joiner j(200, kI);
  j.add_frame(1000, 7);
  j.add_tracks(at(1000), 0);
  const auto out = j.drain(0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sequence, 7u);
  EXPECT_EQ(j.joined(), 1u);
}

TEST(Joiner, WaitsForLateFrameInfo) {
  Joiner j(200, kI);
  j.add_tracks(at(1000), 0);
  EXPECT_TRUE(j.drain(50).empty());
  j.add_frame(1000, 3);
  const auto out = j.drain(60);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sequence, 3u);
}

TEST(Joiner, EstimatesTheSequenceWhenFrameInfoNeverComes) {
  Joiner j(200, kI);
  j.add_frame(1'000'000'000, 10);
  j.add_tracks(at(1'000'000'000), 0);
  ASSERT_EQ(j.drain(0).size(), 1u);
  j.add_tracks(at(1'000'000'000 + 3 * kI), 10);  // three frames later, its FrameInfo was lost
  EXPECT_TRUE(j.drain(100).empty());
  const auto out = j.drain(211);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sequence, 13u);
  EXPECT_EQ(j.unjoined(), 1u);
}

TEST(Joiner, KeepsArrivalOrderBehindAPacketThatIsStillWaiting) {
  Joiner j(200, kI);
  j.add_tracks(at(1000), 0);
  j.add_frame(2000, 2);
  j.add_tracks(at(2000), 0);
  EXPECT_TRUE(j.drain(10).empty()) << "the second packet must not overtake the first";
  j.add_frame(1000, 1);
  const auto out = j.drain(20);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].sequence, 1u);
  EXPECT_EQ(out[1].sequence, 2u);
}

TEST(Joiner, ForgetsFramesItNoLongerNeeds) {
  Joiner j(200, kI);
  for (std::uint32_t i = 0; i < 5000; ++i) j.add_frame(i, i);
  j.add_tracks(at(4999), 0);
  const auto out = j.drain(0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].sequence, 4999u);
}
