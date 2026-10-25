// The migration's main guarantee: opr_core produces exactly the telemetry visionbox 2.3 produced,
// byte for byte, on every recorded flight. The golden files are visionbox's own output
// (`visionbox --replay <scenario>:<seed> --record ...`); CI regenerates them from the legacy binary
// and fails if they differ from the committed ones.
#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

#include "opr_core/pipeline.hpp"
#include "opr_core/telemetry.hpp"
#include "opr_sim/scene.hpp"

namespace {

std::vector<std::string> read_lines(const std::string& path) {
  std::ifstream in(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) lines.push_back(line);
  return lines;
}

struct Flight {
  std::string scenario;
  int seed;
};

std::vector<Flight> flights() {
  std::vector<Flight> out;
  for (const auto& s : opr::sim::scenario_names()) {
    for (int seed = 1; seed <= 3; ++seed) out.push_back({s, seed});
  }
  return out;
}

class Equivalence : public ::testing::TestWithParam<Flight> {};

TEST_P(Equivalence, SameBytesAsVisionbox23) {
  const Flight f = GetParam();
  const auto golden = read_lines(std::string(OPR_GOLDEN_DIR) + "/" + f.scenario + "_" + std::to_string(f.seed) + ".hex");
  opr::sim::Scene scene(f.scenario, static_cast<std::uint64_t>(f.seed));
  ASSERT_EQ(golden.size(), static_cast<std::size_t>(scene.frame_count())) << "golden file missing or truncated";
  opr::core::Pipeline pipeline({}, {}, {});
  opr::sim::Frame frame;
  std::size_t i = 0;
  while (scene.next(frame)) {
    opr::core::View<const std::uint8_t> yuyv{frame.yuyv.data(), 640, 480, 640 * 2};
    const auto bytes = pipeline.process(yuyv, frame.sequence, frame.stamp_ns);
    const auto hex = opr::core::telemetry::to_hex(bytes);
    ASSERT_EQ(hex, golden[i]) << f.scenario << " seed " << f.seed << " frame " << i;
    ++i;
  }
}

INSTANTIATE_TEST_SUITE_P(AllFlights, Equivalence, ::testing::ValuesIn(flights()),
                         [](const ::testing::TestParamInfo<Flight>& info) {
                           return info.param.scenario + "_" + std::to_string(info.param.seed);
                         });

}  // namespace
