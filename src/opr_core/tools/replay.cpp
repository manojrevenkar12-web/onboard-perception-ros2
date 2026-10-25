// Replays a simulated flight through opr_core and writes one telemetry datagram per line (hex),
// the same output as `visionbox --replay scenario:seed --record file`.
// Usage: opr_replay <scenario> <seed> <out.hex>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "opr_core/pipeline.hpp"
#include "opr_core/telemetry.hpp"
#include "opr_sim/scene.hpp"

int main(int argc, char** argv) {
  if (argc != 4) {
    std::fprintf(stderr, "usage: %s <scenario> <seed> <out.hex>\n", argv[0]);
    return 2;
  }
  opr::sim::Scene scene(argv[1], std::strtoull(argv[2], nullptr, 10));
  opr::core::Pipeline pipeline({}, {}, {});
  std::ofstream out(argv[3]);
  opr::sim::Frame f;
  while (scene.next(f)) {
    opr::core::View<const std::uint8_t> yuyv{f.yuyv.data(), 640, 480, 640 * 2};
    out << opr::core::telemetry::to_hex(pipeline.process(yuyv, f.sequence, f.stamp_ns)) << "\n";
  }
  return 0;
}
