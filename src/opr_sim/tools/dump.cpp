// Writes one frame of a scenario as a PGM image (the Y channel) and prints its ground truth.
// Usage: opr_sim_dump <scenario> <seed> <sequence> <out.pgm>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "opr_sim/scene.hpp"

int main(int argc, char** argv) {
  if (argc != 5) {
    std::fprintf(stderr, "usage: %s <scenario> <seed> <sequence> <out.pgm>\n", argv[0]);
    return 2;
  }
  opr::sim::Scene scene(argv[1], std::strtoull(argv[2], nullptr, 10));
  opr::sim::Frame frame;
  scene.render(static_cast<std::uint32_t>(std::strtoul(argv[3], nullptr, 10)), frame);
  const auto& c = scene.calibration();
  std::ofstream out(argv[4], std::ios::binary);
  out << "P5\n" << c.width << " " << c.height << "\n255\n";
  for (std::size_t i = 0; i < frame.yuyv.size(); i += 2) out.put(static_cast<char>(frame.yuyv[i]));
  for (const auto& t : frame.truth) {
    std::printf("target %u at (%.2f, %.2f)%s\n", t.id, t.x, t.y, t.visible ? "" : " hidden");
  }
  return 0;
}
