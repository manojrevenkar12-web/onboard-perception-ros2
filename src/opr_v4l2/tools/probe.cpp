// Opens a camera with the production driver and reports what it delivers: frame rate, drops,
// timestamp source and the age of each frame when it is dequeued. Run it on the vehicle before
// blaming the pipeline. Usage: opr_v4l2_probe <device|sim://...> [seconds] [width height]
#include <time.h>

#include <cstdio>
#include <cstdlib>
#include <string>

#include "opr_v4l2/camera.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <device|sim://...> [seconds] [width height]\n", argv[0]);
    return 2;
  }
  opr::v4l2::Config cfg;
  cfg.device = argv[1];
  const double seconds = argc > 2 ? std::atof(argv[2]) : 5.0;
  if (argc > 4) {
    cfg.width = std::atoi(argv[3]);
    cfg.height = std::atoi(argv[4]);
  }
  opr::v4l2::Camera cam(cfg, opr::v4l2::syscalls_for(cfg.device));
  if (cam.open() != opr::v4l2::Status::kOk) {
    std::fprintf(stderr, "open failed: %s\n", cam.error().c_str());
    return 1;
  }
  auto now = [] {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + ts.tv_nsec * 1e-9;
  };
  const double start = now();
  double max_age_ms = 0;
  const char* source = "?";
  while (now() - start < seconds) {
    opr::v4l2::Buffer b;
    const auto s = cam.grab(500, b);
    if (s == opr::v4l2::Status::kTimeout) continue;
    if (s != opr::v4l2::Status::kOk) {
      std::fprintf(stderr, "grab: %s (%s)\n", opr::v4l2::to_string(s), cam.error().c_str());
      break;
    }
    const double age = (now() - b.stamp_ns() * 1e-9) * 1e3;
    if (age > max_age_ms) max_age_ms = age;
    switch (b.stamp_source()) {
      case opr::v4l2::StampSource::kStartOfExposure: source = "start of exposure"; break;
      case opr::v4l2::StampSource::kEndOfFrame: source = "end of frame (corrected)"; break;
      case opr::v4l2::StampSource::kDequeue: source = "dequeue time (no driver timestamp)"; break;
    }
  }
  const auto& st = cam.stats();
  const double elapsed = now() - start;
  std::printf("frames %llu (%.1f fps), dropped %llu, corrupted %llu, timeouts %llu\n",
              static_cast<unsigned long long>(st.frames), st.frames / elapsed,
              static_cast<unsigned long long>(st.dropped), static_cast<unsigned long long>(st.corrupted),
              static_cast<unsigned long long>(st.timeouts));
  std::printf("timestamps: %s; oldest frame at dequeue %.1f ms\n", source, max_age_ms);
  return 0;
}
