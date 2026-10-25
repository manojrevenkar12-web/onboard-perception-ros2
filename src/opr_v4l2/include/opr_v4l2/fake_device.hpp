// A simulated V4L2 capture device: answers the driver's ioctls like a UVC camera would and fills
// its buffers with frames from opr_sim. Used by the tests and by the camera node when its device
// is "sim://<scenario>?option=value&...":
//
//   seed=N          simulator seed (1)
//   pace_hz=F       produce frames on the wall clock at F Hz and drop them when no buffer is
//                   queued, like a sensor; 0 = a frame is ready whenever a buffer is (lockstep) (0)
//   stamps=sim|clock  buffer timestamps from the simulator (reproducible) or CLOCK_MONOTONIC at
//                   production (for latency measurements) (sim)
//   loop=0|1        restart the flight when it ends instead of going quiet (0)
//   unplug_at=K     vanish before world frame K: ioctls fail with ENODEV, poll reports POLLERR (-1)
//   unplug_ms=M     come back M ms later; open() fails with ENOENT until then (500)
//   corrupt_every=N flag every Nth frame V4L2_BUF_FLAG_ERROR (0 = never)
//   eof_stamps=1    report end-of-frame timestamps instead of start-of-exposure (0)
//   no_timestamps=1 report no monotonic timestamp at all (0)
//   wrong_size=1    answer S_FMT with 320x240, as a driver without the mode would (0)
#pragma once

#include <linux/videodev2.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "opr_sim/scene.hpp"
#include "opr_v4l2/syscalls.hpp"

namespace opr::v4l2 {

struct FakeOptions {
  std::string scenario = "crossing";
  std::uint64_t seed = 1;
  double pace_hz = 0.0;
  bool clock_stamps = false;
  bool loop = false;
  long unplug_at = -1;
  int unplug_ms = 500;
  int corrupt_every = 0;
  bool eof_stamps = false;
  bool no_timestamps = false;
  bool wrong_size = false;
};

/// Parses "sim://scenario?k=v&..." (throws std::invalid_argument on unknown keys or scenarios).
FakeOptions parse_sim_url(const std::string& url);

class FakeDevice : public Syscalls {
 public:
  explicit FakeDevice(FakeOptions options);

  int open(const char* path, int flags) override;
  int close(int fd) override;
  int ioctl(int fd, unsigned long request, void* arg) override;
  void* mmap(std::size_t length, int fd, off_t offset) override;
  int munmap(void* addr, std::size_t length) override;
  int poll(int fd, short events, int timeout_ms, short* revents) override;

  // Introspection for tests.
  long world_frame() const { return world_; }
  int open_count() const { return opens_; }
  int mapped() const { return mapped_; }
  bool streaming() const { return streaming_; }
  const opr::sim::Scene& scene() const { return scene_; }

 private:
  using Clock = std::chrono::steady_clock;
  static constexpr int kFd = 1000;
  static constexpr std::size_t kPage = 1 << 20;

  bool gone();
  void produce_due();
  bool produce_one();

  FakeOptions opt_;
  opr::sim::Scene scene_;
  opr::sim::Frame frame_;
  int width_;
  int height_;
  bool is_open_ = false;
  bool streaming_ = false;
  bool unplugged_ = false;
  bool unplug_done_ = false;
  Clock::time_point replug_at_{};
  Clock::time_point stream_start_{};
  long produced_since_start_ = 0;
  long world_ = 0;              // simulator frame index (the world keeps moving across reconnects)
  std::uint32_t sequence_ = 0;  // per-stream buffer sequence, restarts at STREAMON
  std::vector<std::vector<std::uint8_t>> buffers_;
  std::deque<int> queued_;
  struct Done {
    int index;
    v4l2_buffer meta;
  };
  std::deque<Done> done_;
  int opens_ = 0;
  int mapped_ = 0;
};

}  // namespace opr::v4l2
