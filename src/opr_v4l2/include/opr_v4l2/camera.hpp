// A V4L2 memory-mapped streaming capture driver.
//
// open(): QUERYCAP (capture + streaming), S_FMT (refuses a driver that silently changes the size
// or pixel format), S_PARM (frame interval), REQBUFS, QUERYBUF + mmap per buffer, QBUF all,
// STREAMON. grab(): poll, DQBUF, and hand the caller a Buffer that points into the mapped memory.
// The Buffer goes back to the driver (QBUF) when it is destroyed, so a consumer that holds frames
// too long starves the queue and the drop counter shows it.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "opr_v4l2/syscalls.hpp"

namespace opr::v4l2 {

struct Config {
  std::string device = "/dev/video0";
  int width = 640;
  int height = 480;
  std::uint32_t fourcc = 0x56595559;  // 'YUYV'
  int buffers = 4;
  int fps = 30;
};

enum class Status { kOk, kTimeout, kDisconnected, kError };
const char* to_string(Status s);

enum class StampSource {
  kStartOfExposure,  // buffer timestamp, CLOCK_MONOTONIC, start of exposure
  kEndOfFrame,       // buffer timestamp, CLOCK_MONOTONIC, end of readout; corrected by one interval
  kDequeue,          // driver gave no monotonic timestamp: time of DQBUF (late by up to a frame)
};

struct Stats {
  std::uint64_t frames = 0;        // delivered to the caller
  std::uint64_t dropped = 0;       // gaps in the driver's sequence numbers
  std::uint64_t corrupted = 0;     // buffers flagged V4L2_BUF_FLAG_ERROR or short, requeued unseen
  std::uint64_t timeouts = 0;
  std::uint64_t disconnects = 0;
  std::uint64_t opens = 0;
};

class Camera;

/// A dequeued frame. Movable, not copyable; requeues itself on destruction.
class Buffer {
 public:
  Buffer() = default;
  Buffer(Buffer&& other) noexcept;
  Buffer& operator=(Buffer&& other) noexcept;
  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;
  ~Buffer();

  explicit operator bool() const { return camera_ != nullptr; }
  const std::uint8_t* data() const { return data_; }
  std::size_t bytes() const { return bytes_; }
  std::size_t step() const { return step_; }
  std::uint32_t sequence() const { return sequence_; }
  std::uint64_t stamp_ns() const { return stamp_ns_; }
  StampSource stamp_source() const { return source_; }
  int index() const { return index_; }
  /// Gives the buffer back to the driver now.
  void release();

 private:
  friend class Camera;
  Camera* camera_ = nullptr;
  std::uint64_t generation_ = 0;
  int index_ = -1;
  const std::uint8_t* data_ = nullptr;
  std::size_t bytes_ = 0;
  std::size_t step_ = 0;
  std::uint32_t sequence_ = 0;
  std::uint64_t stamp_ns_ = 0;
  StampSource source_ = StampSource::kDequeue;
};

class Camera {
 public:
  Camera(Config config, std::shared_ptr<Syscalls> sys);
  ~Camera();
  Camera(const Camera&) = delete;
  Camera& operator=(const Camera&) = delete;

  /// Opens and starts streaming. On failure returns kError or kDisconnected and `error()` says why.
  Status open();
  /// Stops capture but keeps the device open and its buffers mapped (a paused sensor delivers no
  /// stale frames when it resumes). Outstanding Buffers become inert.
  void stop_streaming();
  /// Queues every buffer and starts capture again after stop_streaming().
  Status start_streaming();
  bool streaming() const { return streaming_; }

  /// Stops streaming and unmaps buffers. Outstanding Buffers become inert (they will not requeue
  /// into a closed device). Safe to call twice.
  void close();
  bool is_open() const { return fd_ >= 0; }

  /// Waits up to timeout_ms for a frame. kDisconnected means the device is gone (unplugged or
  /// reset): close() and open() again.
  Status grab(int timeout_ms, Buffer& out);

  const Stats& stats() const { return stats_; }
  const std::string& error() const { return error_; }
  const Config& config() const { return config_; }
  /// Buffers the driver currently owns (queued for capture).
  int queued() const { return queued_; }

 private:
  friend class Buffer;
  struct Mapping {
    void* addr = nullptr;
    std::size_t length = 0;
  };
  void requeue(int index, std::uint64_t generation);
  Status fail(Status s, const std::string& what);

  Config config_;
  std::shared_ptr<Syscalls> sys_;
  int fd_ = -1;
  std::vector<Mapping> maps_;
  std::size_t step_ = 0;
  std::size_t image_bytes_ = 0;
  std::uint64_t generation_ = 0;  // bumped on every open, so stale Buffers never requeue
  bool have_last_sequence_ = false;
  std::uint32_t last_sequence_ = 0;
  std::uint64_t interval_ns_ = 0;
  int queued_ = 0;
  bool streaming_ = false;
  Stats stats_;
  std::string error_;
};

}  // namespace opr::v4l2
