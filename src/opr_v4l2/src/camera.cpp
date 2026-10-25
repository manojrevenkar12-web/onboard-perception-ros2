#include "opr_v4l2/camera.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <time.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace opr::v4l2 {

namespace {

std::uint64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}

bool gone(int err) { return err == ENODEV || err == ENXIO || err == EIO; }

std::string fourcc_name(std::uint32_t f) {
  std::string s(4, ' ');
  for (int i = 0; i < 4; ++i) s[i] = static_cast<char>((f >> (8 * i)) & 0xff);
  return s;
}

}  // namespace

const char* to_string(Status s) {
  switch (s) {
    case Status::kOk:
      return "ok";
    case Status::kTimeout:
      return "timeout";
    case Status::kDisconnected:
      return "disconnected";
    case Status::kError:
      return "error";
  }
  return "?";
}

// ------------------------------------------------------------------------------------- Buffer

Buffer::Buffer(Buffer&& o) noexcept { *this = std::move(o); }

Buffer& Buffer::operator=(Buffer&& o) noexcept {
  if (this != &o) {
    release();
    camera_ = std::exchange(o.camera_, nullptr);
    generation_ = o.generation_;
    index_ = o.index_;
    data_ = o.data_;
    bytes_ = o.bytes_;
    step_ = o.step_;
    sequence_ = o.sequence_;
    stamp_ns_ = o.stamp_ns_;
    source_ = o.source_;
  }
  return *this;
}

Buffer::~Buffer() { release(); }

void Buffer::release() {
  if (camera_ != nullptr) {
    camera_->requeue(index_, generation_);
    camera_ = nullptr;
    data_ = nullptr;
  }
}

// ------------------------------------------------------------------------------------- Camera

Camera::Camera(Config config, std::shared_ptr<Syscalls> sys) : config_(std::move(config)), sys_(std::move(sys)) {}

Camera::~Camera() { close(); }

Status Camera::fail(Status s, const std::string& what) {
  error_ = what + (errno ? std::string(": ") + std::strerror(errno) : std::string());
  close();
  return s;
}

Status Camera::open() {
  close();
  errno = 0;
  error_.clear();
  ++generation_;
  fd_ = sys_->open(config_.device.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) return fail(gone(errno) || errno == ENOENT ? Status::kDisconnected : Status::kError, "open " + config_.device);
  ++stats_.opens;

  v4l2_capability cap{};
  if (sys_->ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) return fail(Status::kError, "VIDIOC_QUERYCAP");
  const std::uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
  if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) {
    errno = 0;
    return fail(Status::kError, config_.device + " is not a video capture device");
  }
  if (!(caps & V4L2_CAP_STREAMING)) {
    errno = 0;
    return fail(Status::kError, config_.device + " does not support streaming I/O");
  }

  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = static_cast<std::uint32_t>(config_.width);
  fmt.fmt.pix.height = static_cast<std::uint32_t>(config_.height);
  fmt.fmt.pix.pixelformat = config_.fourcc;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (sys_->ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) return fail(Status::kError, "VIDIOC_S_FMT");
  // Drivers may adjust the request to the nearest mode they support. A silently different size
  // would break the calibration downstream, so refuse it.
  if (fmt.fmt.pix.width != static_cast<std::uint32_t>(config_.width) ||
      fmt.fmt.pix.height != static_cast<std::uint32_t>(config_.height) || fmt.fmt.pix.pixelformat != config_.fourcc) {
    errno = 0;
    return fail(Status::kError, "device offers " + std::to_string(fmt.fmt.pix.width) + "x" +
                                    std::to_string(fmt.fmt.pix.height) + " " + fourcc_name(fmt.fmt.pix.pixelformat) +
                                    ", configured " + std::to_string(config_.width) + "x" +
                                    std::to_string(config_.height) + " " + fourcc_name(config_.fourcc));
  }
  step_ = fmt.fmt.pix.bytesperline;
  image_bytes_ = fmt.fmt.pix.sizeimage;

  v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = static_cast<std::uint32_t>(config_.fps);
  if (sys_->ioctl(fd_, VIDIOC_S_PARM, &parm) == 0 && parm.parm.capture.timeperframe.denominator != 0) {
    interval_ns_ = 1'000'000'000ULL * parm.parm.capture.timeperframe.numerator /
                   parm.parm.capture.timeperframe.denominator;
  } else {
    interval_ns_ = 1'000'000'000ULL / static_cast<std::uint64_t>(config_.fps);  // not all drivers support S_PARM
  }

  v4l2_requestbuffers req{};
  req.count = static_cast<std::uint32_t>(config_.buffers);
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (sys_->ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) return fail(Status::kError, "VIDIOC_REQBUFS");
  if (req.count < 2) {
    errno = 0;
    return fail(Status::kError, "driver granted only " + std::to_string(req.count) + " buffer(s)");
  }
  maps_.assign(req.count, Mapping{});
  for (std::uint32_t i = 0; i < req.count; ++i) {
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = i;
    if (sys_->ioctl(fd_, VIDIOC_QUERYBUF, &b) < 0) return fail(Status::kError, "VIDIOC_QUERYBUF");
    void* addr = sys_->mmap(b.length, fd_, static_cast<off_t>(b.m.offset));
    if (addr == MAP_FAILED || addr == nullptr) return fail(Status::kError, "mmap");
    maps_[i] = {addr, b.length};
  }
  return start_streaming();
}

Status Camera::start_streaming() {
  if (fd_ < 0) {
    errno = 0;
    error_ = "not open";
    return Status::kError;
  }
  if (streaming_) return Status::kOk;
  for (std::size_t i = 0; i < maps_.size(); ++i) {
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = static_cast<std::uint32_t>(i);
    if (sys_->ioctl(fd_, VIDIOC_QBUF, &b) < 0) return fail(gone(errno) ? Status::kDisconnected : Status::kError, "VIDIOC_QBUF");
  }
  queued_ = static_cast<int>(maps_.size());
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (sys_->ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    return fail(gone(errno) ? Status::kDisconnected : Status::kError, "VIDIOC_STREAMON");
  }
  streaming_ = true;
  have_last_sequence_ = false;
  return Status::kOk;
}

void Camera::stop_streaming() {
  if (fd_ < 0 || !streaming_) return;
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  sys_->ioctl(fd_, VIDIOC_STREAMOFF, &type);  // returns every buffer to us, queued or filled
  streaming_ = false;
  queued_ = 0;
  ++generation_;  // a Buffer from the old stream must not be queued into the next one
}

void Camera::close() {
  if (fd_ < 0) return;
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  sys_->ioctl(fd_, VIDIOC_STREAMOFF, &type);  // may fail on a vanished device; nothing to do then
  streaming_ = false;
  for (auto& m : maps_) {
    if (m.addr != nullptr) sys_->munmap(m.addr, m.length);
  }
  maps_.clear();
  v4l2_requestbuffers req{};
  req.count = 0;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  sys_->ioctl(fd_, VIDIOC_REQBUFS, &req);
  sys_->close(fd_);
  fd_ = -1;
  queued_ = 0;
  ++generation_;  // Buffers handed out before this point must not requeue
}

void Camera::requeue(int index, std::uint64_t generation) {
  if (fd_ < 0 || !streaming_ || generation != generation_) return;
  v4l2_buffer b{};
  b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  b.memory = V4L2_MEMORY_MMAP;
  b.index = static_cast<std::uint32_t>(index);
  if (sys_->ioctl(fd_, VIDIOC_QBUF, &b) == 0) ++queued_;
}

Status Camera::grab(int timeout_ms, Buffer& out) {
  out.release();
  if (fd_ < 0 || !streaming_) {
    errno = 0;
    error_ = fd_ < 0 ? "not open" : "not streaming";
    return Status::kError;
  }
  for (;;) {
    short revents = 0;
    errno = 0;
    const int r = sys_->poll(fd_, POLLIN, timeout_ms, &revents);
    if (r < 0) {
      if (errno == EINTR) continue;
      if (gone(errno)) {
        ++stats_.disconnects;
        return fail(Status::kDisconnected, "poll");
      }
      return fail(Status::kError, "poll");
    }
    if (r == 0) {
      ++stats_.timeouts;
      return Status::kTimeout;
    }
    if (revents & (POLLERR | POLLHUP | POLLNVAL)) {
      ++stats_.disconnects;
      errno = ENODEV;
      return fail(Status::kDisconnected, "device reported an error condition");
    }

    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    errno = 0;
    if (sys_->ioctl(fd_, VIDIOC_DQBUF, &b) < 0) {
      if (errno == EAGAIN) continue;  // woken without a buffer ready
      if (gone(errno)) {
        ++stats_.disconnects;
        return fail(Status::kDisconnected, "VIDIOC_DQBUF");
      }
      return fail(Status::kError, "VIDIOC_DQBUF");
    }
    --queued_;
    if (have_last_sequence_ && b.sequence > last_sequence_ + 1) stats_.dropped += b.sequence - last_sequence_ - 1;
    have_last_sequence_ = true;
    last_sequence_ = b.sequence;

    if ((b.flags & V4L2_BUF_FLAG_ERROR) || b.bytesused < image_bytes_ || b.index >= maps_.size()) {
      ++stats_.corrupted;
      requeue(static_cast<int>(b.index), generation_);
      continue;
    }

    StampSource source = StampSource::kDequeue;
    std::uint64_t stamp = 0;
    if ((b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
      stamp = static_cast<std::uint64_t>(b.timestamp.tv_sec) * 1'000'000'000ULL +
              static_cast<std::uint64_t>(b.timestamp.tv_usec) * 1000ULL;
      if ((b.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) == V4L2_BUF_FLAG_TSTAMP_SRC_EOF) {
        // End of readout: the exposure began about one frame interval earlier.
        source = StampSource::kEndOfFrame;
        stamp = stamp > interval_ns_ ? stamp - interval_ns_ : 0;
      } else {
        source = StampSource::kStartOfExposure;
      }
    } else {
      stamp = monotonic_ns();
    }

    out.camera_ = this;
    out.generation_ = generation_;
    out.index_ = static_cast<int>(b.index);
    out.data_ = static_cast<const std::uint8_t*>(maps_[b.index].addr);
    out.bytes_ = b.bytesused;
    out.step_ = step_;
    out.sequence_ = b.sequence;
    out.stamp_ns_ = stamp;
    out.source_ = source;
    ++stats_.frames;
    return Status::kOk;
  }
}

}  // namespace opr::v4l2
