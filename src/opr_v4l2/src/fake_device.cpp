#include "opr_v4l2/fake_device.hpp"

#include <sys/mman.h>
#include <time.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace opr::v4l2 {

namespace {

std::uint64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}

int fail(int err) {
  errno = err;
  return -1;
}

}  // namespace

FakeOptions parse_sim_url(const std::string& url) {
  const std::string prefix = "sim://";
  if (url.rfind(prefix, 0) != 0) throw std::invalid_argument("not a sim:// device: " + url);
  FakeOptions o;
  const std::string rest = url.substr(prefix.size());
  const auto q = rest.find('?');
  o.scenario = rest.substr(0, q);
  if (q == std::string::npos) return o;
  std::stringstream ss(rest.substr(q + 1));
  std::string kv;
  while (std::getline(ss, kv, '&')) {
    if (kv.empty()) continue;
    const auto eq = kv.find('=');
    if (eq == std::string::npos) throw std::invalid_argument("expected key=value in " + url);
    const std::string k = kv.substr(0, eq);
    const std::string v = kv.substr(eq + 1);
    if (k == "seed") o.seed = std::stoull(v);
    else if (k == "pace_hz") o.pace_hz = std::stod(v);
    else if (k == "stamps") {
      if (v != "sim" && v != "clock") throw std::invalid_argument("stamps must be sim or clock");
      o.clock_stamps = v == "clock";
    } else if (k == "loop") o.loop = v == "1";
    else if (k == "unplug_at") o.unplug_at = std::stol(v);
    else if (k == "unplug_ms") o.unplug_ms = std::stoi(v);
    else if (k == "corrupt_every") o.corrupt_every = std::stoi(v);
    else if (k == "eof_stamps") o.eof_stamps = v == "1";
    else if (k == "no_timestamps") o.no_timestamps = v == "1";
    else if (k == "wrong_size") o.wrong_size = v == "1";
    else throw std::invalid_argument("unknown sim:// option: " + k);
  }
  return o;
}

FakeDevice::FakeDevice(FakeOptions options)
    : opt_(std::move(options)),
      scene_(opt_.scenario, opt_.seed),
      width_(scene_.calibration().width),
      height_(scene_.calibration().height) {}

bool FakeDevice::gone() {
  if (!unplugged_) return false;
  if (Clock::now() >= replug_at_ && !is_open_) unplugged_ = false;
  return unplugged_;
}

int FakeDevice::open(const char* /*path*/, int /*flags*/) {
  if (gone()) return fail(ENOENT);
  if (is_open_) return fail(EBUSY);
  is_open_ = true;
  ++opens_;
  return kFd;
}

int FakeDevice::close(int fd) {
  if (fd != kFd || !is_open_) return fail(EBADF);
  is_open_ = false;
  streaming_ = false;
  buffers_.clear();
  queued_.clear();
  done_.clear();
  return 0;
}

bool FakeDevice::produce_one() {
  const long n = scene_.frame_count();
  if (!opt_.loop && world_ >= n) return false;
  if (opt_.unplug_at >= 0 && !unplug_done_ && world_ >= opt_.unplug_at) {
    unplug_done_ = true;
    unplugged_ = true;
    replug_at_ = Clock::now() + std::chrono::milliseconds(opt_.unplug_ms);
    // The world keeps moving while the cable is out.
    world_ += static_cast<long>(opt_.unplug_ms * 30 / 1000);
    return false;
  }
  const std::uint32_t seq = sequence_++;
  if (queued_.empty()) {  // sensor exposed a frame but had nowhere to put it
    ++world_;
    return true;
  }
  const int index = queued_.front();
  queued_.pop_front();
  scene_.render(static_cast<std::uint32_t>(world_ % n), frame_);
  const long lap = world_ / n;
  std::uint64_t stamp = frame_.stamp_ns + static_cast<std::uint64_t>(lap) * static_cast<std::uint64_t>(n) * 33'333'000ULL;
  if (opt_.clock_stamps) stamp = monotonic_ns();
  auto& mem = buffers_[static_cast<std::size_t>(index)];
  std::memcpy(mem.data(), frame_.yuyv.data(), std::min(mem.size(), frame_.yuyv.size()));

  v4l2_buffer meta{};
  meta.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  meta.memory = V4L2_MEMORY_MMAP;
  meta.index = static_cast<std::uint32_t>(index);
  meta.bytesused = static_cast<std::uint32_t>(frame_.yuyv.size());
  meta.sequence = seq;
  meta.field = V4L2_FIELD_NONE;
  if (opt_.eof_stamps) stamp += 1'000'000'000ULL / 30;  // readout ends one interval after exposure starts
  meta.timestamp.tv_sec = static_cast<time_t>(stamp / 1'000'000'000ULL);
  meta.timestamp.tv_usec = static_cast<suseconds_t>((stamp % 1'000'000'000ULL) / 1000ULL);
  meta.flags = V4L2_BUF_FLAG_DONE;
  if (!opt_.no_timestamps) {
    meta.flags |= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    meta.flags |= opt_.eof_stamps ? V4L2_BUF_FLAG_TSTAMP_SRC_EOF : V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
  }
  if (opt_.corrupt_every > 0 && (world_ + 1) % opt_.corrupt_every == 0) meta.flags |= V4L2_BUF_FLAG_ERROR;
  done_.push_back({index, meta});
  ++world_;
  return true;
}

void FakeDevice::produce_due() {
  if (!streaming_ || unplugged_) return;
  if (opt_.pace_hz <= 0.0) {
    // Lockstep: one frame per queued buffer, as fast as the consumer returns them.
    while (!queued_.empty() && done_.empty() && !unplugged_) {
      if (!produce_one()) break;
    }
    return;
  }
  const auto elapsed = std::chrono::duration<double>(Clock::now() - stream_start_).count();
  const auto due = static_cast<long>(elapsed * opt_.pace_hz) + 1;
  while (produced_since_start_ < due && !unplugged_) {
    if (!produce_one()) break;
    ++produced_since_start_;
  }
}

int FakeDevice::ioctl(int fd, unsigned long request, void* arg) {
  if (fd != kFd || !is_open_) return fail(EBADF);
  if (gone()) return fail(ENODEV);
  switch (request) {
    case VIDIOC_QUERYCAP: {
      auto* cap = static_cast<v4l2_capability*>(arg);
      std::memset(cap, 0, sizeof(*cap));
      std::strncpy(reinterpret_cast<char*>(cap->driver), "opr_fake", sizeof(cap->driver) - 1);
      std::strncpy(reinterpret_cast<char*>(cap->card), "Simulated UVC camera", sizeof(cap->card) - 1);
      cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;
      cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
      return 0;
    }
    case VIDIOC_S_FMT: {
      auto* f = static_cast<v4l2_format*>(arg);
      if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) return fail(EINVAL);
      if (streaming_) return fail(EBUSY);
      // Like a real driver: adjust to the one mode we have instead of failing.
      f->fmt.pix.width = static_cast<std::uint32_t>(opt_.wrong_size ? width_ / 2 : width_);
      f->fmt.pix.height = static_cast<std::uint32_t>(opt_.wrong_size ? height_ / 2 : height_);
      f->fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      f->fmt.pix.field = V4L2_FIELD_NONE;
      f->fmt.pix.bytesperline = f->fmt.pix.width * 2;
      f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;
      return 0;
    }
    case VIDIOC_S_PARM: {
      auto* p = static_cast<v4l2_streamparm*>(arg);
      p->parm.capture.timeperframe.numerator = 1;
      p->parm.capture.timeperframe.denominator = 30;
      return 0;
    }
    case VIDIOC_REQBUFS: {
      auto* r = static_cast<v4l2_requestbuffers*>(arg);
      if (r->memory != V4L2_MEMORY_MMAP) return fail(EINVAL);
      if (streaming_) return fail(EBUSY);
      if (mapped_ > 0 && r->count > 0) return fail(EBUSY);
      const std::uint32_t n = std::min<std::uint32_t>(r->count, 8);
      buffers_.assign(n, std::vector<std::uint8_t>(static_cast<std::size_t>(width_) * height_ * 2));
      queued_.clear();
      done_.clear();
      r->count = n;
      return 0;
    }
    case VIDIOC_QUERYBUF: {
      auto* b = static_cast<v4l2_buffer*>(arg);
      if (b->index >= buffers_.size()) return fail(EINVAL);
      b->length = static_cast<std::uint32_t>(buffers_[b->index].size());
      b->m.offset = static_cast<std::uint32_t>(b->index * kPage);
      return 0;
    }
    case VIDIOC_QBUF: {
      auto* b = static_cast<v4l2_buffer*>(arg);
      if (b->index >= buffers_.size()) return fail(EINVAL);
      const int i = static_cast<int>(b->index);
      if (std::find(queued_.begin(), queued_.end(), i) != queued_.end()) return fail(EINVAL);
      for (const auto& d : done_) {
        if (d.index == i) return fail(EINVAL);
      }
      queued_.push_back(i);
      return 0;
    }
    case VIDIOC_DQBUF: {
      auto* b = static_cast<v4l2_buffer*>(arg);
      if (!streaming_) return fail(EINVAL);
      produce_due();
      if (gone()) return fail(ENODEV);
      if (done_.empty()) return fail(EAGAIN);
      *b = done_.front().meta;
      done_.pop_front();
      return 0;
    }
    case VIDIOC_STREAMON:
      if (buffers_.empty()) return fail(EINVAL);
      streaming_ = true;
      sequence_ = 0;
      produced_since_start_ = 0;
      stream_start_ = Clock::now();
      return 0;
    case VIDIOC_STREAMOFF:
      streaming_ = false;
      queued_.clear();
      done_.clear();
      return 0;
    default:
      return fail(ENOTTY);
  }
}

void* FakeDevice::mmap(std::size_t length, int fd, off_t offset) {
  if (fd != kFd || !is_open_) {
    errno = EBADF;
    return MAP_FAILED;
  }
  const auto index = static_cast<std::size_t>(offset) / kPage;
  if (index >= buffers_.size() || length != buffers_[index].size()) {
    errno = EINVAL;
    return MAP_FAILED;
  }
  ++mapped_;
  return buffers_[index].data();
}

int FakeDevice::munmap(void* /*addr*/, std::size_t /*length*/) {
  if (mapped_ > 0) --mapped_;
  return 0;
}

int FakeDevice::poll(int fd, short /*events*/, int timeout_ms, short* revents) {
  *revents = 0;
  if (fd != kFd || !is_open_) {
    *revents = POLLNVAL;
    return 1;
  }
  const auto deadline = Clock::now() + std::chrono::milliseconds(std::max(timeout_ms, 0));
  for (;;) {
    if (gone()) {
      *revents = POLLERR;
      return 1;
    }
    produce_due();
    if (gone()) {
      *revents = POLLERR;
      return 1;
    }
    if (!done_.empty()) {
      *revents = POLLIN;
      return 1;
    }
    if (Clock::now() >= deadline) return 0;
    if (opt_.pace_hz <= 0.0) {
      // Lockstep with nothing ready: no buffer is queued or the flight is over. Nothing can
      // change until the caller acts, so wait out the timeout like an idle sensor would.
      std::this_thread::sleep_until(deadline);
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
}

}  // namespace opr::v4l2
