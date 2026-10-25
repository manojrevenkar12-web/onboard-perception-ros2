// The handful of system calls a V4L2 capture driver makes, behind an interface so the driver can
// be tested against a simulated device (FakeDevice) and run against /dev/videoN unchanged.
#pragma once

#include <poll.h>
#include <sys/types.h>

#include <cstddef>
#include <memory>
#include <string>

namespace opr::v4l2 {

class Syscalls {
 public:
  virtual ~Syscalls() = default;
  virtual int open(const char* path, int flags) = 0;
  virtual int close(int fd) = 0;
  /// Returns -1 and sets errno on failure, like ioctl(2).
  virtual int ioctl(int fd, unsigned long request, void* arg) = 0;
  virtual void* mmap(std::size_t length, int fd, off_t offset) = 0;
  virtual int munmap(void* addr, std::size_t length) = 0;
  virtual int poll(int fd, short events, int timeout_ms, short* revents) = 0;
};

/// The real kernel.
std::shared_ptr<Syscalls> real_syscalls();

/// "sim://<scenario>?seed=N&pace_hz=F&unplug_at=K&unplug_ms=M&..." gives a FakeDevice fed by the
/// simulator; anything else gives the real kernel. See fake_device.hpp for the options.
std::shared_ptr<Syscalls> syscalls_for(const std::string& device);

}  // namespace opr::v4l2
