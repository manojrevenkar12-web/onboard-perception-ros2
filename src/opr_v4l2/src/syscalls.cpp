#include "opr_v4l2/syscalls.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>

#include "opr_v4l2/fake_device.hpp"

namespace opr::v4l2 {

namespace {

class Kernel final : public Syscalls {
 public:
  int open(const char* path, int flags) override { return ::open(path, flags); }
  int close(int fd) override { return ::close(fd); }
  int ioctl(int fd, unsigned long request, void* arg) override {
    int r;
    do {
      r = ::ioctl(fd, request, arg);
    } while (r < 0 && errno == EINTR);
    return r;
  }
  void* mmap(std::size_t length, int fd, off_t offset) override {
    return ::mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
  }
  int munmap(void* addr, std::size_t length) override { return ::munmap(addr, length); }
  int poll(int fd, short events, int timeout_ms, short* revents) override {
    pollfd p{fd, events, 0};
    const int r = ::poll(&p, 1, timeout_ms);
    *revents = p.revents;
    return r;
  }
};

}  // namespace

std::shared_ptr<Syscalls> real_syscalls() { return std::make_shared<Kernel>(); }

std::shared_ptr<Syscalls> syscalls_for(const std::string& device) {
  if (device.rfind("sim://", 0) == 0) return std::make_shared<FakeDevice>(parse_sim_url(device));
  return real_syscalls();
}

}  // namespace opr::v4l2
