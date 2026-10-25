#include "opr_shm/segment.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <random>
#include <stdexcept>
#include <system_error>

namespace opr::shm {

static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "process-shared atomics must be lock-free");

struct SegmentHeader {
  std::uint64_t magic;
  std::uint32_t version;
  std::uint32_t slots;
  std::uint64_t slot_bytes;
  std::uint64_t stride;
  std::uint64_t epoch;
  char boot_id[40];
  std::atomic<std::uint64_t> committed;
};

struct SlotHeader {
  std::atomic<std::uint64_t> seq;          // odd while being written
  std::atomic<std::uint64_t> lease_until;  // CLOCK_MONOTONIC ns; 0 = free
  Meta meta;
};

namespace {

constexpr std::size_t kHeaderBytes = 4096;
constexpr std::size_t kSlotHeaderBytes = 256;
static_assert(sizeof(SegmentHeader) <= kHeaderBytes, "segment header too large");
static_assert(sizeof(SlotHeader) <= kSlotHeaderBytes, "slot header too large");

std::size_t round_up(std::size_t v, std::size_t to) { return (v + to - 1) / to * to; }

void set_error(ReadError* out, ReadError e) {
  if (out) *out = e;
}

}  // namespace

const char* to_string(ReadError e) {
  switch (e) {
    case ReadError::kNone:
      return "none";
    case ReadError::kNotVisible:
      return "segment not visible";
    case ReadError::kStale:
      return "stale handle";
    case ReadError::kOverwritten:
      return "overwritten";
    case ReadError::kTorn:
      return "torn read";
    case ReadError::kCorrupt:
      return "corrupt segment";
  }
  return "?";
}

std::uint64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}

std::string boot_id() {
  std::ifstream in("/proc/sys/kernel/random/boot_id");
  std::string id;
  std::getline(in, id);
  return id;
}

std::string segment_name_for(const std::string& topic) {
  std::string s = "/opr";
  for (char c : topic) s.push_back(c == '/' ? '_' : c);
  s += "_" + std::to_string(::getpid());
  return s.size() > 200 ? s.substr(0, 200) : s;
}

// ------------------------------------------------------------------------------------ Writer

Writer::Writer(const std::string& name, std::uint32_t slots, std::size_t slot_bytes)
    : name_(name), slots_(slots), slot_bytes_(slot_bytes) {
  if (slots < 2) throw std::invalid_argument("need at least two slots");
  if (name.empty() || name[0] != '/') throw std::invalid_argument("shm name must start with '/'");
  stride_ = round_up(kSlotHeaderBytes + slot_bytes, 4096);
  size_ = kHeaderBytes + stride_ * slots;
  ::shm_unlink(name.c_str());  // a crashed predecessor may have left one behind
  const int fd = ::shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0660);
  if (fd < 0) throw std::system_error(errno, std::generic_category(), "shm_open " + name);
  if (::ftruncate(fd, static_cast<off_t>(size_)) != 0) {
    const int err = errno;
    ::close(fd);
    ::shm_unlink(name.c_str());
    throw std::system_error(err, std::generic_category(), "ftruncate " + name);
  }
  base_ = ::mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  const int err = errno;
  ::close(fd);
  if (base_ == MAP_FAILED) {
    base_ = nullptr;
    ::shm_unlink(name.c_str());
    throw std::system_error(err, std::generic_category(), "mmap " + name);
  }
  std::random_device rd;
  epoch_ = (static_cast<std::uint64_t>(rd()) << 32) ^ rd() ^ monotonic_ns();
  boot_id_ = boot_id();

  auto* h = new (base_) SegmentHeader{};
  h->version = kVersion;
  h->slots = slots;
  h->slot_bytes = slot_bytes;
  h->stride = stride_;
  h->epoch = epoch_;
  std::strncpy(h->boot_id, boot_id_.c_str(), sizeof(h->boot_id) - 1);
  h->committed.store(0);
  for (std::uint32_t i = 0; i < slots; ++i) {
    auto* s = new (slot(i)) SlotHeader{};
    s->seq.store(0);
    s->lease_until.store(0);
  }
  // Magic last: a reader that maps a half-initialised segment sees no magic and backs off.
  std::atomic_thread_fence(std::memory_order_seq_cst);
  h->magic = kMagic;
}

Writer::~Writer() {
  if (base_ != nullptr) ::munmap(base_, size_);
  ::shm_unlink(name_.c_str());
}

SlotHeader* Writer::slot(std::uint32_t i) const {
  return reinterpret_cast<SlotHeader*>(static_cast<std::uint8_t*>(base_) + kHeaderBytes + stride_ * i);
}

std::uint8_t* Writer::payload(std::uint32_t i) const {
  return reinterpret_cast<std::uint8_t*>(slot(i)) + kSlotHeaderBytes;
}

Writer::Loan Writer::loan(std::size_t bytes) {
  if (loaned_) throw std::logic_error("one loan at a time");
  if (bytes > slot_bytes_) {
    ++stats_.too_large;
    return {};
  }
  for (std::uint32_t k = 0; k < slots_; ++k) {
    const std::uint32_t i = (next_ + k) % slots_;
    SlotHeader* s = slot(i);
    if (s->lease_until.load() > monotonic_ns()) {
      ++stats_.lease_skips;
      continue;
    }
    const std::uint64_t seq = s->seq.load();
    s->seq.store(seq + 1);  // odd: being written; readers that look now will refuse the slot
    if (s->lease_until.load() > monotonic_ns()) {
      // A reader leased it between our two looks; it saw the old, still intact contents.
      s->seq.store(seq);
      ++stats_.lease_skips;
      continue;
    }
    next_ = (i + 1) % slots_;
    loaned_ = true;
    return {payload(i), slot_bytes_, i};
  }
  ++stats_.no_free_slot;
  return {};
}

Handle Writer::commit(const Loan& loan, const Meta& meta) {
  if (!loaned_ || !loan) throw std::logic_error("commit without a loan");
  SlotHeader* s = slot(loan.slot);
  s->meta = meta;
  const std::uint64_t seq = s->seq.load() + 1;  // even again, one generation later
  s->seq.store(seq);
  reinterpret_cast<SegmentHeader*>(base_)->committed.fetch_add(1);
  loaned_ = false;
  ++stats_.committed;
  return {name_, epoch_, boot_id_, loan.slot, seq};
}

void Writer::cancel(const Loan& loan) {
  if (!loaned_ || !loan) return;
  // Bytes may have been written: move to a new generation so no old handle trusts them.
  SlotHeader* s = slot(loan.slot);
  s->seq.store(s->seq.load() + 1);
  loaned_ = false;
}

// ------------------------------------------------------------------------------------ Reader

Reader::~Reader() { unmap(); }

void Reader::unmap() {
  if (base_ != nullptr) ::munmap(base_, size_);
  base_ = nullptr;
  size_ = 0;
  name_.clear();
  epoch_ = 0;
}

SlotHeader* Reader::slot(std::uint32_t i) const {
  return reinterpret_cast<SlotHeader*>(static_cast<std::uint8_t*>(base_) + kHeaderBytes + stride_ * i);
}

bool Reader::map(const Handle& h, ReadError* error) {
  if (base_ != nullptr && name_ == h.segment && epoch_ == h.epoch) return true;
  if (!h.boot_id.empty()) {
    static const std::string mine = boot_id();
    if (!mine.empty() && mine != h.boot_id) {
      set_error(error, ReadError::kNotVisible);  // published on another machine
      return false;
    }
  }
  unmap();
  const int fd = ::shm_open(h.segment.c_str(), O_RDWR, 0);
  if (fd < 0) {
    set_error(error, ReadError::kNotVisible);
    return false;
  }
  struct stat st {};
  if (::fstat(fd, &st) != 0 || static_cast<std::size_t>(st.st_size) < kHeaderBytes) {
    ::close(fd);
    set_error(error, ReadError::kCorrupt);
    return false;
  }
  void* base = ::mmap(nullptr, static_cast<std::size_t>(st.st_size), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  ::close(fd);
  if (base == MAP_FAILED) {
    set_error(error, ReadError::kNotVisible);
    return false;
  }
  const auto* hdr = static_cast<const SegmentHeader*>(base);
  const std::size_t expected = kHeaderBytes + hdr->stride * hdr->slots;
  if (hdr->magic != kMagic || hdr->version != kVersion || expected > static_cast<std::size_t>(st.st_size)) {
    ::munmap(base, static_cast<std::size_t>(st.st_size));
    set_error(error, ReadError::kCorrupt);
    return false;
  }
  if (hdr->epoch != h.epoch) {
    ::munmap(base, static_cast<std::size_t>(st.st_size));
    set_error(error, ReadError::kStale);
    return false;
  }
  base_ = base;
  size_ = static_cast<std::size_t>(st.st_size);
  name_ = h.segment;
  epoch_ = h.epoch;
  slots_ = hdr->slots;
  stride_ = hdr->stride;
  slot_bytes_ = hdr->slot_bytes;
  return true;
}

std::optional<View> Reader::acquire(const Handle& h, std::uint32_t lease_ms, ReadError* error) {
  set_error(error, ReadError::kNone);
  if (!map(h, error)) return std::nullopt;
  if (h.slot >= slots_ || (h.seq & 1U) != 0) {
    set_error(error, ReadError::kCorrupt);
    return std::nullopt;
  }
  SlotHeader* s = slot(h.slot);
  if (s->seq.load() != h.seq) {
    set_error(error, ReadError::kOverwritten);
    return std::nullopt;
  }
  const std::uint64_t until = monotonic_ns() + static_cast<std::uint64_t>(lease_ms) * 1'000'000ULL;
  std::uint64_t cur = s->lease_until.load();
  while (cur < until && !s->lease_until.compare_exchange_weak(cur, until)) {
  }
  if (s->seq.load() != h.seq) {  // the writer got in between our first look and the lease
    set_error(error, ReadError::kOverwritten);
    return std::nullopt;
  }
  View v;
  v.meta = s->meta;
  if (v.meta.data_bytes > slot_bytes_) {
    set_error(error, ReadError::kCorrupt);
    return std::nullopt;
  }
  v.data = reinterpret_cast<const std::uint8_t*>(s) + kSlotHeaderBytes;
  v.slot = h.slot;
  v.seq = h.seq;
  v.lease_until_ns = until;
  return v;
}

bool Reader::validate(const View& v) const {
  if (base_ == nullptr || v.slot >= slots_) return false;
  return slot(v.slot)->seq.load() == v.seq;
}

void Reader::release(const View& v) {
  if (base_ == nullptr || v.slot >= slots_) return;
  std::uint64_t expected = v.lease_until_ns;
  // Only clear our own lease; another reader may have extended it.
  slot(v.slot)->lease_until.compare_exchange_strong(expected, 0);
}

bool Reader::copy(const Handle& h, std::uint8_t* dst, std::size_t capacity, Meta* meta, ReadError* error) {
  auto v = acquire(h, 100, error);
  if (!v) return false;
  if (v->meta.data_bytes > capacity) {
    release(*v);
    set_error(error, ReadError::kCorrupt);
    return false;
  }
  std::memcpy(dst, v->data, v->meta.data_bytes);
  const bool ok = validate(*v);
  release(*v);
  if (!ok) {
    set_error(error, ReadError::kTorn);
    return false;
  }
  if (meta) *meta = v->meta;
  return true;
}

}  // namespace opr::shm
