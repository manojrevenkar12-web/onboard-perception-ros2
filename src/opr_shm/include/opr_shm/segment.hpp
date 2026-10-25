// A pool of image slots in POSIX shared memory, shared between processes and containers on one
// host without copying pixels through the middleware.
//
// Protocol (all atomics sequentially consistent, which is what makes the lease check sound):
//   writer: pick the next slot whose lease has expired; seq <- seq + 1 (odd: being written);
//           re-check the lease and back off (seq <- seq - 1) if a reader took it meanwhile;
//           write pixels and metadata; seq <- seq + 1 (even: stable). Publish {slot, seq}.
//   reader: check slot.seq == handle.seq; lease_until <- now + lease; check seq again.
//           Read in place for as long as the lease lasts, then validate() (seq unchanged) before
//           trusting what was computed. A reader that crashes holds its slot only until the lease
//           expires, so a dead consumer cannot wedge the publisher.
//
// Leases use CLOCK_MONOTONIC, which is one clock for every process and container on a kernel.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace opr::shm {

inline constexpr std::uint64_t kMagic = 0x3153474D49525053ULL;  // "SPRIMGS1" little-endian
inline constexpr std::uint32_t kVersion = 1;

struct Meta {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t step = 0;
  std::uint32_t data_bytes = 0;
  std::uint64_t stamp_ns = 0;
  char encoding[32] = {};
  char frame_id[64] = {};
};

struct Handle {
  std::string segment;
  std::uint64_t epoch = 0;
  std::string boot_id;
  std::uint32_t slot = 0;
  std::uint64_t seq = 0;
};

/// Why a read failed. Each maps to a counter in the transport's diagnostics.
enum class ReadError {
  kNone,
  kNotVisible,   // segment cannot be opened here: other host, or /dev/shm not shared with us
  kStale,        // handle is from a previous incarnation of the publisher (epoch differs)
  kOverwritten,  // the publisher reused the slot before we got to it (we are too slow)
  kTorn,         // the slot changed while we were reading it (lease expired mid-read)
  kCorrupt,      // header does not look like ours
};
const char* to_string(ReadError e);

std::uint64_t monotonic_ns();
/// The kernel's boot id, or "" if it cannot be read.
std::string boot_id();
/// A segment name derived from a topic, unique per process: "/opr" + topic with '/' -> '_' + pid.
std::string segment_name_for(const std::string& topic);

struct SegmentHeader;
struct SlotHeader;

struct WriterStats {
  std::uint64_t committed = 0;
  std::uint64_t no_free_slot = 0;  // every slot leased: the frame was not published
  std::uint64_t lease_skips = 0;   // slots passed over because a reader held them
  std::uint64_t too_large = 0;
};

class Writer {
 public:
  /// Creates (or replaces) the segment. Throws std::system_error if /dev/shm is not usable.
  Writer(const std::string& name, std::uint32_t slots, std::size_t slot_bytes);
  ~Writer();
  Writer(const Writer&) = delete;
  Writer& operator=(const Writer&) = delete;

  struct Loan {
    std::uint8_t* data = nullptr;
    std::size_t capacity = 0;
    std::uint32_t slot = 0;
    explicit operator bool() const { return data != nullptr; }
  };

  /// Reserves a slot to write into, or an empty Loan if none is free or `bytes` is too large.
  Loan loan(std::size_t bytes);
  /// Publishes a loaned slot and returns the handle readers need.
  Handle commit(const Loan& loan, const Meta& meta);
  /// Gives a loan back without publishing it.
  void cancel(const Loan& loan);

  const std::string& name() const { return name_; }
  std::uint64_t epoch() const { return epoch_; }
  std::uint32_t slots() const { return slots_; }
  std::size_t slot_bytes() const { return slot_bytes_; }
  const WriterStats& stats() const { return stats_; }

 private:
  SlotHeader* slot(std::uint32_t i) const;
  std::uint8_t* payload(std::uint32_t i) const;

  std::string name_;
  std::uint32_t slots_;
  std::size_t slot_bytes_;
  std::size_t stride_ = 0;
  std::size_t size_ = 0;
  void* base_ = nullptr;
  std::uint64_t epoch_ = 0;
  std::string boot_id_;
  std::uint32_t next_ = 0;
  bool loaned_ = false;
  WriterStats stats_;
};

/// A slot a reader holds under lease. Pointers are valid until the lease expires.
struct View {
  const std::uint8_t* data = nullptr;
  Meta meta;
  std::uint32_t slot = 0;
  std::uint64_t seq = 0;
  std::uint64_t lease_until_ns = 0;
};

class Reader {
 public:
  Reader() = default;
  ~Reader();
  Reader(const Reader&) = delete;
  Reader& operator=(const Reader&) = delete;

  /// Maps the handle's segment (cached; remapped when the publisher restarts) and leases the slot.
  std::optional<View> acquire(const Handle& h, std::uint32_t lease_ms, ReadError* error = nullptr);
  /// True if the slot still holds what acquire() saw: anything computed from it can be trusted.
  bool validate(const View& v) const;
  /// Ends the lease early so the publisher may reuse the slot.
  void release(const View& v);
  /// Copies the image out (for consumers that need to own it). Validated after the copy.
  bool copy(const Handle& h, std::uint8_t* dst, std::size_t capacity, Meta* meta, ReadError* error = nullptr);

  bool mapped() const { return base_ != nullptr; }

 private:
  bool map(const Handle& h, ReadError* error);
  void unmap();
  SlotHeader* slot(std::uint32_t i) const;

  std::string name_;
  std::uint64_t epoch_ = 0;
  void* base_ = nullptr;
  std::size_t size_ = 0;
  std::uint32_t slots_ = 0;
  std::size_t stride_ = 0;
  std::size_t slot_bytes_ = 0;
};

}  // namespace opr::shm
