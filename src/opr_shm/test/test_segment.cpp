#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "opr_shm/segment.hpp"

using namespace opr::shm;

namespace {

std::string unique(const char* tag) { return "/opr_test_" + std::string(tag) + "_" + std::to_string(::getpid()); }

Handle put(Writer& w, std::uint8_t fill, std::size_t bytes = 1000) {
  auto loan = w.loan(bytes);
  if (!loan) return {};
  std::memset(loan.data, fill, bytes);
  Meta m;
  m.width = static_cast<std::uint32_t>(bytes);
  m.height = 1;
  m.step = static_cast<std::uint32_t>(bytes);
  m.data_bytes = static_cast<std::uint32_t>(bytes);
  m.stamp_ns = fill;
  std::strncpy(m.encoding, "mono8", sizeof(m.encoding) - 1);
  return w.commit(loan, m);
}

}  // namespace

TEST(Segment, RoundTripsAnImage) {
  Writer w(unique("rt"), 3, 4096);
  const Handle h = put(w, 0x5a);
  Reader r;
  ReadError e;
  auto v = r.acquire(h, 50, &e);
  ASSERT_TRUE(v) << to_string(e);
  EXPECT_EQ(v->meta.data_bytes, 1000u);
  EXPECT_STREQ(v->meta.encoding, "mono8");
  EXPECT_EQ(v->data[0], 0x5a);
  EXPECT_EQ(v->data[999], 0x5a);
  EXPECT_TRUE(r.validate(*v));
  r.release(*v);
}

TEST(Segment, ALeasedSlotIsNeverOverwritten) {
  Writer w(unique("lease"), 3, 4096);
  Reader r;
  const Handle first = put(w, 1);
  auto held = r.acquire(first, 2000);
  ASSERT_TRUE(held);
  for (int i = 0; i < 10; ++i) ASSERT_TRUE(put(w, static_cast<std::uint8_t>(10 + i)).segment.size());
  EXPECT_TRUE(r.validate(*held)) << "writer reused a leased slot";
  EXPECT_EQ(held->data[0], 1);
  EXPECT_GT(w.stats().lease_skips, 0u);
  r.release(*held);
}

TEST(Segment, WhenEverySlotIsLeasedThePublisherDropsTheFrame) {
  Writer w(unique("full"), 2, 4096);
  Reader r;
  auto a = r.acquire(put(w, 1), 2000);
  auto b = r.acquire(put(w, 2), 2000);
  ASSERT_TRUE(a && b);
  EXPECT_FALSE(w.loan(100));
  EXPECT_EQ(w.stats().no_free_slot, 1u);
  r.release(*a);
  EXPECT_TRUE(put(w, 3).segment.size());
}

TEST(Segment, ADeadReadersLeaseExpires) {
  Writer w(unique("dead"), 2, 4096);
  Reader r;
  ASSERT_TRUE(r.acquire(put(w, 1), 30));
  ASSERT_TRUE(r.acquire(put(w, 2), 30));  // never released: the reader "crashed"
  EXPECT_FALSE(w.loan(100));
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_TRUE(put(w, 3).segment.size());
}

TEST(Segment, ALateReaderIsToldItWasOverwritten) {
  Writer w(unique("late"), 2, 4096);
  const Handle old = put(w, 1);
  put(w, 2);
  put(w, 3);  // reuses old's slot
  Reader r;
  ReadError e;
  EXPECT_FALSE(r.acquire(old, 50, &e));
  EXPECT_EQ(e, ReadError::kOverwritten);
}

TEST(Segment, AHandleFromARestartedPublisherIsStale) {
  const std::string name = unique("restart");
  Handle old;
  {
    Writer w(name, 2, 4096);
    old = put(w, 1);
  }
  Writer w2(name, 2, 4096);
  const Handle fresh = put(w2, 2);
  Reader r;
  ReadError e;
  EXPECT_FALSE(r.acquire(old, 50, &e));
  EXPECT_EQ(e, ReadError::kStale);
  auto v = r.acquire(fresh, 50, &e);
  ASSERT_TRUE(v) << to_string(e);
  EXPECT_EQ(v->data[0], 2);
}

TEST(Segment, ASegmentThatIsNotHereIsNotVisible) {
  Reader r;
  ReadError e;
  Handle h;
  h.segment = "/opr_test_does_not_exist";
  h.epoch = 1;
  EXPECT_FALSE(r.acquire(h, 10, &e));
  EXPECT_EQ(e, ReadError::kNotVisible);

  Writer w(unique("boot"), 2, 4096);
  Handle other = put(w, 1);
  other.boot_id = "00000000-0000-0000-0000-000000000000";
  EXPECT_FALSE(r.acquire(other, 10, &e));
  EXPECT_EQ(e, ReadError::kNotVisible);
}

TEST(Segment, CancelInvalidatesWhateverWasWritten) {
  Writer w(unique("cancel"), 2, 4096);
  const Handle h = put(w, 7);
  auto loan = w.loan(10);  // next slot
  w.cancel(loan);
  auto loan2 = w.loan(10);  // wraps to h's slot
  std::memset(loan2.data, 9, 10);
  w.cancel(loan2);
  Reader r;
  ReadError e;
  EXPECT_FALSE(r.acquire(h, 10, &e));
  EXPECT_EQ(e, ReadError::kOverwritten);
}

TEST(Segment, RefusesOversizedFramesAndBadNames) {
  Writer w(unique("big"), 2, 100);
  EXPECT_FALSE(w.loan(101));
  EXPECT_EQ(w.stats().too_large, 1u);
  EXPECT_THROW(Writer("no_slash", 2, 100), std::invalid_argument);
  EXPECT_THROW(Writer(unique("one"), 1, 100), std::invalid_argument);
}

// Two processes, a fast writer and a reader with short leases. Every frame the reader accepts
// (validated after reading) must be internally consistent: all bytes equal its stamp's low byte.
// Torn or overwritten frames are allowed, and counted, but never accepted.
TEST(Segment, CrossProcessReadsAreNeverTorn) {
  const std::string name = unique("stress");
  int pipefd[2];
  ASSERT_EQ(::pipe(pipefd), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(pipefd[0]);
    Writer w(name, 3, 64 * 1024);
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
    std::uint64_t n = 0;
    while (std::chrono::steady_clock::now() < end) {
      auto loan = w.loan(64 * 1024);
      if (!loan) continue;
      const auto fill = static_cast<std::uint8_t>(n);
      for (std::size_t i = 0; i < 64 * 1024; i += 4096) std::memset(loan.data + i, fill, 4096);
      Meta m;
      m.data_bytes = 64 * 1024;
      m.stamp_ns = n++;
      const Handle h = w.commit(loan, m);
      struct {
        std::uint64_t epoch, seq;
        std::uint32_t slot;
      } msg{h.epoch, h.seq, h.slot};
      if (::write(pipefd[1], &msg, sizeof(msg)) != sizeof(msg)) break;
    }
    ::close(pipefd[1]);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ::_exit(0);
  }
  ::close(pipefd[1]);
  Reader r;
  std::uint64_t accepted = 0, refused = 0, caught = 0, bad = 0, unchecked_bad = 0;
  struct {
    std::uint64_t epoch, seq;
    std::uint32_t slot;
  } msg{};
  while (::read(pipefd[0], &msg, sizeof(msg)) == static_cast<ssize_t>(sizeof(msg))) {
    Handle h{name, msg.epoch, "", msg.slot, msg.seq};
    auto v = r.acquire(h, 1);
    if (!v) {
      ++refused;
      continue;
    }
    // Every 7th frame, dawdle past the lease so the writer may reuse the slot mid-read:
    // validate() must then refuse the frame.
    if ((accepted + caught) % 7 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(3));
    const auto want = static_cast<std::uint8_t>(v->meta.stamp_ns);
    bool consistent = true;
    for (std::size_t i = 0; i < v->meta.data_bytes; i += 509) consistent &= v->data[i] == want;
    if (!consistent) ++unchecked_bad;  // what a reader without validate() would have used
    if (r.validate(*v)) {
      ++accepted;
      if (!consistent) ++bad;
    } else {
      ++caught;  // the slot changed under us after the lease ran out
    }
    r.release(*v);
  }
  int status = 0;
  ::waitpid(child, &status, 0);
  EXPECT_GT(accepted, 100u);
  EXPECT_GT(caught, 0u) << "the test never provoked a reuse mid-read; it proves nothing";
  EXPECT_EQ(bad, 0u) << accepted << " accepted, " << refused << " refused";
  std::printf("accepted %llu, overwritten before acquire %llu, changed mid-read and caught %llu, "
              "inconsistent but accepted %llu (without validate: %llu)\n",
              static_cast<unsigned long long>(accepted), static_cast<unsigned long long>(refused),
              static_cast<unsigned long long>(caught), static_cast<unsigned long long>(bad),
              static_cast<unsigned long long>(unchecked_bad));
}
