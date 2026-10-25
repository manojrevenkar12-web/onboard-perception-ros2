#include <gtest/gtest.h>

#include <time.h>

#include <chrono>
#include <cstring>
#include <thread>

#include "opr_v4l2/camera.hpp"
#include "opr_v4l2/fake_device.hpp"

using opr::v4l2::Buffer;
using opr::v4l2::Camera;
using opr::v4l2::Config;
using opr::v4l2::FakeDevice;
using opr::v4l2::Status;
using opr::v4l2::StampSource;

namespace {
std::shared_ptr<FakeDevice> fake(const std::string& url) {
  return std::make_shared<FakeDevice>(opr::v4l2::parse_sim_url(url));
}
Config cfg(const std::string& url, int buffers = 4) {
  Config c;
  c.device = url;
  c.buffers = buffers;
  return c;
}
}  // namespace

TEST(Camera, StreamsTheSimulatorsFramesWithTheirExposureStamps) {
  auto dev = fake("sim://crossing?seed=4");
  Camera cam(cfg("sim://crossing?seed=4"), dev);
  ASSERT_EQ(cam.open(), Status::kOk) << cam.error();
  EXPECT_EQ(dev->mapped(), 4);
  opr::sim::Scene scene("crossing", 4);
  opr::sim::Frame want;
  for (std::uint32_t i = 0; i < 10; ++i) {
    Buffer b;
    ASSERT_EQ(cam.grab(100, b), Status::kOk) << cam.error();
    scene.render(i, want);
    EXPECT_EQ(b.sequence(), i);
    EXPECT_EQ(b.stamp_ns(), want.stamp_ns);
    EXPECT_EQ(b.stamp_source(), StampSource::kStartOfExposure);
    ASSERT_EQ(b.bytes(), want.yuyv.size());
    EXPECT_EQ(b.step(), 1280u);
    EXPECT_EQ(0, std::memcmp(b.data(), want.yuyv.data(), want.yuyv.size()));
  }
  EXPECT_EQ(cam.stats().frames, 10u);
  EXPECT_EQ(cam.stats().dropped, 0u);
  cam.close();
  EXPECT_EQ(dev->mapped(), 0);
  EXPECT_FALSE(dev->streaming());
}

TEST(Camera, BuffersGoBackToTheDriverWhenReleased) {
  auto dev = fake("sim://fast");
  Camera cam(cfg("sim://fast", 3), dev);
  ASSERT_EQ(cam.open(), Status::kOk);
  EXPECT_EQ(cam.queued(), 3);
  Buffer a, b;
  ASSERT_EQ(cam.grab(100, a), Status::kOk);
  ASSERT_EQ(cam.grab(100, b), Status::kOk);
  EXPECT_EQ(cam.queued(), 1);
  a.release();
  EXPECT_EQ(cam.queued(), 2);
  Buffer moved = std::move(b);
  EXPECT_FALSE(b);
  EXPECT_EQ(cam.queued(), 2);
  moved = Buffer();
  EXPECT_EQ(cam.queued(), 3);
}

TEST(Camera, ASlowConsumerShowsUpAsDrops) {
  // A sensor at 200 Hz and a consumer that holds every frame for 20 ms: most exposures find no
  // free buffer. The driver must count them from the sequence gaps, not hide them.
  auto dev = fake("sim://swarm?pace_hz=200&loop=1");
  Camera cam(cfg("sim://swarm?pace_hz=200&loop=1", 2), dev);
  ASSERT_EQ(cam.open(), Status::kOk);
  for (int i = 0; i < 15; ++i) {
    Buffer b;
    ASSERT_EQ(cam.grab(200, b), Status::kOk) << cam.error();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_GT(cam.stats().dropped, 20u);
}

TEST(Camera, RefusesASilentlyDifferentResolution) {
  auto dev = fake("sim://crossing?wrong_size=1");
  Camera cam(cfg("sim://crossing?wrong_size=1"), dev);
  EXPECT_EQ(cam.open(), Status::kError);
  EXPECT_NE(cam.error().find("device offers 320x240"), std::string::npos) << cam.error();
  EXPECT_FALSE(cam.is_open());
}

TEST(Camera, SkipsAndCountsCorruptedBuffers) {
  auto dev = fake("sim://crossing?corrupt_every=3");
  Camera cam(cfg("sim://crossing?corrupt_every=3"), dev);
  ASSERT_EQ(cam.open(), Status::kOk);
  std::vector<std::uint32_t> seen;
  for (int i = 0; i < 6; ++i) {
    Buffer b;
    ASSERT_EQ(cam.grab(100, b), Status::kOk);
    seen.push_back(b.sequence());
  }
  EXPECT_EQ(seen, (std::vector<std::uint32_t>{0, 1, 3, 4, 6, 7}));
  EXPECT_EQ(cam.stats().corrupted, 2u);
}

TEST(Camera, CorrectsEndOfFrameStampsToExposureStart) {
  auto soe = fake("sim://fast?seed=2");
  auto eof = fake("sim://fast?seed=2&eof_stamps=1");
  Camera a(cfg("x"), soe), b(cfg("x"), eof);
  ASSERT_EQ(a.open(), Status::kOk);
  ASSERT_EQ(b.open(), Status::kOk);
  Buffer fa, fb;
  ASSERT_EQ(a.grab(100, fa), Status::kOk);
  ASSERT_EQ(b.grab(100, fb), Status::kOk);
  EXPECT_EQ(fb.stamp_source(), StampSource::kEndOfFrame);
  // The correction is one nominal frame interval; V4L2 timestamps have microsecond resolution.
  EXPECT_NEAR(static_cast<double>(fa.stamp_ns()), static_cast<double>(fb.stamp_ns()), 1000.0);
}

TEST(Camera, FallsBackToDequeueTimeWithoutDriverTimestamps) {
  auto dev = fake("sim://fast?no_timestamps=1");
  Camera cam(cfg("x"), dev);
  ASSERT_EQ(cam.open(), Status::kOk);
  Buffer b;
  ASSERT_EQ(cam.grab(100, b), Status::kOk);
  EXPECT_EQ(b.stamp_source(), StampSource::kDequeue);
  EXPECT_GT(b.stamp_ns(), 0u);
}

TEST(Camera, SurvivesAnUnplugAndReconnects) {
  auto dev = fake("sim://crossing?unplug_at=5&unplug_ms=150");
  Camera cam(cfg("x"), dev);
  ASSERT_EQ(cam.open(), Status::kOk);
  Buffer held;
  for (int i = 0; i < 5; ++i) {
    Buffer b;
    ASSERT_EQ(cam.grab(100, b), Status::kOk);
    if (i == 4) held = std::move(b);
  }
  Buffer b;
  EXPECT_EQ(cam.grab(100, b), Status::kDisconnected);
  EXPECT_FALSE(cam.is_open());
  EXPECT_EQ(cam.stats().disconnects, 1u);
  held.release();  // must not requeue into the dead stream
  EXPECT_EQ(cam.open(), Status::kDisconnected) << "device node is gone while unplugged";
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  ASSERT_EQ(cam.open(), Status::kOk) << cam.error();
  ASSERT_EQ(cam.grab(100, b), Status::kOk);
  EXPECT_EQ(b.sequence(), 0u) << "a new stream starts counting again";
  EXPECT_GT(dev->world_frame(), 5) << "the world kept moving while unplugged";
  EXPECT_EQ(cam.stats().opens, 2u);
}

TEST(Camera, TimesOutWhenTheFlightEnds) {
  auto dev = fake("sim://fast");
  Camera cam(cfg("x"), dev);
  ASSERT_EQ(cam.open(), Status::kOk);
  int frames = 0;
  for (;;) {
    Buffer b;
    const auto s = cam.grab(20, b);
    if (s == Status::kTimeout) break;
    ASSERT_EQ(s, Status::kOk);
    ++frames;
  }
  EXPECT_EQ(frames, dev->scene().frame_count());
  EXPECT_EQ(cam.stats().timeouts, 1u);
}

TEST(Camera, GrabOnAClosedCameraIsAnError) {
  Camera cam(cfg("x"), fake("sim://fast"));
  Buffer b;
  EXPECT_EQ(cam.grab(10, b), Status::kError);
}

TEST(SimUrl, ParsesOptionsAndRejectsTypos) {
  const auto o = opr::v4l2::parse_sim_url("sim://clutter?seed=9&pace_hz=15&stamps=clock&loop=1");
  EXPECT_EQ(o.scenario, "clutter");
  EXPECT_EQ(o.seed, 9u);
  EXPECT_DOUBLE_EQ(o.pace_hz, 15.0);
  EXPECT_TRUE(o.clock_stamps);
  EXPECT_TRUE(o.loop);
  EXPECT_THROW(opr::v4l2::parse_sim_url("sim://clutter?sed=9"), std::invalid_argument);
  EXPECT_THROW(opr::v4l2::parse_sim_url("sim://clutter?stamps=gps"), std::invalid_argument);
  EXPECT_THROW(opr::v4l2::parse_sim_url("/dev/video0"), std::invalid_argument);
  EXPECT_THROW(opr::v4l2::FakeDevice(opr::v4l2::parse_sim_url("sim://hover")), std::invalid_argument);
}

TEST(Camera, APausedStreamDeliversNoStaleFrames) {
  // Paced sensor: while the stream is stopped nothing is captured, so the first frame after
  // start_streaming() is fresh and no drops are counted for the pause.
  auto dev = fake("sim://fast?pace_hz=100&loop=1&stamps=clock");
  Camera cam(cfg("x"), dev);
  ASSERT_EQ(cam.open(), Status::kOk);
  Buffer held;
  ASSERT_EQ(cam.grab(100, held), Status::kOk);
  cam.stop_streaming();
  EXPECT_FALSE(cam.streaming());
  Buffer b;
  EXPECT_EQ(cam.grab(10, b), Status::kError) << "grab while paused";
  held.release();  // must not requeue into the stopped stream
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const std::uint64_t resumed = static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
  ASSERT_EQ(cam.start_streaming(), Status::kOk);
  ASSERT_EQ(cam.grab(100, b), Status::kOk);
  EXPECT_GE(b.stamp_ns(), resumed) << "first frame after resume was exposed before it";
  EXPECT_EQ(b.sequence(), 0u);
  EXPECT_EQ(cam.stats().dropped, 0u);
}
