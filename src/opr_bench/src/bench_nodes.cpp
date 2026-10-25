// Transport benchmark: a publisher of large frames and a subscriber that measures what each way of
// moving them costs. Both are ordinary nodes, so the same code runs in one process (intra-process)
// or two (raw middleware, shm with one copy, shm in place).
//
// The publisher stamps each frame with CLOCK_MONOTONIC just before handing it over. The
// subscriber touches every 64th byte (as any consumer must read its pixels) and records latency,
// frames received and its own CPU time; the publisher records its CPU time. Results go to JSON.
#include <sys/resource.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <opr_msgs/msg/shm_image.hpp>
#include <opr_shm/segment.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "opr_shm_transport/image_publisher.hpp"

namespace opr::bench {

namespace {

std::uint64_t mono_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}

double cpu_seconds() {
  rusage u{};
  getrusage(RUSAGE_SELF, &u);
  return u.ru_utime.tv_sec + u.ru_utime.tv_usec * 1e-6 + u.ru_stime.tv_sec + u.ru_stime.tv_usec * 1e-6;
}

double percentile(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const auto i = static_cast<std::size_t>(p * static_cast<double>(v.size() - 1) + 0.5);
  return v[std::min(i, v.size() - 1)];
}

rclcpp::QoS qos(bool reliable) {
  auto q = rclcpp::QoS(rclcpp::KeepLast(2));
  return reliable ? q.reliable() : q.best_effort();
}

}  // namespace

class BenchPublisher : public rclcpp::Node {
 public:
  explicit BenchPublisher(const rclcpp::NodeOptions& o) : Node("bench_pub", o) {
    width_ = static_cast<std::uint32_t>(declare_parameter("width", 1920));
    height_ = static_cast<std::uint32_t>(declare_parameter("height", 1080));
    bpp_ = static_cast<std::uint32_t>(declare_parameter("bytes_per_pixel", 2));
    frames_ = static_cast<int>(declare_parameter("frames", 300));
    rate_ = declare_parameter("rate_hz", 30.0);
    wait_for_ = static_cast<int>(declare_parameter("wait_for_subscribers", 1));
    out_ = declare_parameter("output", std::string(""));
    const bool reliable = declare_parameter("reliable", false);
    pixels_.assign(static_cast<std::size_t>(width_) * height_ * bpp_, 0);
    for (std::size_t i = 0; i < pixels_.size(); ++i) pixels_[i] = static_cast<std::uint8_t>(i * 7);
    auto raw = create_publisher<sensor_msgs::msg::Image>("bench/image", qos(reliable));
    auto shm = create_publisher<opr_msgs::msg::ShmImage>("bench/image/shm", qos(reliable));
    pub_ = std::make_unique<opr::shm_transport::ImagePublisher>(raw, shm, "/bench/image", 4);
    thread_ = std::thread([this] { run(); });
  }
  ~BenchPublisher() override {
    if (thread_.joinable()) thread_.join();
  }

 private:
  void run() {
    while (rclcpp::ok() && pub_->raw_subscribers() + pub_->shm_subscribers() < static_cast<std::size_t>(wait_for_)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let discovery settle
    const double cpu0 = cpu_seconds();
    const auto period = std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / rate_));
    auto next = std::chrono::steady_clock::now();
    for (int i = 0; i < frames_ && rclcpp::ok(); ++i) {
      std::this_thread::sleep_until(next);
      next += period;
      std::memcpy(pixels_.data(), &i, sizeof(i));
      std_msgs::msg::Header h;
      const std::uint64_t now = mono_ns();
      h.stamp.sec = static_cast<std::int32_t>(now / 1'000'000'000ULL);
      h.stamp.nanosec = static_cast<std::uint32_t>(now % 1'000'000'000ULL);
      h.frame_id = std::to_string(i);
      pub_->publish(h, width_, height_, width_ * bpp_, bpp_ == 2 ? "yuv422_yuy2" : "mono8", pixels_.data(), pixels_.size());
    }
    const double cpu = cpu_seconds() - cpu0;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!out_.empty()) {
      std::ofstream f(out_);
      f << "{\"publisher_cpu_s\": " << cpu << ", \"sent\": " << frames_ << "}\n";
    }
    rclcpp::shutdown();
  }

  std::uint32_t width_, height_, bpp_;
  int frames_, wait_for_;
  double rate_;
  std::string out_;
  std::vector<std::uint8_t> pixels_;
  std::unique_ptr<opr::shm_transport::ImagePublisher> pub_;
  std::thread thread_;
};

class BenchSubscriber : public rclcpp::Node {
 public:
  explicit BenchSubscriber(const rclcpp::NodeOptions& o) : Node("bench_sub", o) {
    mode_ = declare_parameter("mode", std::string("raw"));  // raw | shm | zero_copy
    out_ = declare_parameter("output", std::string(""));
    const bool reliable = declare_parameter("reliable", false);
    if (mode_ == "zero_copy") {
      shm_sub_ = create_subscription<opr_msgs::msg::ShmImage>(
          "bench/image/shm", qos(reliable), [this](const opr_msgs::msg::ShmImage::ConstSharedPtr& m) { on_handle(*m); });
    } else {
      it_sub_ = image_transport::create_subscription(
          this, "bench/image", [this](const sensor_msgs::msg::Image::ConstSharedPtr& m) { on_image(*m); }, mode_,
          qos(reliable).get_rmw_qos_profile());
    }
    cpu0_ = cpu_seconds();
    rclcpp::on_shutdown([this] { write(); });
  }

 private:
  std::uint64_t touch(const std::uint8_t* d, std::size_t n) {
    std::uint64_t s = 0;
    for (std::size_t i = 0; i < n; i += 64) s += d[i];
    return s;
  }

  void record(std::uint64_t stamp_ns, std::uint64_t checksum) {
    std::lock_guard<std::mutex> lock(mutex_);
    sink_ += checksum;
    latencies_ms_.push_back(static_cast<double>(mono_ns() - stamp_ns) / 1e6);
  }

  void on_image(const sensor_msgs::msg::Image& m) {
    const std::uint64_t stamp = static_cast<std::uint64_t>(m.header.stamp.sec) * 1'000'000'000ULL + m.header.stamp.nanosec;
    record(stamp, touch(m.data.data(), m.data.size()));
  }

  void on_handle(const opr_msgs::msg::ShmImage& m) {
    opr::shm::ReadError err;
    auto v = reader_.acquire(opr::shm_transport::to_handle(m), 50, &err);
    if (!v) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++errors_;
      return;
    }
    const auto sum = touch(v->data, v->meta.data_bytes);
    const bool ok = reader_.validate(*v);
    reader_.release(*v);
    if (!ok) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++errors_;
      return;
    }
    record(v->meta.stamp_ns, sum);
  }

  // Called from the shutdown handler, which runs on the signal thread while callbacks may still run.
  void write() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (out_.empty() || written_) return;
    written_ = true;
    std::ofstream f(out_);
    const double mean = latencies_ms_.empty()
                            ? 0.0
                            : std::accumulate(latencies_ms_.begin(), latencies_ms_.end(), 0.0) / latencies_ms_.size();
    f << "{\"mode\": \"" << mode_ << "\", \"received\": " << latencies_ms_.size() << ", \"errors\": " << errors_
      << ", \"latency_ms_mean\": " << mean << ", \"latency_ms_p50\": " << percentile(latencies_ms_, 0.5)
      << ", \"latency_ms_p99\": " << percentile(latencies_ms_, 0.99)
      << ", \"latency_ms_max\": " << percentile(latencies_ms_, 1.0)
      << ", \"subscriber_cpu_s\": " << (cpu_seconds() - cpu0_) << "}\n";
  }

  std::string mode_, out_;
  image_transport::Subscriber it_sub_;
  rclcpp::Subscription<opr_msgs::msg::ShmImage>::SharedPtr shm_sub_;
  opr::shm::Reader reader_;
  std::vector<double> latencies_ms_;
  std::uint64_t errors_ = 0, sink_ = 0;
  double cpu0_ = 0;
  bool written_ = false;
  std::mutex mutex_;
};

}  // namespace opr::bench

RCLCPP_COMPONENTS_REGISTER_NODE(opr::bench::BenchPublisher)
RCLCPP_COMPONENTS_REGISTER_NODE(opr::bench::BenchSubscriber)
