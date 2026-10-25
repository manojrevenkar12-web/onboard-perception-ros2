// Camera node: owns the V4L2 device and publishes every frame once, to whichever audiences exist
// (in-process subscribers, shared-memory readers, raw subscribers).
//
// Lifecycle: configure opens the device (a missing or misconfigured camera fails configure, so a
// deployment notices before arming); activate starts capture; deactivate stops it; cleanup closes
// the device. While active, an unplugged or reset camera is reopened with backoff and the node stays
// active: diagnostics say ERROR until frames flow again. The frame sequence published in FrameInfo
// counts across reconnects, so downstream sees the gap as lost frames rather than a restart.
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <opr_msgs/msg/frame_info.hpp>
#include <opr_v4l2/camera.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "opr_pipeline/common.hpp"
#include "opr_shm_transport/image_publisher.hpp"

namespace opr::pipeline {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CameraNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit CameraNode(const rclcpp::NodeOptions& options) : LifecycleNode("camera", options) {
    device_ = declare_parameter("device", std::string("sim://crossing?pace_hz=30&loop=1"));
    width_ = static_cast<int>(declare_parameter("width", 640));
    height_ = static_cast<int>(declare_parameter("height", 480));
    fps_ = static_cast<int>(declare_parameter("fps", 30));
    buffers_ = static_cast<int>(declare_parameter("buffers", 4));
    frame_id_ = declare_parameter("frame_id", std::string("camera_optical"));
    qos_mode_ = declare_parameter("image_qos", std::string("sensor"));
    shm_slots_ = static_cast<int>(declare_parameter("shm_slots", 4));
    grab_timeout_ms_ = static_cast<int>(declare_parameter("grab_timeout_ms", 200));
    backoff_min_ms_ = static_cast<int>(declare_parameter("reconnect_backoff_min_ms", 100));
    backoff_max_ms_ = static_cast<int>(declare_parameter("reconnect_backoff_max_ms", 2000));
    if (options.use_intra_process_comms()) RCLCPP_INFO(get_logger(), "intra-process communication enabled");
    // Standalone and composed deployments without a lifecycle manager configure and activate
    // themselves; a manager (or the tests) leaves autostart off and drives the transitions.
    if (declare_parameter("autostart", false)) {
      autostart_timer_ = create_wall_timer(std::chrono::milliseconds(0), [this] {
        autostart_timer_->cancel();
        if (configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_FATAL(get_logger(), "autostart: configure failed");
          return;
        }
        activate();
      });
    }
  }

  ~CameraNode() override { stop(); }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    opr::v4l2::Config cfg;
    cfg.device = device_;
    cfg.width = width_;
    cfg.height = height_;
    cfg.fps = fps_;
    cfg.buffers = buffers_;
    try {
      camera_ = std::make_unique<opr::v4l2::Camera>(cfg, opr::v4l2::syscalls_for(device_));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "bad device '%s': %s", device_.c_str(), e.what());
      return CallbackReturn::FAILURE;
    }
    const auto status = camera_->open();
    if (status != opr::v4l2::Status::kOk) {
      RCLCPP_ERROR(get_logger(), "cannot open %s: %s", device_.c_str(), camera_->error().c_str());
      camera_.reset();
      return CallbackReturn::FAILURE;
    }
    // Validated and mapped, but not capturing until activate: an inactive camera must not fill its
    // buffers with frames that will be stale by the time anyone reads them.
    camera_->stop_streaming();
    RCLCPP_INFO(get_logger(), "opened %s %dx%d @ %d fps, %d buffers", device_.c_str(), width_, height_, fps_, buffers_);
    take_snapshot();
    raw_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", image_qos(qos_mode_));
    shm_pub_ = create_publisher<opr_msgs::msg::ShmImage>("image_raw/shm", image_qos(qos_mode_));
    info_pub_ = create_publisher<opr_msgs::msg::FrameInfo>("frame_info", result_qos());
    images_ = std::make_unique<opr::shm_transport::ImagePublisher>(
        raw_pub_, shm_pub_, std::string(get_fully_qualified_name()) + "/image_raw", static_cast<std::uint32_t>(shm_slots_));

    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this, 1.0);
    diagnostics_->setHardwareID(device_);
    freq_min_ = fps_ * 0.9;
    freq_max_ = fps_ * 1.1;
    freq_ = std::make_unique<diagnostic_updater::FrequencyStatus>(
        diagnostic_updater::FrequencyStatusParam(&freq_min_, &freq_max_, 0.1, 5), "frame rate");
    diagnostics_->add(*freq_);
    diagnostics_->add("driver", this, &CameraNode::diagnose);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override {
    LifecycleNode::on_activate(state);
    if (camera_->is_open() && camera_->start_streaming() != opr::v4l2::Status::kOk) {
      RCLCPP_WARN(get_logger(), "camera lost while inactive (%s); reconnecting", camera_->error().c_str());
    }
    resumed_ = true;
    running_ = true;
    thread_ = std::thread([this] { run(); });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override {
    stop();
    camera_->stop_streaming();
    take_snapshot();
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    stop();
    diagnostics_.reset();
    freq_.reset();
    images_.reset();
    raw_pub_.reset();
    shm_pub_.reset();
    info_pub_.reset();
    camera_.reset();
    take_snapshot();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    stop();
    diagnostics_.reset();
    camera_.reset();
    return CallbackReturn::SUCCESS;
  }

 private:
  void stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
  }

  struct Snapshot {
    bool configured = false;
    bool open = false;
    std::string error;
    opr::v4l2::Stats stats;
  };

  void take_snapshot() {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.configured = camera_ != nullptr;
    snapshot_.open = camera_ && camera_->is_open();
    snapshot_.error = camera_ ? camera_->error() : std::string();
    snapshot_.stats = camera_ ? camera_->stats() : opr::v4l2::Stats{};
  }

  // Runs on the executor. Reads a snapshot the capture thread keeps, so diagnostics never wait for
  // a grab (which would stall every node sharing this executor).
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    Snapshot snap;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      snap = snapshot_;
    }
    if (!snap.configured) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "not configured");
      return;
    }
    const auto& s = snap.stats;
    if (!snap.open) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "camera disconnected: " + snap.error);
    } else if (s.dropped > last_reported_drops_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "sensor frames dropped since last report");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "streaming");
    }
    last_reported_drops_ = s.dropped;
    const auto pub = images_ ? images_->stats() : opr::shm_transport::PublisherStats{};
    stat.add("frames", s.frames);
    stat.add("dropped by driver", s.dropped);
    stat.add("corrupted buffers", s.corrupted);
    stat.add("timeouts", s.timeouts);
    stat.add("disconnects", s.disconnects);
    stat.add("opens", s.opens);
    stat.add("published raw", pub.raw);
    stat.add("published shm", pub.shm);
    stat.add("shm frames with no free slot", pub.shm_dropped);
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    stat.add("timestamp source", stamp_source_name_);
  }

  // Capture thread: the only code that touches camera_ while the node is active.
  void run() {
    int backoff_ms = backoff_min_ms_;
    while (running_) {
      if (!camera_->is_open()) {
        const auto s = camera_->open();
        take_snapshot();
        if (s != opr::v4l2::Status::kOk) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "camera unavailable (%s), retrying in %d ms",
                               camera_->error().c_str(), backoff_ms);
          sleep_while_running(backoff_ms);
          backoff_ms = std::min(backoff_ms * 2, backoff_max_ms_);
          continue;
        }
        RCLCPP_INFO(get_logger(), "camera reconnected");
        backoff_ms = backoff_min_ms_;
        new_stream_ = true;
      }
      opr::v4l2::Buffer buf;
      const auto s = camera_->grab(grab_timeout_ms_, buf);
      if (s == opr::v4l2::Status::kOk) {
        publish(buf);
      } else if (s != opr::v4l2::Status::kTimeout) {
        RCLCPP_ERROR(get_logger(), "capture stopped: %s (%s)", opr::v4l2::to_string(s), camera_->error().c_str());
        new_stream_ = true;
      }
      take_snapshot();
    }
  }

  void sleep_while_running(int ms) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (running_ && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  void publish(opr::v4l2::Buffer& buf) {
    std::uint32_t dropped_before = 0;
    if (!have_frame_) {
      sequence_ = buf.sequence();
    } else if (resumed_) {
      sequence_ += 1;  // a deliberate pause (deactivate/activate) is not frame loss
    } else if (new_stream_) {
      // The driver's counter restarted; estimate what we missed from the exposure times.
      const double interval = 1e9 / fps_;
      const double gap = buf.stamp_ns() > last_stamp_ ? (buf.stamp_ns() - last_stamp_) / interval : 1.0;
      const auto steps = static_cast<std::uint32_t>(std::max(1.0, std::round(gap)));
      dropped_before = steps - 1;
      sequence_ += steps;
    } else {
      const std::uint32_t steps = buf.sequence() - last_driver_sequence_;
      dropped_before = steps > 0 ? steps - 1 : 0;
      sequence_ += steps;
    }
    have_frame_ = true;
    new_stream_ = false;
    resumed_ = false;
    last_driver_sequence_ = buf.sequence();
    last_stamp_ = buf.stamp_ns();

    std_msgs::msg::Header header;
    header.stamp = to_time(buf.stamp_ns());
    header.frame_id = frame_id_;

    opr_msgs::msg::FrameInfo info;
    info.header = header;
    info.sequence = sequence_;
    info.driver_sequence = buf.sequence();
    info.dropped_before = dropped_before;
    info.queue_depth = static_cast<std::uint32_t>(camera_->queued());
    {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    switch (buf.stamp_source()) {
      case opr::v4l2::StampSource::kStartOfExposure:
        info.stamp_source = opr_msgs::msg::FrameInfo::STAMP_START_OF_EXPOSURE;
        stamp_source_name_ = "start of exposure";
        break;
      case opr::v4l2::StampSource::kEndOfFrame:
        info.stamp_source = opr_msgs::msg::FrameInfo::STAMP_END_OF_FRAME;
        stamp_source_name_ = "end of frame, corrected";
        break;
      case opr::v4l2::StampSource::kDequeue:
        info.stamp_source = opr_msgs::msg::FrameInfo::STAMP_DEQUEUE;
        stamp_source_name_ = "dequeue time (driver gave none)";
        break;
    }
    }
    // FrameInfo first: anything that joins on the stamp later has it before the results arrive.
    info_pub_->publish(info);
    images_->publish(header, static_cast<std::uint32_t>(width_), static_cast<std::uint32_t>(height_),
                     static_cast<std::uint32_t>(buf.step()), "yuv422_yuy2", buf.data(), buf.bytes());
    buf.release();
    freq_->tick();
  }

  std::string device_;
  int width_, height_, fps_, buffers_;
  std::string frame_id_, qos_mode_;
  int shm_slots_, grab_timeout_ms_, backoff_min_ms_, backoff_max_ms_;

  std::unique_ptr<opr::v4l2::Camera> camera_;
  std::mutex snapshot_mutex_;
  Snapshot snapshot_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp_lifecycle::LifecyclePublisher<opr_msgs::msg::ShmImage>::SharedPtr shm_pub_;
  rclcpp_lifecycle::LifecyclePublisher<opr_msgs::msg::FrameInfo>::SharedPtr info_pub_;
  std::unique_ptr<opr::shm_transport::ImagePublisher> images_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::unique_ptr<diagnostic_updater::FrequencyStatus> freq_;
  double freq_min_ = 0, freq_max_ = 0;

  rclcpp::TimerBase::SharedPtr autostart_timer_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  bool have_frame_ = false;
  bool new_stream_ = true;
  bool resumed_ = false;
  std::uint32_t sequence_ = 0;
  std::uint32_t last_driver_sequence_ = 0;
  std::uint64_t last_stamp_ = 0;
  std::uint64_t last_reported_drops_ = 0;
  std::string stamp_source_name_ = "none yet";
};

}  // namespace opr::pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(opr::pipeline::CameraNode)
