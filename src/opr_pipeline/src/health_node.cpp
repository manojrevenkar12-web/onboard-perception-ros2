// Health node: one answer to "can the autonomy stack trust perception right now?".
// Reads every node's diagnostics and the track stream, and publishes opr_msgs/Health at 5 Hz:
// the worst level, which component caused it, and how old the newest tracks are. No tracks for
// `stale_ms` is ERROR whatever the diagnostics say: a silent pipeline must never look healthy.
#include <chrono>
#include <map>
#include <memory>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <opr_msgs/msg/health.hpp>
#include <opr_msgs/msg/track_array.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "opr_pipeline/common.hpp"

namespace opr::pipeline {

class HealthNode : public rclcpp::Node {
 public:
  explicit HealthNode(const rclcpp::NodeOptions& options) : Node("perception_health", options) {
    stale_ms_ = declare_parameter("stale_ms", 500.0);
    diag_stale_ms_ = declare_parameter("diagnostics_stale_ms", 3000.0);
    pub_ = create_publisher<opr_msgs::msg::Health>("perception/health", rclcpp::QoS(10).reliable());
    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(100), [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& m) {
          const auto now = std::chrono::steady_clock::now();
          for (const auto& s : m->status) latest_[s.name] = {s.level, s.message, now};
        });
    tracks_sub_ = create_subscription<opr_msgs::msg::TrackArray>(
        "tracks", result_qos(), [this](const opr_msgs::msg::TrackArray::ConstSharedPtr& m) {
          last_tracks_ = std::chrono::steady_clock::now();
          last_stamp_ns_ = to_ns(m->header.stamp);
          have_tracks_ = true;
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] { publish(); });
  }

 private:
  struct Entry {
    std::uint8_t level;
    std::string message;
    std::chrono::steady_clock::time_point seen;
  };

  void publish() {
    const auto now = std::chrono::steady_clock::now();
    opr_msgs::msg::Health h;
    h.header.stamp = get_clock()->now();
    h.level = opr_msgs::msg::Health::OK;
    h.message = "all components OK";
    for (const auto& [name, e] : latest_) {
      const double age = std::chrono::duration<double, std::milli>(now - e.seen).count();
      std::uint8_t level = e.level;
      std::string msg = e.message;
      if (age > diag_stale_ms_) {
        level = opr_msgs::msg::Health::STALE;
        msg = "no diagnostics for " + std::to_string(static_cast<int>(age)) + " ms";
      }
      if (level > h.level) {
        h.level = level;
        h.worst = name;
        h.message = msg;
      }
    }
    if (!have_tracks_) {
      h.level = opr_msgs::msg::Health::ERROR;
      h.worst = "tracks";
      h.message = "no tracks received yet";
      h.track_age_ms = -1.0f;
    } else {
      const double silent = std::chrono::duration<double, std::milli>(now - last_tracks_).count();
      // Age from the exposure stamp only when stamps are on this machine's monotonic clock
      // (replayed flights use the simulator's clock, which is unrelated).
      const auto mono_ns = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
      const double stamp_age = (mono_ns - last_stamp_ns_) / 1e6;
      h.track_age_ms = (last_stamp_ns_ <= mono_ns && stamp_age < 60'000.0) ? static_cast<float>(stamp_age) : -1.0f;
      if (silent > stale_ms_) {
        h.level = opr_msgs::msg::Health::ERROR;
        h.worst = "tracks";
        h.message = "no tracks for " + std::to_string(static_cast<int>(silent)) + " ms";
      }
    }
    pub_->publish(h);
  }

  double stale_ms_, diag_stale_ms_;
  std::map<std::string, Entry> latest_;
  bool have_tracks_ = false;
  std::chrono::steady_clock::time_point last_tracks_{};
  std::uint64_t last_stamp_ns_ = 0;
  rclcpp::Publisher<opr_msgs::msg::Health>::SharedPtr pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<opr_msgs::msg::TrackArray>::SharedPtr tracks_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace opr::pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(opr::pipeline::HealthNode)
