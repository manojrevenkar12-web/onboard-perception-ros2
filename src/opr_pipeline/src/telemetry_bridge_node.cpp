// Telemetry bridge: keeps the ground station working unchanged. Turns TrackArray into visionbox
// wire-format v1 datagrams (opr_core/telemetry.hpp) on the same UDP port, and can record them to
// a file (one hex datagram per line), which is what the equivalence tests compare.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <opr_core/telemetry.hpp>
#include <opr_msgs/msg/frame_info.hpp>
#include <opr_msgs/msg/track_array.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "opr_pipeline/common.hpp"
#include "opr_pipeline/joiner.hpp"

namespace opr::pipeline {

class TelemetryBridgeNode : public rclcpp::Node {
 public:
  explicit TelemetryBridgeNode(const rclcpp::NodeOptions& options)
      : Node("telemetry_bridge", options),
        joiner_(static_cast<std::uint64_t>(declare_parameter("join_timeout_ms", 200)),
                1'000'000'000ULL / static_cast<std::uint64_t>(declare_parameter("fps", 30))) {
    const auto host = declare_parameter("udp_host", std::string("127.0.0.1"));
    const auto port = declare_parameter("udp_port", 14560);
    const auto record = declare_parameter("record_path", std::string(""));
    if (port > 0) {
      sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
      dst_.sin_family = AF_INET;
      dst_.sin_port = htons(static_cast<std::uint16_t>(port));
      if (::inet_pton(AF_INET, host.c_str(), &dst_.sin_addr) != 1) {
        throw std::invalid_argument("udp_host must be an IPv4 address, got " + host);
      }
    }
    if (!record.empty()) {
      record_.open(record, std::ios::out | std::ios::trunc);
      if (!record_) throw std::runtime_error("cannot write " + record);
      RCLCPP_INFO(get_logger(), "recording telemetry to %s", record.c_str());
    }
    info_sub_ = create_subscription<opr_msgs::msg::FrameInfo>(
        "frame_info", rclcpp::QoS(rclcpp::KeepLast(300)).reliable(),
        [this](const opr_msgs::msg::FrameInfo::ConstSharedPtr& m) {
          std::lock_guard<std::mutex> lock(mutex_);
          joiner_.add_frame(to_ns(m->header.stamp), m->sequence);
          flush();
        });
    tracks_sub_ = create_subscription<opr_msgs::msg::TrackArray>(
        "tracks", result_qos(), [this](const opr_msgs::msg::TrackArray::ConstSharedPtr& m) { on_tracks(m); });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
      std::lock_guard<std::mutex> lock(mutex_);
      flush();
    });
    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this, 1.0);
    diagnostics_->setHardwareID("telemetry");
    diagnostics_->add("telemetry", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (send_errors_ > 0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "UDP send failing");
      } else if (joiner_.unjoined() > 0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "some packets sent with an estimated sequence");
      } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "sending");
      }
      stat.add("sent", sent_);
      stat.add("joined with frame info", joiner_.joined());
      stat.add("estimated sequence", joiner_.unjoined());
      stat.add("send errors", send_errors_);
    });
  }

  ~TelemetryBridgeNode() override {
    if (sock_ >= 0) ::close(sock_);
  }

 private:
  static std::uint64_t now_ms() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
  }

  void on_tracks(const opr_msgs::msg::TrackArray::ConstSharedPtr& m) {
    opr::core::telemetry::Packet p;
    p.stamp_ns = to_ns(m->header.stamp);
    p.tracks.reserve(m->tracks.size());
    for (const auto& t : m->tracks) {
      opr::core::telemetry::TrackRecord r;
      r.id = t.id;
      r.x = static_cast<float>(t.x);
      r.y = static_cast<float>(t.y);
      r.vx = static_cast<float>(t.vx);
      r.vy = static_cast<float>(t.vy);
      r.hits = static_cast<std::uint8_t>(t.hits > 255 ? 255 : t.hits);
      r.coasting = t.coasting;
      p.tracks.push_back(r);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    joiner_.add_tracks(p, now_ms());
    flush();
  }

  void flush() {
    for (const auto& p : joiner_.drain(now_ms())) {
      const auto bytes = opr::core::telemetry::encode(p);
      if (sock_ >= 0 &&
          ::sendto(sock_, bytes.data(), bytes.size(), 0, reinterpret_cast<sockaddr*>(&dst_), sizeof(dst_)) < 0) {
        ++send_errors_;
      }
      if (record_) {
        record_ << opr::core::telemetry::to_hex(bytes) << '\n';
        record_.flush();
      }
      ++sent_;
    }
  }

  std::mutex mutex_;
  Joiner joiner_;
  int sock_ = -1;
  sockaddr_in dst_{};
  std::ofstream record_;
  rclcpp::Subscription<opr_msgs::msg::FrameInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<opr_msgs::msg::TrackArray>::SharedPtr tracks_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::uint64_t sent_ = 0, send_errors_ = 0;
};

}  // namespace opr::pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(opr::pipeline::TelemetryBridgeNode)
