// Tracker node: detections in, confirmed tracks out (opr_core::Tracker, visionbox 2.3 behaviour).
#include <memory>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <opr_core/track.hpp>
#include <opr_msgs/msg/detection_array.hpp>
#include <opr_msgs/msg/track_array.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "opr_pipeline/common.hpp"

namespace opr::pipeline {

class TrackerNode : public rclcpp::Node {
 public:
  explicit TrackerNode(const rclcpp::NodeOptions& options) : Node("tracker", options), tracker_(declare_tracker(this)) {
    pub_ = create_publisher<opr_msgs::msg::TrackArray>("tracks", result_qos());
    sub_ = create_subscription<opr_msgs::msg::DetectionArray>(
        "detections", result_qos(), [this](const opr_msgs::msg::DetectionArray::ConstSharedPtr& m) { on_detections(m); });
    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this, 1.0);
    diagnostics_->setHardwareID("tracker");
    diagnostics_->add("tracker", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      if (out_of_order_ > reported_out_of_order_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "frames arrived out of order and were ignored");
      } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "tracking");
      }
      reported_out_of_order_ = out_of_order_;
      stat.add("frames", frames_);
      stat.add("out of order", out_of_order_);
      stat.add("confirmed tracks", confirmed_);
    });
  }

 private:
  void on_detections(const opr_msgs::msg::DetectionArray::ConstSharedPtr& m) {
    std::vector<opr::core::Detection> dets;
    dets.reserve(m->detections.size());
    for (const auto& d : m->detections) dets.push_back({d.x, d.y, d.strength, d.area});
    const auto r = tracker_.step(to_ns(m->header.stamp), dets);
    if (!r.accepted) {
      ++out_of_order_;
      return;
    }
    auto out = std::make_unique<opr_msgs::msg::TrackArray>();
    out->header = m->header;
    out->born = static_cast<std::uint32_t>(r.born);
    out->dropped = static_cast<std::uint32_t>(r.dropped);
    for (const auto& t : tracker_.confirmed()) {
      opr_msgs::msg::Track o;
      o.id = t.id;
      o.x = t.x[0];
      o.y = t.x[1];
      o.vx = t.x[2];
      o.vy = t.x[3];
      o.hits = static_cast<std::uint32_t>(t.hits);
      o.coasting = t.misses > 0;
      out->tracks.push_back(o);
    }
    confirmed_ = out->tracks.size();
    pub_->publish(std::move(out));
    ++frames_;
  }

  opr::core::Tracker tracker_;
  rclcpp::Publisher<opr_msgs::msg::TrackArray>::SharedPtr pub_;
  rclcpp::Subscription<opr_msgs::msg::DetectionArray>::SharedPtr sub_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::uint64_t frames_ = 0, out_of_order_ = 0, reported_out_of_order_ = 0;
  std::size_t confirmed_ = 0;
};

}  // namespace opr::pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(opr::pipeline::TrackerNode)
