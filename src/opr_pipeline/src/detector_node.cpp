// Detector node. The node that is meant to run in a separate container (a third-party or
// separately released perception module), so it supports three ways of getting pixels:
//   transport:=raw   sensor_msgs/Image through the middleware (or intra-process when composed)
//   transport:=shm   image_transport shm plugin: one copy out of shared memory
//   zero_copy:=true  reads the leased shared-memory slot in place: no copy at all. If the slot
//                    changes while it is being read (the lease ran out), the result is discarded.
// If the publisher's segment is not visible (other host, /dev/shm not shared), zero-copy mode
// falls back to raw by itself, like the plugin does.
#include <array>
#include <chrono>
#include <memory>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <opr_core/detect.hpp>
#include <opr_msgs/msg/detection_array.hpp>
#include <opr_msgs/msg/shm_image.hpp>
#include <opr_shm/segment.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "opr_pipeline/common.hpp"
#include "opr_shm_transport/image_publisher.hpp"

namespace opr::pipeline {

class DetectorNode : public rclcpp::Node {
 public:
  explicit DetectorNode(const rclcpp::NodeOptions& options) : Node("detector", options) {
    detector_.set_params(declare_detector(this));
    transport_ = declare_parameter("transport", std::string("raw"));
    zero_copy_ = declare_parameter("zero_copy", false);
    lease_ms_ = static_cast<std::uint32_t>(declare_parameter("lease_ms", 50));
    qos_mode_ = declare_parameter("image_qos", std::string("sensor"));
    pub_ = create_publisher<opr_msgs::msg::DetectionArray>("detections", result_qos());
    if (zero_copy_) {
      shm_sub_ = create_subscription<opr_msgs::msg::ShmImage>(
          "image_rect/shm", image_qos(qos_mode_), [this](const opr_msgs::msg::ShmImage::ConstSharedPtr& m) { on_handle(m); });
      RCLCPP_INFO(get_logger(), "reading image_rect in place from shared memory (lease %u ms)", lease_ms_);
    } else {
      subscribe_image(transport_);
    }
    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this, 1.0);
    diagnostics_->setHardwareID("detector");
    diagnostics_->add("detector", this, &DetectorNode::diagnose);
  }

 private:
  void subscribe_image(const std::string& transport) {
    it_sub_ = image_transport::create_subscription(
        this, "image_rect", [this](const sensor_msgs::msg::Image::ConstSharedPtr& m) { on_image(m); }, transport,
        image_qos(qos_mode_).get_rmw_qos_profile());
    RCLCPP_INFO(get_logger(), "subscribed to %s with transport '%s'", it_sub_.getTopic().c_str(), transport.c_str());
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (fell_back_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "shared memory not visible: using raw transport");
    } else if (errors_[static_cast<int>(opr::shm::ReadError::kTorn)] > 0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "frames changed while being read (lease too short)");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "detecting");
    }
    stat.add("frames", frames_);
    stat.add("mean ms", frames_ ? total_ms_ / static_cast<double>(frames_) : 0.0);
    stat.add("input", zero_copy_ ? std::string("shm zero-copy") : transport_);
    for (int e = 1; e < 6; ++e) {
      stat.add(std::string("shm ") + opr::shm::to_string(static_cast<opr::shm::ReadError>(e)), errors_[e]);
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& m) {
    if (m->encoding != "mono8") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "expected mono8, got %s", m->encoding.c_str());
      return;
    }
    const auto start = std::chrono::steady_clock::now();
    auto dets = detector_.detect({m->data.data(), static_cast<int>(m->width), static_cast<int>(m->height), m->step});
    publish(m->header, dets, start);
  }

  void on_handle(const opr_msgs::msg::ShmImage::ConstSharedPtr& m) {
    if (fell_back_) return;
    const auto start = std::chrono::steady_clock::now();
    opr::shm::ReadError err = opr::shm::ReadError::kNone;
    auto view = reader_.acquire(opr::shm_transport::to_handle(*m), lease_ms_, &err);
    if (!view) {
      ++errors_[static_cast<int>(err)];
      if (err == opr::shm::ReadError::kNotVisible) {
        fell_back_ = true;
        RCLCPP_WARN(get_logger(), "segment %s not visible here: falling back to raw image_rect", m->segment.c_str());
        shm_sub_.reset();
        subscribe_image("raw");
      }
      return;
    }
    auto dets = detector_.detect({view->data, static_cast<int>(m->width), static_cast<int>(m->height), m->step});
    const bool intact = reader_.validate(*view);
    reader_.release(*view);
    if (!intact) {  // the publisher reused the slot under us: these detections may be garbage
      ++errors_[static_cast<int>(opr::shm::ReadError::kTorn)];
      return;
    }
    publish(m->header, dets, start);
  }

  void publish(const std_msgs::msg::Header& header, const std::vector<opr::core::Detection>& dets,
               std::chrono::steady_clock::time_point start) {
    auto out = std::make_unique<opr_msgs::msg::DetectionArray>();
    out->header = header;
    out->detections.reserve(dets.size());
    for (const auto& d : dets) {
      opr_msgs::msg::Detection o;
      o.x = d.x;
      o.y = d.y;
      o.strength = d.strength;
      o.area = d.area;
      out->detections.push_back(o);
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    out->processing_ms = static_cast<float>(ms);
    pub_->publish(std::move(out));
    ++frames_;
    total_ms_ += ms;
  }

  opr::core::Detector detector_;
  std::string transport_;
  bool zero_copy_ = false;
  std::uint32_t lease_ms_ = 50;
  std::string qos_mode_;
  rclcpp::Publisher<opr_msgs::msg::DetectionArray>::SharedPtr pub_;
  image_transport::Subscriber it_sub_;
  rclcpp::Subscription<opr_msgs::msg::ShmImage>::SharedPtr shm_sub_;
  opr::shm::Reader reader_;
  bool fell_back_ = false;
  std::array<std::uint64_t, 6> errors_{};
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::uint64_t frames_ = 0;
  double total_ms_ = 0;
};

}  // namespace opr::pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(opr::pipeline::DetectorNode)
