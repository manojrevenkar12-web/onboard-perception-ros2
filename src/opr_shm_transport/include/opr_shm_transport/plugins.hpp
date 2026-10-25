// image_transport plugins for the "shm" transport, so any ROS 2 node can publish or subscribe to
// leased shared-memory images with a transport hint, and fall back to raw when it cannot see the
// publisher's segment (another host, or a container that does not share /dev/shm).
#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>

#include <image_transport/simple_publisher_plugin.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>
#include <opr_msgs/msg/shm_image.hpp>
#include <opr_shm/segment.hpp>

#include "opr_shm_transport/image_publisher.hpp"

namespace opr::shm_transport {

class ShmPublisher : public image_transport::SimplePublisherPlugin<opr_msgs::msg::ShmImage> {
 public:
  std::string getTransportName() const override { return "shm"; }

 protected:
  void advertiseImpl(rclcpp::Node* node, const std::string& base_topic, rmw_qos_profile_t custom_qos,
                     rclcpp::PublisherOptions options) override;
  void publish(const sensor_msgs::msg::Image& message, const PublisherT& publisher) const override;

 private:
  mutable std::unique_ptr<SlotWriter> slots_;
};

struct SubscriberStats {
  std::atomic<std::uint64_t> received{0};
  std::atomic<std::uint64_t> fallback_frames{0};
  std::array<std::atomic<std::uint64_t>, 6> errors{};  // indexed by opr::shm::ReadError
};

class ShmSubscriber : public image_transport::SimpleSubscriberPlugin<opr_msgs::msg::ShmImage> {
 public:
  std::string getTransportName() const override { return "shm"; }
  void shutdown() override;
  bool falling_back() const { return fallback_.load(); }
  const SubscriberStats& stats() const { return stats_; }

 protected:
  void subscribeImpl(rclcpp::Node* node, const std::string& base_topic, const Callback& callback,
                     rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions options) override;
  void internalCallback(const opr_msgs::msg::ShmImage::ConstSharedPtr& message, const Callback& user_cb) override;

 private:
  void start_fallback(const std::string& why);

  rclcpp::Node* node_ = nullptr;
  std::string base_topic_;
  Callback callback_;
  rmw_qos_profile_t qos_{};
  rclcpp::SubscriptionOptions options_;
  opr::shm::Reader reader_;
  std::atomic<bool> fallback_{false};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_;
  SubscriberStats stats_;
};

}  // namespace opr::shm_transport
