// Publishes an image the cheapest way each audience can take it:
//   <topic>      sensor_msgs/Image, only if someone subscribes (in-process subscribers get it
//                through intra-process communication without serialisation)
//   <topic>/shm  opr_msgs/ShmImage handles into a leased shared-memory slot, only if someone
//                subscribes there (other processes and containers on this host)
// The same topic names as image_transport's raw and "shm" transports, so standard subscribers
// with the opr_shm_transport plugin, rviz and ros2 bag all keep working.
#pragma once

#include <cstdint>
#include <memory>
#include <atomic>
#include <mutex>
#include <optional>
#include <string>

#include <opr_msgs/msg/shm_image.hpp>
#include <opr_shm/segment.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace opr::shm_transport {

struct PublisherStats {
  std::uint64_t raw = 0;          // frames sent as sensor_msgs/Image
  std::uint64_t shm = 0;          // frames committed to shared memory
  std::uint64_t shm_dropped = 0;  // no free slot (every slot leased by a slow reader)
  std::uint64_t segments = 0;     // segments created (one more each time the frame size grows)
};

/// Turns a ROS handle message into an opr_shm handle.
opr::shm::Handle to_handle(const opr_msgs::msg::ShmImage& msg);

/// Owns a topic's shared-memory segment: copies a frame into a free slot and returns the handle
/// message to publish. Thread-safe. Used by ImagePublisher and by the image_transport plugin.
class SlotWriter {
 public:
  SlotWriter(std::string topic, std::uint32_t slots) : topic_(std::move(topic)), slots_(slots) {}

  /// Returns nothing if every slot is leased (the frame is dropped for shared-memory readers).
  std::optional<opr_msgs::msg::ShmImage> write(const std_msgs::msg::Header& header, std::uint32_t width,
                                               std::uint32_t height, std::uint32_t step,
                                               const std::string& encoding, const std::uint8_t* data,
                                               std::size_t bytes);
  PublisherStats stats() const;

 private:
  std::string topic_;
  std::uint32_t slots_;
  mutable std::mutex mutex_;
  std::unique_ptr<opr::shm::Writer> writer_;
  PublisherStats stats_;
};

class ImagePublisher {
 public:
  ImagePublisher(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw,
                 rclcpp::Publisher<opr_msgs::msg::ShmImage>::SharedPtr shm, std::string topic,
                 std::uint32_t slots = 4);

  /// Copies `data` once per audience that exists. Returns false if nobody was listening.
  bool publish(const std_msgs::msg::Header& header, std::uint32_t width, std::uint32_t height,
               std::uint32_t step, const std::string& encoding, const std::uint8_t* data, std::size_t bytes);

  /// For callers that already own a message (raw path takes it without a copy).
  bool publish(sensor_msgs::msg::Image::UniquePtr image);

  PublisherStats stats() const;
  std::size_t raw_subscribers() const { return raw_->get_subscription_count(); }
  std::size_t shm_subscribers() const { return shm_->get_subscription_count(); }

 private:
  bool to_shm(const std_msgs::msg::Header& header, std::uint32_t width, std::uint32_t height, std::uint32_t step,
              const std::string& encoding, const std::uint8_t* data, std::size_t bytes);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_;
  rclcpp::Publisher<opr_msgs::msg::ShmImage>::SharedPtr shm_;
  SlotWriter slots_;
  std::atomic<std::uint64_t> raw_count_{0};
};

}  // namespace opr::shm_transport
