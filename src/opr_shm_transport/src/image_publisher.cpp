#include "opr_shm_transport/image_publisher.hpp"

#include <cstring>

namespace opr::shm_transport {

opr::shm::Handle to_handle(const opr_msgs::msg::ShmImage& m) {
  return {m.segment, m.epoch, m.boot_id, m.slot, m.seq};
}

std::optional<opr_msgs::msg::ShmImage> SlotWriter::write(const std_msgs::msg::Header& header, std::uint32_t width,
                                                         std::uint32_t height, std::uint32_t step,
                                                         const std::string& encoding, const std::uint8_t* data,
                                                         std::size_t bytes) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!writer_ || writer_->slot_bytes() < bytes) {
    // Sized for this frame with some headroom; a larger frame later gets a new segment (and a new
    // epoch, so readers remap instead of trusting the old layout).
    writer_.reset();
    writer_ = std::make_unique<opr::shm::Writer>(opr::shm::segment_name_for(topic_), slots_, bytes + bytes / 8);
    ++stats_.segments;
  }
  auto loan = writer_->loan(bytes);
  if (!loan) {
    ++stats_.shm_dropped;
    return std::nullopt;
  }
  std::memcpy(loan.data, data, bytes);
  opr::shm::Meta meta;
  meta.width = width;
  meta.height = height;
  meta.step = step;
  meta.data_bytes = static_cast<std::uint32_t>(bytes);
  meta.stamp_ns = static_cast<std::uint64_t>(header.stamp.sec) * 1'000'000'000ULL + header.stamp.nanosec;
  std::strncpy(meta.encoding, encoding.c_str(), sizeof(meta.encoding) - 1);
  std::strncpy(meta.frame_id, header.frame_id.c_str(), sizeof(meta.frame_id) - 1);
  const opr::shm::Handle h = writer_->commit(loan, meta);
  ++stats_.shm;

  opr_msgs::msg::ShmImage msg;
  msg.header = header;
  msg.segment = h.segment;
  msg.epoch = h.epoch;
  msg.boot_id = h.boot_id;
  msg.slot = h.slot;
  msg.seq = h.seq;
  msg.width = width;
  msg.height = height;
  msg.step = step;
  msg.encoding = encoding;
  msg.data_bytes = static_cast<std::uint32_t>(bytes);
  return msg;
}

PublisherStats SlotWriter::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

ImagePublisher::ImagePublisher(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw,
                               rclcpp::Publisher<opr_msgs::msg::ShmImage>::SharedPtr shm, std::string topic,
                               std::uint32_t slots)
    : raw_(std::move(raw)), shm_(std::move(shm)), slots_(std::move(topic), slots) {}

bool ImagePublisher::to_shm(const std_msgs::msg::Header& header, std::uint32_t width, std::uint32_t height,
                            std::uint32_t step, const std::string& encoding, const std::uint8_t* data,
                            std::size_t bytes) {
  auto msg = slots_.write(header, width, height, step, encoding, data, bytes);
  if (!msg) return false;
  shm_->publish(*msg);
  return true;
}

bool ImagePublisher::publish(const std_msgs::msg::Header& header, std::uint32_t width, std::uint32_t height,
                             std::uint32_t step, const std::string& encoding, const std::uint8_t* data,
                             std::size_t bytes) {
  bool any = false;
  if (shm_->get_subscription_count() > 0) any |= to_shm(header, width, height, step, encoding, data, bytes);
  if (raw_->get_subscription_count() > 0) {
    auto img = std::make_unique<sensor_msgs::msg::Image>();
    img->header = header;
    img->width = width;
    img->height = height;
    img->step = step;
    img->encoding = encoding;
    img->data.assign(data, data + bytes);
    raw_->publish(std::move(img));
    ++raw_count_;
    any = true;
  }
  return any;
}

bool ImagePublisher::publish(sensor_msgs::msg::Image::UniquePtr image) {
  bool any = false;
  if (shm_->get_subscription_count() > 0) {
    any |= to_shm(image->header, image->width, image->height, image->step, image->encoding, image->data.data(),
                  image->data.size());
  }
  if (raw_->get_subscription_count() > 0) {
    raw_->publish(std::move(image));
    ++raw_count_;
    any = true;
  }
  return any;
}

PublisherStats ImagePublisher::stats() const {
  PublisherStats s = slots_.stats();
  s.raw = raw_count_.load();
  return s;
}

}  // namespace opr::shm_transport
