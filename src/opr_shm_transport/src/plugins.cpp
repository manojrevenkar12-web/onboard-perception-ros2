#include "opr_shm_transport/plugins.hpp"

#include <cstdlib>

#include <pluginlib/class_list_macros.hpp>

namespace opr::shm_transport {

namespace {
std::uint32_t slots_from_env() {
  const char* v = std::getenv("OPR_SHM_SLOTS");
  const long n = v ? std::strtol(v, nullptr, 10) : 4;
  return n >= 2 && n <= 64 ? static_cast<std::uint32_t>(n) : 4;
}
}  // namespace

void ShmPublisher::advertiseImpl(rclcpp::Node* node, const std::string& base_topic, rmw_qos_profile_t custom_qos,
                                 rclcpp::PublisherOptions options) {
  slots_ = std::make_unique<SlotWriter>(base_topic, slots_from_env());
  SimplePublisherPlugin::advertiseImpl(node, base_topic, custom_qos, options);
}

void ShmPublisher::publish(const sensor_msgs::msg::Image& m, const PublisherT& publisher) const {
  auto handle = slots_->write(m.header, m.width, m.height, m.step, m.encoding, m.data.data(), m.data.size());
  if (handle) publisher->publish(*handle);
}

void ShmSubscriber::subscribeImpl(rclcpp::Node* node, const std::string& base_topic, const Callback& callback,
                                  rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions options) {
  node_ = node;
  base_topic_ = base_topic;
  callback_ = callback;
  qos_ = custom_qos;
  options_ = options;
  SimpleSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos, options);
}

void ShmSubscriber::shutdown() {
  raw_.reset();
  SimpleSubscriberPlugin::shutdown();
}

void ShmSubscriber::start_fallback(const std::string& why) {
  if (fallback_.exchange(true)) return;
  RCLCPP_WARN(node_->get_logger(),
              "shm transport: %s; falling back to the raw transport on %s. Frames will be serialised "
              "through the middleware. To share memory between containers run them with a shared IPC "
              "namespace and /dev/shm (docker: --ipc=host or --ipc=container:<name>).",
              why.c_str(), base_topic_.c_str());
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_), qos_);
  auto cb = callback_;
  auto* stats = &stats_;
  raw_ = node_->create_subscription<sensor_msgs::msg::Image>(
      base_topic_, qos,
      [cb, stats](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        ++stats->fallback_frames;
        cb(msg);
      },
      options_);
}

void ShmSubscriber::internalCallback(const opr_msgs::msg::ShmImage::ConstSharedPtr& m, const Callback& user_cb) {
  if (fallback_.load()) return;
  auto img = std::make_shared<sensor_msgs::msg::Image>();
  img->data.resize(m->data_bytes);
  opr::shm::ReadError err = opr::shm::ReadError::kNone;
  opr::shm::Meta meta;
  if (reader_.copy(to_handle(*m), img->data.data(), img->data.size(), &meta, &err)) {
    img->header = m->header;
    img->width = m->width;
    img->height = m->height;
    img->step = m->step;
    img->encoding = m->encoding;
    ++stats_.received;
    user_cb(img);
    return;
  }
  ++stats_.errors[static_cast<std::size_t>(err)];
  if (err == opr::shm::ReadError::kNotVisible) {
    start_fallback("segment " + m->segment + " is not visible from this process");
  }
  // Overwritten, torn or stale frames are dropped: a newer one is already on its way.
}

}  // namespace opr::shm_transport

PLUGINLIB_EXPORT_CLASS(opr::shm_transport::ShmPublisher, image_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(opr::shm_transport::ShmSubscriber, image_transport::SubscriberPlugin)
