// Rectify node: YUYV (or mono8) in, undistorted mono8 out. In a composed container the input
// arrives by intra-process pointer and the output leaves the same way; across processes it uses
// the shm transport.
#include <chrono>
#include <memory>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <opr_core/image.hpp>
#include <opr_core/rectify.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "opr_pipeline/common.hpp"
#include "opr_shm_transport/image_publisher.hpp"

namespace opr::pipeline {

class RectifyNode : public rclcpp::Node {
 public:
  explicit RectifyNode(const rclcpp::NodeOptions& options) : Node("rectify", options) {
    const auto model = declare_camera(this);
    const auto transport = declare_parameter("transport", std::string("raw"));
    const auto qos_mode = declare_parameter("image_qos", std::string("sensor"));
    const auto slots = declare_parameter("shm_slots", 4);
    rectifier_ = std::make_unique<opr::core::Rectifier>(model);
    mono_.resize(static_cast<std::size_t>(model.width) * model.height);

    auto raw = create_publisher<sensor_msgs::msg::Image>("image_rect", image_qos(qos_mode));
    auto shm = create_publisher<opr_msgs::msg::ShmImage>("image_rect/shm", image_qos(qos_mode));
    out_ = std::make_unique<opr::shm_transport::ImagePublisher>(
        raw, shm, std::string(get_fully_qualified_name()) + "/image_rect", static_cast<std::uint32_t>(slots));
    sub_ = image_transport::create_subscription(
        this, "image_raw", [this](const sensor_msgs::msg::Image::ConstSharedPtr& m) { on_image(m); }, transport,
        image_qos(qos_mode).get_rmw_qos_profile());
    RCLCPP_INFO(get_logger(), "subscribed to %s with transport '%s'", sub_.getTopic().c_str(), transport.c_str());

    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this, 1.0);
    diagnostics_->setHardwareID("rectify");
    diagnostics_->add("rectify", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      if (rejected_ > 0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "frames with the wrong size or encoding");
      } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "rectifying");
      }
      stat.add("frames", frames_);
      stat.add("rejected", rejected_);
      stat.add("mean ms", frames_ ? total_ms_ / static_cast<double>(frames_) : 0.0);
    });
  }

 private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& in) {
    const auto start = std::chrono::steady_clock::now();
    const auto& m = rectifier_->model();
    const bool size_ok = static_cast<int>(in->width) == m.width && static_cast<int>(in->height) == m.height;
    opr::core::ConstImageView mono;
    if (size_ok && in->encoding == "yuv422_yuy2" && in->step >= in->width * 2 && in->data.size() >= in->step * in->height) {
      opr::core::ImageView dst{mono_.data(), m.width, m.height, static_cast<std::size_t>(m.width)};
      opr::core::yuyv_to_mono({in->data.data(), m.width, m.height, in->step}, dst);
      mono = {mono_.data(), m.width, m.height, static_cast<std::size_t>(m.width)};
    } else if (size_ok && in->encoding == "mono8" && in->data.size() >= in->step * in->height) {
      mono = {in->data.data(), m.width, m.height, in->step};
    } else {
      ++rejected_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "rejected %ux%u %s frame (expected %dx%d yuv422_yuy2 or mono8)",
                            in->width, in->height, in->encoding.c_str(), m.width, m.height);
      return;
    }
    auto out = std::make_unique<sensor_msgs::msg::Image>();
    out->header = in->header;
    out->width = in->width;
    out->height = in->height;
    out->encoding = "mono8";
    out->step = in->width;
    out->data.resize(static_cast<std::size_t>(out->step) * out->height);
    rectifier_->apply(mono, {out->data.data(), m.width, m.height, out->step});
    out_->publish(std::move(out));
    ++frames_;
    total_ms_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  }

  std::unique_ptr<opr::core::Rectifier> rectifier_;
  std::vector<std::uint8_t> mono_;
  std::unique_ptr<opr::shm_transport::ImagePublisher> out_;
  image_transport::Subscriber sub_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::uint64_t frames_ = 0;
  std::uint64_t rejected_ = 0;
  double total_ms_ = 0;
};

}  // namespace opr::pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(opr::pipeline::RectifyNode)
