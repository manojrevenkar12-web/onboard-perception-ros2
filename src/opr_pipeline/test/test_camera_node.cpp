// Drives the camera node's lifecycle directly against the simulated device.
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <opr_msgs/msg/frame_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <class_loader/class_loader.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;

namespace {

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_camera(const std::string& device) {
  static class_loader::ClassLoader loader("libopr_pipeline_components.so");
  auto factory = loader.createInstance<rclcpp_components::NodeFactory>(
      "rclcpp_components::NodeFactoryTemplate<opr::pipeline::CameraNode>");
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"device", device}, {"image_qos", "reliable"}});
  auto wrapper = factory->create_node_instance(opts);
  return std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(wrapper.get_node_instance());
}

class CameraNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void listen() {
    probe_ = std::make_shared<rclcpp::Node>("probe");
    info_ = probe_->create_subscription<opr_msgs::msg::FrameInfo>(
        "frame_info", rclcpp::QoS(300).reliable(), [this](const opr_msgs::msg::FrameInfo& m) { infos_.push_back(m); });
    images_ = probe_->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::QoS(50).reliable(), [this](const sensor_msgs::msg::Image::ConstSharedPtr&) { ++frames_; });
  }

  bool spin_until(rclcpp::executors::SingleThreadedExecutor& ex, const std::function<bool()>& done,
                  std::chrono::milliseconds limit = 5s) {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      ex.spin_some(10ms);
      if (done()) return true;
    }
    return false;
  }

  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::Subscription<opr_msgs::msg::FrameInfo>::SharedPtr info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr images_;
  std::vector<opr_msgs::msg::FrameInfo> infos_;
  int frames_ = 0;
};

}  // namespace

TEST_F(CameraNodeTest, ConfigureActivateStreamDeactivateCleanup) {
  auto cam = make_camera("sim://fast?pace_hz=60");
  listen();
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(cam->get_node_base_interface());
  ex.add_node(probe_);
  EXPECT_EQ(cam->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_until(ex, [] { return false; }, 300ms);
  EXPECT_EQ(frames_, 0) << "an inactive camera publishes nothing";
  EXPECT_EQ(cam->activate().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(spin_until(ex, [&] { return frames_ >= 20 && infos_.size() >= 20; }));
  for (std::size_t i = 1; i < infos_.size(); ++i) {
    EXPECT_EQ(infos_[i].sequence, infos_[i - 1].sequence + 1);
    EXPECT_EQ(infos_[i].stamp_source, opr_msgs::msg::FrameInfo::STAMP_START_OF_EXPOSURE);
  }
  EXPECT_EQ(cam->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  spin_until(ex, [] { return false; }, 200ms);
  const int after = frames_;
  spin_until(ex, [] { return false; }, 300ms);
  EXPECT_EQ(frames_, after) << "frames after deactivate";
  EXPECT_EQ(cam->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(CameraNodeTest, AMissingCameraFailsConfigure) {
  auto cam = make_camera("/dev/video-that-does-not-exist");
  EXPECT_EQ(cam->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(CameraNodeTest, AWrongResolutionFailsConfigure) {
  auto cam = make_camera("sim://fast?wrong_size=1");
  EXPECT_EQ(cam->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(CameraNodeTest, AnUnplugIsALoudGapNotARestart) {
  // 60 Hz sensor, cable out for 400 ms after world frame 30: about 12 sensor frames never arrive.
  auto cam = make_camera("sim://swarm?pace_hz=60&unplug_at=30&unplug_ms=400&stamps=clock");
  listen();
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(cam->get_node_base_interface());
  ex.add_node(probe_);
  ASSERT_EQ(cam->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(cam->activate().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(spin_until(ex, [&] { return infos_.size() >= 60; }, 8s)) << infos_.size();
  std::uint32_t biggest_gap = 0;
  std::size_t restarts = 0;
  for (std::size_t i = 1; i < infos_.size(); ++i) {
    ASSERT_GT(infos_[i].sequence, infos_[i - 1].sequence) << "sequence must keep counting across reconnects";
    biggest_gap = std::max(biggest_gap, infos_[i].sequence - infos_[i - 1].sequence - 1);
    if (infos_[i].driver_sequence < infos_[i - 1].driver_sequence) ++restarts;
  }
  EXPECT_EQ(restarts, 1u) << "the driver's own counter restarted once (new stream)";
  EXPECT_GE(biggest_gap, 15u) << "a 400 ms outage plus reconnect time must show as lost frames";
  cam->deactivate();
}
