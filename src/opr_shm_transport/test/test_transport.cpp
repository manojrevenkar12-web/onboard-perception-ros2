#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include <image_transport/image_transport.hpp>
#include <opr_msgs/msg/shm_image.hpp>
#include <rclcpp/rclcpp.hpp>

#include "opr_shm_transport/image_publisher.hpp"

using namespace std::chrono_literals;
using sensor_msgs::msg::Image;

namespace {

Image::UniquePtr frame(std::uint8_t fill, int seq) {
  auto m = std::make_unique<Image>();
  m->header.frame_id = "camera";
  m->header.stamp.sec = 1;
  m->header.stamp.nanosec = static_cast<std::uint32_t>(seq);
  m->width = 320;
  m->height = 240;
  m->step = 320;
  m->encoding = "mono8";
  m->data.assign(320 * 240, fill);
  m->data[0] = static_cast<std::uint8_t>(seq);
  return m;
}

template <typename Pred>
bool spin_until(rclcpp::executors::SingleThreadedExecutor& ex, Pred done, std::chrono::milliseconds limit = 5s) {
  const auto end = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < end) {
    ex.spin_some(10ms);
    if (done()) return true;
  }
  return false;
}

class Transport : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

}  // namespace

TEST_F(Transport, ImageTransportShmCarriesTheExactBytes) {
  auto pub_node = std::make_shared<rclcpp::Node>("shm_pub_test");
  auto sub_node = std::make_shared<rclcpp::Node>("shm_sub_test");
  auto pub = image_transport::create_publisher(pub_node.get(), "/t1/image");
  std::vector<Image::ConstSharedPtr> got;
  auto sub = image_transport::create_subscription(
      sub_node.get(), "/t1/image", [&](const Image::ConstSharedPtr& m) { got.push_back(m); }, "shm");
  EXPECT_EQ(sub.getTransport(), "shm");
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(pub_node);
  ex.add_node(sub_node);
  ASSERT_TRUE(spin_until(ex, [&] { return pub.getNumSubscribers() > 0; }));
  for (int i = 0; i < 5; ++i) {
    pub.publish(*frame(static_cast<std::uint8_t>(40 + i), i));
    ASSERT_TRUE(spin_until(ex, [&] { return got.size() == static_cast<std::size_t>(i + 1); }));
  }
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(got[i]->data[0], i);
    EXPECT_EQ(got[i]->data[1000], 40 + i);
    EXPECT_EQ(got[i]->header.stamp.nanosec, static_cast<std::uint32_t>(i));
    EXPECT_EQ(got[i]->encoding, "mono8");
    EXPECT_EQ(got[i]->width, 320u);
  }
}

TEST_F(Transport, FallsBackToRawWhenTheSegmentIsNotVisible) {
  // A publisher in "another container": its handles point at a segment this process cannot open.
  auto node = std::make_shared<rclcpp::Node>("shm_fallback_test");
  auto handles = node->create_publisher<opr_msgs::msg::ShmImage>("/t2/image/shm", rclcpp::SensorDataQoS());
  auto raw = node->create_publisher<Image>("/t2/image", rclcpp::SensorDataQoS());
  std::vector<Image::ConstSharedPtr> got;
  auto sub_node = std::make_shared<rclcpp::Node>("shm_fallback_sub");
  auto sub = image_transport::create_subscription(
      sub_node.get(), "/t2/image", [&](const Image::ConstSharedPtr& m) { got.push_back(m); }, "shm",
      rmw_qos_profile_sensor_data);
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  ex.add_node(sub_node);
  ASSERT_TRUE(spin_until(ex, [&] { return handles->get_subscription_count() > 0; }));
  opr_msgs::msg::ShmImage h;
  h.segment = "/opr_segment_in_another_container";
  h.epoch = 42;
  h.data_bytes = 320 * 240;
  handles->publish(h);
  ASSERT_TRUE(spin_until(ex, [&] { return raw->get_subscription_count() > 0; }))
      << "subscriber did not fall back to the raw topic";
  for (int i = 0; i < 20 && got.empty(); ++i) {
    raw->publish(*frame(7, 3));
    spin_until(ex, [&] { return !got.empty(); }, 100ms);
  }
  ASSERT_FALSE(got.empty());
  EXPECT_EQ(got[0]->data[5], 7);
  handles->publish(h);  // further handles are ignored, not an error storm
  spin_until(ex, [] { return false; }, 100ms);
}

TEST_F(Transport, DualPublisherOnlyPaysForAudiencesThatExist) {
  auto node = std::make_shared<rclcpp::Node>("dual_pub_test");
  auto raw = node->create_publisher<Image>("/t3/image", rclcpp::SensorDataQoS());
  auto shm = node->create_publisher<opr_msgs::msg::ShmImage>("/t3/image/shm", rclcpp::SensorDataQoS());
  opr::shm_transport::ImagePublisher pub(raw, shm, "/t3/image");
  std::vector<std::uint8_t> pixels(640 * 480, 9);
  std_msgs::msg::Header header;
  EXPECT_FALSE(pub.publish(header, 640, 480, 640, "mono8", pixels.data(), pixels.size()));
  EXPECT_EQ(pub.stats().shm, 0u);
  EXPECT_EQ(pub.stats().raw, 0u);

  auto sub_node = std::make_shared<rclcpp::Node>("dual_sub_test");
  std::vector<opr_msgs::msg::ShmImage> handles_seen;
  auto s = sub_node->create_subscription<opr_msgs::msg::ShmImage>(
      "/t3/image/shm", rclcpp::SensorDataQoS(), [&](const opr_msgs::msg::ShmImage& m) { handles_seen.push_back(m); });
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  ex.add_node(sub_node);
  ASSERT_TRUE(spin_until(ex, [&] { return shm->get_subscription_count() > 0; }));
  EXPECT_TRUE(pub.publish(header, 640, 480, 640, "mono8", pixels.data(), pixels.size()));
  ASSERT_TRUE(spin_until(ex, [&] { return !handles_seen.empty(); }));
  EXPECT_EQ(pub.stats().shm, 1u);
  EXPECT_EQ(pub.stats().raw, 0u) << "nobody subscribed to raw, so no raw copy was made";
  opr::shm::Reader reader;
  auto v = reader.acquire(opr::shm_transport::to_handle(handles_seen[0]), 10);
  ASSERT_TRUE(v);
  EXPECT_EQ(v->data[100], 9);
}
