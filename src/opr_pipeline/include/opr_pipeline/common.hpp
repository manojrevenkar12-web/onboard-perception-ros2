// Helpers shared by the pipeline nodes: parameters that map onto opr_core's structs, stamps, QoS.
#pragma once

#include <cstdint>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <opr_core/params.hpp>
#include <rclcpp/rclcpp.hpp>

namespace opr::pipeline {

inline builtin_interfaces::msg::Time to_time(std::uint64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<std::int32_t>(ns / 1'000'000'000ULL);
  t.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000ULL);
  return t;
}

inline std::uint64_t to_ns(const builtin_interfaces::msg::Time& t) {
  return static_cast<std::uint64_t>(t.sec) * 1'000'000'000ULL + t.nanosec;
}

/// "sensor": best effort, keep last 2 (a late frame is worth less than the next one).
/// "reliable": keep last `depth`, used by the equivalence tests where no frame may be lost.
inline rclcpp::QoS image_qos(const std::string& mode, int depth = 10) {
  if (mode == "reliable") return rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(depth))).reliable();
  if (mode != "sensor") throw std::invalid_argument("image_qos must be 'sensor' or 'reliable', got " + mode);
  return rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
}

/// Results are small and must not be lost.
inline rclcpp::QoS result_qos() { return rclcpp::QoS(rclcpp::KeepLast(50)).reliable(); }

template <typename NodeT>
opr::core::CameraModel declare_camera(NodeT* node) {
  opr::core::CameraModel m;
  m.width = static_cast<int>(node->declare_parameter("width", m.width));
  m.height = static_cast<int>(node->declare_parameter("height", m.height));
  m.fx = node->declare_parameter("fx", m.fx);
  m.fy = node->declare_parameter("fy", m.fy);
  m.cx = node->declare_parameter("cx", m.cx);
  m.cy = node->declare_parameter("cy", m.cy);
  m.k1 = node->declare_parameter("k1", m.k1);
  m.k2 = node->declare_parameter("k2", m.k2);
  m.p1 = node->declare_parameter("p1", m.p1);
  m.p2 = node->declare_parameter("p2", m.p2);
  m.k3 = node->declare_parameter("k3", m.k3);
  return m;
}

template <typename NodeT>
opr::core::DetectorParams declare_detector(NodeT* node) {
  opr::core::DetectorParams p;
  p.bg_radius = static_cast<int>(node->declare_parameter("bg_radius", p.bg_radius));
  p.threshold = static_cast<int>(node->declare_parameter("threshold", p.threshold));
  p.min_area = static_cast<int>(node->declare_parameter("min_area", p.min_area));
  p.max_area = static_cast<int>(node->declare_parameter("max_area", p.max_area));
  return p;
}

template <typename NodeT>
opr::core::TrackerParams declare_tracker(NodeT* node) {
  opr::core::TrackerParams p;
  p.gate = node->declare_parameter("gate", p.gate);
  p.accel_noise = node->declare_parameter("accel_noise", p.accel_noise);
  p.meas_noise = node->declare_parameter("meas_noise", p.meas_noise);
  p.init_pos_var = node->declare_parameter("init_pos_var", p.init_pos_var);
  p.init_vel_var = node->declare_parameter("init_vel_var", p.init_vel_var);
  p.confirm_hits = static_cast<int>(node->declare_parameter("confirm_hits", p.confirm_hits));
  p.max_misses = static_cast<int>(node->declare_parameter("max_misses", p.max_misses));
  return p;
}

}  // namespace opr::pipeline
