// Pairs each TrackArray with the FrameInfo of the frame it came from (by exposure stamp), so the
// telemetry carries the camera's frame sequence like visionbox did. FrameInfo is published before
// the image, so it nearly always arrives first; if it does not arrive within `timeout_ms` the
// packet goes out anyway with a sequence estimated from the stamps, and is counted as unjoined.
// Output order is input order.
#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <vector>

#include <opr_core/telemetry.hpp>

namespace opr::pipeline {

class Joiner {
 public:
  Joiner(std::uint64_t timeout_ms, std::uint64_t frame_interval_ns)
      : timeout_ms_(timeout_ms), interval_ns_(frame_interval_ns) {}

  void add_frame(std::uint64_t stamp_ns, std::uint32_t sequence) {
    frames_[stamp_ns] = sequence;
    if (frames_.size() > 2000) frames_.erase(frames_.begin());
  }

  /// `packet.sequence` is ignored and filled in.
  void add_tracks(const opr::core::telemetry::Packet& packet, std::uint64_t now_ms) {
    pending_.push_back({packet, now_ms});
  }

  /// Packets that are ready, in arrival order.
  std::vector<opr::core::telemetry::Packet> drain(std::uint64_t now_ms) {
    std::vector<opr::core::telemetry::Packet> out;
    while (!pending_.empty()) {
      Pending& p = pending_.front();
      auto it = frames_.find(p.packet.stamp_ns);
      if (it != frames_.end()) {
        p.packet.sequence = it->second;
        ++joined_;
      } else if (now_ms >= p.arrived_ms + timeout_ms_) {
        p.packet.sequence = estimate(p.packet.stamp_ns);
        ++unjoined_;
      } else {
        break;
      }
      last_sequence_ = p.packet.sequence;
      last_stamp_ = p.packet.stamp_ns;
      have_last_ = true;
      out.push_back(p.packet);
      frames_.erase(frames_.begin(), frames_.upper_bound(p.packet.stamp_ns));
      pending_.pop_front();
    }
    return out;
  }

  std::uint64_t joined() const { return joined_; }
  std::uint64_t unjoined() const { return unjoined_; }
  std::size_t pending() const { return pending_.size(); }

 private:
  std::uint32_t estimate(std::uint64_t stamp) const {
    if (!have_last_ || stamp <= last_stamp_ || interval_ns_ == 0) return have_last_ ? last_sequence_ + 1 : 0;
    const std::uint64_t steps = (stamp - last_stamp_ + interval_ns_ / 2) / interval_ns_;
    return last_sequence_ + static_cast<std::uint32_t>(steps == 0 ? 1 : steps);
  }

  struct Pending {
    opr::core::telemetry::Packet packet;
    std::uint64_t arrived_ms;
  };
  std::uint64_t timeout_ms_;
  std::uint64_t interval_ns_;
  std::map<std::uint64_t, std::uint32_t> frames_;
  std::deque<Pending> pending_;
  bool have_last_ = false;
  std::uint32_t last_sequence_ = 0;
  std::uint64_t last_stamp_ = 0;
  std::uint64_t joined_ = 0;
  std::uint64_t unjoined_ = 0;
};

}  // namespace opr::pipeline
