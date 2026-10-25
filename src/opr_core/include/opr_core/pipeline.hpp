#pragma once

#include <cstdint>
#include <vector>

#include "opr_core/detect.hpp"
#include "opr_core/image.hpp"
#include "opr_core/params.hpp"
#include "opr_core/rectify.hpp"
#include "opr_core/track.hpp"

namespace opr::core {

/// The whole chain in one object, with visionbox 2.3's behaviour. The ROS 2 nodes split it across
/// processes; this is what the equivalence tests and the single-process fallback use.
class Pipeline {
 public:
  Pipeline(const CameraModel& camera, const DetectorParams& detector, const TrackerParams& tracker);

  /// Processes one YUYV frame and returns the telemetry datagram for it.
  std::vector<std::uint8_t> process(View<const std::uint8_t> yuyv, std::uint32_t sequence, std::uint64_t stamp_ns);

  const std::vector<Detection>& last_detections() const { return detections_; }
  const Tracker& tracker() const { return tracker_; }
  ConstImageView rectified() const;

 private:
  Rectifier rectifier_;
  Detector detector_;
  Tracker tracker_;
  std::vector<std::uint8_t> mono_;
  std::vector<std::uint8_t> rect_;
  std::vector<Detection> detections_;
};

}  // namespace opr::core
