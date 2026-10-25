#include "opr_core/pipeline.hpp"

#include "opr_core/telemetry.hpp"

namespace opr::core {

Pipeline::Pipeline(const CameraModel& camera, const DetectorParams& detector, const TrackerParams& tracker)
    : rectifier_(camera), detector_(detector), tracker_(tracker) {
  const std::size_t n = static_cast<std::size_t>(camera.width) * camera.height;
  mono_.resize(n);
  rect_.resize(n);
}

ConstImageView Pipeline::rectified() const {
  const auto& m = rectifier_.model();
  return {rect_.data(), m.width, m.height, static_cast<std::size_t>(m.width)};
}

std::vector<std::uint8_t> Pipeline::process(View<const std::uint8_t> yuyv, std::uint32_t sequence,
                                            std::uint64_t stamp_ns) {
  const auto& m = rectifier_.model();
  const auto w = static_cast<std::size_t>(m.width);
  ImageView mono{mono_.data(), m.width, m.height, w};
  ImageView rect{rect_.data(), m.width, m.height, w};
  yuyv_to_mono(yuyv, mono);
  rectifier_.apply({mono_.data(), m.width, m.height, w}, rect);
  detections_ = detector_.detect(rectified());
  tracker_.step(stamp_ns, detections_);
  return telemetry::encode(sequence, stamp_ns, tracker_.confirmed());
}

}  // namespace opr::core
