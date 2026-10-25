#pragma once

#include <cstdint>
#include <vector>

#include "opr_core/image.hpp"
#include "opr_core/params.hpp"

namespace opr::core {

/// Maps an ideal (undistorted) pixel to where the lens images it (Brown-Conrady model).
void distort_point(const CameraModel& m, double xu, double yu, double& xd, double& yd);

/// Undistorts mono8 images with a precomputed table and 8-bit bilinear weights.
/// Building the table costs about one frame; applying it is one pass with no floating point.
class Rectifier {
 public:
  explicit Rectifier(const CameraModel& model);

  const CameraModel& model() const { return model_; }

  /// `src` and `dst` must both be model().width x model().height. Pixels whose source falls
  /// outside the sensor are written as 0.
  void apply(ConstImageView src, ImageView dst) const;

 private:
  CameraModel model_;
  std::vector<std::int32_t> offset_;  // y0 * width + x0 of the top-left source pixel, or -1
  std::vector<std::uint16_t> wx_;     // 0..256
  std::vector<std::uint16_t> wy_;
  std::vector<std::int32_t> row_;     // y0 per output pixel (offsets are relative to a dense image)
};

}  // namespace opr::core
