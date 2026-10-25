#pragma once

#include <cstdint>
#include <vector>

#include "opr_core/image.hpp"
#include "opr_core/params.hpp"

namespace opr::core {

struct Detection {
  double x = 0.0;  // full-resolution pixel coordinates of the contrast-weighted centroid
  double y = 0.0;
  double strength = 0.0;  // mean contrast above the local background
  int area = 0;           // half-resolution pixels
};

/// Finds small bright blobs against a slowly varying background:
/// 2x2 box downscale, local mean by integral image, threshold, 8-connected labelling.
/// Allocates once per image size; detect() itself does not allocate after the first call
/// except for the output.
class Detector {
 public:
  explicit Detector(DetectorParams params = {}) : params_(params) {}

  const DetectorParams& params() const { return params_; }
  void set_params(const DetectorParams& p) { params_ = p; }

  /// Detections in order of first appearance in raster order (deterministic).
  std::vector<Detection> detect(ConstImageView rectified);

 private:
  void resize(int width, int height);
  int local_mean(int x, int y) const;
  std::int32_t root(std::int32_t a);

  DetectorParams params_;
  int hw_ = 0;
  int hh_ = 0;
  std::vector<std::uint8_t> half_;
  std::vector<std::uint32_t> integral_;
  std::vector<int> contrast_;
  std::vector<std::int32_t> labels_;
  std::vector<std::int32_t> parent_;
};

}  // namespace opr::core
