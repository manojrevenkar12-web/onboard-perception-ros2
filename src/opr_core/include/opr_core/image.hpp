#pragma once

#include <cstddef>
#include <cstdint>

namespace opr::core {

/// A non-owning view of an 8-bit image. `step` is the distance in bytes between rows, so a view
/// can point into a camera buffer, a shared-memory slot or a ROS message without copying.
template <typename T>
struct View {
  T* data = nullptr;
  int width = 0;
  int height = 0;
  std::size_t step = 0;

  T* row(int y) const { return data + static_cast<std::size_t>(y) * step; }
};

using ImageView = View<std::uint8_t>;
using ConstImageView = View<const std::uint8_t>;

/// Copies the luma channel of a YUYV (YUV 4:2:2, Y0 U Y1 V) image into `dst` (mono8).
void yuyv_to_mono(View<const std::uint8_t> yuyv, ImageView dst);

}  // namespace opr::core
