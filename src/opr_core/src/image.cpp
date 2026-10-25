#include "opr_core/image.hpp"

namespace opr::core {

void yuyv_to_mono(View<const std::uint8_t> yuyv, ImageView dst) {
  for (int y = 0; y < dst.height; ++y) {
    const std::uint8_t* s = yuyv.row(y);
    std::uint8_t* d = dst.row(y);
    for (int x = 0; x < dst.width; ++x) d[x] = s[2 * x];
  }
}

}  // namespace opr::core
