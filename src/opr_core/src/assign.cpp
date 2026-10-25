#include "opr_core/assign.hpp"

#include <stdexcept>

namespace opr::core {

std::vector<int> hungarian(const std::vector<std::vector<double>>& a) {
  const int n = static_cast<int>(a.size());
  const int m = n ? static_cast<int>(a[0].size()) : 0;
  if (n > m) throw std::invalid_argument("hungarian: more rows than columns");
  const double inf = 1e18;
  std::vector<double> u(n + 1, 0), v(m + 1, 0), minv(m + 1);
  std::vector<int> p(m + 1, 0), way(m + 1, 0);
  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    minv.assign(m + 1, inf);
    std::vector<char> used(m + 1, 0);
    do {
      used[j0] = 1;
      const int i0 = p[j0];
      int j1 = 0;
      double delta = inf;
      for (int j = 1; j <= m; ++j) {
        if (used[j]) continue;
        const double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }
  std::vector<int> row_to_col(n, -1);
  for (int j = 1; j <= m; ++j) {
    if (p[j] != 0) row_to_col[p[j] - 1] = j - 1;
  }
  return row_to_col;
}

}  // namespace opr::core
