#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>

#include "opr_core/assign.hpp"
#include "opr_sim/scene.hpp"

TEST(Hungarian, MatchesBruteForceOnRandomMatrices) {
  opr::sim::Rng rng(99);
  for (int trial = 0; trial < 300; ++trial) {
    const int n = 1 + rng.uniform(0, 5);
    const int m = n + rng.uniform(0, 2);
    std::vector<std::vector<double>> c(n, std::vector<double>(m));
    for (auto& row : c) {
      for (auto& v : row) v = rng.uniform(0, 50) + 0.25 * rng.uniform(0, 3);
    }
    const auto a = opr::core::hungarian(c);
    double got = 0;
    for (int i = 0; i < n; ++i) got += c[i][a[i]];
    std::vector<int> cols(m);
    std::iota(cols.begin(), cols.end(), 0);
    double best = 1e18;
    do {
      double s = 0;
      for (int i = 0; i < n; ++i) s += c[i][cols[i]];
      best = std::min(best, s);
    } while (std::next_permutation(cols.begin(), cols.end()));
    ASSERT_DOUBLE_EQ(got, best) << "trial " << trial;
    std::vector<int> used(a.begin(), a.end());
    std::sort(used.begin(), used.end());
    EXPECT_TRUE(std::adjacent_find(used.begin(), used.end()) == used.end());
  }
}

TEST(Hungarian, RejectsMoreRowsThanColumns) {
  EXPECT_THROW(opr::core::hungarian({{1, 2}, {3, 4}, {5, 6}}), std::invalid_argument);
}

TEST(Hungarian, EmptyIsEmpty) { EXPECT_TRUE(opr::core::hungarian({}).empty()); }
