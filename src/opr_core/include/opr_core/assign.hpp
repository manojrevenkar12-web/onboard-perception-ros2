#pragma once

#include <vector>

namespace opr::core {

/// Minimum-cost assignment (Hungarian algorithm, O(n^2 m)) for an n x m cost matrix with n <= m.
/// Returns, for each row, the column it is assigned to.
std::vector<int> hungarian(const std::vector<std::vector<double>>& cost);

}  // namespace opr::core
