#include "opr_core/track.hpp"

#include <algorithm>
#include <cstring>

#include "opr_core/assign.hpp"

namespace opr::core {

// The arithmetic below keeps visionbox 2.3's operation order exactly; the equivalence tests
// compare the two bit for bit. Restructure the data, not the sums.

void Tracker::predict(Track& t, double dt) const {
  t.x[0] += t.x[2] * dt;
  t.x[1] += t.x[3] * dt;
  const double F[4][4] = {{1, 0, dt, 0}, {0, 1, 0, dt}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  double FP[4][4];
  double FPFt[4][4];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double s = 0;
      for (int k = 0; k < 4; ++k) s += F[i][k] * t.P[k][j];
      FP[i][j] = s;
    }
  }
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double s = 0;
      for (int k = 0; k < 4; ++k) s += FP[i][k] * F[j][k];
      FPFt[i][j] = s;
    }
  }
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double q = params_.accel_noise;
  const double Q[4][4] = {{dt4 / 4 * q, 0, dt3 / 2 * q, 0},
                          {0, dt4 / 4 * q, 0, dt3 / 2 * q},
                          {dt3 / 2 * q, 0, dt2 * q, 0},
                          {0, dt3 / 2 * q, 0, dt2 * q}};
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) t.P[i][j] = FPFt[i][j] + Q[i][j];
  }
}

double Tracker::mahalanobis2(const Track& t, const Detection& d) const {
  const double r = params_.meas_noise;
  const double s00 = t.P[0][0] + r, s01 = t.P[0][1], s10 = t.P[1][0], s11 = t.P[1][1] + r;
  const double det = s00 * s11 - s01 * s10;
  const double y0 = d.x - t.x[0], y1 = d.y - t.x[1];
  const double i00 = s11 / det, i01 = -s01 / det, i10 = -s10 / det, i11 = s00 / det;
  return y0 * (i00 * y0 + i01 * y1) + y1 * (i10 * y0 + i11 * y1);
}

void Tracker::update(Track& t, const Detection& d) const {
  const double r = params_.meas_noise;
  const double s00 = t.P[0][0] + r, s01 = t.P[0][1], s10 = t.P[1][0], s11 = t.P[1][1] + r;
  const double det = s00 * s11 - s01 * s10;
  const double i00 = s11 / det, i01 = -s01 / det, i10 = -s10 / det, i11 = s00 / det;
  double K[4][2];
  for (int row = 0; row < 4; ++row) {
    K[row][0] = t.P[row][0] * i00 + t.P[row][1] * i10;
    K[row][1] = t.P[row][0] * i01 + t.P[row][1] * i11;
  }
  const double y0 = d.x - t.x[0], y1 = d.y - t.x[1];
  for (int row = 0; row < 4; ++row) t.x[row] += K[row][0] * y0 + K[row][1] * y1;
  double NP[4][4];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) NP[i][j] = t.P[i][j] - (K[i][0] * t.P[0][j] + K[i][1] * t.P[1][j]);
  }
  std::memcpy(t.P, NP, sizeof(NP));
  t.hits++;
  t.misses = 0;
}

StepResult Tracker::step(std::uint64_t stamp_ns, const std::vector<Detection>& dets) {
  StepResult result;
  // visionbox 2.3 subtracted unsigned stamps, so a stamp going backwards became a dt of 584
  // years. Ignoring the frame is the fix (docs/MIGRATION.md, finding 3).
  if (last_stamp_ != 0 && stamp_ns <= last_stamp_) {
    result.accepted = false;
    return result;
  }
  const double dt = last_stamp_ == 0 ? 0.0 : static_cast<double>(stamp_ns - last_stamp_) * 1e-9;
  last_stamp_ = stamp_ns;
  if (dt > 0) {
    for (auto& t : tracks_) predict(t, dt);
  }

  const double big = 1e6;
  const std::size_t nt = tracks_.size();
  const std::size_t nd = dets.size();
  std::vector<int> det_of(nt, -1);
  std::vector<char> det_used(nd, 0);
  if (nt > 0 && nd > 0) {
    const std::size_t n = std::max(nt, nd);
    std::vector<std::vector<double>> cost(n, std::vector<double>(n, big));
    for (std::size_t i = 0; i < nt; ++i) {
      for (std::size_t j = 0; j < nd; ++j) {
        const double m = mahalanobis2(tracks_[i], dets[j]);
        if (m <= params_.gate) cost[i][j] = m;
      }
    }
    const std::vector<int> a = hungarian(cost);
    for (std::size_t i = 0; i < nt; ++i) {
      const int j = a[i];
      if (j >= 0 && static_cast<std::size_t>(j) < nd && cost[i][j] < big) {
        det_of[i] = j;
        det_used[j] = 1;
      }
    }
  }
  for (std::size_t i = 0; i < nt; ++i) {
    if (det_of[i] >= 0) {
      update(tracks_[i], dets[det_of[i]]);
      ++result.assigned;
    } else {
      tracks_[i].misses++;
    }
  }
  std::vector<Track> keep;
  keep.reserve(tracks_.size() + nd);
  for (auto& t : tracks_) {
    const bool confirmed = t.confirmed(params_);
    if ((!confirmed && t.misses > 0) || (confirmed && t.misses > params_.max_misses)) {
      ++result.dropped;
      continue;
    }
    keep.push_back(t);
  }
  tracks_.swap(keep);
  for (std::size_t j = 0; j < nd; ++j) {
    if (det_used[j]) continue;
    Track t;
    t.id = next_id_++;
    t.x[0] = dets[j].x;
    t.x[1] = dets[j].y;
    t.P[0][0] = params_.init_pos_var;
    t.P[1][1] = params_.init_pos_var;
    t.P[2][2] = params_.init_vel_var;
    t.P[3][3] = params_.init_vel_var;
    t.hits = 1;
    tracks_.push_back(t);
    ++result.born;
  }
  return result;
}

std::vector<Track> Tracker::confirmed() const {
  std::vector<Track> out;
  for (const auto& t : tracks_) {
    if (t.confirmed(params_)) out.push_back(t);
  }
  std::stable_sort(out.begin(), out.end(), [](const Track& a, const Track& b) { return a.id < b.id; });
  return out;
}

void Tracker::reset() {
  tracks_.clear();
  next_id_ = 1;
  last_stamp_ = 0;
}

}  // namespace opr::core
