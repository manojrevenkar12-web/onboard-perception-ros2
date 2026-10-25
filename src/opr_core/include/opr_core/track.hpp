#pragma once

#include <cstdint>
#include <vector>

#include "opr_core/detect.hpp"
#include "opr_core/params.hpp"

namespace opr::core {

struct Track {
  std::uint32_t id = 0;
  double x[4] = {0, 0, 0, 0};  // position (px) and velocity (px/s) in rectified image coordinates
  double P[4][4] = {};
  int hits = 0;
  int misses = 0;

  bool confirmed(const TrackerParams& p) const { return hits >= p.confirm_hits; }
};

struct StepResult {
  bool accepted = true;  // false if the stamp did not move forward; the frame was ignored
  int assigned = 0;
  int born = 0;
  int dropped = 0;
};

/// Constant-velocity Kalman tracker with gated Hungarian association.
/// Tentative tracks die on their first miss; confirmed tracks coast for max_misses frames.
class Tracker {
 public:
  explicit Tracker(TrackerParams params = {}) : params_(params) {}

  const TrackerParams& params() const { return params_; }

  StepResult step(std::uint64_t stamp_ns, const std::vector<Detection>& detections);

  /// Confirmed tracks sorted by id (what the ground station is told about).
  std::vector<Track> confirmed() const;
  const std::vector<Track>& all() const { return tracks_; }

  void reset();

 private:
  void predict(Track& t, double dt) const;
  double mahalanobis2(const Track& t, const Detection& d) const;
  void update(Track& t, const Detection& d) const;

  TrackerParams params_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_ = 1;
  std::uint64_t last_stamp_ = 0;
};

}  // namespace opr::core
