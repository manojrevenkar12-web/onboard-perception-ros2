// Measures what the tracker (visionbox 2.3's behaviour, unchanged) does on every recorded flight,
// against the simulator's ground truth. This characterises the starting point of the migration;
// it is not a claim that the tracker is good.
//
//   recall        share of target-frames (target visible and at least 8 px inside the image) that
//                 have a confirmed track within 4 px
//   ghost frames  confirmed track-frames that match no target (merged blobs, clutter)
//   id switches   times a target's matched track id changed
//   error         distance between matched track and target, px
//   confirm delay frames from a target's first appearance to its first match
//
// Usage: opr_evaluate <out.json> [<out.md>]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <vector>

#include "opr_core/assign.hpp"
#include "opr_core/pipeline.hpp"
#include "opr_sim/scene.hpp"

namespace {

struct Totals {
  long target_frames = 0, matched = 0, ghost_frames = 0, track_frames = 0, id_switches = 0;
  std::vector<double> errors;
  std::vector<double> confirm_delays;
};

double pct(std::vector<double> v, double p) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1) + 0.5)];
}

double mean(const std::vector<double>& v) {
  if (v.empty()) return 0;
  double s = 0;
  for (double x : v) s += x;
  return s / static_cast<double>(v.size());
}

void evaluate(const std::string& scenario, int seed, Totals& t) {
  opr::sim::Scene scene(scenario, static_cast<std::uint64_t>(seed));
  opr::core::Pipeline pipeline({}, {}, {});
  const auto& cal = scene.calibration();
  std::map<std::uint32_t, std::uint32_t> last_id;      // target -> last matched track
  std::map<std::uint32_t, std::uint32_t> first_seen;   // target -> first evaluable frame
  std::map<std::uint32_t, bool> confirmed_once;
  opr::sim::Frame f;
  while (scene.next(f)) {
    pipeline.process({f.yuyv.data(), cal.width, cal.height, static_cast<std::size_t>(cal.width) * 2}, f.sequence, f.stamp_ns);
    const auto tracks = pipeline.tracker().confirmed();
    std::vector<const opr::sim::Target*> targets;
    for (const auto& g : f.truth) {
      if (g.visible && g.x >= 8 && g.y >= 8 && g.x <= cal.width - 8 && g.y <= cal.height - 8) targets.push_back(&g);
    }
    t.target_frames += static_cast<long>(targets.size());
    t.track_frames += static_cast<long>(tracks.size());
    for (const auto* g : targets) first_seen.emplace(g->id, f.sequence);
    std::vector<int> track_of(targets.size(), -1);
    std::vector<char> track_used(tracks.size(), 0);
    if (!targets.empty() && !tracks.empty()) {
      const std::size_t n = std::max(targets.size(), tracks.size());
      std::vector<std::vector<double>> cost(n, std::vector<double>(n, 1e6));
      for (std::size_t i = 0; i < targets.size(); ++i) {
        for (std::size_t j = 0; j < tracks.size(); ++j) {
          const double d = std::hypot(tracks[j].x[0] - targets[i]->x, tracks[j].x[1] - targets[i]->y);
          if (d <= 4.0) cost[i][j] = d;
        }
      }
      const auto a = opr::core::hungarian(cost);
      for (std::size_t i = 0; i < targets.size(); ++i) {
        const int j = a[i];
        if (j >= 0 && static_cast<std::size_t>(j) < tracks.size() && cost[i][j] < 1e6) {
          track_of[i] = j;
          track_used[j] = 1;
        }
      }
    }
    for (std::size_t i = 0; i < targets.size(); ++i) {
      if (track_of[i] < 0) continue;
      const auto& tr = tracks[static_cast<std::size_t>(track_of[i])];
      const auto id = targets[i]->id;
      ++t.matched;
      t.errors.push_back(std::hypot(tr.x[0] - targets[i]->x, tr.x[1] - targets[i]->y));
      auto it = last_id.find(id);
      if (it != last_id.end() && it->second != tr.id) ++t.id_switches;
      last_id[id] = tr.id;
      if (!confirmed_once[id]) {
        confirmed_once[id] = true;
        t.confirm_delays.push_back(static_cast<double>(f.sequence - first_seen[id]));
      }
    }
    for (std::size_t j = 0; j < tracks.size(); ++j) {
      if (!track_used[j]) ++t.ghost_frames;
    }
  }
}

std::string row_json(const std::string& name, const Totals& t) {
  std::ostringstream o;
  o << std::fixed << std::setprecision(4);
  o << "    \"" << name << "\": {\"target_frames\": " << t.target_frames << ", \"recall\": "
    << (t.target_frames ? static_cast<double>(t.matched) / t.target_frames : 0.0) << ", \"track_frames\": "
    << t.track_frames << ", \"ghost_frames\": " << t.ghost_frames << ", \"id_switches\": " << t.id_switches
    << ", \"error_px_mean\": " << mean(t.errors) << ", \"error_px_p95\": " << pct(t.errors, 0.95)
    << ", \"confirm_delay_frames_mean\": " << mean(t.confirm_delays) << "}";
  return o.str();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <out.json> [<out.md>]\n", argv[0]);
    return 2;
  }
  std::vector<std::pair<std::string, Totals>> rows;
  Totals all;
  for (const auto& s : opr::sim::scenario_names()) {
    Totals t;
    for (int seed = 1; seed <= 3; ++seed) evaluate(s, seed, t);
    all.target_frames += t.target_frames;
    all.matched += t.matched;
    all.ghost_frames += t.ghost_frames;
    all.track_frames += t.track_frames;
    all.id_switches += t.id_switches;
    all.errors.insert(all.errors.end(), t.errors.begin(), t.errors.end());
    all.confirm_delays.insert(all.confirm_delays.end(), t.confirm_delays.begin(), t.confirm_delays.end());
    rows.emplace_back(s, t);
  }
  rows.emplace_back("all", all);
  std::ofstream json(argv[1]);
  json << "{\n  \"flights_per_scenario\": 3,\n  \"scenarios\": {\n";
  for (std::size_t i = 0; i < rows.size(); ++i) json << row_json(rows[i].first, rows[i].second) << (i + 1 < rows.size() ? ",\n" : "\n");
  json << "  }\n}\n";
  if (argc > 2) {
    std::ofstream md(argv[2]);
    md << std::fixed << std::setprecision(2);
    md << "# Tracking on the recorded flights (visionbox 2.3 behaviour)\n\n"
       << "Generated by `opr_evaluate`; 3 flights per scenario, 640x480 at 30 Hz, ground truth from the simulator.\n\n"
       << "| Scenario | Target-frames | Recall | Ghost track-frames | ID switches | Error mean / p95 (px) | Frames to confirm |\n"
       << "|---|---:|---:|---:|---:|---:|---:|\n";
    for (const auto& [name, t] : rows) {
      md << "| " << (name == "all" ? "**all**" : name) << " | " << t.target_frames << " | "
         << (t.target_frames ? 100.0 * t.matched / t.target_frames : 0.0) << "% | " << t.ghost_frames << " of "
         << t.track_frames << " | " << t.id_switches << " | " << mean(t.errors) << " / " << pct(t.errors, 0.95)
         << " | " << mean(t.confirm_delays) << " |\n";
    }
  }
  return 0;
}
