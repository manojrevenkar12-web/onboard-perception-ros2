// Deterministic camera simulator: renders YUYV frames of moving targets seen through a lens with
// radial and tangential distortion, and keeps the ground truth for every frame.
//
// Everything here uses integer arithmetic and the four basic IEEE operations only (no sin, exp,
// pow): a frame rendered on an x86 laptop is bit-identical to one rendered on an aarch64 Jetson,
// which is what lets recorded outputs be compared byte for byte across machines.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace opr::sim {

/// Pinhole intrinsics plus Brown-Conrady distortion (k1, k2, k3 radial; p1, p2 tangential).
struct Calibration {
  int width = 640;
  int height = 480;
  double fx = 520.0;
  double fy = 520.0;
  double cx = 319.5;
  double cy = 239.5;
  double k1 = -0.28;
  double k2 = 0.09;
  double p1 = 0.0006;
  double p2 = -0.0004;
  double k3 = -0.012;
};

/// One target in the world, in ideal (undistorted) pixel coordinates.
struct Target {
  std::uint32_t id = 0;
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;  // pixels per second
  double vy = 0.0;
  double radius = 6.0;
  int contrast = 90;  // grey levels above the local background
  bool visible = true;
};

struct Frame {
  std::uint32_t sequence = 0;
  std::uint64_t stamp_ns = 0;          // capture time on the camera's monotonic clock
  std::vector<std::uint8_t> yuyv;      // width * height * 2 bytes, Y0 U Y1 V
  std::vector<Target> truth;           // targets as they were when this frame was exposed
  std::vector<std::uint32_t> flashes;  // number of one-frame clutter flashes drawn (for evaluation)
};

/// Names of the built-in scenarios. Each is a short flight segment with a known difficulty.
std::vector<std::string> scenario_names();

/// Small deterministic PRNG (SplitMix64). std:: distributions differ between standard libraries.
class Rng {
 public:
  explicit Rng(std::uint64_t seed) : state_(seed) {}
  std::uint64_t next();
  /// Uniform integer in [lo, hi].
  int uniform(int lo, int hi);
  /// Uniform double in [0, 1) with 53 random bits.
  double unit();

 private:
  std::uint64_t state_;
};

class Scene {
 public:
  /// Throws std::invalid_argument for an unknown scenario name.
  Scene(const std::string& scenario, std::uint64_t seed, Calibration calib = {});

  const Calibration& calibration() const { return calib_; }
  const std::string& scenario() const { return scenario_; }
  int frame_count() const { return frame_count_; }
  double fps() const { return fps_; }

  /// Renders the next frame. Returns false after the last frame.
  bool next(Frame& out);

  /// Renders the frame with this sequence number without advancing (used by the fake camera,
  /// which may be asked for frames at its own pace).
  void render(std::uint32_t sequence, Frame& out) const;

 private:
  struct Event {
    std::uint32_t target_id;
    std::uint32_t from;  // first frame
    std::uint32_t to;    // last frame (inclusive)
  };

  std::vector<Target> targets_at(std::uint32_t sequence) const;
  int background(double x, double y) const;
  int gain_permille(std::uint32_t sequence) const;

  std::string scenario_;
  std::uint64_t seed_;
  Calibration calib_;
  int frame_count_ = 0;
  int next_sequence_ = 0;
  double fps_ = 30.0;
  int sensor_noise_ = 3;
  int flashes_per_frame_ = 0;
  bool exposure_ramp_ = false;
  std::vector<Target> start_;
  std::vector<Event> occlusions_;
  // Distorted pixel -> ideal pixel coordinates, computed once (row major, x then y).
  std::vector<double> ideal_x_;
  std::vector<double> ideal_y_;
  std::vector<int> background_;  // static scene, before exposure gain and noise
};

/// Maps an ideal (undistorted) pixel to where the lens images it. Basic operations only.
void distort(const Calibration& c, double xu, double yu, double& xd, double& yd);

/// Inverse of distort() by fixed-point iteration (20 rounds). Basic operations only.
void undistort(const Calibration& c, double xd, double yd, double& xu, double& yu);

}  // namespace opr::sim
