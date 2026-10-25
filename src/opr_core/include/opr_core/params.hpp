// Parameters of the perception chain. Defaults are the legacy visionbox.ini field values; the ROS 2
// nodes declare them as parameters with the same names (config/pipeline.yaml).
#pragma once

namespace opr::core {

struct CameraModel {
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

struct DetectorParams {
  int bg_radius = 8;   // half-resolution pixels around each pixel that define its background
  int threshold = 18;  // grey levels above that background
  int min_area = 4;    // half-resolution pixels
  int max_area = 400;
};

struct TrackerParams {
  double gate = 13.8;  // squared Mahalanobis distance, chi-square 2 dof at 99.9%
  double accel_noise = 2000.0;
  double meas_noise = 1.0;
  double init_pos_var = 4.0;
  double init_vel_var = 90000.0;
  int confirm_hits = 3;
  int max_misses = 5;
};

}  // namespace opr::core
