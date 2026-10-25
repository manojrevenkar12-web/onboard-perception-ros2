// visionbox 2.3 — onboard target tracker (LEGACY, frozen).
//
// This is the program the migration started from, kept as it was so that its behaviour can be
// replayed and compared with the ROS 2 pipeline at any time (see docs/MIGRATION.md). It is one
// file with global state: capture with read(), undistortion, blob detection, a Kalman tracker and
// a UDP telemetry stream that the ground station still parses. Do not fix bugs here; record them in
// docs/MIGRATION.md and fix them in opr_core behind a flag that the equivalence tests understand.
//
// Usage:
//   visionbox --config visionbox.ini [--device /dev/video0 | --replay scenario:seed]
//             [--udp 127.0.0.1:14560] [--record out.hex]
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "opr_sim/scene.hpp"

// ---------------------------------------------------------------- configuration (globals)
static int W = 640, H = 480;
static double FX = 520.0, FY = 520.0, CX = 319.5, CY = 239.5;
static double K1 = -0.28, K2 = 0.09, P1 = 0.0006, P2 = -0.0004, K3 = -0.012;
static int BG_RADIUS = 8;      // half-resolution pixels
static int THRESHOLD = 18;     // grey levels above local mean
static int MIN_AREA = 4;       // half-resolution pixels
static int MAX_AREA = 400;
static double GATE = 13.8;     // chi-square, 2 dof
static double ACCEL_NOISE = 2000.0;
static double MEAS_NOISE = 1.0;
static double INIT_POS_VAR = 4.0;
static double INIT_VEL_VAR = 90000.0;
static int CONFIRM_HITS = 3;
static int MAX_MISSES = 5;

static void load_config(const char* path) {
  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "visionbox: cannot read %s, using defaults\n", path);
    return;
  }
  std::map<std::string, std::string> kv;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#' || line[0] == ';' || line[0] == '[') continue;
    auto eq = line.find('=');
    if (eq == std::string::npos) continue;
    std::string k = line.substr(0, eq), v = line.substr(eq + 1);
    while (!k.empty() && k.back() == ' ') k.pop_back();
    while (!v.empty() && v.front() == ' ') v.erase(0, 1);
    kv[k] = v;
  }
  auto d = [&](const char* k, double& x) { if (kv.count(k)) x = std::atof(kv[k].c_str()); };
  auto i = [&](const char* k, int& x) { if (kv.count(k)) x = std::atoi(kv[k].c_str()); };
  i("width", W); i("height", H);
  d("fx", FX); d("fy", FY); d("cx", CX); d("cy", CY);
  d("k1", K1); d("k2", K2); d("p1", P1); d("p2", P2); d("k3", K3);
  i("bg_radius", BG_RADIUS); i("threshold", THRESHOLD);
  i("min_area", MIN_AREA); i("max_area", MAX_AREA);
  d("gate", GATE); d("accel_noise", ACCEL_NOISE); d("meas_noise", MEAS_NOISE);
  d("init_pos_var", INIT_POS_VAR); d("init_vel_var", INIT_VEL_VAR);
  i("confirm_hits", CONFIRM_HITS); i("max_misses", MAX_MISSES);
}

// ---------------------------------------------------------------- undistortion table (globals)
static std::vector<int32_t> g_lut_off;   // -1 = outside the source image
static std::vector<uint16_t> g_lut_wx, g_lut_wy;

static void build_lut() {
  g_lut_off.assign((size_t)W * H, -1);
  g_lut_wx.assign((size_t)W * H, 0);
  g_lut_wy.assign((size_t)W * H, 0);
  for (int v = 0; v < H; v++) {
    for (int u = 0; u < W; u++) {
      double x = (u - CX) / FX;
      double y = (v - CY) / FY;
      double r2 = x * x + y * y;
      double radial = 1.0 + r2 * (K1 + r2 * (K2 + r2 * K3));
      double dx = 2.0 * P1 * x * y + P2 * (r2 + 2.0 * x * x);
      double dy = P1 * (r2 + 2.0 * y * y) + 2.0 * P2 * x * y;
      double xd = (x * radial + dx) * FX + CX;
      double yd = (y * radial + dy) * FY + CY;
      double fx0 = std::floor(xd), fy0 = std::floor(yd);
      int x0 = (int)fx0, y0 = (int)fy0;
      if (x0 < 0 || y0 < 0 || x0 + 1 >= W || y0 + 1 >= H) continue;
      size_t i = (size_t)v * W + u;
      g_lut_off[i] = y0 * W + x0;
      g_lut_wx[i] = (uint16_t)((xd - fx0) * 256.0 + 0.5);
      g_lut_wy[i] = (uint16_t)((yd - fy0) * 256.0 + 0.5);
    }
  }
}

// ---------------------------------------------------------------- image buffers (globals)
static std::vector<uint8_t> g_raw;     // YUYV from the camera
static std::vector<uint8_t> g_grey;    // Y plane
static std::vector<uint8_t> g_rect;    // undistorted
static std::vector<uint8_t> g_half;    // half resolution
static std::vector<uint32_t> g_integral;
static std::vector<int32_t> g_labels;
static std::vector<int32_t> g_parent;

static void undistort_frame() {
  for (size_t i = 0; i < g_grey.size(); i++) g_grey[i] = g_raw[i * 2];
  for (size_t i = 0; i < g_rect.size(); i++) {
    int32_t off = g_lut_off[i];
    if (off < 0) { g_rect[i] = 0; continue; }
    uint32_t wx = g_lut_wx[i], wy = g_lut_wy[i];
    const uint8_t* p = &g_grey[off];
    uint32_t top = p[0] * (256 - wx) + p[1] * wx;
    uint32_t bot = p[W] * (256 - wx) + p[W + 1] * wx;
    g_rect[i] = (uint8_t)((top * (256 - wy) + bot * wy + 32768) >> 16);
  }
}

// ---------------------------------------------------------------- detection
struct Det { double x, y, strength; int area; };
static std::vector<Det> g_dets;

static int32_t find_root(int32_t a) {
  while (g_parent[a] != a) { g_parent[a] = g_parent[g_parent[a]]; a = g_parent[a]; }
  return a;
}

static void detect() {
  int hw = W / 2, hh = H / 2;
  for (int y = 0; y < hh; y++)
    for (int x = 0; x < hw; x++) {
      const uint8_t* p = &g_rect[(size_t)(2 * y) * W + 2 * x];
      g_half[(size_t)y * hw + x] = (uint8_t)((p[0] + p[1] + p[W] + p[W + 1] + 2) >> 2);
    }
  // integral image with a zero row and column
  int iw = hw + 1;
  for (int x = 0; x < iw; x++) g_integral[x] = 0;
  for (int y = 1; y <= hh; y++) {
    uint32_t row = 0;
    g_integral[(size_t)y * iw] = 0;
    for (int x = 1; x <= hw; x++) {
      row += g_half[(size_t)(y - 1) * hw + (x - 1)];
      g_integral[(size_t)y * iw + x] = g_integral[(size_t)(y - 1) * iw + x] + row;
    }
  }
  // threshold against the local mean and label (two-pass, 8-connected)
  g_parent.clear();
  for (int y = 0; y < hh; y++) {
    for (int x = 0; x < hw; x++) {
      int x0 = x - BG_RADIUS < 0 ? 0 : x - BG_RADIUS;
      int y0 = y - BG_RADIUS < 0 ? 0 : y - BG_RADIUS;
      int x1 = x + BG_RADIUS >= hw ? hw - 1 : x + BG_RADIUS;
      int y1 = y + BG_RADIUS >= hh ? hh - 1 : y + BG_RADIUS;
      uint32_t sum = g_integral[(size_t)(y1 + 1) * iw + (x1 + 1)] - g_integral[(size_t)y0 * iw + (x1 + 1)] -
                     g_integral[(size_t)(y1 + 1) * iw + x0] + g_integral[(size_t)y0 * iw + x0];
      uint32_t count = (uint32_t)((x1 - x0 + 1) * (y1 - y0 + 1));
      int bg = (int)((sum + count / 2) / count);
      size_t i = (size_t)y * hw + x;
      if ((int)g_half[i] - bg < THRESHOLD) { g_labels[i] = -1; continue; }
      int32_t best = -1;
      const int nx[4] = {-1, -1, 0, 1}, ny[4] = {0, -1, -1, -1};
      for (int k = 0; k < 4; k++) {
        int xx = x + nx[k], yy = y + ny[k];
        if (xx < 0 || yy < 0 || xx >= hw) continue;
        int32_t l = g_labels[(size_t)yy * hw + xx];
        if (l < 0) continue;
        l = find_root(l);
        if (best < 0) best = l;
        else if (l != best) {
          if (l < best) { g_parent[best] = l; best = l; } else { g_parent[l] = best; }
        }
      }
      if (best < 0) { best = (int32_t)g_parent.size(); g_parent.push_back(best); }
      g_labels[i] = best;
    }
  }
  // second pass: accumulate per root, in order of first appearance
  struct Acc { int64_t w, wx, wy; int area; };
  std::vector<Acc> acc(g_parent.size(), Acc{0, 0, 0, 0});
  std::vector<int32_t> order;
  std::vector<char> seen(g_parent.size(), 0);
  for (int y = 0; y < hh; y++) {
    for (int x = 0; x < hw; x++) {
      size_t i = (size_t)y * hw + x;
      if (g_labels[i] < 0) continue;
      int32_t r = find_root(g_labels[i]);
      if (!seen[r]) { seen[r] = 1; order.push_back(r); }
      int x0 = x - BG_RADIUS < 0 ? 0 : x - BG_RADIUS;
      int y0 = y - BG_RADIUS < 0 ? 0 : y - BG_RADIUS;
      int x1 = x + BG_RADIUS >= hw ? hw - 1 : x + BG_RADIUS;
      int y1 = y + BG_RADIUS >= hh ? hh - 1 : y + BG_RADIUS;
      uint32_t sum = g_integral[(size_t)(y1 + 1) * iw + (x1 + 1)] - g_integral[(size_t)y0 * iw + (x1 + 1)] -
                     g_integral[(size_t)(y1 + 1) * iw + x0] + g_integral[(size_t)y0 * iw + x0];
      uint32_t count = (uint32_t)((x1 - x0 + 1) * (y1 - y0 + 1));
      int bg = (int)((sum + count / 2) / count);
      int64_t w = (int)g_half[i] - bg;
      acc[r].w += w; acc[r].wx += w * x; acc[r].wy += w * y; acc[r].area++;
    }
  }
  g_dets.clear();
  for (int32_t r : order) {
    if (acc[r].area < MIN_AREA || acc[r].area > MAX_AREA) continue;
    Det d;
    d.x = (double)acc[r].wx / (double)acc[r].w * 2.0 + 0.5;
    d.y = (double)acc[r].wy / (double)acc[r].w * 2.0 + 0.5;
    d.strength = (double)acc[r].w / (double)acc[r].area;
    d.area = acc[r].area;
    g_dets.push_back(d);
  }
}

// ---------------------------------------------------------------- tracking (globals)
struct Trk {
  uint32_t id;
  double x[4];
  double P[4][4];
  int hits, misses;
};
static std::vector<Trk> g_tracks;
static uint32_t g_next_id = 1;
static uint64_t g_last_stamp = 0;

static void predict(Trk& t, double dt) {
  t.x[0] += t.x[2] * dt;
  t.x[1] += t.x[3] * dt;
  double F[4][4] = {{1, 0, dt, 0}, {0, 1, 0, dt}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  double FP[4][4], FPFt[4][4];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      double s = 0;
      for (int k = 0; k < 4; k++) s += F[i][k] * t.P[k][j];
      FP[i][j] = s;
    }
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      double s = 0;
      for (int k = 0; k < 4; k++) s += FP[i][k] * F[j][k];
      FPFt[i][j] = s;
    }
  double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
  double q = ACCEL_NOISE;
  double Q[4][4] = {{dt4 / 4 * q, 0, dt3 / 2 * q, 0},
                    {0, dt4 / 4 * q, 0, dt3 / 2 * q},
                    {dt3 / 2 * q, 0, dt2 * q, 0},
                    {0, dt3 / 2 * q, 0, dt2 * q}};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) t.P[i][j] = FPFt[i][j] + Q[i][j];
}

// squared Mahalanobis distance of a detection from a track's predicted position
static double maha(const Trk& t, const Det& d) {
  double s00 = t.P[0][0] + MEAS_NOISE, s01 = t.P[0][1], s10 = t.P[1][0], s11 = t.P[1][1] + MEAS_NOISE;
  double det = s00 * s11 - s01 * s10;
  double y0 = d.x - t.x[0], y1 = d.y - t.x[1];
  double i00 = s11 / det, i01 = -s01 / det, i10 = -s10 / det, i11 = s00 / det;
  return y0 * (i00 * y0 + i01 * y1) + y1 * (i10 * y0 + i11 * y1);
}

static void update(Trk& t, const Det& d) {
  double s00 = t.P[0][0] + MEAS_NOISE, s01 = t.P[0][1], s10 = t.P[1][0], s11 = t.P[1][1] + MEAS_NOISE;
  double det = s00 * s11 - s01 * s10;
  double i00 = s11 / det, i01 = -s01 / det, i10 = -s10 / det, i11 = s00 / det;
  double K[4][2];
  for (int r = 0; r < 4; r++) {
    K[r][0] = t.P[r][0] * i00 + t.P[r][1] * i10;
    K[r][1] = t.P[r][0] * i01 + t.P[r][1] * i11;
  }
  double y0 = d.x - t.x[0], y1 = d.y - t.x[1];
  for (int r = 0; r < 4; r++) t.x[r] += K[r][0] * y0 + K[r][1] * y1;
  double NP[4][4];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) NP[i][j] = t.P[i][j] - (K[i][0] * t.P[0][j] + K[i][1] * t.P[1][j]);
  std::memcpy(t.P, NP, sizeof(NP));
  t.hits++;
  t.misses = 0;
}

// Hungarian algorithm (minimisation), n rows <= m columns, 1-based internally.
static std::vector<int> hungarian(const std::vector<std::vector<double>>& a) {
  int n = (int)a.size(), m = n ? (int)a[0].size() : 0;
  const double INF = 1e18;
  std::vector<double> u(n + 1, 0), v(m + 1, 0), minv(m + 1);
  std::vector<int> p(m + 1, 0), way(m + 1, 0);
  for (int i = 1; i <= n; i++) {
    p[0] = i;
    int j0 = 0;
    minv.assign(m + 1, INF);
    std::vector<char> used(m + 1, 0);
    do {
      used[j0] = 1;
      int i0 = p[j0], j1 = 0;
      double delta = INF;
      for (int j = 1; j <= m; j++)
        if (!used[j]) {
          double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
          if (minv[j] < delta) { delta = minv[j]; j1 = j; }
        }
      for (int j = 0; j <= m; j++)
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; } else minv[j] -= delta;
      j0 = j1;
    } while (p[j0] != 0);
    do { int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
  }
  std::vector<int> row_to_col(n, -1);
  for (int j = 1; j <= m; j++) if (p[j] != 0) row_to_col[p[j] - 1] = j - 1;
  return row_to_col;
}

static void track(uint64_t stamp_ns) {
  double dt = g_last_stamp == 0 ? 0.0 : (double)(stamp_ns - g_last_stamp) * 1e-9;
  g_last_stamp = stamp_ns;
  if (dt > 0) for (auto& t : g_tracks) predict(t, dt);

  const double BIG = 1e6;
  size_t nt = g_tracks.size(), nd = g_dets.size();
  std::vector<int> det_of(nt, -1);
  std::vector<char> det_used(nd, 0);
  if (nt > 0 && nd > 0) {
    size_t n = nt > nd ? nt : nd;
    std::vector<std::vector<double>> cost(n, std::vector<double>(n, BIG));
    for (size_t i = 0; i < nt; i++)
      for (size_t j = 0; j < nd; j++) {
        double m = maha(g_tracks[i], g_dets[j]);
        if (m <= GATE) cost[i][j] = m;
      }
    std::vector<int> a = hungarian(cost);
    for (size_t i = 0; i < nt; i++) {
      int j = a[i];
      if (j >= 0 && (size_t)j < nd && cost[i][j] < BIG) { det_of[i] = j; det_used[j] = 1; }
    }
  }
  for (size_t i = 0; i < nt; i++) {
    if (det_of[i] >= 0) update(g_tracks[i], g_dets[det_of[i]]);
    else g_tracks[i].misses++;
  }
  // drop tentative tracks on their first miss, confirmed ones after MAX_MISSES
  std::vector<Trk> keep;
  for (auto& t : g_tracks) {
    bool confirmed = t.hits >= CONFIRM_HITS;
    if (!confirmed && t.misses > 0) continue;
    if (confirmed && t.misses > MAX_MISSES) continue;
    keep.push_back(t);
  }
  g_tracks.swap(keep);
  for (size_t j = 0; j < nd; j++) {
    if (det_used[j]) continue;
    Trk t;
    t.id = g_next_id++;
    t.x[0] = g_dets[j].x; t.x[1] = g_dets[j].y; t.x[2] = 0; t.x[3] = 0;
    std::memset(t.P, 0, sizeof(t.P));
    t.P[0][0] = INIT_POS_VAR; t.P[1][1] = INIT_POS_VAR;
    t.P[2][2] = INIT_VEL_VAR; t.P[3][3] = INIT_VEL_VAR;
    t.hits = 1; t.misses = 0;
    g_tracks.push_back(t);
  }
}

// ---------------------------------------------------------------- telemetry (wire format v1)
static uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t)(p[i] << 8);
    for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

static void put(std::vector<uint8_t>& b, uint64_t v, int n) {
  for (int i = 0; i < n; i++) b.push_back((uint8_t)(v >> (8 * i)));
}
static void putf(std::vector<uint8_t>& b, float f) {
  uint32_t u;
  std::memcpy(&u, &f, 4);
  put(b, u, 4);
}

static std::vector<uint8_t> encode(uint32_t seq, uint64_t stamp) {
  std::vector<uint8_t> b;
  b.push_back('V'); b.push_back('B'); b.push_back('X'); b.push_back('1');
  uint16_t n = 0;
  for (auto& t : g_tracks) if (t.hits >= CONFIRM_HITS) n++;
  put(b, 1, 2);
  put(b, n, 2);
  put(b, seq, 4);
  put(b, stamp, 8);
  std::vector<const Trk*> out;
  for (auto& t : g_tracks) if (t.hits >= CONFIRM_HITS) out.push_back(&t);
  for (size_t i = 1; i < out.size(); i++)  // insertion sort by id
    for (size_t j = i; j > 0 && out[j - 1]->id > out[j]->id; j--) std::swap(out[j - 1], out[j]);
  for (auto* t : out) {
    put(b, t->id, 4);
    putf(b, (float)t->x[0]); putf(b, (float)t->x[1]); putf(b, (float)t->x[2]); putf(b, (float)t->x[3]);
    b.push_back((uint8_t)(t->hits > 255 ? 255 : t->hits));
    b.push_back((uint8_t)(t->misses > 0 ? 1 : 0));
  }
  put(b, crc16(b.data(), b.size()), 2);
  return b;
}

// ---------------------------------------------------------------- main loop
static uint64_t now_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

int main(int argc, char** argv) {
  const char* config = nullptr;
  const char* device = nullptr;
  const char* replay = nullptr;
  const char* udp = nullptr;
  const char* record = nullptr;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string a = argv[i];
    if (a == "--config") config = argv[i + 1];
    else if (a == "--device") device = argv[i + 1];
    else if (a == "--replay") replay = argv[i + 1];
    else if (a == "--udp") udp = argv[i + 1];
    else if (a == "--record") record = argv[i + 1];
  }
  if (config) load_config(config);
  build_lut();
  g_raw.resize((size_t)W * H * 2);
  g_grey.resize((size_t)W * H);
  g_rect.resize((size_t)W * H);
  g_half.resize((size_t)(W / 2) * (H / 2));
  g_integral.resize((size_t)(W / 2 + 1) * (H / 2 + 1));
  g_labels.resize((size_t)(W / 2) * (H / 2));

  int sock = -1;
  sockaddr_in dst{};
  if (udp) {
    std::string s = udp;
    auto c = s.find(':');
    dst.sin_family = AF_INET;
    dst.sin_port = htons((uint16_t)std::atoi(s.substr(c + 1).c_str()));
    inet_pton(AF_INET, s.substr(0, c).c_str(), &dst.sin_addr);
    sock = socket(AF_INET, SOCK_DGRAM, 0);
  }
  FILE* rec = record ? std::fopen(record, "w") : nullptr;

  opr::sim::Scene* scene = nullptr;
  int fd = -1;
  if (replay) {
    std::string s = replay;
    auto c = s.find(':');
    opr::sim::Calibration cal;
    cal.width = W; cal.height = H;
    scene = new opr::sim::Scene(s.substr(0, c), std::strtoull(s.substr(c + 1).c_str(), nullptr, 10), cal);
  } else {
    fd = open(device ? device : "/dev/video0", O_RDWR);
    if (fd < 0) { std::perror("visionbox: open"); return 1; }
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = W;
    fmt.fmt.pix.height = H;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { std::perror("visionbox: S_FMT"); return 1; }
  }

  uint32_t seq = 0;
  opr::sim::Frame frame;
  for (;;) {
    uint64_t stamp;
    if (scene) {
      if (!scene->next(frame)) break;
      std::memcpy(g_raw.data(), frame.yuyv.data(), g_raw.size());
      stamp = frame.stamp_ns;
      seq = frame.sequence;
    } else {
      ssize_t n = read(fd, g_raw.data(), g_raw.size());
      if (n != (ssize_t)g_raw.size()) { std::perror("visionbox: read"); break; }
      stamp = now_ns();  // NOTE: time after read() returns, not exposure time
      seq++;
    }
    undistort_frame();
    detect();
    track(stamp);
    std::vector<uint8_t> msg = encode(seq, stamp);
    if (sock >= 0) sendto(sock, msg.data(), msg.size(), 0, (sockaddr*)&dst, sizeof(dst));
    if (rec) {
      for (uint8_t b : msg) std::fprintf(rec, "%02x", b);
      std::fprintf(rec, "\n");
    }
  }
  if (rec) std::fclose(rec);
  delete scene;
  return 0;
}
