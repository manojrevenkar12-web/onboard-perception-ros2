#include "opr_core/telemetry.hpp"

#include <cstring>
#include <stdexcept>

namespace opr::core::telemetry {

namespace {

constexpr std::size_t kHeader = 20;
constexpr std::size_t kRecord = 22;

void put(std::vector<std::uint8_t>& b, std::uint64_t v, int n) {
  for (int i = 0; i < n; ++i) b.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
}

void put_f32(std::vector<std::uint8_t>& b, float f) {
  std::uint32_t u = 0;
  std::memcpy(&u, &f, 4);
  put(b, u, 4);
}

std::uint64_t get(const std::vector<std::uint8_t>& b, std::size_t at, int n) {
  std::uint64_t v = 0;
  for (int i = 0; i < n; ++i) v |= static_cast<std::uint64_t>(b[at + i]) << (8 * i);
  return v;
}

float get_f32(const std::vector<std::uint8_t>& b, std::size_t at) {
  const auto u = static_cast<std::uint32_t>(get(b, at, 4));
  float f = 0;
  std::memcpy(&f, &u, 4);
  return f;
}

}  // namespace

std::uint16_t crc16(const std::uint8_t* p, std::size_t n) {
  std::uint16_t c = 0xFFFF;
  for (std::size_t i = 0; i < n; ++i) {
    c ^= static_cast<std::uint16_t>(p[i] << 8);
    for (int b = 0; b < 8; ++b) {
      c = (c & 0x8000) ? static_cast<std::uint16_t>((c << 1) ^ 0x1021) : static_cast<std::uint16_t>(c << 1);
    }
  }
  return c;
}

std::vector<std::uint8_t> encode(const Packet& packet) {
  std::vector<std::uint8_t> b;
  b.reserve(kHeader + kRecord * packet.tracks.size() + 2);
  b.insert(b.end(), {'V', 'B', 'X', '1'});
  put(b, 1, 2);
  put(b, packet.tracks.size(), 2);
  put(b, packet.sequence, 4);
  put(b, packet.stamp_ns, 8);
  for (const auto& t : packet.tracks) {
    put(b, t.id, 4);
    put_f32(b, t.x);
    put_f32(b, t.y);
    put_f32(b, t.vx);
    put_f32(b, t.vy);
    b.push_back(t.hits);
    b.push_back(t.coasting ? 1 : 0);
  }
  put(b, crc16(b.data(), b.size()), 2);
  return b;
}

std::vector<std::uint8_t> encode(std::uint32_t sequence, std::uint64_t stamp_ns, const std::vector<Track>& tracks) {
  Packet p;
  p.sequence = sequence;
  p.stamp_ns = stamp_ns;
  for (const auto& t : tracks) {
    TrackRecord r;
    r.id = t.id;
    r.x = static_cast<float>(t.x[0]);
    r.y = static_cast<float>(t.x[1]);
    r.vx = static_cast<float>(t.x[2]);
    r.vy = static_cast<float>(t.x[3]);
    r.hits = static_cast<std::uint8_t>(t.hits > 255 ? 255 : t.hits);
    r.coasting = t.misses > 0;
    p.tracks.push_back(r);
  }
  return encode(p);
}

std::optional<Packet> decode(const std::vector<std::uint8_t>& b, std::string* error) {
  auto fail = [&](const char* why) -> std::optional<Packet> {
    if (error) *error = why;
    return std::nullopt;
  };
  if (b.size() < kHeader + 2) return fail("too short");
  if (b[0] != 'V' || b[1] != 'B' || b[2] != 'X' || b[3] != '1') return fail("bad magic");
  if (get(b, 4, 2) != 1) return fail("unsupported version");
  const auto n = static_cast<std::size_t>(get(b, 6, 2));
  if (b.size() != kHeader + kRecord * n + 2) return fail("length does not match track count");
  const auto crc = static_cast<std::uint16_t>(get(b, b.size() - 2, 2));
  if (crc != crc16(b.data(), b.size() - 2)) return fail("bad crc");
  Packet p;
  p.sequence = static_cast<std::uint32_t>(get(b, 8, 4));
  p.stamp_ns = get(b, 12, 8);
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t at = kHeader + kRecord * i;
    TrackRecord r;
    r.id = static_cast<std::uint32_t>(get(b, at, 4));
    r.x = get_f32(b, at + 4);
    r.y = get_f32(b, at + 8);
    r.vx = get_f32(b, at + 12);
    r.vy = get_f32(b, at + 16);
    r.hits = b[at + 20];
    r.coasting = (b[at + 21] & 1) != 0;
    p.tracks.push_back(r);
  }
  return p;
}

std::string to_hex(const std::vector<std::uint8_t>& bytes) {
  static const char* digits = "0123456789abcdef";
  std::string s;
  s.reserve(bytes.size() * 2);
  for (auto b : bytes) {
    s.push_back(digits[b >> 4]);
    s.push_back(digits[b & 15]);
  }
  return s;
}

std::vector<std::uint8_t> from_hex(const std::string& hex) {
  if (hex.size() % 2 != 0) throw std::invalid_argument("odd-length hex");
  auto nib = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    throw std::invalid_argument("not hex");
  };
  std::vector<std::uint8_t> out(hex.size() / 2);
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = static_cast<std::uint8_t>(nib(hex[2 * i]) * 16 + nib(hex[2 * i + 1]));
  }
  return out;
}

}  // namespace opr::core::telemetry
