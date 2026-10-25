// visionbox telemetry, wire format v1: what the ground station has parsed since 2.x and still does.
//
//   offset  size  field
//   0       4     magic "VBX1"
//   4       2     version (1)
//   6       2     track count n
//   8       4     frame sequence
//   12      8     exposure stamp, ns
//   20      22*n  per track: u32 id, f32 x, f32 y, f32 vx, f32 vy, u8 hits (capped 255), u8 flags
//   20+22n  2     CRC-16/CCITT-FALSE over every byte before it
//
// All little-endian. flags bit 0: coasting (no detection this frame).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "opr_core/track.hpp"

namespace opr::core::telemetry {

struct TrackRecord {
  std::uint32_t id = 0;
  float x = 0, y = 0, vx = 0, vy = 0;
  std::uint8_t hits = 0;
  bool coasting = false;
};

struct Packet {
  std::uint32_t sequence = 0;
  std::uint64_t stamp_ns = 0;
  std::vector<TrackRecord> tracks;
};

std::uint16_t crc16(const std::uint8_t* data, std::size_t size);

/// Tracks must already be the confirmed ones, sorted by id (Tracker::confirmed()).
std::vector<std::uint8_t> encode(std::uint32_t sequence, std::uint64_t stamp_ns, const std::vector<Track>& tracks);
std::vector<std::uint8_t> encode(const Packet& packet);

/// Returns nothing for a datagram that is too short, has the wrong magic or version, a length that
/// does not match its count, or a bad CRC; `error` says which.
std::optional<Packet> decode(const std::vector<std::uint8_t>& bytes, std::string* error = nullptr);

std::string to_hex(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> from_hex(const std::string& hex);

}  // namespace opr::core::telemetry
