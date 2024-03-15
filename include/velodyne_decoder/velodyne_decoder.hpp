#pragma once

#include "pcl_types/pcl_types.hpp"

#include <bit>
#include <cstdint>
#include <numbers>

namespace velodyne_decoder {

using VelodynePacket = uint8_t[1206];

constexpr float kDEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;

template <std::size_t N>
  requires(N == 1)
uint8_t getBytes(const uint8_t bytes[1]) {
  return bytes[0];
}

template <std::size_t N, std::endian Endian = std::endian::native>
  requires(N == 2 && Endian == std::endian::little)
uint16_t getBytes(const uint8_t bytes[2]) {
  return (static_cast<uint16_t>(bytes[1]) << 8) |
         static_cast<uint16_t>(bytes[0]);
}

template <std::size_t N, std::endian Endian = std::endian::native>
  requires(N == 2 && Endian == std::endian::big)
uint16_t getBytes(const uint8_t bytes[2]) {
  return (static_cast<uint16_t>(bytes[0]) << 8) |
         static_cast<uint16_t>(bytes[1]);
}

}; // namespace velodyne_decoder
