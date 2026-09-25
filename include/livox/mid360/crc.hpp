// SPDX-License-Identifier: Apache-2.0
// CRC algorithms defined by the Livox Mid-360 communication protocol (section "CRC Algorithm").
//
//   CRC-16/CCITT-FALSE : poly 0x1021, init 0xFFFF, xorout 0x0000, refin=false, refout=false
//   CRC-32             : poly 0x04C11DB7, init 0xFFFFFFFF, xorout 0xFFFFFFFF, refin=true,
//   refout=true
//                        (identical to the CRC-32 used by zlib / Ethernet / PNG)
//
// Both are pure functions, usable at compile time, and have no I/O dependency.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace livox::mid360::crc {

namespace detail {

consteval std::array<std::uint16_t, 256> make_crc16_table() {
  std::array<std::uint16_t, 256> t{};
  for (std::uint32_t i = 0; i < 256; ++i) {
    std::uint16_t c = static_cast<std::uint16_t>(i << 8);
    for (int k = 0; k < 8; ++k) {
      const std::uint32_t shifted = static_cast<std::uint32_t>(c) << 1;
      c = static_cast<std::uint16_t>((c & 0x8000u) ? (shifted ^ 0x1021u) : shifted);
    }
    t[i] = c;
  }
  return t;
}

consteval std::array<std::uint32_t, 256> make_crc32_table() {
  // Reflected form of 0x04C11DB7 is 0xEDB88320.
  std::array<std::uint32_t, 256> t{};
  for (std::uint32_t i = 0; i < 256; ++i) {
    std::uint32_t c = i;
    for (int k = 0; k < 8; ++k) {
      c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
    t[i] = c;
  }
  return t;
}

inline constexpr auto kCrc16Table = make_crc16_table();
inline constexpr auto kCrc32Table = make_crc32_table();

}  // namespace detail

/// Incremental CRC-16/CCITT-FALSE. Pass the previous result as `state` to continue.
[[nodiscard]] constexpr std::uint16_t crc16_ccitt_false(std::span<const std::byte> data,
                                                        std::uint16_t state = 0xFFFF) noexcept {
  for (std::byte b : data) {
    const auto idx = static_cast<std::uint8_t>((state >> 8) ^ static_cast<std::uint8_t>(b));
    state = static_cast<std::uint16_t>((state << 8) ^ detail::kCrc16Table[idx]);
  }
  return state;
}

/// Incremental CRC-32 (zlib-compatible). Pass the previous *final* result as `state`
/// to continue, exactly like zlib's crc32(). A fresh computation starts from 0.
[[nodiscard]] constexpr std::uint32_t crc32(std::span<const std::byte> data,
                                            std::uint32_t state = 0) noexcept {
  std::uint32_t c = state ^ 0xFFFFFFFFu;
  for (std::byte b : data) {
    c = detail::kCrc32Table[(c ^ static_cast<std::uint8_t>(b)) & 0xFFu] ^ (c >> 8);
  }
  return c ^ 0xFFFFFFFFu;
}

}  // namespace livox::mid360::crc
