// SPDX-License-Identifier: Apache-2.0
// HMS (health management system) diagnostic code decoding for the Mid-360.
// Reference: wiki "HMS Diagnostic Code Introduction" (key 0x8011 hms_code, uint32[8]).
//
//   byte[3:2] abnormal ID | byte[1] reserved | byte[0] abnormal level
#pragma once

#include <cstdint>
#include <string_view>

#include "livox/mid360/export.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360 {

enum class HmsLevel : std::uint8_t {
  kNone = 0,     ///< slot unused
  kInfo = 1,
  kWarning = 2,
  kError = 3,
  kFatal = 4,
};

struct HmsCode {
  std::uint16_t abnormal_id;
  HmsLevel level;
  std::uint8_t reserved;
  std::uint32_t raw;

  [[nodiscard]] constexpr bool active() const noexcept { return raw != 0; }
};

[[nodiscard]] constexpr HmsCode decode_hms(std::uint32_t raw) noexcept {
  return {static_cast<std::uint16_t>(raw >> 16), static_cast<HmsLevel>(raw & 0xFFu),
          static_cast<std::uint8_t>((raw >> 8) & 0xFFu), raw};
}

[[nodiscard]] std::string_view to_string(HmsLevel l) noexcept;

/// Human-readable description for an abnormal ID from the official table, or "" if unknown.
[[nodiscard]] std::string_view hms_description(std::uint16_t abnormal_id) noexcept;

/// Suggested action from the official table, or "" if unknown.
[[nodiscard]] std::string_view hms_suggestion(std::uint16_t abnormal_id) noexcept;

}  // namespace livox::mid360
LIVOX_MID360_API_END
