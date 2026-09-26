// Point tag byte (issue #34): four 2-bit confidence fields, section "Tag Information".
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "livox/mid360/export.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

/// Confidence that a point is a normal return for one tag category; higher = more likely noise.
enum class TagConfidence : std::uint8_t
{
  kHigh = 0,
  kMedium = 1,
  kLow = 2,
  kReserved = 3,
};
[[nodiscard]] std::string_view to_string(TagConfidence c) noexcept;

/// Decoded tag byte: bit 0-1 adjacent glue, 2-3 rain / fog / dust, 4-5 other, 6-7 reserved.
struct TagInfo
{
  TagConfidence adjacent_glue;  ///< bit 0-1: glue points between adjacent objects
  TagConfidence particles;      ///< bit 2-3: rain, fog, dust
  TagConfidence other;          ///< bit 4-5: other properties
  TagConfidence reserved;       ///< bit 6-7
};
[[nodiscard]] std::string to_string(const TagInfo & t);

[[nodiscard]] constexpr TagInfo decode_tag(std::uint8_t tag) noexcept
{
  return {
    static_cast<TagConfidence>(tag & 0x3u), static_cast<TagConfidence>((tag >> 2) & 0x3u),
    static_cast<TagConfidence>((tag >> 4) & 0x3u), static_cast<TagConfidence>((tag >> 6) & 0x3u)};
}

/// True when the adjacent-glue or particle confidence is worse than `worst_accepted`; the
/// `other` and `reserved` fields are not consulted. With the default any degradation is noise.
[[nodiscard]] constexpr bool is_noise(
  std::uint8_t tag, TagConfidence worst_accepted = TagConfidence::kHigh) noexcept
{
  const TagInfo t = decode_tag(tag);
  return t.adjacent_glue > worst_accepted || t.particles > worst_accepted;
}

}  // namespace livox::mid360
LIVOX_MID360_API_END
