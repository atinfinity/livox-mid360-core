// SPDX-License-Identifier: Apache-2.0
// Debug raw data collection (issue #93): command 0x0303 switches on a diagnostic stream that
// Livox support asks for ("debug raw data" in the protocol document, "debug point cloud" in
// Livox-SDK2). The stream itself is opaque to this SDK. The request layout follows the
// protocol document rev v1.4.12; ports and behaviour follow Livox-SDK2 and are unverified on
// hardware (#11). Unrelated to the firmware log (firmware_log.hpp).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>

#include "livox/mid360/export.hpp"
#include "livox/mid360/protocol.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

inline constexpr std::size_t kDebugDataControlSize = 9;

/// 0x0303 request: start (`enable`) or stop the stream towards `host_ip:host_port`.
struct DebugDataControlRequest
{
  bool enable = true;
  std::array<std::uint8_t, 4> host_ip{};
  std::uint16_t host_port = kDefaultHostDebugDataPort;
  /// [unverified] `reserved` in the protocol document; SDK2 calls it `bandwidth` (Mbps) and
  /// sends 0.
  std::uint16_t reserved = 0;

  friend bool operator==(const DebugDataControlRequest &, const DebugDataControlRequest &) =
    default;
};

/// 0x0303 payload: `{enable u8, host_ip u8[4], host_port u16, reserved u16}`, little-endian.
/// The ACK is a single `ret_code`: parse_simple_ack().
[[nodiscard]] std::array<std::byte, kDebugDataControlSize> encode_debug_data_control(
  const DebugDataControlRequest & req) noexcept;
/// `enable` is any non-zero byte; bytes after the ninth are ignored.
[[nodiscard]] std::expected<DebugDataControlRequest, ParseError> parse_debug_data_control(
  std::span<const std::byte> data) noexcept;

}  // namespace livox::mid360
LIVOX_MID360_API_END
