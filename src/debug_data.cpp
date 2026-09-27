// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/debug_data.hpp"

#include "livox/mid360/bytes.hpp"

namespace livox::mid360
{

using bytes::read_le;
using bytes::write_le;

std::array<std::byte, kDebugDataControlSize> encode_debug_data_control(
  const DebugDataControlRequest & req) noexcept
{
  std::array<std::byte, kDebugDataControlSize> out{};
  write_le<std::uint8_t>(out, 0, req.enable ? 1 : 0);
  for (std::size_t i = 0; i < req.host_ip.size(); ++i) {
    write_le<std::uint8_t>(out, 1 + i, req.host_ip[i]);
  }
  write_le<std::uint16_t>(out, 5, req.host_port);
  write_le<std::uint16_t>(out, 7, req.reserved);
  return out;
}

std::expected<DebugDataControlRequest, ParseError> parse_debug_data_control(
  std::span<const std::byte> data) noexcept
{
  if (data.size() < kDebugDataControlSize) {
    return std::unexpected(ParseError::kTooShort);
  }
  DebugDataControlRequest req;
  req.enable = read_le<std::uint8_t>(data, 0) != 0;
  for (std::size_t i = 0; i < req.host_ip.size(); ++i) {
    req.host_ip[i] = read_le<std::uint8_t>(data, 1 + i);
  }
  req.host_port = read_le<std::uint16_t>(data, 5);
  req.reserved = read_le<std::uint16_t>(data, 7);
  return req;
}

}  // namespace livox::mid360
