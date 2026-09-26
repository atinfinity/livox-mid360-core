// SPDX-License-Identifier: Apache-2.0
// discover(): datagram -> DiscoveredDevice step.
//
// Input layout: [from ip 4][from port u16][datagram...]
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "fuzz_check.hpp"
#include "livox/mid360/protocol.hpp"
#include "session_detail.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  using namespace livox::mid360;
  constexpr std::size_t kPrefix = 6;
  if (size < kPrefix) return 0;
  Endpoint from;
  std::memcpy(from.ip.data(), data, 4);
  from.port = static_cast<std::uint16_t>(data[4] | (data[5] << 8));
  const std::span<const std::byte> datagram{
    reinterpret_cast<const std::byte *>(data + kPrefix), size - kPrefix};
  const auto dev = detail::parse_discovered_device(datagram, from);
  if (!dev) return 0;
  const auto frame = parse_command_frame(datagram);
  fuzz::require(
    frame.has_value() && frame->header.cmd_id == 0x0000 && frame->header.cmd_type == CmdType::kAck);
  fuzz::require(dev->from == from);
  fuzz::require(dev->serial_number.size() <= 16);
  fuzz::require(
    std::find(dev->serial_number.begin(), dev->serial_number.end(), '\0') ==
    dev->serial_number.end());
  return 0;
}
