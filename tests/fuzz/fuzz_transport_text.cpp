// SPDX-License-Identifier: Apache-2.0
// parse_endpoint on arbitrary text, plus the to_string / parse_endpoint round trip.
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "fuzz_check.hpp"
#include "livox/mid360/transport.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  using namespace livox::mid360;
  const std::string_view text{reinterpret_cast<const char *>(data), size};
  if (const auto ep = parse_endpoint(text)) {
    const auto again = parse_endpoint(to_string(*ep));
    fuzz::require(again.has_value() && *again == *ep);
  }
  if (size >= 6) {
    Endpoint ep;
    for (std::size_t i = 0; i < 4; ++i) ep.ip[i] = data[i];
    ep.port = static_cast<std::uint16_t>(data[4] | (data[5] << 8));
    const auto again = parse_endpoint(to_string(ep));
    fuzz::require(again.has_value() && *again == ep);
    fuzz::require(parse_endpoint(ip_to_string(ep.ip)) == Endpoint{ep.ip, 0});
  }
  return 0;
}
