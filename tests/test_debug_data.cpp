// SPDX-License-Identifier: Apache-2.0
// Debug raw data control (0x0303) codec (issue #93).
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "livox/mid360/debug_data.hpp"

using namespace livox::mid360;

namespace
{
std::vector<std::byte> bytes(std::initializer_list<int> v)
{
  std::vector<std::byte> out;
  for (const int b : v) {
    out.push_back(static_cast<std::byte>(b));
  }
  return out;
}

std::vector<std::byte> to_vec(std::span<const std::byte> s) { return {s.begin(), s.end()}; }
}  // namespace

TEST_CASE("debug data: constants", "[debug_data]")
{
  CHECK(static_cast<std::uint16_t>(CmdId::kDebugDataControl) == 0x0303);
  CHECK(kDebugDataPort == 60301);
  CHECK(kDefaultHostDebugDataPort == 44332);
  CHECK(kDebugDataControlSize == 9);
}

TEST_CASE("debug data: control request layout", "[debug_data]")
{
  // enable, host_ip, host_port (little-endian), reserved (little-endian)
  const auto on = encode_debug_data_control(
    {.enable = true, .host_ip = {192, 168, 1, 5}, .host_port = 0x1234, .reserved = 0xABCD});
  CHECK(to_vec(on) == bytes({1, 192, 168, 1, 5, 0x34, 0x12, 0xCD, 0xAB}));

  const auto off = encode_debug_data_control({.enable = false, .host_ip = {10, 0, 0, 1}});
  // 44332 = 0xAD2C, the SDK2 host port; reserved defaults to 0
  CHECK(to_vec(off) == bytes({0, 10, 0, 0, 1, 0x2C, 0xAD, 0, 0}));
}

TEST_CASE("debug data: control request round trip", "[debug_data]")
{
  const DebugDataControlRequest req{
    .enable = true, .host_ip = {172, 16, 254, 3}, .host_port = 65535, .reserved = 100};
  const auto dec = parse_debug_data_control(encode_debug_data_control(req));
  REQUIRE(dec.has_value());
  CHECK(*dec == req);

  const DebugDataControlRequest stop{.enable = false, .host_ip = {}, .host_port = 0};
  const auto dec_stop = parse_debug_data_control(encode_debug_data_control(stop));
  REQUIRE(dec_stop.has_value());
  CHECK(*dec_stop == stop);
}

TEST_CASE("debug data: control request parsing", "[debug_data]")
{
  const auto enc = encode_debug_data_control({.host_ip = {192, 168, 1, 5}});

  SECTION("too short")
  {
    for (std::size_t n = 0; n < kDebugDataControlSize; ++n) {
      const auto r = parse_debug_data_control(std::span(enc).first(n));
      REQUIRE_FALSE(r.has_value());
      CHECK(r.error() == ParseError::kTooShort);
    }
  }
  SECTION("any non-zero enable byte enables")
  {
    auto raw = enc;
    raw[0] = std::byte{0xFF};
    CHECK(parse_debug_data_control(raw).value().enable);
    raw[0] = std::byte{0};
    CHECK_FALSE(parse_debug_data_control(raw).value().enable);
  }
  SECTION("trailing bytes are ignored")
  {
    auto longer = to_vec(enc);
    longer.push_back(std::byte{0x55});
    const auto r = parse_debug_data_control(longer);
    REQUIRE(r.has_value());
    CHECK(r->host_ip == std::array<std::uint8_t, 4>{192, 168, 1, 5});
    CHECK(r->host_port == kDefaultHostDebugDataPort);
  }
}
