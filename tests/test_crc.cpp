// SPDX-License-Identifier: Apache-2.0
#include <catch2/catch_test_macros.hpp>

#include "livox/mid360/crc.hpp"
#include "test_util.hpp"

using namespace livox::mid360;

TEST_CASE("CRC-16/CCITT-FALSE check value", "[crc]") {
  const auto d = bytes_of("123456789");
  CHECK(crc::crc16_ccitt_false(d) == 0x29B1);
  CHECK(crc::crc16_ccitt_false({}) == 0xFFFF);
}

TEST_CASE("CRC-16 is incremental", "[crc]") {
  const auto d = bytes_of("123456789");
  const auto s = std::span<const std::byte>(d);
  const auto part = crc::crc16_ccitt_false(s.first(4));
  CHECK(crc::crc16_ccitt_false(s.subspan(4), part) == 0x29B1);
}

TEST_CASE("CRC-32 check value (zlib compatible)", "[crc]") {
  const auto d = bytes_of("123456789");
  CHECK(crc::crc32(d) == 0xCBF43926);
  CHECK(crc::crc32({}) == 0);
}

TEST_CASE("CRC-32 is incremental like zlib", "[crc]") {
  const auto d = bytes_of("123456789");
  const auto s = std::span<const std::byte>(d);
  const auto part = crc::crc32(s.first(3));
  CHECK(crc::crc32(s.subspan(3), part) == 0xCBF43926);
}

TEST_CASE("CRC is constexpr", "[crc]") {
  constexpr std::array<std::byte, 1> one{std::byte{0x00}};
  constexpr auto c16 = crc::crc16_ccitt_false(one);
  constexpr auto c32 = crc::crc32(one);
  STATIC_CHECK(c16 == 0xE1F0);
  STATIC_CHECK(c32 == 0xD202EF8D);
}
