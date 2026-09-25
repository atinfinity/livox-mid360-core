// SPDX-License-Identifier: Apache-2.0
#include <catch2/catch_test_macros.hpp>

#include "livox/mid360/bytes.hpp"

using namespace livox::mid360;

TEST_CASE("little-endian round trip", "[bytes]") {
  std::array<std::byte, 20> buf{};
  bytes::write_le<std::uint16_t>(buf, 0, 0x1234);
  bytes::write_le<std::uint32_t>(buf, 2, 0xDEADBEEF);
  bytes::write_le<std::int64_t>(buf, 6, -2);
  bytes::write_le<float>(buf, 16, 1.5f);
  CHECK(buf[0] == std::byte{0x34});
  CHECK(buf[1] == std::byte{0x12});
  CHECK(buf[2] == std::byte{0xEF});
  CHECK(bytes::read_le<std::uint16_t>(buf, 0) == 0x1234);
  CHECK(bytes::read_le<std::uint32_t>(buf, 2) == 0xDEADBEEF);
  CHECK(bytes::read_le<std::int64_t>(buf, 6) == -2);
  CHECK(bytes::read_le<float>(buf, 16) == 1.5f);
}
