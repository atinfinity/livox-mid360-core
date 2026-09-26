// SPDX-License-Identifier: Apache-2.0
// Firmware log (0x03xx) codec round trips (issue #44).
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "livox/mid360/firmware_log.hpp"

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
}  // namespace

TEST_CASE("firmware log: control request codec", "[firmware_log]")
{
  const auto enc = encode_firmware_log_control({.log_type = FirmwareLogType::kException});
  CHECK(enc == std::array{std::byte{1}, std::byte{1}});
  const auto dec = parse_firmware_log_control(enc);
  REQUIRE(dec.has_value());
  CHECK(dec->log_type == FirmwareLogType::kException);
  CHECK(dec->enable);
  const auto off =
    encode_firmware_log_control({.log_type = FirmwareLogType::kRealTime, .enable = false});
  CHECK(off == std::array{std::byte{0}, std::byte{0}});
  CHECK(!parse_firmware_log_control(std::span(enc).first(1)).has_value());
}

TEST_CASE("firmware log: push packet codec", "[firmware_log]")
{
  const FirmwareLogPushHeader h{
    .log_type = FirmwareLogType::kRealTime,
    .file_index = 3,
    .file_num = 1,
    .flags = FirmwareLogFlags{kFirmwareLogFlagAck | kFirmwareLogFlagBegin},
    .timestamp = 0x11223344,
    .rsvd = 0,
    .trans_index = 0x01020304,
    .data_length = 4};
  const auto data = bytes({0xde, 0xad, 0xbe, 0xef});
  const auto enc = encode_firmware_log_push(h, data);
  REQUIRE(enc.has_value());
  REQUIRE(enc->size() == kFirmwareLogPushHeaderSize + 4);
  CHECK((*enc)[0] == std::byte{0});
  CHECK((*enc)[1] == std::byte{3});
  CHECK((*enc)[3] == std::byte{0x03});
  CHECK((*enc)[4] == std::byte{0x44});  // little-endian timestamp
  CHECK((*enc)[14] == std::byte{4});    // data_length

  const auto view = parse_firmware_log_push(*enc);
  REQUIRE(view.has_value());
  CHECK(view->header.file_index == 3);
  CHECK(view->header.flags.ack_requested());
  CHECK(view->header.flags.file_begin());
  CHECK(!view->header.flags.file_end());
  CHECK(view->header.timestamp == 0x11223344);
  CHECK(view->header.trans_index == 0x01020304);
  CHECK(view->data.size() == 4);
  CHECK(view->data[0] == std::byte{0xde});
  CHECK(to_string(view->header).find("file=3/1") != std::string::npos);
  CHECK(to_string(FirmwareLogType::kException) == "exception");

  SECTION("length errors")
  {
    CHECK(parse_firmware_log_push(std::span(*enc).first(10)).error() == ParseError::kTooShort);
    CHECK(parse_firmware_log_push(std::span(*enc).first(18)).error() == ParseError::kTruncated);
    auto longer = *enc;
    longer.push_back(std::byte{0});
    CHECK(parse_firmware_log_push(longer).error() == ParseError::kLengthMismatch);
    const std::vector<std::byte> big(kFirmwareLogPushDataMaxSize + 1);
    CHECK(!encode_firmware_log_push(h, big).has_value());
  }
}

TEST_CASE("firmware log: push ack codec", "[firmware_log]")
{
  const FirmwareLogPushAck ack{
    .ret_code = RetCode::kSuccess,
    .log_type = FirmwareLogType::kException,
    .file_index = 7,
    .trans_index = 0xAABBCCDD};
  const auto enc = encode_firmware_log_push_ack(ack);
  CHECK(
    enc == std::array{
             std::byte{0}, std::byte{1}, std::byte{7}, std::byte{0xDD}, std::byte{0xCC},
             std::byte{0xBB}, std::byte{0xAA}});
  const auto dec = parse_firmware_log_push_ack(enc);
  REQUIRE(dec.has_value());
  CHECK(dec->log_type == FirmwareLogType::kException);
  CHECK(dec->file_index == 7);
  CHECK(dec->trans_index == 0xAABBCCDD);
  CHECK(!parse_firmware_log_push_ack(std::span(enc).first(6)).has_value());
}
