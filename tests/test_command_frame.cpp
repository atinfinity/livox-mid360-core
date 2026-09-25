// SPDX-License-Identifier: Apache-2.0
#include <catch2/catch_test_macros.hpp>

#include "livox/mid360/crc.hpp"
#include "livox/mid360/protocol.hpp"
#include "test_util.hpp"

using namespace livox::mid360;

TEST_CASE("discovery request frame layout", "[cmd]") {
  auto f = build_command_frame({.seq_num = 1, .cmd_id = 0x0000});
  REQUIRE(f);
  REQUIRE(f->size() == 24);
  // sof, version, length=24
  CHECK((*f)[0] == std::byte{0xAA});
  CHECK((*f)[1] == std::byte{0x00});
  CHECK((*f)[2] == std::byte{24});
  CHECK((*f)[3] == std::byte{0});
  // seq 1
  CHECK((*f)[4] == std::byte{1});
  // cmd_type REQ, sender host
  CHECK((*f)[10] == std::byte{0});
  CHECK((*f)[11] == std::byte{0});
  // crc32 of empty data is 0
  for (int i = 20; i < 24; ++i) CHECK((*f)[static_cast<std::size_t>(i)] == std::byte{0});
  // header crc matches
  const auto c16 = crc::crc16_ccitt_false(std::span<const std::byte>(*f).first(18));
  CHECK(bytes_of({static_cast<unsigned>(c16 & 0xFF), static_cast<unsigned>(c16 >> 8)}) ==
        std::vector<std::byte>(f->begin() + 18, f->begin() + 20));
}

TEST_CASE("encode then parse round trip with payload", "[cmd]") {
  const auto payload = bytes_of("hello");
  CommandFrameSpec spec{.seq_num = 0xA5A5A5A5,
                        .cmd_id = 0x0100,
                        .cmd_type = CmdType::kReq,
                        .sender_type = SenderType::kHost,
                        .data = payload};
  auto f = build_command_frame(spec);
  REQUIRE(f);
  REQUIRE(f->size() == 29);
  auto v = parse_command_frame(*f);
  REQUIRE(v);
  CHECK(v->header.length == 29);
  CHECK(v->header.seq_num == 0xA5A5A5A5);
  CHECK(v->header.cmd_id == 0x0100);
  CHECK(v->header.cmd_type == CmdType::kReq);
  CHECK(v->header.sender_type == SenderType::kHost);
  CHECK(v->header.crc32 == crc::crc32(payload));
  CHECK(std::vector<std::byte>(v->data.begin(), v->data.end()) == payload);
}

TEST_CASE("encode into fixed buffer", "[cmd]") {
  std::array<std::byte, 24> small{};
  const auto payload = bytes_of("x");
  CHECK(encode_command_frame(small, {.data = payload}).error() == EncodeError::kBufferTooSmall);
  CHECK(encode_command_frame(small, {}).value() == 24);
  std::vector<std::byte> huge(kCommandDataMaxSize + 1);
  CHECK(build_command_frame({.data = huge}).error() == EncodeError::kDataTooLarge);
  std::vector<std::byte> max(kCommandDataMaxSize);
  auto f = build_command_frame({.data = max});
  REQUIRE(f);
  CHECK(f->size() == kCommandFrameMaxSize);
  CHECK(parse_command_frame(*f).has_value());
}

TEST_CASE("parse rejects malformed frames", "[cmd]") {
  auto ok = build_command_frame({.seq_num = 3, .cmd_id = 0x0200, .data = bytes_of("ab")}).value();
  CHECK(parse_command_frame(std::span(ok).first(10)).error() == ParseError::kTooShort);

  auto bad = ok;
  bad[0] = std::byte{0x55};
  CHECK(parse_command_frame(bad).error() == ParseError::kBadSof);

  bad = ok;
  bad[1] = std::byte{1};
  CHECK(parse_command_frame(bad).error() == ParseError::kBadVersion);

  bad = ok;
  bad[2] = std::byte{200};  // length larger than buffer
  CHECK(parse_command_frame(bad).error() == ParseError::kLengthMismatch);

  bad = ok;
  bad[4] = std::byte{9};  // seq changed -> header crc mismatch
  CHECK(parse_command_frame(bad).error() == ParseError::kBadCrc16);

  bad = ok;
  bad[24] = std::byte{'z'};  // payload changed
  CHECK(parse_command_frame(bad).error() == ParseError::kBadCrc32);
}

TEST_CASE("trailing bytes after length are ignored", "[cmd]") {
  auto ok = build_command_frame({.cmd_id = 0x0000}).value();
  ok.push_back(std::byte{0xFF});
  auto v = parse_command_frame(ok);
  REQUIRE(v);
  CHECK(v->data.empty());
}

TEST_CASE("to_string coverage", "[cmd]") {
  CHECK(to_string(RetCode::kSuccess) == "SUCCESS");
  CHECK(to_string(RetCode::kParamReadOnly) == "PARAM_RD_ONLY");
  CHECK(to_string(static_cast<RetCode>(0x7F)) == "UNKNOWN");
  CHECK(to_string(WorkState::kSampling) == "SAMPLING");
  CHECK(to_string(CmdId::kInfoPush) == "INFO_PUSH");
  CHECK(to_string(ParseError::kBadCrc16) == "bad CRC16");
}
