// SPDX-License-Identifier: Apache-2.0
// The pure datagram -> result helpers behind Session / discover() (src/session_detail.hpp).
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <span>
#include <vector>

#include "generated/golden_vectors.hpp"
#include "session_detail.hpp"

using namespace livox::mid360;

namespace {

std::span<const std::byte> span_of(const unsigned char* p, std::size_t n) {
  return std::as_bytes(std::span<const unsigned char>(p, n));
}
#define GOLDEN(name) span_of(golden::name, golden::name##_len)

const Ipv4 kLidarIp{192, 168, 1, 12};
const Endpoint kFrom{kLidarIp, kCommandPort};

RawAck raw_ack(std::uint16_t cmd_id, std::vector<std::byte> data) {
  RawAck a;
  a.cmd_id = cmd_id;
  a.seq_num = 1;
  a.data = std::move(data);
  return a;
}

std::vector<std::byte> bytes(std::initializer_list<unsigned char> il) {
  std::vector<std::byte> v;
  for (auto c : il) v.push_back(static_cast<std::byte>(c));
  return v;
}

/// Builds an ACK frame with arbitrary payload for the negative cases.
std::vector<std::byte> ack_frame(std::uint32_t seq, std::uint16_t cmd_id,
                                 std::span<const std::byte> data) {
  CommandFrameSpec spec;
  spec.seq_num = seq;
  spec.cmd_id = cmd_id;
  spec.cmd_type = CmdType::kAck;
  spec.sender_type = SenderType::kLidar;
  spec.data = data;
  const auto f = build_command_frame(spec);
  REQUIRE(f.has_value());
  return *f;
}

}  // namespace

TEST_CASE("match_ack classifies datagrams", "[session][detail]") {
  const auto ack = GOLDEN(discovery_ack);  // seq 1, cmd 0x0000, ACK from lidar

  SECTION("matching ACK") {
    const auto m = detail::match_ack(ack, kFrom, 1, 0x0000, kLidarIp);
    REQUIRE(m.has_value());
    CHECK(m->header.seq_num == 1);
    CHECK(m->header.cmd_id == 0x0000);
    CHECK(m->data.size() == ack.size() - kCommandHeaderSize);
  }
  SECTION("other seq / cmd_id / source ip are late") {
    CHECK(detail::match_ack(ack, kFrom, 2, 0x0000, kLidarIp).error() ==
          detail::AckMismatch::kLate);
    CHECK(detail::match_ack(ack, kFrom, 1, 0x0101, kLidarIp).error() ==
          detail::AckMismatch::kLate);
    CHECK(detail::match_ack(ack, Endpoint{{192, 168, 1, 99}, kCommandPort}, 1, 0x0000, kLidarIp)
              .error() == detail::AckMismatch::kLate);
  }
  SECTION("source port is not part of the match") {
    CHECK(detail::match_ack(ack, Endpoint{kLidarIp, 1}, 1, 0x0000, kLidarIp).has_value());
  }
  SECTION("requests and pushes are not ACKs") {
    CHECK(detail::match_ack(GOLDEN(param_config_req), kFrom, 2, 0x0100, kLidarIp).error() ==
          detail::AckMismatch::kNotAck);
    CHECK(detail::match_ack(GOLDEN(info_push), kFrom, 7, 0x0102, kLidarIp).error() ==
          detail::AckMismatch::kNotAck);
  }
  SECTION("corrupted frame is bad") {
    std::vector<std::byte> bad(ack.begin(), ack.end());
    bad.back() ^= std::byte{0xFF};
    CHECK(detail::match_ack(bad, kFrom, 1, 0x0000, kLidarIp).error() ==
          detail::AckMismatch::kBadFrame);
    CHECK(detail::match_ack({}, kFrom, 1, 0x0000, kLidarIp).error() ==
          detail::AckMismatch::kBadFrame);
  }
}

TEST_CASE("parse_discovered_device", "[session][detail]") {
  const Endpoint from{kLidarIp, kDiscoveryPort};
  SECTION("golden ACK") {
    const auto dev = detail::parse_discovered_device(GOLDEN(discovery_ack), from);
    REQUIRE(dev.has_value());
    CHECK(dev->serial_number == "47MDL9K0010001");
    CHECK(dev->ip == kLidarIp);
    CHECK(dev->cmd_port == kCommandPort);
    CHECK(dev->dev_type == 9);
    CHECK(dev->from == from);
  }
  SECTION("ret_code != 0, wrong cmd_id and requests are ignored") {
    auto data = bytes({0x01, 9});
    data.resize(2 + 16 + 4 + 2);
    CHECK_FALSE(detail::parse_discovered_device(ack_frame(1, 0x0000, data), from));
    CHECK_FALSE(detail::parse_discovered_device(GOLDEN(param_inquire_ack), from));
    CHECK_FALSE(detail::parse_discovered_device(GOLDEN(discovery_req), from));
  }
  SECTION("serial number without NUL terminator is truncated to 16 chars") {
    auto data = bytes({0x00, 9});
    for (char c : std::string_view("0123456789ABCDEF")) data.push_back(static_cast<std::byte>(c));
    data.resize(2 + 16 + 4 + 2);
    const auto dev = detail::parse_discovered_device(ack_frame(1, 0x0000, data), from);
    REQUIRE(dev.has_value());
    CHECK(dev->serial_number == "0123456789ABCDEF");
  }
}

TEST_CASE("typed ACK conversions", "[session][detail]") {
  SECTION("discovery") {
    const auto frame = parse_command_frame(GOLDEN(discovery_ack));
    REQUIRE(frame.has_value());
    const auto r = detail::to_discovery_ack(
        raw_ack(0x0000, {frame->data.begin(), frame->data.end()}));
    REQUIRE(r.has_value());
    CHECK(r->serial_number_view() == "47MDL9K0010001");
    const auto bad = detail::to_discovery_ack(raw_ack(0x0000, bytes({0x00})));
    REQUIRE_FALSE(bad.has_value());
    CHECK(bad.error().kind == SessionErrorKind::kBadResponse);
    CHECK(bad.error().cmd_id == 0x0000);
  }
  SECTION("configure: 0x00 and 0x21 succeed, others are rejected") {
    CHECK(detail::to_config_ack(raw_ack(0x0100, bytes({0x00, 0, 0}))).has_value());
    const auto reboot = detail::to_config_ack(raw_ack(0x0100, bytes({0x21, 0, 0})));
    REQUIRE(reboot.has_value());
    CHECK(reboot->ret_code == RetCode::kParamRebootEffect);
    const auto rej = detail::to_config_ack(raw_ack(0x0100, bytes({0x22, 0x00, 0x80})));
    REQUIRE_FALSE(rej.has_value());
    CHECK(rej.error().kind == SessionErrorKind::kLidarRejected);
    CHECK(rej.error().ret_code == static_cast<RetCode>(0x22));
    CHECK(rej.error().error_key == 0x8000);
  }
  SECTION("inquire: views stay inside raw across a move") {
    const auto frame = parse_command_frame(GOLDEN(param_inquire_ack));
    REQUIRE(frame.has_value());
    auto r = detail::to_inquire_result(
        raw_ack(0x0101, {frame->data.begin(), frame->data.end()}));
    REQUIRE(r.has_value());
    REQUIRE(r->values.size() == 3);
    const InquireResult moved = std::move(*r);
    for (const auto& kv : moved.values) {
      CHECK(kv.value.data() >= moved.raw.data());
      CHECK(kv.value.data() + kv.value.size() <= moved.raw.data() + moved.raw.size());
    }
    const auto ws = detail::to_work_state(moved);
    REQUIRE(ws.has_value());
    CHECK(*ws == WorkState::kSampling);
  }
  SECTION("inquire rejection reports the first key") {
    auto data = bytes({0x20, 1, 0, 0x34, 0x12, 0, 0});  // ret 0x20, 1 key, key 0x1234 len 0
    const auto rej = detail::to_inquire_result(raw_ack(0x0101, data));
    REQUIRE_FALSE(rej.has_value());
    CHECK(rej.error().kind == SessionErrorKind::kLidarRejected);
    CHECK(rej.error().error_key == 0x1234);
  }
  SECTION("work state missing from the result") {
    InquireResult empty;
    const auto ws = detail::to_work_state(empty);
    REQUIRE_FALSE(ws.has_value());
    CHECK(ws.error().kind == SessionErrorKind::kBadResponse);
  }
  SECTION("simple ACK") {
    CHECK(detail::to_simple_ack(raw_ack(0x0200, bytes({0x00}))).has_value());
    CHECK(detail::to_simple_ack(raw_ack(0x0200, bytes({0x01}))).error().kind ==
          SessionErrorKind::kLidarRejected);
    CHECK(detail::to_simple_ack(raw_ack(0x0200, {})).error().kind ==
          SessionErrorKind::kBadResponse);
  }
}
