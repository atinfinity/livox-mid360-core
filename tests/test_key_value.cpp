// SPDX-License-Identifier: Apache-2.0
#include <catch2/catch_test_macros.hpp>

#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"
#include "test_util.hpp"

using namespace livox::mid360;

TEST_CASE("key-value list round trip", "[kv]") {
  const auto a = bytes_of({0x01});
  const auto b = bytes_of({0xC0, 0xA8, 0x01, 0x05, 0x35, 0xDB, 0x34, 0xDB});
  const KeyValue kvs[] = {{0x001A, a}, {0x0006, b}};
  std::vector<std::byte> out;
  append_key_value_list(out, kvs);
  REQUIRE(out.size() == 4 + 1 + 4 + 8);
  CHECK(out == bytes_of({0x1A, 0x00, 0x01, 0x00, 0x01, 0x06, 0x00, 0x08, 0x00, 0xC0, 0xA8, 0x01,
                         0x05, 0x35, 0xDB, 0x34, 0xDB}));
  auto parsed = parse_key_value_list(out, 2);
  REQUIRE(parsed);
  REQUIRE(parsed->size() == 2);
  CHECK((*parsed)[0].key == 0x001A);
  CHECK((*parsed)[0].value.size() == 1);
  CHECK((*parsed)[1].key == 0x0006);
  CHECK((*parsed)[1].value.size() == 8);
  CHECK(find_key(*parsed, Key::kPointCloudHostIpCfg).has_value());
  CHECK_FALSE(find_key(*parsed, Key::kSn).has_value());
}

TEST_CASE("key-value list errors", "[kv]") {
  const auto buf = bytes_of({0x1A, 0x00, 0x05, 0x00, 0x01});  // claims 5 bytes, has 1
  CHECK(parse_key_value_list(buf, 1).error() == ParseError::kTruncated);
  const auto short_hdr = bytes_of({0x1A, 0x00, 0x01});
  CHECK(parse_key_value_list(short_hdr, 1).error() == ParseError::kTruncated);
  const auto two = bytes_of({0x1A, 0x00, 0x01, 0x00, 0x01, 0x1C, 0x00, 0x01, 0x00, 0x01});
  CHECK(parse_key_value_list(two, 1).error() == ParseError::kKeyNumMismatch);  // leftovers
  CHECK(parse_key_value_list(two, 2).has_value());
  CHECK(parse_key_value_list(two, 3).error() == ParseError::kTruncated);
  CHECK(parse_key_value_list({}, 0).value().empty());
}

TEST_CASE("0x0100 request payload", "[kv]") {
  const auto one = bytes_of({0x01});
  const KeyValue kvs[] = {{0x001A, one}};
  CHECK(encode_param_config_request(kvs) ==
        bytes_of({0x01, 0x00, 0x00, 0x00, 0x1A, 0x00, 0x01, 0x00, 0x01}));
}

TEST_CASE("0x0101 request payload", "[kv]") {
  const std::uint16_t keys[] = {0x8000, 0x8006};
  CHECK(encode_param_inquire_request(keys) ==
        bytes_of({0x02, 0x00, 0x00, 0x00, 0x00, 0x80, 0x06, 0x80}));
}

TEST_CASE("control command payloads", "[kv]") {
  CHECK(encode_reboot_request(1000) == bytes_of({0xE8, 0x03}));
  CHECK(encode_factory_reset_request().size() == 16);
  auto g = encode_set_gps_timestamp_request(0x0102030405060708ULL);
  CHECK(g == bytes_of({0x02, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01}));
}

TEST_CASE("ack parsers", "[kv]") {
  auto d =
      parse_discovery_ack(bytes_of({0x00, 0x09, 'S', 'N', '1', 0, 0,   0,   0, 0,  0,    0,
                                    0,    0,    0,   0,   0,   0, 192, 168, 1, 12, 0x24, 0xDB}));
  REQUIRE(d);
  CHECK(d->ret_code == RetCode::kSuccess);
  CHECK(d->dev_type == 9);
  CHECK(d->serial_number_view() == "SN1");
  CHECK(d->lidar_ip == Ipv4{192, 168, 1, 12});
  CHECK(d->cmd_port == 56100);
  CHECK(parse_discovery_ack(bytes_of({0})).error() == ParseError::kTruncated);

  auto c = parse_param_config_ack(bytes_of({0x22, 0x00, 0x80}));
  REQUIRE(c);
  CHECK(c->ret_code == RetCode::kParamReadOnly);
  CHECK(c->error_key == 0x8000);

  const auto qb = bytes_of({0x00, 0x01, 0x00, 0x06, 0x80, 0x01, 0x00, 0x01});
  auto q = parse_param_inquire_ack(qb);
  REQUIRE(q);
  CHECK(q->ret_code == RetCode::kSuccess);
  REQUIRE(q->values.size() == 1);
  CHECK(q->values[0].key == 0x8006);
  CHECK(decode_work_state(q->values[0].value).value() == WorkState::kSampling);

  const auto pb = bytes_of({0x01, 0x00, 0x00, 0x00, 0x06, 0x80, 0x01, 0x00, 0x09});
  auto p = parse_info_push(pb);
  REQUIRE(p);
  REQUIRE(p->values.size() == 1);
  CHECK(decode_work_state(p->values[0].value).value() == WorkState::kReady);
  CHECK(parse_info_push(bytes_of({0x01, 0x00, 0x00, 0x00})).error() == ParseError::kTruncated);

  CHECK(parse_simple_ack(bytes_of({0x02})).value().ret_code == RetCode::kNotPermitNow);
  CHECK(parse_simple_ack({}).error() == ParseError::kTruncated);
}
