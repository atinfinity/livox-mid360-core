// SPDX-License-Identifier: Apache-2.0
// Cross-checks the C++ implementation against byte sequences produced independently by the
// Python reference implementation (tools/livox_mid360_proto.py).
#include <catch2/catch_test_macros.hpp>

#include "generated/golden_vectors.hpp"
#include "livox/mid360/crc.hpp"
#include "livox/mid360/hms.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"
#include "test_util.hpp"

using namespace livox::mid360;

#define GOLDEN(name) span_of(golden::name, golden::name##_len)

namespace {
std::vector<std::byte> to_vec(std::span<const std::byte> s) {
  return {s.begin(), s.end()};
}
}  // namespace

TEST_CASE("golden: crc check values agree", "[golden]") {
  CHECK(crc::crc16_ccitt_false(bytes_of("123456789")) == golden::crc16_check);
  CHECK(crc::crc32(bytes_of("123456789")) == golden::crc32_check);
  auto f = build_command_frame({.seq_num = 1, .cmd_id = 0}).value();
  CHECK(crc::crc16_ccitt_false(std::span<const std::byte>(f).first(18)) ==
        golden::crc16_discovery_header);
}

TEST_CASE("golden: request frames byte-identical", "[golden]") {
  CHECK(build_command_frame({.seq_num = 1, .cmd_id = 0x0000}).value() ==
        to_vec(GOLDEN(discovery_req)));

  const auto h5 = encode_host_ip_config({{192, 168, 1, 5}, 56201, 56200});
  const auto h6 = encode_host_ip_config({{192, 168, 1, 5}, 56301, 56300});
  const auto h7 = encode_host_ip_config({{192, 168, 1, 5}, 56401, 56400});
  const auto one = encode_u8(1);
  const KeyValue kvs[] = {{0x0005, h5}, {0x0006, h6}, {0x0007, h7}, {0x0000, one}, {0x001C, one}};
  const auto cfg = encode_param_config_request(kvs);
  CHECK(build_command_frame({.seq_num = 2, .cmd_id = 0x0100, .data = cfg}).value() ==
        to_vec(GOLDEN(param_config_req)));

  const KeyValue smp[] = {{0x001A, one}};
  const auto smp_data = encode_param_config_request(smp);
  CHECK(build_command_frame({.seq_num = 3, .cmd_id = 0x0100, .data = smp_data}).value() ==
        to_vec(GOLDEN(set_sampling_req)));

  const std::uint16_t keys[] = {0x8000, 0x8002, 0x8006};
  const auto inq = encode_param_inquire_request(keys);
  CHECK(build_command_frame({.seq_num = 4, .cmd_id = 0x0101, .data = inq}).value() ==
        to_vec(GOLDEN(param_inquire_req)));

  const auto rb = encode_reboot_request(100);
  CHECK(build_command_frame({.seq_num = 5, .cmd_id = 0x0200, .data = rb}).value() ==
        to_vec(GOLDEN(reboot_req)));
  const auto fr = encode_factory_reset_request();
  CHECK(build_command_frame({.seq_num = 6, .cmd_id = 0x0201, .data = fr}).value() ==
        to_vec(GOLDEN(factory_reset_req)));
  const auto gps = encode_set_gps_timestamp_request(1700000000000000000ULL);
  CHECK(build_command_frame({.seq_num = 8, .cmd_id = 0x0202, .data = gps}).value() ==
        to_vec(GOLDEN(gps_time_req)));
}

TEST_CASE("golden: LiDAR-originated frames parse", "[golden]") {
  auto d = parse_command_frame(GOLDEN(discovery_ack));
  REQUIRE(d);
  CHECK(d->header.cmd_type == CmdType::kAck);
  CHECK(d->header.sender_type == SenderType::kLidar);
  auto da = parse_discovery_ack(d->data);
  REQUIRE(da);
  CHECK(da->serial_number_view() == "47MDL9K0010001");
  CHECK(da->lidar_ip == Ipv4{192, 168, 1, 12});
  CHECK(da->cmd_port == 56100);

  auto ok = parse_command_frame(GOLDEN(param_config_ack_ok));
  REQUIRE(ok);
  CHECK(parse_param_config_ack(ok->data)->ret_code == RetCode::kSuccess);
  auto err = parse_command_frame(GOLDEN(param_config_ack_err));
  REQUIRE(err);
  auto ea = parse_param_config_ack(err->data);
  CHECK(ea->ret_code == RetCode::kParamReadOnly);
  CHECK(ea->error_key == 0x8000);

  auto iq = parse_command_frame(GOLDEN(param_inquire_ack));
  REQUIRE(iq);
  auto ia = parse_param_inquire_ack(iq->data);
  REQUIRE(ia);
  REQUIRE(ia->values.size() == 3);
  CHECK(decode_string(*find_key(ia->values, Key::kSn)) == "47MDL9K0010001");
  CHECK(decode_version(*find_key(ia->values, Key::kVersionApp))->v ==
        std::array<std::uint8_t, 4>{13, 18, 2, 44});
  CHECK(decode_work_state(*find_key(ia->values, Key::kCurWorkState)).value() ==
        WorkState::kSampling);

  auto pu = parse_command_frame(GOLDEN(info_push));
  REQUIRE(pu);
  CHECK(pu->header.cmd_id == static_cast<std::uint16_t>(CmdId::kInfoPush));
  auto push = parse_info_push(pu->data);
  REQUIRE(push);
  REQUIRE(push->values.size() == 4);
  CHECK(decode_i32(*find_key(push->values, Key::kCoreTemp)).value() == 4321);
  auto diag = decode_diag_status(*find_key(push->values, Key::kLidarDiagStatus)).value();
  CHECK(diag.scan == 2);
  auto hms = decode_hms_codes(*find_key(push->values, Key::kHmsCode)).value();
  CHECK(decode_hms(hms[0]).abnormal_id == 0x0102);
  CHECK(decode_hms(hms[1]).level == HmsLevel::kFatal);
  CHECK_FALSE(decode_hms(hms[2]).active());
}

TEST_CASE("golden: data packets", "[golden]") {
  auto p32 = parse_data_packet(GOLDEN(pcl32));
  REQUIRE(p32);
  CHECK(p32->header.length == 1380);
  CHECK(p32->header.dot_num == 96);
  CHECK(p32->header.udp_cnt == 17);
  CHECK(p32->header.time_type == TimeType::kPtp);
  CHECK(p32->header.timestamp_ns == 1700000000123456789ULL);
  auto pts = decode_all_cartesian32(*p32);
  REQUIRE(pts.size() == 96);
  CHECK(pts[0].x_mm == 1000);
  CHECK(pts[95].x_mm == 1095);
  CHECK(pts[95].y_mm == -2000 + 3 * 95);
  CHECK(pts[95].z_mm == 500 - 95);
  CHECK(pts[10].reflectivity == (10 * 7) % 256);
  CHECK(pts[5].tag == ((5 % 4) | ((5 % 3) << 2)));

  auto p16 = parse_data_packet(GOLDEN(pcl16));
  REQUIRE(p16);
  CHECK(p16->header.length == 36 + 96 * 8);
  CHECK(decode_cartesian16(*p16, 95).y_cm == -200 + 3 * 95);

  auto ps = parse_data_packet(GOLDEN(pcl_sph));
  REQUIRE(ps);
  CHECK(ps->header.time_type == TimeType::kGps);
  CHECK(decode_spherical(*ps, 1).theta_centideg == 180);
  CHECK(decode_spherical(*ps, 1).phi_centideg == 360);

  auto imu = parse_data_packet(GOLDEN(imu));
  REQUIRE(imu);
  CHECK(imu->header.data_type == DataType::kImu);
  CHECK(imu->header.dot_num == 1);
  CHECK(imu->header.udp_cnt == 42);
  auto s = decode_imu(*imu, 0);
  CHECK(s.gyro_x == 0.01f);
  CHECK(s.gyro_y == -0.02f);
  CHECK(s.acc_z == 1.0f);
}
