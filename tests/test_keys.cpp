// SPDX-License-Identifier: Apache-2.0
#include <catch2/catch_test_macros.hpp>

#include "livox/mid360/keys.hpp"
#include "test_util.hpp"

using namespace livox::mid360;

TEST_CASE("key metadata", "[keys]") {
  STATIC_CHECK(is_read_only(Key::kSn));
  STATIC_CHECK_FALSE(is_read_only(Key::kWorkTgtMode));
  CHECK(to_string(Key::kPointCloudHostIpCfg) == "pointcloud_host_ipcfg");
  CHECK(key_value_length(Key::kHmsCode) == 32);
  CHECK(key_value_length(Key::kInstallAttitude) == 24);
  CHECK(key_value_length(Key::kProductInfo) == 64);
  CHECK(key_value_length(Key::kLidarDiagStatus) == 2);
}

TEST_CASE("host ip config", "[keys]") {
  HostIpConfig c{{192, 168, 1, 5}, 56301, 56300};
  auto e = encode_host_ip_config(c);
  CHECK(std::vector<std::byte>(e.begin(), e.end()) ==
        bytes_of({192, 168, 1, 5, 0xED, 0xDB, 0xEC, 0xDB}));
  auto d = decode_host_ip_config(e);
  REQUIRE(d);
  CHECK(d->ip == c.ip);
  CHECK(d->dst_port == 56301);
  CHECK(d->src_port == 56300);
  CHECK(decode_host_ip_config(std::span(e).first(7)).error() == KeyError::kWrongLength);
}

TEST_CASE("lidar ip config", "[keys]") {
  LidarIpConfig c{{192, 168, 1, 12}, {255, 255, 255, 0}, {192, 168, 1, 1}};
  auto d = decode_lidar_ip_config(encode_lidar_ip_config(c));
  REQUIRE(d);
  CHECK(d->ip == c.ip);
  CHECK(d->netmask == c.netmask);
  CHECK(d->gateway == c.gateway);
}

TEST_CASE("install attitude and fov", "[keys]") {
  InstallAttitude a{1.5f, -2.5f, 90.0f, 10, -20, 30};
  auto e = encode_install_attitude(a);
  CHECK(e.size() == 24);
  auto d = decode_install_attitude(e);
  REQUIRE(d);
  CHECK(d->roll_deg == 1.5f);
  CHECK(d->pitch_deg == -2.5f);
  CHECK(d->yaw_deg == 90.0f);
  CHECK(d->x_mm == 10);
  CHECK(d->y_mm == -20);
  CHECK(d->z_mm == 30);

  FovConfig f{0, 359, -7, 52, 0};
  auto fd = decode_fov_config(encode_fov_config(f));
  REQUIRE(fd);
  CHECK(fd->yaw_stop_deg == 359);
  CHECK(fd->pitch_start_deg == -7);
  CHECK(fd->pitch_stop_deg == 52);
}

TEST_CASE("func io and imu sensor config", "[keys]") {
  auto io = decode_func_io_config(encode_func_io_config({0, 0, 1, 2}));
  REQUIRE(io);
  CHECK(io->out0 == 1);
  CHECK(io->out1 == 2);

  ImuSensorConfig c{ImuOutputRate::k500Hz, ImuAccelRange::k16g, ImuGyroRange::k250dps};
  auto e = encode_imu_sensor_config(c);
  CHECK(std::vector<std::byte>(e.begin(), e.end()) == bytes_of({1, 2, 3}));
  auto d = decode_imu_sensor_config(e);
  REQUIRE(d);
  CHECK(d->output_rate == ImuOutputRate::k500Hz);
  CHECK(d->gyro_range == ImuGyroRange::k250dps);
  CHECK(decode_imu_sensor_config(bytes_of({0, 0, 8})).error() == KeyError::kOutOfRange);
}

TEST_CASE("read-only decoders", "[keys]") {
  auto v = decode_version(bytes_of({13, 18, 2, 44}));
  REQUIRE(v);
  CHECK(v->v == std::array<std::uint8_t, 4>{13, 18, 2, 44});
  auto m = decode_mac(bytes_of({1, 2, 3, 4, 5, 6}));
  REQUIRE(m);
  CHECK((*m)[5] == 6);
  CHECK(decode_work_state(bytes_of({0x07})).error() == KeyError::kOutOfRange);
  CHECK(decode_work_state(bytes_of({0x06})).value() == WorkState::kMotorStartup);
  auto ds = decode_diag_status(bytes_of({0x21, 0x30}));
  REQUIRE(ds);
  CHECK(ds->system == 1);
  CHECK(ds->scan == 2);
  CHECK(ds->ranging == 0);
  CHECK(ds->communication == 3);
  CHECK(decode_i32(bytes_of({0xE1, 0x10, 0, 0})).value() == 4321);
  CHECK(decode_i64(bytes_of({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF})).value() == -1);
  CHECK(decode_u64(bytes_of({1, 0, 0, 0, 0, 0, 0, 0})).value() == 1);
  CHECK(decode_u32(bytes_of({1, 0, 0})).error() == KeyError::kWrongLength);
  std::vector<std::byte> hms(32);
  hms[0] = std::byte{0x02};
  hms[2] = std::byte{0x02};
  hms[3] = std::byte{0x01};
  auto h = decode_hms_codes(hms);
  REQUIRE(h);
  CHECK((*h)[0] == 0x01020002);
  CHECK((*h)[1] == 0);
  CHECK(decode_string(bytes_of("Mid-360 2021/12/01\0\0\0")) == "Mid-360 2021/12/01");
  CHECK(decode_string(bytes_of("abc")) == "abc");
}
