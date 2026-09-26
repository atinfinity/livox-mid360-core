// SPDX-License-Identifier: Apache-2.0
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <limits>
#include <vector>

#include "livox/mid360/keys.hpp"
#include "test_util.hpp"

using namespace livox::mid360;

TEST_CASE("key metadata", "[keys]")
{
  STATIC_CHECK(is_read_only(Key::kSn));
  STATIC_CHECK_FALSE(is_read_only(Key::kWorkTgtMode));
  CHECK(to_string(Key::kPointCloudHostIpCfg) == "pointcloud_host_ipcfg");
  CHECK(key_value_length(Key::kHmsCode) == 32);
  CHECK(key_value_length(Key::kInstallAttitude) == 24);
  CHECK(key_value_length(Key::kProductInfo) == 64);
  CHECK(key_value_length(Key::kLidarDiagStatus) == 2);
}

TEST_CASE("u8 enum and flag codecs (issue #57)", "[keys]")
{
  CHECK(encode_bool(true) == encode_u8(1));
  CHECK(decode_bool(encode_u8(0)).value() == false);
  CHECK(decode_bool(encode_u8(2)).error() == KeyError::kOutOfRange);
  CHECK(decode_data_type(encode_u8(3)).value() == DataType::kSpherical);
  CHECK(decode_data_type(encode_u8(0)).error() == KeyError::kOutOfRange);  // IMU is not settable
  CHECK(decode_data_type(encode_u8(4)).error() == KeyError::kOutOfRange);
  CHECK(
    decode_detect_mode(encode_enum_u8(DetectMode::kSensitive)).value() == DetectMode::kSensitive);
  CHECK(decode_detect_mode(encode_u8(2)).error() == KeyError::kOutOfRange);
  CHECK(decode_time_sync_type(encode_u8(2)).value() == TimeSyncType::kGps);
  CHECK(decode_time_sync_type(encode_u8(3)).error() == KeyError::kOutOfRange);
  CHECK(decode_fw_type(encode_u8(1)).value() == FwType::kApp);
  CHECK(decode_fw_type(std::span<const std::byte>{}).error() == KeyError::kWrongLength);
  const FovEnable both{.fov0 = true, .fov1 = true};
  CHECK(encode_fov_enable(both) == encode_u8(3));
  CHECK(encode_fov_enable({.fov0 = false, .fov1 = true}) == encode_u8(2));
  auto d = decode_fov_enable(encode_u8(1));
  REQUIRE(d);
  CHECK((d->fov0 && !d->fov1));
  CHECK(decode_fov_enable(encode_u8(4)).error() == KeyError::kOutOfRange);
}

TEST_CASE("key_traits (issue #57)", "[keys]")
{
  STATIC_CHECK(typed_key<Key::kFovCfg0>);
  STATIC_CHECK(writable_key<Key::kFovCfg0>);
  STATIC_CHECK(!writable_key<Key::kHmsCode>);
  STATIC_CHECK(!typed_key<Key::kSpeedMode>);
  STATIC_CHECK(std::is_same_v<key_value_t<Key::kMac>, std::array<std::uint8_t, 6>>);
  const auto bytes = key_traits<Key::kImuHostIpCfg>::encode({{10, 0, 0, 1}, 56301, 56300});
  STATIC_CHECK(std::tuple_size_v<decltype(bytes)> == 8);
  auto back = key_traits<Key::kImuHostIpCfg>::decode(bytes);
  REQUIRE(back);
  CHECK(back->dst_port == 56301);
  auto sn = key_traits<Key::kSn>::decode(bytes_of("ABC\0\0"));
  REQUIRE(sn);
  CHECK(*sn == "ABC");
}

TEST_CASE("host ip config", "[keys]")
{
  HostIpConfig c{{192, 168, 1, 5}, 56301, 56300};
  auto e = encode_host_ip_config(c);
  CHECK(
    std::vector<std::byte>(e.begin(), e.end()) ==
    bytes_of({192, 168, 1, 5, 0xED, 0xDB, 0xEC, 0xDB}));
  auto d = decode_host_ip_config(e);
  REQUIRE(d);
  CHECK(d->ip == c.ip);
  CHECK(d->dst_port == 56301);
  CHECK(d->src_port == 56300);
  CHECK(decode_host_ip_config(std::span(e).first(7)).error() == KeyError::kWrongLength);
}

TEST_CASE("lidar ip config", "[keys]")
{
  LidarIpConfig c{{192, 168, 1, 12}, {255, 255, 255, 0}, {192, 168, 1, 1}};
  auto d = decode_lidar_ip_config(encode_lidar_ip_config(c));
  REQUIRE(d);
  CHECK(d->ip == c.ip);
  CHECK(d->netmask == c.netmask);
  CHECK(d->gateway == c.gateway);
}

TEST_CASE("install attitude and fov", "[keys]")
{
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
  // 1.5f = 0x3FC00000, -2.5f = 0xC0200000, 90.0f = 0x42B40000, little-endian, then int32 mm.
  const std::vector<std::byte> golden =
    bytes_of({0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x20, 0xC0, 0x00, 0x00, 0xB4, 0x42,
              0x0A, 0x00, 0x00, 0x00, 0xEC, 0xFF, 0xFF, 0xFF, 0x1E, 0x00, 0x00, 0x00});
  CHECK(std::vector<std::byte>(e.begin(), e.end()) == golden);

  CHECK(install_attitude_valid(a));
  CHECK(install_attitude_valid({.roll_deg = -180.0F, .pitch_deg = 180.0F, .yaw_deg = 0.0F}));
  CHECK_FALSE(install_attitude_valid({.roll_deg = 180.5F}));
  CHECK_FALSE(install_attitude_valid({.pitch_deg = -181.0F}));
  CHECK_FALSE(install_attitude_valid({.yaw_deg = std::numeric_limits<float>::infinity()}));
  CHECK_FALSE(install_attitude_valid({.yaw_deg = std::numeric_limits<float>::quiet_NaN()}));

  FovConfig f{0, 359, -7, 52, 0};
  auto fd = decode_fov_config(encode_fov_config(f));
  REQUIRE(fd);
  CHECK(fd->yaw_stop_deg == 359);
  CHECK(fd->pitch_start_deg == -7);
  CHECK(fd->pitch_stop_deg == 52);
}

TEST_CASE("func io and imu sensor config", "[keys]")
{
  const FuncIoConfig cfg{.out0 = FuncOut::kFollowInput, .out1 = FuncOut::kSafetyZone};
  auto io = decode_func_io_config(encode_func_io_config(cfg));
  REQUIRE(io);
  CHECK(io->in0 == FuncIn0::kPps);
  CHECK(io->in1 == FuncIn1::kGps);
  CHECK(io->out0 == FuncOut::kFollowInput);
  CHECK(io->out1 == FuncOut::kSafetyZone);
  CHECK(func_io_config_valid(cfg));
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange): out-of-enum value on purpose
  CHECK_FALSE(func_io_config_valid({.in0 = static_cast<FuncIn0>(1)}));
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange): out-of-enum value on purpose
  CHECK_FALSE(func_io_config_valid({.in1 = static_cast<FuncIn1>(1)}));
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange): out-of-enum value on purpose
  CHECK_FALSE(func_io_config_valid({.out0 = static_cast<FuncOut>(3)}));
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange): out-of-enum value on purpose
  CHECK_FALSE(func_io_config_valid({.out1 = static_cast<FuncOut>(255)}));
  CHECK(decode_func_io_config(bytes_of({0, 0, 0, 3})).error() == KeyError::kOutOfRange);
  CHECK(decode_func_io_config(bytes_of({1, 0, 0, 0})).error() == KeyError::kOutOfRange);
  CHECK(decode_func_io_config(bytes_of({0, 0, 2})).error() == KeyError::kWrongLength);

  ImuSensorConfig c{ImuOutputRate::k500Hz, ImuAccelRange::k16g, ImuGyroRange::k250dps};
  auto e = encode_imu_sensor_config(c);
  CHECK(std::vector<std::byte>(e.begin(), e.end()) == bytes_of({1, 2, 3}));
  auto d = decode_imu_sensor_config(e);
  REQUIRE(d);
  CHECK(d->output_rate == ImuOutputRate::k500Hz);
  CHECK(d->gyro_range == ImuGyroRange::k250dps);
  CHECK(decode_imu_sensor_config(bytes_of({0, 0, 8})).error() == KeyError::kOutOfRange);
}

TEST_CASE("read-only decoders", "[keys]")
{
  auto v = decode_version(bytes_of({13, 18, 2, 44}));
  REQUIRE(v);
  CHECK(v->v == std::array<std::uint8_t, 4>{13, 18, 2, 44});
  auto m = decode_mac(bytes_of({1, 2, 3, 4, 5, 6}));
  REQUIRE(m);
  CHECK((*m)[5] == 6);
  CHECK(decode_work_state(bytes_of({0x07})).error() == KeyError::kOutOfRange);
  CHECK(decode_work_state(bytes_of({0x06})).value() == WorkState::kMotorStartup);
  const LidarIpConfig good{
    .ip = {192, 168, 1, 12}, .netmask = {255, 255, 255, 0}, .gateway = {192, 168, 1, 1}};
  CHECK(lidar_ip_config_valid(good));
  CHECK(lidar_ip_config_valid({.ip = {10, 0, 0, 2}, .netmask = {255, 0, 0, 0}, .gateway = {}}));
  CHECK_FALSE(lidar_ip_config_valid({.ip = {}, .netmask = {255, 255, 255, 0}, .gateway = {}}));
  CHECK_FALSE(lidar_ip_config_valid({.ip = {255, 255, 255, 255}, .netmask = {255, 255, 255, 0}}));
  CHECK_FALSE(lidar_ip_config_valid({.ip = {192, 168, 1, 0}, .netmask = {255, 255, 255, 0}}));
  CHECK_FALSE(lidar_ip_config_valid({.ip = {192, 168, 1, 255}, .netmask = {255, 255, 255, 0}}));
  CHECK_FALSE(lidar_ip_config_valid({.ip = {192, 168, 1, 12}, .netmask = {255, 0, 255, 0}}));
  CHECK_FALSE(lidar_ip_config_valid({.ip = {192, 168, 1, 12}, .netmask = {}}));
  CHECK_FALSE(lidar_ip_config_valid({.ip = {192, 168, 1, 12}, .netmask = {255, 255, 255, 255}}));
  CHECK_FALSE(lidar_ip_config_valid({.ip = {192, 168, 1, 12}, .netmask = {255, 255, 255, 254}}));
  CHECK_FALSE(lidar_ip_config_valid(
    {.ip = {192, 168, 1, 12}, .netmask = {255, 255, 255, 0}, .gateway = {192, 168, 2, 1}}));
  CHECK_FALSE(lidar_ip_config_valid(
    {.ip = {192, 168, 1, 12}, .netmask = {255, 255, 255, 0}, .gateway = {192, 168, 1, 12}}));
  CHECK_FALSE(lidar_ip_config_valid(
    {.ip = {192, 168, 1, 12}, .netmask = {255, 255, 255, 0}, .gateway = {192, 168, 1, 255}}));
  auto ds = decode_diag_status(bytes_of({0x21, 0x30}));
  REQUIRE(ds);
  CHECK(ds->system == DiagLevel::kWarning);
  CHECK(ds->scan == DiagLevel::kError);
  CHECK(ds->ranging == DiagLevel::kNormal);
  CHECK(ds->communication == DiagLevel::kSafetyError);
  CHECK(ds->worst() == DiagLevel::kSafetyError);
  CHECK_FALSE(ds->normal());
  CHECK(
    *ds == DiagStatus{
             .system = DiagLevel::kWarning,
             .scan = DiagLevel::kError,
             .ranging = DiagLevel::kNormal,
             .communication = DiagLevel::kSafetyError});
  CHECK(DiagStatus{}.normal());
  CHECK(decode_diag_status(bytes_of({0x04, 0x00})).error() == KeyError::kOutOfRange);
  CHECK(decode_diag_status(bytes_of({0x00, 0x40})).error() == KeyError::kOutOfRange);
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
