// SPDX-License-Identifier: Apache-2.0
// DeviceIdentity (issue #38): decode_identity() on hand-built key lists, to_string(), and
// Device::identity() against tools/livox_mid360_sim.py with the identity CLI options.
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "livox/mid360/mid360.hpp"
#include "sim_process.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{

std::vector<std::byte> bytes_of(std::string_view s, std::size_t width)
{
  std::vector<std::byte> out(width, std::byte{0});
  for (std::size_t i = 0; i < s.size() && i < width; ++i) {
    out[i] = static_cast<std::byte>(s[i]);
  }
  return out;
}

struct Fixture
{
  std::optional<SimProcess> sim;
  std::string err;
  std::unique_ptr<Context> context;

  explicit Fixture(std::vector<std::string> extra = {})
  {
    extra.insert(extra.end(), {"--rate-multiplier", "0.25", "--push-rate", "10"});
    sim = SimProcess::start(err, std::move(extra));
    if (sim) {
      ContextOptions o;
      o.bind_address = {127, 0, 0, 1};
      o.push_port = o.point_port = o.imu_port = 0;
      auto c = Context::create(o);
      REQUIRE(c.has_value());
      context = std::move(*c);
    }
  }

  [[nodiscard]] std::unique_ptr<Device> open() const
  {
    DeviceOptions o;
    o.session.host_command_port = 0;
    o.session.request = {.timeout = 500ms, .attempts = 3};
    auto d = Device::open(
      *context,
      DiscoveredDevice{
        .serial_number = sim->sn(),
        .ip = {127, 0, 0, 1},
        .cmd_port = sim->ports().cmd,
        .dev_type = 9,
        .from = Endpoint::loopback(sim->ports().cmd)},
      o);
    if (!d) {
      FAIL(to_string(d.error()));
    }
    return std::move(*d);
  }
};

}  // namespace

TEST_CASE("to_string(Version) is aa.bb.cc.dd", "[lidar_info]")
{
  CHECK(to_string(Version{.v = {13, 18, 0, 244}}) == "13.18.0.244");
  CHECK(to_string(Version{}) == "0.0.0.0");
}

TEST_CASE("decode_identity fills every field from a key list", "[lidar_info]")
{
  const auto sn = bytes_of("SN123", 16);
  const auto product = bytes_of("MID-360", 64);
  const std::array<std::byte, 4> app{std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}};
  const std::array<std::byte, 4> loader{std::byte{5}, std::byte{6}, std::byte{7}, std::byte{8}};
  const std::array<std::byte, 4> hw{std::byte{9}, std::byte{10}, std::byte{11}, std::byte{12}};
  const std::array<std::byte, 6> mac{std::byte{0xaa}, std::byte{0xbb}, std::byte{0xcc},
                                     std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
  const std::vector<KeyValue> kvs{
    {static_cast<std::uint16_t>(Key::kSn), sn},
    {static_cast<std::uint16_t>(Key::kProductInfo), product},
    {static_cast<std::uint16_t>(Key::kVersionApp), app},
    {static_cast<std::uint16_t>(Key::kVersionLoader), loader},
    {static_cast<std::uint16_t>(Key::kVersionHardware), hw},
    {static_cast<std::uint16_t>(Key::kMac), mac},
  };
  const auto id = decode_identity(kvs);
  CHECK(id.serial_number == "SN123");
  CHECK(id.product_info == "MID-360");
  CHECK(id.version_app.v == std::array<std::uint8_t, 4>{1, 2, 3, 4});
  CHECK(id.version_loader.v == std::array<std::uint8_t, 4>{5, 6, 7, 8});
  CHECK(id.version_hardware.v == std::array<std::uint8_t, 4>{9, 10, 11, 12});
  CHECK(id.mac == std::array<std::uint8_t, 6>{0xaa, 0xbb, 0xcc, 0x01, 0x02, 0x03});
  CHECK(
    to_string(id) ==
    "sn=SN123 product_info=MID-360 version_app=1.2.3.4 version_loader=5.6.7.8 "
    "version_hardware=9.10.11.12 mac=aa:bb:cc:01:02:03");
}

TEST_CASE("decode_identity tolerates missing and malformed keys", "[lidar_info]")
{
  const auto sn = bytes_of("ONLY", 16);
  const std::array<std::byte, 2> short_version{std::byte{1}, std::byte{2}};
  const std::vector<KeyValue> kvs{
    {static_cast<std::uint16_t>(Key::kSn), sn},
    {static_cast<std::uint16_t>(Key::kVersionApp), short_version},
  };
  const auto id = decode_identity(kvs);
  CHECK(id.serial_number == "ONLY");
  CHECK(id.product_info.empty());
  CHECK(id.version_app.v == std::array<std::uint8_t, 4>{});
  CHECK(id.mac == std::array<std::uint8_t, 6>{});
  CHECK(decode_identity({}).serial_number.empty());
}

TEST_CASE("kIdentityKeys lists 0x8000-0x8005 in order", "[lidar_info]")
{
  for (std::size_t i = 0; i < kIdentityKeys.size(); ++i) {
    CHECK(static_cast<std::uint16_t>(kIdentityKeys[i]) == 0x8000 + i);
  }
}

TEST_CASE("Device::identity reads the simulator defaults", "[lidar_info][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();
  auto id = dev->identity();
  REQUIRE(id.has_value());
  CHECK(id->serial_number == f.sim->sn());
  CHECK(id->product_info == "MID360-SIM");
  CHECK(to_string(id->version_app) == "0.0.0.1");
  CHECK(to_string(id->version_loader) == "0.0.0.1");
  CHECK(to_string(id->version_hardware) == "0.0.0.1");
  CHECK(id->mac == std::array<std::uint8_t, 6>{2, 0, 0, 0, 0, 1});
  CHECK(to_string(*id).starts_with("sn=" + f.sim->sn() + " product_info=MID360-SIM"));
}

TEST_CASE("Device::identity reflects the simulator identity options", "[lidar_info][sim]")
{
  Fixture f(
    {"--sn", "TESTSN01", "--product-info", "MID-360", "--version-app", "13.18.0.244",
     "--version-loader", "1.2.3.4", "--version-hardware", "5.6.7.8"});
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();
  auto id = dev->identity();
  REQUIRE(id.has_value());
  CHECK(id->serial_number == "TESTSN01");
  CHECK(id->product_info == "MID-360");
  CHECK(to_string(id->version_app) == "13.18.0.244");
  CHECK(to_string(id->version_loader) == "1.2.3.4");
  CHECK(to_string(id->version_hardware) == "5.6.7.8");
}

TEST_CASE("Device::identity fails when the LiDAR does not answer", "[lidar_info][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();
  REQUIRE(f.sim->control(R"({"cmd":"silence","seconds":1.5})"));
  // The simulator serves its control line and the command socket from one select() wake:
  // without this the request can be answered before the silence takes effect.
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());
  auto id = dev->identity(RequestOptions{.timeout = 50ms, .attempts = 1});
  REQUIRE_FALSE(id.has_value());
  CHECK(id.error().kind == DeviceError::Kind::kSession);
}

// --- settings / status (issue #41) -------------------------------------------------------

namespace
{
template <Key K>
KeyValue kv(std::span<const std::byte> bytes)
{
  return {static_cast<std::uint16_t>(K), bytes};
}
}  // namespace

TEST_CASE("to_string of the typed key values", "[lidar_info]")
{
  CHECK(
    to_string(HostIpConfig{.ip = {192, 168, 1, 5}, .dst_port = 56301, .src_port = 56201}) ==
    "192.168.1.5:56301<-56201");
  CHECK(
    to_string(LidarIpConfig{
      .ip = {192, 168, 1, 12}, .netmask = {255, 255, 255, 0}, .gateway = {192, 168, 1, 1}}) ==
    "192.168.1.12/255.255.255.0/192.168.1.1");
  CHECK(
    to_string(InstallAttitude{
      .roll_deg = 1.5F, .pitch_deg = -2, .yaw_deg = 0, .x_mm = 10, .y_mm = 20, .z_mm = 30}) ==
    "r1.5/p-2.0/y0.0/x10/y20/z30");
  CHECK(
    to_string(FovConfig{
      .yaw_start_deg = 0, .yaw_stop_deg = 360, .pitch_start_deg = -7, .pitch_stop_deg = 52}) ==
    "yaw0-360/pitch-7-52");
  CHECK(to_string(FovEnable{.fov0 = true, .fov1 = false}) == "fov0:1,fov1:0");
  CHECK(
    to_string(FuncIoConfig{.out0 = FuncOut::kFollowInput, .out1 = FuncOut::kSafetyZone}) ==
    "pps/gps/follow_input/safety_zone");
  CHECK(to_string(ImuSensorConfig{}) == "200Hz/4g/2000dps");
  CHECK(
    to_string(ImuSensorConfig{
      .output_rate = ImuOutputRate::k50Hz,
      .accel_range = ImuAccelRange::k32g,
      .gyro_range = ImuGyroRange::k15_625dps}) == "50Hz/32g/15.625dps");
  CHECK(
    to_string(DiagStatus{
      .system = DiagLevel::kNormal,
      .scan = DiagLevel::kWarning,
      .ranging = DiagLevel::kError,
      .communication = DiagLevel::kSafetyError}) == "sys0/scan1/rng2/comm3");
  CHECK(to_string(DiagLevel::kSafetyError) == "safety_error");
  CHECK(to_string(DetectMode::kSensitive) == "sensitive");
  CHECK(to_string(TimeSyncType::kPtp) == "ptp");
  CHECK(to_string(FwType::kApp) == "app");
}

TEST_CASE("decode_settings fills every key and skips bad ones", "[lidar_info]")
{
  const auto data_type = std::vector<std::byte>{std::byte{2}};
  const auto pattern = std::vector<std::byte>{std::byte{0}};
  const auto tgt = std::vector<std::byte>{std::byte{1}};
  const auto on = std::vector<std::byte>{std::byte{1}};
  const auto fov_en = std::vector<std::byte>{std::byte{3}};
  const auto bad_detect = std::vector<std::byte>{std::byte{7}};
  const auto fov = encode_fov_config(
    FovConfig{.yaw_start_deg = 10, .yaw_stop_deg = 20, .pitch_start_deg = 0, .pitch_stop_deg = 5});
  const auto imu = encode_imu_sensor_config(ImuSensorConfig{
    .output_rate = ImuOutputRate::k500Hz,
    .accel_range = ImuAccelRange::k8g,
    .gyro_range = ImuGyroRange::k500dps});
  const std::vector<KeyValue> kvs{
    kv<Key::kPclDataType>(data_type), kv<Key::kPatternMode>(pattern),   kv<Key::kFovCfg1>(fov),
    kv<Key::kFovCfgEn>(fov_en),       kv<Key::kDetectMode>(bad_detect), kv<Key::kWorkTgtMode>(tgt),
    kv<Key::kImuDataEn>(on),          kv<Key::kTimeFilter>(on),         kv<Key::kImuSensorCfg>(imu),
  };
  const auto s = decode_settings(kvs);
  CHECK(s.pcl_data_type == DataType::kCartesian16);
  CHECK(s.pattern_mode == ScanPattern::kNonRepetitive);
  CHECK_FALSE(s.lidar_ipcfg.has_value());
  CHECK_FALSE(s.fov_cfg0.has_value());
  REQUIRE(s.fov_cfg1.has_value());
  CHECK(s.fov_cfg1->yaw_stop_deg == 20);
  REQUIRE(s.fov_cfg_en.has_value());
  CHECK((s.fov_cfg_en->fov0 && s.fov_cfg_en->fov1));
  CHECK_FALSE(s.detect_mode.has_value());  // 7 is out of range
  CHECK(s.work_tgt_mode == WorkState::kSampling);
  CHECK(s.imu_data_en == true);
  CHECK(s.time_filter == true);
  REQUIRE(s.imu_sensor_cfg.has_value());
  CHECK(s.imu_sensor_cfg->gyro_range == ImuGyroRange::k500dps);
  CHECK(
    to_string(s) ==
    "pcl_data_type=CARTESIAN16 pattern_mode=non_repetitive fov_cfg1=yaw10-20/pitch0-5 "
    "fov_cfg_en=fov0:1,fov1:1 "
    "work_tgt_mode=SAMPLING imu_data_en=1 time_filter=1 imu_sensor_cfg=500Hz/8g/500dps");
  CHECK(to_string(decode_settings({})).empty());
}

TEST_CASE("decode_status fills every key and formats the line", "[lidar_info]")
{
  const auto state = std::vector<std::byte>{std::byte{0x02}};
  std::vector<std::byte> temp(4);
  bytes::write_le<std::int32_t>(temp, 0, 3512);
  std::vector<std::byte> cnt(4);
  bytes::write_le<std::uint32_t>(cnt, 0, 7);
  std::vector<std::byte> now(8);
  bytes::write_le<std::uint64_t>(now, 0, 1'000'000'000);
  std::vector<std::byte> offset(8);
  bytes::write_le<std::int64_t>(offset, 0, -5);
  const auto sync = std::vector<std::byte>{std::byte{2}};
  std::vector<std::byte> diag(2);
  bytes::write_le<std::uint16_t>(diag, 0, 0x0021);  // system 1, scan 2
  const auto fw = std::vector<std::byte>{std::byte{1}};
  std::vector<std::byte> hms(32);
  bytes::write_le<std::uint32_t>(hms, 0, 0x01030002);
  bytes::write_le<std::uint32_t>(hms, 8, 0x02010003);
  const std::vector<KeyValue> kvs{
    kv<Key::kCurWorkState>(state),   kv<Key::kCoreTemp>(temp),     kv<Key::kPowerupCnt>(cnt),
    kv<Key::kLocalTimeNow>(now),     kv<Key::kTimeOffset>(offset), kv<Key::kTimeSyncType>(sync),
    kv<Key::kLidarDiagStatus>(diag), kv<Key::kFwType>(fw),         kv<Key::kHmsCode>(hms),
  };
  const auto s = decode_status(kvs);
  CHECK(s.cur_work_state == WorkState::kIdle);
  CHECK(s.core_temp == 3512);
  CHECK(s.powerup_cnt == 7);
  CHECK(s.local_time_now == 1'000'000'000);
  CHECK_FALSE(s.last_sync_time.has_value());
  CHECK(s.time_offset == -5);
  CHECK(s.time_sync_type == TimeSyncType::kGps);
  REQUIRE(s.lidar_diag_status.has_value());
  CHECK(s.lidar_diag_status->scan == DiagLevel::kError);
  CHECK(s.time_ns == 0);
  CHECK(s.fw_type == FwType::kApp);
  REQUIRE(s.hms_code.has_value());
  CHECK((*s.hms_code)[0].raw == 0x01030002);
  CHECK((*s.hms_code)[2].raw == 0x02010003);
  CHECK(
    to_string(s) ==
    "cur_work_state=IDLE core_temp=35.12C powerup_cnt=7 local_time_now=1000000000 "
    "time_offset=-5 time_sync_type=gps lidar_diag_status=sys1/scan2/rng0/comm0 fw_type=app "
    "hms=[0x01030002:" +
      std::string(to_string(decode_hms(0x01030002).level)) +
      ",0x02010003:" + std::string(to_string(decode_hms(0x02010003).level)) + "]");
  CHECK(to_string(decode_status({})).empty());
}

TEST_CASE("kSettingsKeys / kStatusKeys are the modelled keys", "[lidar_info]")
{
  CHECK(kSettingsKeys.size() == 16);
  for (Key k : kSettingsKeys) {
    CHECK(static_cast<std::uint16_t>(k) < 0x8000);
  }
  CHECK(kStatusKeys.front() == Key::kCurWorkState);
  CHECK(kStatusKeys.back() == Key::kHmsCode);
}

TEST_CASE("Device::settings and status read every key from the simulator", "[lidar_info][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();
  const auto s = dev->settings();
  REQUIRE(s.has_value());
  CHECK(s->pcl_data_type.has_value());
  CHECK(s->pattern_mode.has_value());
  CHECK(s->lidar_ipcfg.has_value());
  CHECK(s->state_info_host_ipcfg.has_value());
  CHECK(s->pointcloud_host_ipcfg.has_value());
  CHECK(s->imu_host_ipcfg.has_value());
  CHECK(s->install_attitude.has_value());
  CHECK(s->fov_cfg0.has_value());
  CHECK(s->fov_cfg1.has_value());
  CHECK(s->fov_cfg_en.has_value());
  CHECK(s->detect_mode.has_value());
  CHECK(s->func_io_cfg.has_value());
  CHECK(s->work_tgt_mode.has_value());
  CHECK(s->imu_data_en.has_value());
  CHECK(s->time_filter.has_value());
  CHECK(s->imu_sensor_cfg.has_value());
  // open() pointed the state-info push at this host.
  CHECK(s->state_info_host_ipcfg->ip == std::array<std::uint8_t, 4>{127, 0, 0, 1});
  CHECK_FALSE(to_string(*s).empty());

  const auto st = dev->status();
  REQUIRE(st.has_value());
  CHECK(st->cur_work_state.has_value());
  CHECK(st->core_temp == 3500);
  CHECK(st->powerup_cnt.has_value());
  CHECK(st->local_time_now.has_value());
  CHECK(st->last_sync_time.has_value());
  CHECK(st->time_offset.has_value());
  CHECK(st->time_sync_type == TimeSyncType::kNone);
  CHECK(st->lidar_diag_status.has_value());
  CHECK(st->fw_type == FwType::kLoader);
  REQUIRE(st->hms_code.has_value());
  CHECK(to_string(*st).find("core_temp=35.00C") != std::string::npos);
  CHECK(to_string(*st).find("hms=[]") != std::string::npos);
}

TEST_CASE("Device::pushed_status follows the simulator push", "[lidar_info][sim]")
{
  Fixture f;  // --push-rate 10
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();
  auto wait_for = [&](auto pred) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (auto p = dev->pushed_status(); p && pred(*p)) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  };
  REQUIRE(wait_for([](const LidarStatus & p) { return p.cur_work_state.has_value(); }));
  const auto first = *dev->pushed_status();
  CHECK(first.core_temp == 3500);
  CHECK(first.local_time_now.has_value());
  CHECK(first.hms_code.has_value());
  CHECK(first.lidar_diag_status.has_value());
  CHECK(first.cur_work_state == dev->work_state());

  REQUIRE(f.sim->control(R"({"cmd":"set_state","state":1})"));
  CHECK(wait_for([](const LidarStatus & p) { return p.cur_work_state == WorkState::kSampling; }));
  CHECK(dev->work_state() == WorkState::kSampling);

  REQUIRE(f.sim->control(R"({"cmd":"hms","codes":[16973826]})"));  // 0x01030002
  CHECK(wait_for(
    [](const LidarStatus & p) { return p.hms_code && (*p.hms_code)[0].raw == 0x01030002; }));
  CHECK(dev->hms()[0].raw == 0x01030002);
  CHECK(to_string(*dev->pushed_status()).find("hms=[0x01030002:") != std::string::npos);
}
