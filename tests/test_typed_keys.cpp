// SPDX-License-Identifier: Apache-2.0
// Typed key access on Device (issue #57) against tools/livox_mid360_sim.py: set<K>() /
// get<K>() round trips for every modelled writable key, get<K>() / get_many<>() for the
// read-only keys, rejections, and the compile-time rules of key_traits.
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "livox/mid360/mid360.hpp"
#include "sim_process.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{

struct Fixture
{
  std::optional<SimProcess> sim;
  std::string err;
  std::unique_ptr<Context> context;

  Fixture()
  {
    sim = SimProcess::start(err, {"--rate-multiplier", "0.25", "--push-rate", "10"});
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

/// Writes `value`, reads it back and compares the encodings (the value structs have no ==).
template <Key K>
void roundtrip(Device & dev, const key_value_t<K> & value, bool reboot_required = false)
{
  INFO("key " << to_string(K));
  auto set = dev.set<K>(value);
  REQUIRE(set.has_value());
  CHECK(set->reboot_required == reboot_required);
  auto got = dev.get<K>();
  REQUIRE(got.has_value());
  CHECK(key_traits<K>::encode(*got) == key_traits<K>::encode(value));
}

// ---- compile-time rules ---------------------------------------------------------------
static_assert(typed_key<Key::kPclDataType>);
static_assert(writable_key<Key::kPclDataType>);
static_assert(typed_key<Key::kSn> && !writable_key<Key::kSn>);
static_assert(!typed_key<Key::kSpeedMode>, "Mid-360S / 360L keys are not modelled");
static_assert(!typed_key<Key::kPcFreqMod>, "Mid-360S / 360L keys are not modelled");
static_assert(std::is_same_v<key_value_t<Key::kCoreTemp>, std::int32_t>);
static_assert(std::is_same_v<key_value_t<Key::kSn>, std::string>);
static_assert(std::is_same_v<key_value_t<Key::kImuDataEn>, bool>);
// set<K>() requires writable_key<K>; get<K>() only typed_key<K>.
template <Key K>
concept settable = requires(Device & d, const key_value_t<K> & v) { d.template set<K>(v); };
template <Key K>
concept gettable = requires(Device & d) { d.template get<K>(); };
static_assert(!settable<Key::kCurWorkState>);
static_assert(settable<Key::kWorkTgtMode>);
static_assert(gettable<Key::kCurWorkState>);
static_assert(!gettable<Key::kSpeedMode>);
static_assert(!settable<Key::kSpeedMode>);

}  // namespace

TEST_CASE("Device::set<K> / get<K>: every writable key round-trips", "[device][keys][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();

  SECTION("kPclDataType") { roundtrip<Key::kPclDataType>(*dev, DataType::kCartesian16); }
  SECTION("kPatternMode") { roundtrip<Key::kPatternMode>(*dev, std::uint8_t{0}); }
  SECTION("kLidarIpCfg: a change is acknowledged with 0x21")
  {
    auto cur = dev->get<Key::kLidarIpCfg>();
    REQUIRE(cur.has_value());
    LidarIpConfig changed = *cur;
    changed.ip[3] = static_cast<std::uint8_t>(changed.ip[3] + 1);
    roundtrip<Key::kLidarIpCfg>(*dev, changed, true);
    roundtrip<Key::kLidarIpCfg>(*dev, changed, false);  // unchanged: plain success
  }
  // The host addresses are rewritten with what open() configured, so the streams keep
  // pointing at this Context.
  SECTION("kStateInfoHostIpCfg")
  {
    auto cur = dev->get<Key::kStateInfoHostIpCfg>();
    REQUIRE(cur.has_value());
    roundtrip<Key::kStateInfoHostIpCfg>(*dev, *cur);
  }
  SECTION("kPointCloudHostIpCfg")
  {
    auto cur = dev->get<Key::kPointCloudHostIpCfg>();
    REQUIRE(cur.has_value());
    roundtrip<Key::kPointCloudHostIpCfg>(*dev, *cur);
  }
  SECTION("kImuHostIpCfg")
  {
    auto cur = dev->get<Key::kImuHostIpCfg>();
    REQUIRE(cur.has_value());
    roundtrip<Key::kImuHostIpCfg>(*dev, *cur);
  }
  SECTION("kInstallAttitude")
  {
    roundtrip<Key::kInstallAttitude>(
      *dev, InstallAttitude{
              .roll_deg = 1.5F, .pitch_deg = -2.0F, .yaw_deg = 90.0F, .x_mm = 10, .y_mm = -20,
              .z_mm = 300});
  }
  SECTION("kFovCfg0")
  {
    roundtrip<Key::kFovCfg0>(
      *dev, FovConfig{
              .yaw_start_deg = 10, .yaw_stop_deg = 350, .pitch_start_deg = -5,
              .pitch_stop_deg = 50, .rsvd = 0});
  }
  SECTION("kFovCfg1")
  {
    roundtrip<Key::kFovCfg1>(
      *dev, FovConfig{
              .yaw_start_deg = 0, .yaw_stop_deg = 180, .pitch_start_deg = 0,
              .pitch_stop_deg = 30, .rsvd = 0});
  }
  SECTION("kFovCfgEn") { roundtrip<Key::kFovCfgEn>(*dev, FovEnable{.fov0 = true, .fov1 = true}); }
  SECTION("kDetectMode") { roundtrip<Key::kDetectMode>(*dev, DetectMode::kSensitive); }
  SECTION("kFuncIoCfg")
  {
    roundtrip<Key::kFuncIoCfg>(*dev, FuncIoConfig{.in0 = 0, .in1 = 0, .out0 = 2, .out1 = 1});
  }
  SECTION("kWorkTgtMode: only the ACK is awaited")
  {
    roundtrip<Key::kWorkTgtMode>(*dev, WorkState::kReady);
    roundtrip<Key::kWorkTgtMode>(*dev, WorkState::kIdle);
  }
  SECTION("kImuDataEn") { roundtrip<Key::kImuDataEn>(*dev, true); }
  SECTION("kTimeFilter") { roundtrip<Key::kTimeFilter>(*dev, true); }
  SECTION("kImuSensorCfg")
  {
    roundtrip<Key::kImuSensorCfg>(
      *dev, ImuSensorConfig{
              .output_rate = ImuOutputRate::k500Hz, .accel_range = ImuAccelRange::k16g,
              .gyro_range = ImuGyroRange::k500dps});
  }
}

TEST_CASE("Device::get<K>: every read-only key decodes", "[device][keys][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();

  auto sn = dev->get<Key::kSn>();
  REQUIRE(sn.has_value());
  CHECK(*sn == f.sim->sn());
  auto info = dev->get<Key::kProductInfo>();
  REQUIRE(info.has_value());
  CHECK(*info == "MID360-SIM");
  for (auto v : {dev->get<Key::kVersionApp>(), dev->get<Key::kVersionLoader>(),
                 dev->get<Key::kVersionHardware>()}) {
    REQUIRE(v.has_value());
    CHECK(v->v == std::array<std::uint8_t, 4>{0, 0, 0, 1});
  }
  auto mac = dev->get<Key::kMac>();
  REQUIRE(mac.has_value());
  CHECK((*mac)[0] == 2);
  auto state = dev->get<Key::kCurWorkState>();
  REQUIRE(state.has_value());
  CHECK((*state == WorkState::kIdle || *state == WorkState::kSelfCheck));
  auto temp = dev->get<Key::kCoreTemp>();
  REQUIRE(temp.has_value());
  CHECK(*temp == 3500);
  auto cnt = dev->get<Key::kPowerupCnt>();
  REQUIRE(cnt.has_value());
  CHECK(*cnt == 1);
  auto now = dev->get<Key::kLocalTimeNow>();
  REQUIRE(now.has_value());
  CHECK(*now > 0);
  auto sync = dev->get<Key::kLastSyncTime>();
  REQUIRE(sync.has_value());
  CHECK(*sync == 0);
  auto offset = dev->get<Key::kTimeOffset>();
  REQUIRE(offset.has_value());
  CHECK(*offset == 0);
  auto sync_type = dev->get<Key::kTimeSyncType>();
  REQUIRE(sync_type.has_value());
  CHECK(*sync_type == TimeSyncType::kNone);
  auto diag = dev->get<Key::kLidarDiagStatus>();
  REQUIRE(diag.has_value());
  CHECK(diag->system == 0);
  auto fw = dev->get<Key::kFwType>();
  REQUIRE(fw.has_value());
  CHECK(*fw == FwType::kLoader);
  auto hms = dev->get<Key::kHmsCode>();
  REQUIRE(hms.has_value());
  CHECK((*hms)[0] == 0);
}

TEST_CASE("Device::get_many / set_many: one request for several keys", "[device][keys][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();
  const auto before = dev->session_stats();

  auto set = dev->set_many<Key::kDetectMode, Key::kImuDataEn, Key::kFovCfgEn>(
    DetectMode::kSensitive, true, FovEnable{.fov0 = true, .fov1 = false});
  REQUIRE(set.has_value());
  CHECK_FALSE(set->reboot_required);

  auto got = dev->get_many<Key::kDetectMode, Key::kImuDataEn, Key::kFovCfgEn, Key::kSn>();
  REQUIRE(got.has_value());
  const auto & [mode, imu, fov, sn] = *got;
  CHECK(mode == DetectMode::kSensitive);
  CHECK(imu);
  CHECK(fov.fov0);
  CHECK_FALSE(fov.fov1);
  CHECK(sn == f.sim->sn());
  CHECK(dev->session_stats().requests == before.requests + 2);
}

TEST_CASE("Device::set<K>: LiDAR rejections carry ret_code and error_key", "[device][keys][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();

  SECTION("out of range")
  {
    auto r = dev->set<Key::kWorkTgtMode>(WorkState::kError);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == DeviceError::Kind::kSession);
    REQUIRE(r.error().session.has_value());
    CHECK(r.error().session->kind == SessionErrorKind::kLidarRejected);
    CHECK(r.error().session->ret_code == RetCode::kParamNotSupport);
    CHECK(r.error().session->error_key == static_cast<std::uint16_t>(Key::kWorkTgtMode));
    CHECK_FALSE(r.error().key.has_value());
  }
  SECTION("set_many is all-or-nothing")
  {
    auto r = dev->set_many<Key::kDetectMode, Key::kWorkTgtMode>(
      DetectMode::kSensitive, WorkState::kError);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().session->error_key == static_cast<std::uint16_t>(Key::kWorkTgtMode));
    auto mode = dev->get<Key::kDetectMode>();
    REQUIRE(mode.has_value());
    CHECK(*mode == DetectMode::kNormal);
  }
}

TEST_CASE("Device::get<K>: an undecodable value is kDecodeFailed with the key", "[device][keys][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();
  // pattern_mode is a plain u8 to the simulator; 0x05 is out of range for DataType (1..3)
  // and the simulator only range-checks pcl_data_type, so use the raw configure path to
  // plant an undecodable fov_cfg_en (bits above bit 1).
  const auto bad = encode_u8(0x80);
  const KeyValue kv[] = {{static_cast<std::uint16_t>(Key::kFovCfgEn), bad}};
  REQUIRE(dev->configure(kv).has_value());

  auto r = dev->get<Key::kFovCfgEn>();
  REQUIRE_FALSE(r.has_value());
  CHECK(r.error().kind == DeviceError::Kind::kDecodeFailed);
  CHECK(r.error().key == Key::kFovCfgEn);
  CHECK(to_string(r.error()) == "decode_failed: key fov_cfg_en");

  auto many = dev->get_many<Key::kSn, Key::kFovCfgEn, Key::kDetectMode>();
  REQUIRE_FALSE(many.has_value());
  CHECK(many.error().key == Key::kFovCfgEn);
}
