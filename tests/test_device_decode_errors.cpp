// SPDX-License-Identifier: Apache-2.0
// Argument validation and key decoding branches of device.cpp (issue #98) against
// tools/livox_mid360_sim.py: Device::open() options, set_fov() arguments, inquiry answers
// with a key missing, unsupported or undecodable (fov(), time_sync_status(), settings()),
// rejected set_gps_time() / reboot(), and accepted 0x0100 values the library cannot decode.
// The answers are shaped with the simulator's `inquire_override` and `fail_cmd` controls.
#include <array>
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "livox/mid360/mid360.hpp"
#include "sim_process.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{

bool wait_until(const std::function<bool()> & pred, std::chrono::milliseconds timeout = 5s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return pred();
}

struct Fixture
{
  std::optional<SimProcess> sim;
  std::string err;
  std::unique_ptr<Context> context;
  std::atomic<std::uint64_t> disconnected{0};

  Fixture()
  {
    sim = SimProcess::start(err, {"--rate-multiplier", "0.05", "--push-rate", "10"});
    if (!sim) {
      SKIP("simulator unavailable: " << err);
    }
    ContextOptions o;
    o.bind_address = {127, 0, 0, 1};
    o.push_port = o.point_port = o.imu_port = o.log_port = 0;
    auto c = Context::create(o);
    REQUIRE(c.has_value());
    context = std::move(*c);
  }

  [[nodiscard]] DiscoveredDevice discovered() const
  {
    const Ipv4 ip{127, 0, 0, 1};
    return DiscoveredDevice{
      .serial_number = sim->sn(),
      .ip = ip,
      .cmd_port = sim->ports().cmd,
      .dev_type = 9,
      .from = Endpoint{ip, sim->ports().cmd}};
  }

  [[nodiscard]] DeviceOptions options() const
  {
    DeviceOptions o;
    o.session.host_command_port = 0;
    o.session.request = {.timeout = 200ms, .attempts = 2};
    o.reconnect.enabled = false;
    o.reconnect.discovery_timeout = 200ms;
    o.reconnect.discovery_port = sim->ports().discovery;
    o.lidar_log_port = sim->ports().log;
    return o;
  }

  [[nodiscard]] std::unique_ptr<Device> open()
  {
    auto d = Device::open(*context, discovered(), options());
    if (!d) {
      FAIL(to_string(d.error()));
    }
    REQUIRE((*d)
              ->on_event([this](const Event & e) {
                if (e.kind == Event::Kind::kDisconnected) {
                  ++disconnected;
                }
              })
              .has_value());
    return std::move(*d);
  }

  void control(const std::string & json)
  {
    REQUIRE(sim->control(json));
    REQUIRE(sim->wait_event(R"("event":"control")").has_value());
  }

  void override_value(Key key, const std::string & hex)
  {
    control(std::format(
      R"({{"cmd":"inquire_override","key":{},"value":"{}"}})", static_cast<unsigned>(key), hex));
  }

  void override_flag(Key key, const std::string & flag)
  {
    control(std::format(
      R"({{"cmd":"inquire_override","key":{},"{}":true}})", static_cast<unsigned>(key), flag));
  }
};

void check_rejected(const DeviceError & e, CmdId cmd)
{
  REQUIRE(e.kind == DeviceError::Kind::kSession);
  REQUIRE(e.session.has_value());
  CHECK(e.session->kind == SessionErrorKind::kLidarRejected);
  CHECK(e.session->cmd_id == static_cast<std::uint16_t>(cmd));
  CHECK(e.session->ret_code == static_cast<RetCode>(0x02));
}

}  // namespace

TEST_CASE("Device::open rejects options out of range", "[sim][device]")
{
  Fixture f;
  const auto rejected = [&](const std::function<void(DeviceOptions &)> & edit) {
    DeviceOptions o = f.options();
    edit(o);
    const auto d = Device::open(*f.context, f.discovered(), o);
    REQUIRE_FALSE(d.has_value());
    return d.error().kind == DeviceError::Kind::kInvalidArgument;
  };
  CHECK(rejected([](DeviceOptions & o) { o.frame_policy.window = 0ms; }));
  CHECK(rejected([](DeviceOptions & o) { o.frame_policy.window = -1ms; }));
  CHECK(rejected([](DeviceOptions & o) { o.stats_interval = -1s; }));
  CHECK(rejected([](DeviceOptions & o) { o.reconnect.push_timeout = 0ms; }));
  CHECK(rejected([](DeviceOptions & o) { o.reconnect.initial_backoff = 0ms; }));
  CHECK(rejected([](DeviceOptions & o) {
    o.reconnect.initial_backoff = 200ms;
    o.reconnect.max_backoff = 100ms;
  }));
  CHECK(rejected([](DeviceOptions & o) { o.reconnect.discovery_timeout = 0ms; }));
  // Nothing was registered by the refused calls.
  DeviceOptions o = f.options();
  o.stats_interval = 0s;
  CHECK(Device::open(*f.context, f.discovered(), o).has_value());
}

TEST_CASE("Device::set_fov validates its argument before any I/O", "[sim][device]")
{
  Fixture f;
  auto dev = f.open();

  const auto empty = dev->set_fov(FovSettings{});
  REQUIRE_FALSE(empty.has_value());
  CHECK(empty.error().kind == DeviceError::Kind::kInvalidArgument);
  CHECK_FALSE(empty.error().key.has_value());

  const FovConfig good{.yaw_start_deg = 10, .yaw_stop_deg = 90};
  const auto bad0 =
    dev->set_fov(FovSettings{.fov0 = FovConfig{.yaw_start_deg = 360}, .fov1 = {}, .enable = {}});
  REQUIRE_FALSE(bad0.has_value());
  CHECK(bad0.error().kind == DeviceError::Kind::kInvalidArgument);
  CHECK(bad0.error().key == Key::kFovCfg0);

  const auto bad1 =
    dev->set_fov(FovSettings{.fov0 = good, .fov1 = FovConfig{.pitch_stop_deg = 60}, .enable = {}});
  REQUIRE_FALSE(bad1.has_value());
  CHECK(bad1.error().kind == DeviceError::Kind::kInvalidArgument);
  CHECK(bad1.error().key == Key::kFovCfg1);

  // Nothing reached the LiDAR.
  const auto before = dev->fov();
  REQUIRE(before.has_value());
  CHECK((!before->fov0 || before->fov0->yaw_stop_deg != good.yaw_stop_deg));

  // One field alone is a valid request.
  REQUIRE(dev->set_fov(FovSettings{.fov0 = {}, .fov1 = good, .enable = {}}).has_value());
  const auto after = dev->fov();
  REQUIRE(after.has_value());
  REQUIRE(after->fov1.has_value());
  CHECK(after->fov1->yaw_start_deg == 10);
  CHECK(after->fov1->yaw_stop_deg == 90);
}

TEST_CASE("Device::fov leaves a missing or undecodable key empty", "[sim][device]")
{
  Fixture f;
  auto dev = f.open();
  const auto all = dev->fov();
  REQUIRE(all.has_value());
  REQUIRE(all->fov0.has_value());
  REQUIRE(all->fov1.has_value());
  REQUIRE(all->enable.has_value());

  f.override_flag(Key::kFovCfg0, "omit");
  f.override_value(Key::kFovCfg1, "0102030405");
  f.override_value(Key::kFovCfgEn, "");
  auto r = dev->fov();
  REQUIRE(r.has_value());
  CHECK_FALSE(r->fov0.has_value());
  CHECK_FALSE(r->fov1.has_value());
  CHECK_FALSE(r->enable.has_value());

  f.override_value(Key::kFovCfg0, "0102");
  f.override_flag(Key::kFovCfg1, "omit");
  f.override_flag(Key::kFovCfgEn, "omit");
  r = dev->fov();
  REQUIRE(r.has_value());
  CHECK_FALSE(r->fov0.has_value());
  CHECK_FALSE(r->fov1.has_value());
  CHECK_FALSE(r->enable.has_value());

  f.override_flag(Key::kFovCfg0, "clear");
  r = dev->fov();
  REQUIRE(r.has_value());
  CHECK(r->fov0.has_value());
  CHECK_FALSE(r->fov1.has_value());
}

TEST_CASE("Device::time_sync_status names the key it could not decode", "[sim][device]")
{
  Fixture f;
  auto dev = f.open();
  REQUIRE(dev->time_sync_status().has_value());

  for (const Key key :
       {Key::kLocalTimeNow, Key::kLastSyncTime, Key::kTimeOffset, Key::kTimeSyncType}) {
    CAPTURE(to_string(key));
    f.override_flag(key, "omit");
    const auto missing = dev->time_sync_status();
    REQUIRE_FALSE(missing.has_value());
    CHECK(missing.error().kind == DeviceError::Kind::kDecodeFailed);
    CHECK(missing.error().key == key);

    f.override_value(key, "010203");
    const auto bad = dev->time_sync_status();
    REQUIRE_FALSE(bad.has_value());
    CHECK(bad.error().kind == DeviceError::Kind::kDecodeFailed);
    CHECK(bad.error().key == key);

    f.override_flag(key, "clear");
    CHECK(dev->time_sync_status().has_value());
  }
}

TEST_CASE("Device::settings drops the keys the firmware does not know", "[sim][device]")
{
  Fixture f;
  auto dev = f.open();
  const auto all = dev->settings();
  REQUIRE(all.has_value());
  REQUIRE(all->detect_mode.has_value());
  REQUIRE(all->imu_sensor_cfg.has_value());
  REQUIRE(all->pcl_data_type.has_value());

  SECTION("two unsupported keys")
  {
    f.override_flag(Key::kImuSensorCfg, "unsupported");
    f.override_flag(Key::kDetectMode, "unsupported");
    const auto r = dev->settings();
    REQUIRE(r.has_value());
    CHECK_FALSE(r->detect_mode.has_value());
    CHECK_FALSE(r->imu_sensor_cfg.has_value());
    CHECK(r->pcl_data_type == all->pcl_data_type);
    CHECK(r->fov_cfg_en.has_value());
    CHECK(r->time_filter.has_value());
  }

  SECTION("undecodable and missing values")
  {
    f.override_value(Key::kPclDataType, "0102");
    f.override_value(Key::kLidarIpCfg, "c0a8");
    f.override_flag(Key::kTimeFilter, "omit");
    const auto r = dev->settings();
    REQUIRE(r.has_value());
    CHECK_FALSE(r->pcl_data_type.has_value());
    CHECK_FALSE(r->lidar_ipcfg.has_value());
    CHECK_FALSE(r->time_filter.has_value());
    CHECK(r->detect_mode == all->detect_mode);
  }

  SECTION("another rejection is reported as is")
  {
    f.control(R"({"cmd":"fail_cmd","cmd_id":257,"ret":2})");
    const auto r = dev->settings();
    REQUIRE_FALSE(r.has_value());
    check_rejected(r.error(), CmdId::kParamInquire);
    CHECK(dev->settings().has_value());
  }
}

TEST_CASE("Device: a rejected set_gps_time or reboot keeps the connection", "[sim][device]")
{
  Fixture f;
  auto dev = f.open();

  f.control(R"({"cmd":"fail_cmd","cmd_id":514,"ret":2})");
  const auto gps = dev->set_gps_time(1'000'000'000ULL);
  REQUIRE_FALSE(gps.has_value());
  check_rejected(gps.error(), CmdId::kSetGpsTimestamp);
  CHECK(dev->set_gps_time(1'000'000'000ULL).has_value());

  f.control(R"({"cmd":"fail_cmd","cmd_id":512,"ret":2})");
  const auto reboot = dev->reboot();
  REQUIRE_FALSE(reboot.has_value());
  check_rejected(reboot.error(), CmdId::kReboot);
  // Not declared disconnected: the LiDAR is still there.
  CHECK(dev->connected());
  std::this_thread::sleep_for(100ms);
  CHECK(f.disconnected == 0);
  CHECK(dev->identity().has_value());
}

TEST_CASE(
  "Device: accepted values it cannot decode leave the replayed setup alone", "[sim][device]")
{
  Fixture f;
  auto dev = f.open();
  REQUIRE(dev->set_point_format(DataType::kCartesian16).has_value());

  // The simulator acknowledges without applying, so that only the library sees the values.
  static constexpr std::array<std::byte, 2> kTwo{std::byte{0x07}, std::byte{0x07}};
  static constexpr std::array<std::byte, 3> kThree{std::byte{1}, std::byte{2}, std::byte{3}};
  const std::vector<KeyValue> kvs{
    {static_cast<std::uint16_t>(Key::kPclDataType), kTwo},
    {static_cast<std::uint16_t>(Key::kPatternMode), kTwo},
    {static_cast<std::uint16_t>(Key::kFovCfg0), kThree},
    {static_cast<std::uint16_t>(Key::kFovCfg1), kThree},
    {static_cast<std::uint16_t>(Key::kFovCfgEn), kTwo},
    {static_cast<std::uint16_t>(Key::kImuDataEn), kTwo},
    {static_cast<std::uint16_t>(Key::kDetectMode), kTwo},
    {static_cast<std::uint16_t>(Key::kTimeFilter), kTwo},
    {static_cast<std::uint16_t>(Key::kImuSensorCfg), kTwo},
    {static_cast<std::uint16_t>(Key::kInstallAttitude), kThree},
  };
  f.control(R"({"cmd":"fail_cmd","cmd_id":256,"ret":0})");
  REQUIRE(dev->configure(kvs).has_value());

  dev->disconnect();
  REQUIRE(wait_until([&] { return f.disconnected == 1; }, 1s));
  const auto r = dev->reconnect();
  if (!r) {
    FAIL(to_string(r.error()));
  }
  CHECK(dev->connected());
  CHECK(dev->point_format() == DataType::kCartesian16);
}
