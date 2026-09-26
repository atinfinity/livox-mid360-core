// SPDX-License-Identifier: Apache-2.0
// Typed stored-setting keys (issues #46 / #47 / #54): Device::set_detect_mode() /
// detect_mode(), set_imu_enabled() / imu_enabled(), set_imu_sensor_config() /
// imu_sensor_config(), set_time_filter() / time_filter(), the HostSetup fields behind them
// and their reconnect replay.
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
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

  [[nodiscard]] static DeviceOptions options()
  {
    DeviceOptions o;
    o.session.host_command_port = 0;
    o.session.request = {.timeout = 500ms, .attempts = 3};
    return o;
  }

  [[nodiscard]] DiscoveredDevice discovered() const
  {
    return DiscoveredDevice{
      .serial_number = sim->sn(),
      .ip = {127, 0, 0, 1},
      .cmd_port = sim->ports().cmd,
      .dev_type = 9,
      .from = Endpoint::loopback(sim->ports().cmd)};
  }

  [[nodiscard]] std::unique_ptr<Device> open(const DeviceOptions & o = options()) const
  {
    auto d = Device::open(*context, discovered(), o);
    if (!d) {
      FAIL(to_string(d.error()));
    }
    return std::move(*d);
  }
};

/// IMU sample and reconnect counters.
struct Recorder
{
  std::atomic<std::uint64_t> imu_samples{0};
  std::atomic<std::uint64_t> reconnected{0};

  void attach(Device & d)
  {
    REQUIRE(d.on_imu([this](const ImuData &) { ++imu_samples; }).has_value());
    REQUIRE(d.on_event([this](const Event & e) {
               if (e.kind == Event::Kind::kReconnected) ++reconnected;
             })
              .has_value());
  }

  /// IMU samples delivered over `window`.
  [[nodiscard]] std::uint64_t imu_over(std::chrono::milliseconds window) const
  {
    const auto start = imu_samples.load();
    std::this_thread::sleep_for(window);
    return imu_samples.load() - start;
  }
};

/// An enumerator the enum does not define, built at run time so the analyzer cannot fold it.
template <class E>
E out_of_range(std::uint8_t raw)
{
  volatile std::uint8_t v = raw;
  return static_cast<E>(v);
}

void check_rejected(const DeviceError & e, RetCode code, std::uint16_t key)
{
  CHECK(e.kind == DeviceError::Kind::kSession);
  REQUIRE(e.session.has_value());
  CHECK(e.session->kind == SessionErrorKind::kLidarRejected);
  CHECK(e.session->ret_code == code);
  CHECK(e.session->error_key == key);
}

}  // namespace

TEST_CASE(
  "host_setup_key_values: detect_mode, time_filter and imu_sensor_config follow the FOV keys",
  "[settings][config]")
{
  HostSetup setup;
  setup.fov = FovSettings{
    .fov0 = std::nullopt, .fov1 = std::nullopt, .enable = FovEnable{.fov0 = true, .fov1 = false}};
  setup.detect_mode = DetectMode::kSensitive;
  setup.time_filter = true;
  setup.imu_sensor_config = ImuSensorConfig{
    .output_rate = ImuOutputRate::k500Hz,
    .accel_range = ImuAccelRange::k8g,
    .gyro_range = ImuGyroRange::k1000dps};
  const auto kvs = host_setup_key_values(setup, {192, 168, 1, 5});
  REQUIRE(kvs.values.size() == 9);
  CHECK(kvs.values[4].key == 0x001C);
  CHECK(kvs.values[5].key == 0x0017);
  CHECK(kvs.values[6].key == 0x0018);
  CHECK(kvs.values[7].key == 0x0026);
  CHECK(kvs.values[8].key == 0x002B);
  CHECK(kvs.values[6].value[0] == std::byte{1});
  CHECK(kvs.values[7].value[0] == std::byte{1});
  REQUIRE(kvs.values[8].value.size() == 3);
  CHECK(kvs.values[8].value[0] == std::byte{1});
  CHECK(kvs.values[8].value[1] == std::byte{1});
  CHECK(kvs.values[8].value[2] == std::byte{1});

  // Absent optionals add nothing.
  HostSetup plain;
  CHECK(host_setup_key_values(plain, {192, 168, 1, 5}).values.size() == 5);
}

TEST_CASE("Enum range pre-checks fail before any I/O", "[settings][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  auto dev = f.open();

  const auto mode = dev->set_detect_mode(out_of_range<DetectMode>(2));
  REQUIRE_FALSE(mode.has_value());
  CHECK(mode.error().kind == DeviceError::Kind::kInvalidArgument);
  CHECK(mode.error().key == Key::kDetectMode);

  const ImuSensorConfig bad_rate{
    .output_rate = out_of_range<ImuOutputRate>(4),
    .accel_range = ImuAccelRange::k4g,
    .gyro_range = ImuGyroRange::k2000dps};
  const ImuSensorConfig bad_accel{
    .output_rate = ImuOutputRate::k200Hz,
    .accel_range = out_of_range<ImuAccelRange>(4),
    .gyro_range = ImuGyroRange::k2000dps};
  const ImuSensorConfig bad_gyro{
    .output_rate = ImuOutputRate::k200Hz,
    .accel_range = ImuAccelRange::k4g,
    .gyro_range = out_of_range<ImuGyroRange>(8)};
  for (const auto & cfg : {bad_rate, bad_accel, bad_gyro}) {
    const auto r = dev->set_imu_sensor_config(cfg);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == DeviceError::Kind::kInvalidArgument);
    CHECK(r.error().key == Key::kImuSensorCfg);
  }
  // Nothing reached the LiDAR.
  CHECK(dev->detect_mode().value() == DetectMode::kNormal);
  CHECK(dev->imu_sensor_config().value().output_rate == ImuOutputRate::k200Hz);
}

TEST_CASE("Detect mode, time filter and IMU config round-trip", "[settings][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  auto dev = f.open();

  CHECK(dev->detect_mode().value() == DetectMode::kNormal);
  const auto m = dev->set_detect_mode(DetectMode::kSensitive);
  REQUIRE(m.has_value());
  CHECK_FALSE(m->reboot_required);
  CHECK(dev->detect_mode().value() == DetectMode::kSensitive);

  CHECK_FALSE(dev->time_filter().value());
  REQUIRE(dev->set_time_filter(true).has_value());
  CHECK(dev->time_filter().value());

  const ImuSensorConfig cfg{
    .output_rate = ImuOutputRate::k100Hz,
    .accel_range = ImuAccelRange::k32g,
    .gyro_range = ImuGyroRange::k15_625dps};
  REQUIRE(dev->set_imu_sensor_config(cfg).has_value());
  const auto back = dev->imu_sensor_config();
  REQUIRE(back.has_value());
  CHECK(back->output_rate == ImuOutputRate::k100Hz);
  CHECK(back->accel_range == ImuAccelRange::k32g);
  CHECK(back->gyro_range == ImuGyroRange::k15_625dps);

  // settings() sees the same values.
  const auto s = dev->settings();
  REQUIRE(s.has_value());
  CHECK(s->detect_mode == DetectMode::kSensitive);
  CHECK(s->time_filter == true);
  REQUIRE(s->imu_sensor_cfg.has_value());
  CHECK(s->imu_sensor_cfg->output_rate == ImuOutputRate::k100Hz);

  // The raw typed path still rejects what the pre-check does not cover: the simulator answers
  // 0x03 for a detect mode / time filter byte above 1.
  const auto raw = dev->set<Key::kTimeFilter>(true);
  REQUIRE(raw.has_value());
  KeyValue kv{static_cast<std::uint16_t>(Key::kDetectMode), {}};
  const std::byte two{2};
  kv.value = std::span<const std::byte>(&two, 1);
  const auto rej = dev->configure(std::span<const KeyValue>(&kv, 1));
  REQUIRE_FALSE(rej.has_value());
  check_rejected(rej.error(), RetCode::kOutOfRange, 0x0018);
}

TEST_CASE("IMU enable and output rate drive the IMU stream", "[settings][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;  // outlives the Device
  auto dev = f.open();
  rec.attach(*dev);
  REQUIRE(dev->imu_enabled().value());
  REQUIRE(wait_until([&] { return rec.imu_samples > 0; }));

  // 200 Hz x 0.25 rate multiplier = 50 samples/s; sanitizer builds on a loaded runner
  // deliver far fewer, so only the direction of the change is asserted.
  const auto base = rec.imu_over(1000ms);
  CHECK(base > 0);

  REQUIRE(dev
            ->set_imu_sensor_config(ImuSensorConfig{
              .output_rate = ImuOutputRate::k500Hz,
              .accel_range = ImuAccelRange::k4g,
              .gyro_range = ImuGyroRange::k2000dps})
            .has_value());
  std::this_thread::sleep_for(100ms);      // let the sender pick the new interval up
  const auto fast = rec.imu_over(1000ms);  // 500 Hz x 0.25 = 125 samples/s
  CHECK(fast > base);

  REQUIRE(dev->set_imu_enabled(false).has_value());
  CHECK_FALSE(dev->imu_enabled().value());
  std::this_thread::sleep_for(100ms);
  CHECK(rec.imu_over(300ms) == 0);

  REQUIRE(dev->set_imu_enabled(true).has_value());
  CHECK(dev->imu_enabled().value());
  REQUIRE(wait_until([&] { return rec.imu_over(100ms) > 0; }));
}

TEST_CASE("Firmware without key 0x002B answers kParamNotSupport", "[settings][sim]")
{
  Fixture f({"--imu-cfg-unsupported"});
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  auto dev = f.open();

  const auto w = dev->set_imu_sensor_config(ImuSensorConfig{});
  REQUIRE_FALSE(w.has_value());
  check_rejected(w.error(), RetCode::kParamNotSupport, 0x002B);

  const auto r = dev->imu_sensor_config();
  REQUIRE_FALSE(r.has_value());
  check_rejected(r.error(), RetCode::kParamNotSupport, 0x002B);

  // settings() still succeeds; only the unsupported key is empty.
  const auto s = dev->settings();
  REQUIRE(s.has_value());
  CHECK_FALSE(s->imu_sensor_cfg.has_value());
  CHECK(s->imu_data_en.has_value());

  // The other IMU key is unaffected.
  REQUIRE(dev->set_imu_enabled(false).has_value());
  CHECK_FALSE(dev->imu_enabled().value());
}

TEST_CASE(
  "Reconnect replays detect mode, IMU enable, IMU config and time filter", "[settings][sim]")
{
  Fixture f({"--reboot-silence", "0.3"});
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  DeviceOptions o = Fixture::options();
  o.reconnect.push_timeout = 5000ms;  // the second host below must not trigger a reconnect
  o.reconnect.initial_backoff = 100ms;
  o.reconnect.max_backoff = 400ms;
  o.reconnect.discovery_timeout = 200ms;
  o.host_setup.detect_mode = DetectMode::kSensitive;
  Recorder rec;  // outlives the Device
  auto dev = f.open(o);
  rec.attach(*dev);
  CHECK(dev->detect_mode().value() == DetectMode::kSensitive);

  REQUIRE(dev->set_imu_enabled(false).has_value());
  REQUIRE(dev->set_time_filter(true).has_value());
  REQUIRE(dev
            ->set_imu_sensor_config(ImuSensorConfig{
              .output_rate = ImuOutputRate::k50Hz,
              .accel_range = ImuAccelRange::k16g,
              .gyro_range = ImuGyroRange::k500dps})
            .has_value());
  // A rejected write must not be absorbed.
  REQUIRE_FALSE(dev->set_detect_mode(out_of_range<DetectMode>(2)).has_value());

  // A second host changes everything behind `dev`'s back.
  {
    ContextOptions co;
    co.bind_address = {127, 0, 0, 1};
    co.push_port = co.point_port = co.imu_port = 0;
    auto other_ctx = Context::create(co);
    REQUIRE(other_ctx.has_value());
    DeviceOptions oo = Fixture::options();
    oo.reconnect.enabled = false;
    oo.host_setup.imu_enable = true;
    oo.host_setup.detect_mode = DetectMode::kNormal;
    oo.host_setup.time_filter = false;
    oo.host_setup.imu_sensor_config = ImuSensorConfig{};
    auto other = Device::open(**other_ctx, f.discovered(), oo);
    if (!other) {
      FAIL("second open: " << to_string(other.error()));
    }
    CHECK((*other)->detect_mode().value() == DetectMode::kNormal);
    CHECK((*other)->imu_enabled().value());
    CHECK_FALSE((*other)->time_filter().value());
    CHECK((*other)->imu_sensor_config().value().output_rate == ImuOutputRate::k200Hz);
  }
  // The simulator keeps every setting across a reboot, so what comes back is the replay.
  REQUIRE(dev->reboot().has_value());
  REQUIRE(wait_until([&] { return rec.reconnected == 1; }, 8s));

  CHECK(dev->detect_mode().value() == DetectMode::kSensitive);
  CHECK_FALSE(dev->imu_enabled().value());
  CHECK(dev->time_filter().value());
  const auto cfg = dev->imu_sensor_config();
  REQUIRE(cfg.has_value());
  CHECK(cfg->output_rate == ImuOutputRate::k50Hz);
  CHECK(cfg->accel_range == ImuAccelRange::k16g);
  CHECK(cfg->gyro_range == ImuGyroRange::k500dps);
}
