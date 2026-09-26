// SPDX-License-Identifier: Apache-2.0
// FOV configuration (issue #39): fov_in_range(), FovSettings in HostSetup, Device::set_fov()
// / Device::fov() and the simulator's [unverified] cropping of the point cloud.
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
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

constexpr FovConfig kFront{
  .yaw_start_deg = 0, .yaw_stop_deg = 90, .pitch_start_deg = -5, .pitch_stop_deg = 5, .rsvd = 0};
constexpr FovConfig kBack{
  .yaw_start_deg = 180,
  .yaw_stop_deg = 270,
  .pitch_start_deg = 10,
  .pitch_stop_deg = 20,
  .rsvd = 0};

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

/// (yaw, pitch) in degrees of a delivered point, yaw in [0, 360).
std::pair<double, double> angles(const Point & p)
{
  const auto x = static_cast<double>(p.x);
  const auto y = static_cast<double>(p.y);
  const auto z = static_cast<double>(p.z);
  double yaw = std::atan2(y, x) * 180.0 / M_PI;
  if (yaw < 0) {
    yaw += 360.0;
  }
  const double pitch = std::atan2(z, std::hypot(x, y)) * 180.0 / M_PI;
  return {yaw, pitch};
}

bool inside(const FovConfig & w, const Point & p, double tol = 0.05)
{
  const auto [yaw, pitch] = angles(p);
  return yaw >= w.yaw_start_deg - tol && yaw < w.yaw_stop_deg + tol &&
         pitch >= w.pitch_start_deg - tol && pitch <= w.pitch_stop_deg + tol;
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

/// Frames delivered after attach(); `outside` counts points not inside `window`.
struct Recorder
{
  std::atomic<std::uint64_t> frames{0};
  std::atomic<std::uint64_t> points{0};
  std::atomic<std::uint64_t> outside{0};
  std::atomic<std::uint64_t> reconnected{0};
  FovConfig window = kFront;

  void attach(Device & d)
  {
    REQUIRE(d.on_frame([this](const Frame & f) {
               for (const Point & p : f.points) {
                 ++points;
                 if (!inside(window, p)) {
                   ++outside;
                 }
               }
               ++frames;
             })
              .has_value());
    REQUIRE(d.on_event([this](const Event & e) {
               if (e.kind == Event::Kind::kReconnected) ++reconnected;
             })
              .has_value());
  }
};

bool same(const FovConfig & a, const FovConfig & b)
{
  return a.yaw_start_deg == b.yaw_start_deg && a.yaw_stop_deg == b.yaw_stop_deg &&
         a.pitch_start_deg == b.pitch_start_deg && a.pitch_stop_deg == b.pitch_stop_deg &&
         a.rsvd == b.rsvd;
}

bool same(const std::optional<FovConfig> & a, const FovConfig & b)
{
  return a.has_value() && same(*a, b);
}

}  // namespace

TEST_CASE("fov_in_range: yaw 0..359, pitch -9..59, reversed windows allowed", "[fov]")
{
  CHECK(fov_in_range(kFront));
  CHECK(fov_in_range(
    {.yaw_start_deg = 359,
     .yaw_stop_deg = 0,
     .pitch_start_deg = 59,
     .pitch_stop_deg = -9,
     .rsvd = 0}));
  CHECK(fov_in_range(
    {.yaw_start_deg = 20,
     .yaw_stop_deg = 20,
     .pitch_start_deg = 0,
     .pitch_stop_deg = 0,
     .rsvd = 7}));
  CHECK_FALSE(fov_in_range(
    {.yaw_start_deg = 360,
     .yaw_stop_deg = 0,
     .pitch_start_deg = 0,
     .pitch_stop_deg = 0,
     .rsvd = 0}));
  CHECK_FALSE(fov_in_range(
    {.yaw_start_deg = 0,
     .yaw_stop_deg = -1,
     .pitch_start_deg = 0,
     .pitch_stop_deg = 0,
     .rsvd = 0}));
  CHECK_FALSE(fov_in_range(
    {.yaw_start_deg = 0,
     .yaw_stop_deg = 0,
     .pitch_start_deg = -10,
     .pitch_stop_deg = 0,
     .rsvd = 0}));
  CHECK_FALSE(fov_in_range(
    {.yaw_start_deg = 0,
     .yaw_stop_deg = 0,
     .pitch_start_deg = 0,
     .pitch_stop_deg = 60,
     .rsvd = 0}));
}

TEST_CASE("to_string(FovSettings) lists the present fields", "[fov]")
{
  CHECK(to_string(FovSettings{}).empty());
  const FovSettings s{
    .fov0 = kFront, .fov1 = std::nullopt, .enable = FovEnable{.fov0 = true, .fov1 = false}};
  CHECK(to_string(s) == "fov0=yaw0-90/pitch-5-5 enable=fov0:1,fov1:0");
}

TEST_CASE(
  "host_setup_key_values: FOV keys follow the host keys, absent ones are skipped", "[fov][config]")
{
  HostSetup setup;
  setup.fov = FovSettings{
    .fov0 = kFront, .fov1 = std::nullopt, .enable = FovEnable{.fov0 = true, .fov1 = true}};
  const auto kvs = host_setup_key_values(setup, {192, 168, 1, 5});
  REQUIRE(kvs.values.size() == 7);
  CHECK(kvs.values[4].key == 0x001C);
  CHECK(kvs.values[5].key == 0x0015);
  CHECK(kvs.values[5].value.size() == 20);
  CHECK(kvs.values[6].key == 0x0017);
  CHECK(kvs.values[6].value.size() == 1);
  CHECK(kvs.values[6].value[0] == std::byte{0x03});
  CHECK(same(decode_fov_config(kvs.values[5].value).value(), kFront));
  std::size_t total = 0;
  for (const auto & kv : kvs.values) {
    CHECK(kv.value.data() == kvs.storage.data() + total);
    total += kv.value.size();
  }
  CHECK(total == kvs.storage.size());

  setup.fov = FovSettings{.fov0 = std::nullopt, .fov1 = kBack, .enable = std::nullopt};
  const auto only1 = host_setup_key_values(setup, {192, 168, 1, 5});
  REQUIRE(only1.values.size() == 6);
  CHECK(only1.values[5].key == 0x0016);
  setup.fov = FovSettings{};
  CHECK(host_setup_key_values(setup, {192, 168, 1, 5}).values.size() == 5);
}

TEST_CASE("Device::set_fov / fov round trip and rejections", "[fov][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  auto dev = f.open();

  // Factory default: both windows zero, nothing enabled.
  auto initial = dev->fov();
  REQUIRE(initial.has_value());
  REQUIRE(initial->fov0.has_value());
  CHECK(same(*initial->fov0, FovConfig{}));
  REQUIRE(initial->enable.has_value());
  CHECK_FALSE(initial->enable->fov0);

  // Validated before any I/O.
  const auto empty = dev->set_fov(FovSettings{});
  REQUIRE_FALSE(empty.has_value());
  CHECK(empty.error().kind == DeviceError::Kind::kInvalidArgument);
  CHECK_FALSE(empty.error().key.has_value());
  const auto bad = dev->set_fov(FovSettings{
    .fov0 = kFront,
    .fov1 =
      FovConfig{
        .yaw_start_deg = 0,
        .yaw_stop_deg = 360,
        .pitch_start_deg = 0,
        .pitch_stop_deg = 0,
        .rsvd = 0},
    .enable = std::nullopt});
  REQUIRE_FALSE(bad.has_value());
  CHECK(bad.error().kind == DeviceError::Kind::kInvalidArgument);
  CHECK(bad.error().key == Key::kFovCfg1);

  // All three in one request.
  const FovSettings want{
    .fov0 = kFront, .fov1 = kBack, .enable = FovEnable{.fov0 = true, .fov1 = true}};
  const auto set = dev->set_fov(want);
  REQUIRE(set.has_value());
  CHECK_FALSE(set->reboot_required);
  const auto got = dev->fov();
  REQUIRE(got.has_value());
  CHECK(same(got->fov0, kFront));
  CHECK(same(got->fov1, kBack));
  CHECK(got->enable->fov0);
  CHECK(got->enable->fov1);
  CHECK(to_string(*got) == to_string(want));

  // Only the mask: the windows stay.
  REQUIRE(dev
            ->set_fov(FovSettings{
              .fov0 = std::nullopt,
              .fov1 = std::nullopt,
              .enable = FovEnable{.fov0 = false, .fov1 = true}})
            .has_value());
  const auto after_mask = dev->fov();
  REQUIRE(after_mask.has_value());
  CHECK(same(after_mask->fov0, kFront));
  CHECK_FALSE(after_mask->enable->fov0);
  CHECK(after_mask->enable->fov1);

  // The simulator range-checks too: a raw out-of-range window is answered with 0x03.
  const auto raw = encode_fov_config(
    {.yaw_start_deg = 0, .yaw_stop_deg = 0, .pitch_start_deg = 0, .pitch_stop_deg = 60, .rsvd = 0});
  const KeyValue kv{static_cast<std::uint16_t>(Key::kFovCfg0), raw};
  const auto rejected = dev->configure(std::span<const KeyValue>(&kv, 1));
  REQUIRE_FALSE(rejected.has_value());
  REQUIRE(rejected.error().session.has_value());
  CHECK(rejected.error().session->kind == SessionErrorKind::kLidarRejected);
  CHECK(rejected.error().session->ret_code == RetCode::kOutOfRange);
  CHECK(rejected.error().session->error_key == 0x0015);
  CHECK(same(dev->fov()->fov0, kFront));
}

TEST_CASE("Simulator crops the point cloud to the enabled windows", "[fov][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  auto dev = f.open();
  Recorder rec;
  rec.attach(*dev);
  REQUIRE(
    dev
      ->set_fov(FovSettings{
        .fov0 = kFront, .fov1 = std::nullopt, .enable = FovEnable{.fov0 = true, .fov1 = false}})
      .has_value());
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 3; }));
  CHECK(rec.points > 0);
  CHECK(rec.outside == 0);

  // Disable the window: points outside it appear again.
  REQUIRE(dev
            ->set_fov(FovSettings{
              .fov0 = std::nullopt,
              .fov1 = std::nullopt,
              .enable = FovEnable{.fov0 = false, .fov1 = false}})
            .has_value());
  CHECK(wait_until([&] { return rec.outside > 0; }));
  REQUIRE(dev->stop_sampling().has_value());
}

TEST_CASE("HostSetup::fov is applied at open and replayed after a reconnect", "[fov][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  DeviceOptions o = Fixture::options();
  o.session.request = {.timeout = 200ms, .attempts = 2};
  o.reconnect.push_timeout = 500ms;
  o.reconnect.initial_backoff = 100ms;
  o.reconnect.max_backoff = 400ms;
  o.reconnect.discovery_timeout = 200ms;

  // Out of range: rejected before the request, with the key.
  o.host_setup.fov = FovSettings{
    .fov0 = std::nullopt,
    .fov1 =
      FovConfig{
        .yaw_start_deg = -1,
        .yaw_stop_deg = 0,
        .pitch_start_deg = 0,
        .pitch_stop_deg = 0,
        .rsvd = 0},
    .enable = std::nullopt};
  const auto bad = Device::open(*f.context, f.discovered(), o);
  REQUIRE_FALSE(bad.has_value());
  REQUIRE(bad.error().session.has_value());
  CHECK(bad.error().session->kind == SessionErrorKind::kInvalidArgument);
  CHECK(bad.error().session->error_key == 0x0016);

  o.host_setup.fov = FovSettings{
    .fov0 = kFront, .fov1 = std::nullopt, .enable = FovEnable{.fov0 = true, .fov1 = false}};
  auto dev = f.open(o);
  Recorder rec;
  rec.attach(*dev);
  const auto got = dev->fov();
  REQUIRE(got.has_value());
  CHECK(same(got->fov0, kFront));
  CHECK(got->enable->fov0);

  // Something else changes the LiDAR; the reconnect restores the HostSetup value.
  REQUIRE(
    dev
      ->set_fov(FovSettings{
        .fov0 = kBack, .fov1 = std::nullopt, .enable = FovEnable{.fov0 = false, .fov1 = false}})
      .has_value());
  CHECK(same(dev->fov()->fov0, kBack));
  REQUIRE(f.sim->control(R"({"cmd":"silence","seconds":1.5})"));
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());
  REQUIRE(wait_until([&] { return rec.reconnected == 1; }, 8s));
  const auto after_reconnect = dev->fov();
  REQUIRE(after_reconnect.has_value());
  CHECK(same(after_reconnect->fov0, kFront));
  CHECK(after_reconnect->enable->fov0);
}
