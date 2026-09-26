// SPDX-License-Identifier: Apache-2.0
// Point format, scan pattern and frame policy (issue #40): Device::set_point_format() /
// point_format(), set_scan_pattern() / scan_pattern(), set_frame_policy() / frame_policy(),
// HostSetup::scan_pattern and the reconnect replay of run-time written keys.
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
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

/// Frame headers (points cleared) in delivery order, plus counters.
struct Recorder
{
  std::atomic<std::uint64_t> frames{0};
  std::atomic<std::uint64_t> reconnected{0};
  std::mutex mutex;
  std::vector<Frame> kept;

  void attach(Device & d)
  {
    REQUIRE(d.on_frame([this](Frame && f) {
               const std::lock_guard lock(mutex);
               f.points.clear();
               kept.push_back(std::move(f));
               ++frames;
             })
              .has_value());
    REQUIRE(d.on_event([this](const Event & e) {
               if (e.kind == Event::Kind::kReconnected) ++reconnected;
             })
              .has_value());
  }

  [[nodiscard]] std::vector<DataType> types()
  {
    const std::lock_guard lock(mutex);
    std::vector<DataType> out;
    out.reserve(kept.size());
    for (const Frame & f : kept) {
      out.push_back(f.source_type);
    }
    return out;
  }
};

}  // namespace

TEST_CASE(
  "host_setup_key_values: scan_pattern goes between imu_data_en and the FOV keys",
  "[point_format][config]")
{
  HostSetup setup;
  setup.scan_pattern = ScanPattern::kNonRepetitive;
  setup.fov = FovSettings{
    .fov0 = std::nullopt, .fov1 = std::nullopt, .enable = FovEnable{.fov0 = true, .fov1 = false}};
  const auto kvs = host_setup_key_values(setup, {192, 168, 1, 5});
  REQUIRE(kvs.values.size() == 7);
  CHECK(kvs.values[0].key == 0x0005);
  CHECK(kvs.values[1].key == 0x0006);
  CHECK(kvs.values[2].key == 0x0007);
  CHECK(kvs.values[3].key == 0x0000);
  CHECK(kvs.values[4].key == 0x001C);
  CHECK(kvs.values[5].key == 0x0001);
  CHECK(kvs.values[5].value.size() == 1);
  CHECK(kvs.values[5].value[0] == std::byte{0});
  CHECK(kvs.values[6].key == 0x0017);
  setup.scan_pattern = std::nullopt;
  CHECK(host_setup_key_values(setup, {192, 168, 1, 5}).values.size() == 6);
}

TEST_CASE("ScanPattern codec and to_string", "[point_format]")
{
  CHECK(
    decode_scan_pattern(encode_enum_u8(ScanPattern::kLowRateRepetitive)).value() ==
    ScanPattern::kLowRateRepetitive);
  const auto bad = decode_scan_pattern(encode_u8(3));
  CHECK_FALSE(bad.has_value());
  CHECK(to_string(ScanPattern::kNonRepetitive) == "non_repetitive");
  CHECK(to_string(ScanPattern::kRepetitive) == "repetitive");
  CHECK(to_string(ScanPattern::kLowRateRepetitive) == "low_rate_repetitive");
}

TEST_CASE(
  "Device::set_point_format switches mid-stream, one source_type per frame", "[point_format][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;  // outlives the Device: callbacks may run until the destructor returns
  auto dev = f.open();
  rec.attach(*dev);

  const auto imu = dev->set_point_format(DataType::kImu);
  REQUIRE_FALSE(imu.has_value());
  CHECK(imu.error().kind == DeviceError::Kind::kInvalidArgument);
  CHECK(imu.error().key == Key::kPclDataType);

  REQUIRE(dev->point_format().value() == DataType::kCartesian32);
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 2; }));
  REQUIRE(dev->set_point_format(DataType::kCartesian16).has_value());
  CHECK(dev->point_format().value() == DataType::kCartesian16);
  const auto n16 = rec.frames.load();
  REQUIRE(wait_until([&] { return rec.frames >= n16 + 3; }));
  REQUIRE(dev->set_point_format(DataType::kSpherical).has_value());
  const auto nsph = rec.frames.load();
  REQUIRE(wait_until([&] { return rec.frames >= nsph + 3; }));
  REQUIRE(dev->stop_sampling().has_value());

  const auto types = rec.types();
  REQUIRE(types.size() >= 8);
  // Monotone: 32 ... 32, 16 ... 16, spherical ... spherical, each present.
  std::size_t i = 0;
  const auto skip = [&](DataType t) {
    while (i < types.size() && types[i] == t) {
      ++i;
    }
  };
  skip(DataType::kCartesian32);
  CHECK(i > 0);
  const auto first16 = i;
  skip(DataType::kCartesian16);
  CHECK(i > first16);
  const auto first_sph = i;
  skip(DataType::kSpherical);
  CHECK(i > first_sph);
  CHECK(i == types.size());
  CHECK(dev->stats().bad_packets == 0);
}

TEST_CASE(
  "Device::set_scan_pattern: 0 round-trips, 1 is rejected by the simulator", "[point_format][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  auto dev = f.open();
  CHECK(dev->scan_pattern().value() == ScanPattern::kNonRepetitive);
  const auto ok = dev->set_scan_pattern(ScanPattern::kNonRepetitive);
  REQUIRE(ok.has_value());
  CHECK_FALSE(ok->reboot_required);

  const auto rejected = dev->set_scan_pattern(ScanPattern::kRepetitive);
  REQUIRE_FALSE(rejected.has_value());
  CHECK(rejected.error().kind == DeviceError::Kind::kSession);
  REQUIRE(rejected.error().session.has_value());
  CHECK(rejected.error().session->kind == SessionErrorKind::kLidarRejected);
  CHECK(rejected.error().session->ret_code == RetCode::kParamNotSupport);
  CHECK(rejected.error().session->error_key == 0x0001);
  CHECK(dev->scan_pattern().value() == ScanPattern::kNonRepetitive);
  CHECK(dev->settings().value().pattern_mode == ScanPattern::kNonRepetitive);
}

TEST_CASE("Device::set_frame_policy changes the frame period mid-stream", "[point_format][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  DeviceOptions o = Fixture::options();
  o.frame_policy = {.mode = FramePolicy::Mode::kTimeWindow, .window = 200ms};
  Recorder rec;  // outlives the Device
  auto dev = f.open(o);
  rec.attach(*dev);
  CHECK(dev->frame_policy().window == 200ms);

  const auto bad = dev->set_frame_policy({.mode = FramePolicy::Mode::kTimeWindow, .window = 0ms});
  REQUIRE_FALSE(bad.has_value());
  CHECK(bad.error().kind == DeviceError::Kind::kInvalidArgument);
  CHECK(dev->frame_policy().window == 200ms);

  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 3; }));
  REQUIRE(
    dev->set_frame_policy({.mode = FramePolicy::Mode::kTimeWindow, .window = 50ms}).has_value());
  CHECK(dev->frame_policy().window == 50ms);
  const auto before = rec.frames.load();
  // 4x the frame rate: 12 more frames within ~0.6 s of point time (rate multiplier is on
  // the packet rate only, timestamps are wall clock).
  REQUIRE(wait_until([&] { return rec.frames >= before + 12; }, 3s));
  REQUIRE(dev->stop_sampling().has_value());

  std::vector<std::uint64_t> spans;
  {
    const std::lock_guard lock(rec.mutex);
    for (std::size_t i = before + 1; i < rec.kept.size(); ++i) {
      spans.push_back(rec.kept[i].end_time_ns - rec.kept[i].base_time_ns);
    }
  }
  REQUIRE(spans.size() >= 10);
  for (const auto s : spans) {
    CHECK(s < 100'000'000);  // well under the old 200 ms window
  }
}

TEST_CASE(
  "Reconnect replays the last written point format, scan pattern and FOV", "[point_format][sim]")
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
  o.host_setup.pcl_data_type = DataType::kCartesian32;
  o.host_setup.scan_pattern = ScanPattern::kNonRepetitive;
  Recorder rec;  // outlives the Device
  auto dev = f.open(o);
  rec.attach(*dev);

  constexpr FovConfig kWin{
    .yaw_start_deg = 30, .yaw_stop_deg = 60, .pitch_start_deg = 0, .pitch_stop_deg = 10, .rsvd = 0};
  REQUIRE(dev->set_point_format(DataType::kSpherical).has_value());
  REQUIRE(dev
            ->set_fov(FovSettings{
              .fov0 = kWin, .fov1 = std::nullopt, .enable = FovEnable{.fov0 = true, .fov1 = false}})
            .has_value());
  // A rejected write must not be absorbed.
  REQUIRE_FALSE(dev->set_scan_pattern(ScanPattern::kRepetitive).has_value());

  // Something the Device does not see changes the LiDAR: a second host opens it with other
  // values (and takes the push / point streams, which is what makes `dev` reconnect).
  {
    ContextOptions co;
    co.bind_address = {127, 0, 0, 1};
    co.push_port = co.point_port = co.imu_port = 0;
    auto other_ctx = Context::create(co);
    REQUIRE(other_ctx.has_value());
    DeviceOptions oo = Fixture::options();
    oo.reconnect.enabled = false;
    oo.host_setup.pcl_data_type = DataType::kCartesian16;
    oo.host_setup.fov = FovSettings{
      .fov0 =
        FovConfig{
          .yaw_start_deg = 100,
          .yaw_stop_deg = 200,
          .pitch_start_deg = 0,
          .pitch_stop_deg = 0,
          .rsvd = 0},
      .fov1 = std::nullopt,
      .enable = FovEnable{.fov0 = false, .fov1 = false}};
    auto other = Device::open(**other_ctx, f.discovered(), oo);
    REQUIRE(other.has_value());
    CHECK((*other)->point_format().value() == DataType::kCartesian16);
    CHECK((*other)->fov()->fov0->yaw_start_deg == 100);
  }
  REQUIRE(wait_until([&] { return rec.reconnected >= 1; }, 8s));

  CHECK(dev->point_format().value() == DataType::kSpherical);
  CHECK(dev->scan_pattern().value() == ScanPattern::kNonRepetitive);
  const auto fov = dev->fov();
  REQUIRE(fov.has_value());
  REQUIRE(fov->fov0.has_value());
  CHECK(fov->fov0->yaw_start_deg == 30);
  CHECK(fov->fov0->yaw_stop_deg == 60);
  CHECK(fov->enable->fov0);
}
