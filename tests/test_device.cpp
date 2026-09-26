// SPDX-License-Identifier: Apache-2.0
// Context / Device (issue #6) against tools/livox_mid360_sim.py: registration, frame and IMU
// delivery, drop counting, frame_cnt splitting and fallback, idle close, stop / destruction,
// and 0x0102 push handling (issue #7): work_state(), hms(), kStateChanged / kHms events.
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
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

std::unique_ptr<Context> loopback_context()
{
  ContextOptions o;
  o.bind_address = {127, 0, 0, 1};
  o.push_port = o.point_port = o.imu_port = 0;
  auto c = Context::create(o);
  REQUIRE(c.has_value());
  return std::move(*c);
}

struct Fixture
{
  std::optional<SimProcess> sim;
  std::string err;
  std::unique_ptr<Context> context;

  /// The simulator streams at a quarter of the real rate (500 pkt/s point cloud, 50 Hz IMU)
  /// so that sanitizer builds keep up; per-frame packet counts below assume this. Pushes
  /// come at 10 Hz so that state / HMS tests do not wait a second per step.
  explicit Fixture(std::vector<std::string> args = {})
  {
    args.insert(args.end(), {"--rate-multiplier", "0.25", "--push-rate", "10"});
    sim = SimProcess::start(err, std::move(args));
    if (sim) {
      context = loopback_context();
    }
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

  [[nodiscard]] static DeviceOptions options()
  {
    DeviceOptions o;
    o.session.host_command_port = 0;
    o.session.request = {.timeout = 500ms, .attempts = 3};
    return o;
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

/// Everything the callbacks record, shared with the receive thread.
struct Recorder
{
  std::atomic<std::uint64_t> frames{0};
  std::atomic<std::uint64_t> points{0};
  std::atomic<std::uint64_t> frame_packets{0};  ///< sum of Frame::packets
  std::atomic<std::uint64_t> imu{0};
  std::atomic<std::uint64_t> pcl_packets{0};  ///< on_packet, point cloud only
  std::atomic<std::uint64_t> imu_packets{0};
  std::atomic<std::uint64_t> stats_events{0};
  std::atomic<std::uint64_t> state_events{0};
  std::atomic<std::uint64_t> hms_events{0};
  std::atomic<bool> ok{true};
  std::mutex mutex;
  std::vector<Frame> kept;    ///< headers only (points cleared) of every frame
  std::vector<Event> events;  ///< kStateChanged / kHms in order

  void attach(Device & d)
  {
    REQUIRE(d.on_packet([this](const DataPacketView & p, const ReceiveInfo & info) {
               if (info.host_time_ns == 0 || info.source.ip != Ipv4{127, 0, 0, 1}) ok = false;
               if (p.header.data_type == DataType::kImu) {
                 ++imu_packets;
               } else {
                 ++pcl_packets;
               }
             })
              .has_value());
    REQUIRE(d.on_frame([this](Frame && f) {
               if (f.points.empty() || f.end_time_ns < f.base_time_ns) ok = false;
               for (const Point & p : f.points) {
                 if (p.line > 3 || p.offset_ns > f.end_time_ns - f.base_time_ns) ok = false;
               }
               ++frames;
               points += f.points.size();
               frame_packets += f.packets;
               const std::lock_guard lock(mutex);
               f.points.clear();
               kept.push_back(std::move(f));
             })
              .has_value());
    REQUIRE(d.on_imu([this](const ImuData & s) {
               if (s.time_ns == 0) ok = false;
               ++imu;
             })
              .has_value());
    REQUIRE(d.on_event([this](const Event & e) {
               if (e.kind == Event::Kind::kStats) {
                 ++stats_events;
                 return;
               }
               if (e.time_ns == 0) ok = false;
               const std::lock_guard lock(mutex);
               events.push_back(e);
               if (e.kind == Event::Kind::kStateChanged) ++state_events;
               if (e.kind == Event::Kind::kHms) ++hms_events;
             })
              .has_value());
  }

  [[nodiscard]] Event event(std::size_t i)
  {
    const std::lock_guard lock(mutex);
    REQUIRE(i < events.size());
    return events[i];
  }
};

}  // namespace

TEST_CASE(
  "Context: ephemeral ports are resolved and unknown sources are counted", "[device][context]")
{
  auto ctx = loopback_context();
  const ContextOptions & o = ctx->options();
  CHECK(o.push_port != 0);
  CHECK(o.point_port != 0);
  CHECK(o.imu_port != 0);
  CHECK(o.bind_address == Ipv4{127, 0, 0, 1});
  CHECK(ctx->stats().datagrams == 0);

  const auto sender = UdpSocket::open(Endpoint::loopback(0));
  REQUIRE(sender.has_value());
  const std::byte payload[4]{};
  for (const std::uint16_t port : {o.push_port, o.point_port, o.imu_port}) {
    REQUIRE(sender->send_to(payload, Endpoint::loopback(port)).has_value());
  }
  CHECK(wait_until([&] { return ctx->stats().datagrams == 3; }));
  CHECK(ctx->stats().unknown_source == 3);
}

TEST_CASE("Context: invalid options", "[device][context]")
{
  ContextOptions o;
  o.batch_size = 0;
  const auto c = Context::create(o);
  REQUIRE_FALSE(c.has_value());
  CHECK(c.error().kind == DeviceError::Kind::kInvalidArgument);
  CHECK(to_string(c.error()) == "invalid_argument");
}

TEST_CASE("Event / DeviceError to_string", "[device]")
{
  Event e;
  e.kind = Event::Kind::kStateChanged;
  e.old_state = WorkState::kIdle;
  e.new_state = WorkState::kSampling;
  CHECK(to_string(e) == "state_changed IDLE -> SAMPLING");
  e.kind = Event::Kind::kStats;
  e.stats.packets = 7;
  CHECK(to_string(e).starts_with("stats packets=7 "));
  e.kind = Event::Kind::kHms;
  e.hms[0].raw = 1;
  e.hms_level = HmsLevel::kWarning;
  CHECK(to_string(e) == "hms active=1 level=warning");
  CHECK(to_string(Event::Kind::kDisconnected) == "disconnected");
  DeviceError err;
  err.kind = DeviceError::Kind::kSession;
  SessionError se;
  se.kind = SessionErrorKind::kTimeout;
  se.cmd_id = 0x0100;
  err.session = se;
  CHECK(to_string(err).starts_with("session: timeout"));
  CHECK(to_string(DeviceError::Kind::kAlreadyRegistered) == "already_registered");
}

TEST_CASE("Device: open errors", "[sim][device]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }

  SECTION("unreachable command port wraps the session error")
  {
    DiscoveredDevice d = f.discovered();
    d.cmd_port = 1;
    d.from = Endpoint::loopback(1);
    DeviceOptions o = Fixture::options();
    o.session.request = {.timeout = 50ms, .attempts = 1};
    const auto r = Device::open(*f.context, d, o);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == DeviceError::Kind::kSession);
    REQUIRE(r.error().session.has_value());
    CHECK(r.error().session->kind == SessionErrorKind::kTimeout);
  }
  SECTION("invalid options are rejected before any I/O")
  {
    DeviceOptions o = Fixture::options();
    o.frame_policy.window = 0ns;
    const auto r = Device::open(*f.context, f.discovered(), o);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == DeviceError::Kind::kInvalidArgument);
  }
  SECTION("a second Device with the same IP is refused")
  {
    auto first = f.open();
    const auto second = Device::open(*f.context, f.discovered(), Fixture::options());
    REQUIRE_FALSE(second.has_value());
    CHECK(second.error().kind == DeviceError::Kind::kAlreadyRegistered);
    first.reset();
    auto again = f.open();  // registration freed by the destructor
    CHECK(again->info().serial_number == f.sim->sn());
  }
}

TEST_CASE("Device: frames, IMU, stats and stop", "[sim][device]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  DeviceOptions o = Fixture::options();
  o.stats_interval = 200ms;
  Recorder rec;  // outlives the Device: callbacks may run until the destructor returns
  auto dev = f.open(o);
  CHECK(dev->info().ip == Ipv4{127, 0, 0, 1});
  CHECK(dev->stats().packets == 0);

  rec.attach(*dev);
  // Powers up into SAMPLING (default work_tgt_mode), possibly via MOTORSTARTUP.
  REQUIRE(wait_until([&] { return dev->work_state() == WorkState::kSampling; }));
  REQUIRE(dev->start_sampling().has_value());
  // Callbacks are frozen while sampling.
  const auto locked = dev->on_imu([](const ImuData &) {});
  REQUIRE_FALSE(locked.has_value());
  CHECK(locked.error().kind == DeviceError::Kind::kInvalidState);

  REQUIRE(wait_until([&] { return rec.frames >= 5 && rec.imu >= 10 && rec.stats_events >= 2; }));
  CHECK(rec.ok);
  const DeviceStats s = dev->stats();
  CHECK(s.packets == rec.pcl_packets + rec.imu_packets);
  CHECK(s.frames >= rec.frames);
  CHECK(s.points >= rec.points);
  CHECK(s.imu_samples >= rec.imu);
  CHECK(s.bad_packets == 0);
  CHECK(s.dropped_packets == 0);
  CHECK(s.reordered == 0);
  CHECK(s.frame_cnt_fallback == 0);
  CHECK(s.last_packet_time_ns != 0);
  CHECK(s.time_offset_valid);
  CHECK(dev->session_stats().requests >= 3);
  CHECK(f.context->stats().unknown_source == 0);
  CHECK(f.context->stats().datagrams >= s.packets);
  {
    const std::lock_guard lock(rec.mutex);
    REQUIRE(rec.kept.size() >= 5);
    // 100 ms frames at 500 pkt/s: about 50 packets, 96 points each, consecutive frame_cnt.
    for (std::size_t i = 1; i < rec.kept.size(); ++i) {
      const Frame & a = rec.kept[i - 1];
      const Frame & b = rec.kept[i];
      CHECK(b.index == a.index + 1);
      CHECK(b.frame_cnt == static_cast<std::uint8_t>(a.frame_cnt + 1));
      // (no `b.base >= a.end` check: the simulator's catch-up bursts overlap packet times)
      CHECK(b.source_type == DataType::kCartesian32);
      CHECK(b.packets > 25);
      CHECK(b.packets < 100);
    }
  }
  // Point-cloud datagrams reach on_packet with the same information as the sockets saw.
  CHECK(rec.pcl_packets >= rec.frame_packets);

  // Commands still work while streaming (serialised with the receive side).
  const std::uint16_t keys[] = {static_cast<std::uint16_t>(Key::kCurWorkState)};
  const auto inq = dev->inquire(keys);
  REQUIRE(inq.has_value());
  CHECK(inq->get(Key::kCurWorkState).has_value());

  REQUIRE(dev->stop_sampling().has_value());
  CHECK(dev->on_imu([](const ImuData &) {}).has_value());  // editable again
  // The LiDAR is idle: once the socket queue has drained, the packet count stops moving.
  std::uint64_t settled = dev->stats().packets;
  REQUIRE(wait_until([&] {
    const auto now = dev->stats().packets;
    std::this_thread::sleep_for(300ms);
    const bool stable = dev->stats().packets == now;
    settled = now;
    return stable;
  }));
  CHECK(dev->stats().packets == settled);
  dev.reset();  // before the Context
}

TEST_CASE("Device: pushes drive work_state, hms and events", "[sim][device]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;
  auto dev = f.open();
  rec.attach(*dev);
  for (const HmsCode & c : dev->hms()) {
    CHECK_FALSE(c.active());
  }

  // The first push records the state without a kStateChanged. The simulator powers up
  // into SAMPLING (default work_tgt_mode) after its MOTORSTARTUP delay, so depending on
  // timing the first push carries either MOTORSTARTUP (then one MOTORSTARTUP -> SAMPLING
  // event follows) or SAMPLING (no event).
  REQUIRE(wait_until([&] { return dev->work_state() == WorkState::kSampling; }));
  CHECK(dev->stats().pushes >= 1);
  CHECK(dev->stats().last_push_time_ns != 0);
  CHECK(rec.state_events <= 1);
  if (rec.state_events == 1) {
    CHECK(rec.event(0).old_state == WorkState::kMotorStartup);
    CHECK(rec.event(0).new_state == WorkState::kSampling);
  }
  CHECK(rec.hms_events == 0);  // all slots empty == baseline
  std::size_t base = rec.state_events;

  // Transitions requested by commands are reported by the push, not by the command.
  REQUIRE(dev->stop_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.state_events >= base + 1; }));
  CHECK(rec.event(base).old_state == WorkState::kSampling);
  CHECK(rec.event(base).new_state == WorkState::kIdle);
  CHECK(dev->work_state() == WorkState::kIdle);
  ++base;
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.state_events >= base + 1; }));
  {
    const Event e = rec.event(base);
    CHECK(e.kind == Event::Kind::kStateChanged);
    CHECK(e.old_state == WorkState::kIdle);
    CHECK(e.new_state == WorkState::kSampling);
  }
  CHECK(dev->work_state() == WorkState::kSampling);

  // Two codes: abnormal 0x0001 level error, 0x0002 level warning -> one kHms at level error.
  constexpr std::uint32_t kErr = 0x0001'0003;
  constexpr std::uint32_t kWarn = 0x0002'0002;
  REQUIRE(f.sim->control(R"({"cmd":"hms","codes":[65539,131074]})"));
  REQUIRE(wait_until([&] { return rec.hms_events >= 1; }));
  {
    const Event e = rec.event(base + 1);
    CHECK(e.kind == Event::Kind::kHms);
    CHECK(e.hms_level == HmsLevel::kError);
    CHECK(e.hms[0].raw == kErr);
    CHECK(e.hms[0].abnormal_id == 1);
    CHECK(e.hms[1].raw == kWarn);
    CHECK_FALSE(e.hms[2].active());
  }
  CHECK(dev->hms()[1].level == HmsLevel::kWarning);

  // Same set in another slot order: no event.
  REQUIRE(f.sim->control(R"({"cmd":"hms","codes":[131074,65539]})"));
  const auto pushes_before = dev->stats().pushes;
  REQUIRE(wait_until([&] { return dev->stats().pushes >= pushes_before + 3; }));
  CHECK(rec.hms_events == 1);
  CHECK(dev->hms()[0].raw == kWarn);  // hms() still reflects wire order

  // Error cleared: level drops to warning; then everything cleared: level none.
  REQUIRE(f.sim->control(R"({"cmd":"hms","codes":[131074]})"));
  REQUIRE(wait_until([&] { return rec.hms_events >= 2; }));
  CHECK(rec.event(base + 2).hms_level == HmsLevel::kWarning);
  REQUIRE(f.sim->control(R"({"cmd":"hms","codes":[]})"));
  REQUIRE(wait_until([&] { return rec.hms_events >= 3; }));
  CHECK(rec.event(base + 3).hms_level == HmsLevel::kNone);
  for (const HmsCode & c : rec.event(base + 3).hms) {
    CHECK_FALSE(c.active());
  }
  for (const HmsCode & c : dev->hms()) {
    CHECK_FALSE(c.active());
  }

  // A LiDAR-side transition to ERROR shows up as well.
  REQUIRE(f.sim->control(R"({"cmd":"set_state","state":4})"));
  REQUIRE(wait_until([&] { return rec.state_events >= base + 2; }));
  CHECK(rec.event(base + 4).old_state == WorkState::kSampling);
  CHECK(rec.event(base + 4).new_state == WorkState::kError);
  CHECK(dev->work_state() == WorkState::kError);
  CHECK(rec.ok);
  CHECK(dev->stats().bad_packets == 0);
  dev.reset();
}

TEST_CASE("Device: drop_rate shows up in dropped_packets", "[sim][device]")
{
  Fixture f({"--drop-rate", "0.2"});
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;
  auto dev = f.open();
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 3; }));
  const DeviceStats s = dev->stats();
  CHECK(s.dropped_packets > 0);
  CHECK(s.reordered == 0);
  CHECK(rec.ok);
  std::uint64_t per_frame = 0;
  {
    const std::lock_guard lock(rec.mutex);
    for (const Frame & fr : rec.kept) {
      per_frame += fr.dropped_packets;
    }
  }
  CHECK(per_frame <= s.dropped_packets);
  CHECK(per_frame > 0);
}

TEST_CASE("Device: --frame-ms drives frame_cnt splitting", "[sim][device]")
{
  Fixture f({"--frame-ms", "20"});
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;
  auto dev = f.open();
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 20; }));
  CHECK(dev->stats().frame_cnt_fallback == 0);
  const std::lock_guard lock(rec.mutex);
  std::uint64_t packets = 0;
  for (const Frame & fr : rec.kept) {
    packets += fr.packets;
  }
  CHECK(packets / rec.kept.size() < 25);  // 20 ms at 500 pkt/s = 10 packets
}

TEST_CASE("Device: --frame-ms 0 falls back to the time window", "[sim][device]")
{
  Fixture f({"--frame-ms", "0"});
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  DeviceOptions o = Fixture::options();
  o.frame_policy.window = 50ms;
  Recorder rec;
  auto dev = f.open(o);
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 10; }));
  CHECK(dev->stats().frame_cnt_fallback == 1);
  CHECK(rec.ok);
  const std::lock_guard lock(rec.mutex);
  // The first frame spans the 2 x window grace period; the rest are one window each.
  for (std::size_t i = 1; i < rec.kept.size(); ++i) {
    const Frame & fr = rec.kept[i];
    CHECK(fr.frame_cnt == rec.kept[0].frame_cnt);
    CHECK(fr.end_time_ns - fr.base_time_ns < 60'000'000);
  }
  // Switching the simulator back to a period makes frame_cnt move again, but the assembler
  // stays in time-window mode (fallback is sticky); nothing breaks.
  REQUIRE(f.sim->control(R"({"cmd":"frame_ms","ms":100})"));
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());
}

TEST_CASE("Device: time window mode and kHostReceive", "[sim][device]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  DeviceOptions o = Fixture::options();
  o.frame_policy.mode = FramePolicy::Mode::kTimeWindow;
  o.frame_policy.window = 30ms;
  o.timestamp_policy = TimestampPolicy::kHostReceive;
  Recorder rec;
  auto dev = f.open(o);
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 10; }));
  CHECK(rec.ok);
  CHECK_FALSE(dev->stats().time_offset_valid);
  const std::lock_guard lock(rec.mutex);
  for (const Frame & fr : rec.kept) {
    CHECK(fr.end_time_ns - fr.base_time_ns < 40'000'000);
    CHECK(fr.packets < 30);
  }
}

TEST_CASE("Device: idle close delivers the partial frame", "[sim][device]")
{
  Fixture f({"--frame-ms", "50"});
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;
  auto dev = f.open();
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 3; }));
  REQUIRE(f.sim->control(R"({"cmd":"silence","seconds":1.5})"));
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());
  std::this_thread::sleep_for(600ms);  // > window (100 ms) after the last packet
  // Every point-cloud packet that arrived has been delivered inside a frame.
  CHECK(rec.pcl_packets == rec.frame_packets);
  CHECK(rec.pcl_packets > 0);
  const auto frames_now = rec.frames.load();
  std::this_thread::sleep_for(200ms);
  CHECK(rec.frames == frames_now);  // nothing is invented while silent
}

TEST_CASE("Device: stop discards the partial frame and re-enables callbacks", "[sim][device]")
{
  Fixture f({"--frame-ms", "0"});  // no frame_cnt closes: frames come from the fallback
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  DeviceOptions o = Fixture::options();
  o.frame_policy.window = 1s;  // fallback only after 2 s: everything stays partial
  Recorder rec;
  auto dev = f.open(o);
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.pcl_packets >= 100; }));
  REQUIRE(dev->stop_sampling().has_value());
  CHECK(rec.frames == 0);
  CHECK(dev->on_imu([](const ImuData &) {}).has_value());
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 1; }, 6s));
  CHECK(rec.ok);
}
