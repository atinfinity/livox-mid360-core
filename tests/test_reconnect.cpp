// SPDX-License-Identifier: Apache-2.0
// Reconnection state machine and multi-device registry (issue #8) against
// tools/livox_mid360_sim.py: push-timeout / command-timeout / reboot detection, automatic
// recovery with sampling replay, manual reconnect(), prompt destruction mid-attempt, and two
// LiDARs on one Context looked up by serial number.
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

namespace {

bool wait_until(const std::function<bool()>& pred, std::chrono::milliseconds timeout = 5s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(10ms);
  }
  return pred();
}

std::unique_ptr<Context> loopback_context() {
  ContextOptions o;
  o.bind_address = {127, 0, 0, 1};
  o.push_port = o.point_port = o.imu_port = 0;
  auto c = Context::create(o);
  REQUIRE(c.has_value());
  return std::move(*c);
}

DiscoveredDevice discovered(const SimProcess& sim, Ipv4 ip = {127, 0, 0, 1}) {
  return DiscoveredDevice{.serial_number = sim.sn(),
                          .ip = ip,
                          .cmd_port = sim.ports().cmd,
                          .dev_type = 9,
                          .from = Endpoint{ip, sim.ports().cmd}};
}

/// Pushes at 10 Hz; disconnect after 500 ms of silence; fast backoff so tests stay short.
struct Fixture {
  std::optional<SimProcess> sim;
  std::string err;
  std::unique_ptr<Context> context;

  explicit Fixture(std::vector<std::string> args = {}) {
    args.insert(args.end(), {"--rate-multiplier", "0.25", "--push-rate", "10"});
    sim = SimProcess::start(err, std::move(args));
    if (sim) context = loopback_context();
  }

  [[nodiscard]] static DeviceOptions options() {
    DeviceOptions o;
    o.session.host_command_port = 0;
    o.session.request = {.timeout = 200ms, .attempts = 2};
    o.reconnect.push_timeout = 500ms;
    o.reconnect.initial_backoff = 100ms;
    o.reconnect.max_backoff = 400ms;
    o.reconnect.discovery_timeout = 200ms;
    return o;
  }

  [[nodiscard]] std::unique_ptr<Device> open(DeviceOptions o = options()) const {
    auto d = Device::open(*context, discovered(*sim), o);
    if (!d) FAIL(to_string(d.error()));
    return std::move(*d);
  }
};

struct Recorder {
  std::atomic<std::uint64_t> frames{0};
  std::atomic<std::uint64_t> disconnected{0};
  std::atomic<std::uint64_t> reconnected{0};
  std::mutex mutex;
  std::vector<Event> events;  ///< everything except kStats, in order

  void attach(Device& d) {
    REQUIRE(d.on_frame([this](Frame&&) { ++frames; }).has_value());
    REQUIRE(d.on_event([this](const Event& e) {
               if (e.kind == Event::Kind::kStats) return;
               const std::lock_guard lock(mutex);
               events.push_back(e);
               if (e.kind == Event::Kind::kDisconnected) ++disconnected;
               if (e.kind == Event::Kind::kReconnected) ++reconnected;
             }).has_value());
  }

  [[nodiscard]] std::optional<Event> last(Event::Kind kind) {
    const std::lock_guard lock(mutex);
    for (auto it = events.rbegin(); it != events.rend(); ++it) {
      if (it->kind == kind) return *it;
    }
    return std::nullopt;
  }
};

}  // namespace

TEST_CASE("Reconnect: push timeout, automatic recovery, sampling resumes", "[sim][reconnect]") {
  Fixture f;
  if (!f.sim) SKIP("simulator unavailable: " << f.err);
  Recorder rec;
  auto dev = f.open();
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 3; }));
  CHECK(dev->connected());

  REQUIRE(f.sim->control(R"({"cmd":"silence","seconds":1.5})"));
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());
  REQUIRE(wait_until([&] { return rec.disconnected == 1; }, 2s));
  CHECK_FALSE(dev->connected());
  const auto down = rec.last(Event::Kind::kDisconnected);
  REQUIRE(down.has_value());
  CHECK(down->reason == DisconnectReason::kPushTimeout);
  CHECK(down->time_ns != 0);
  // Commands fail fast while disconnected.
  const auto r = dev->start_sampling();
  REQUIRE_FALSE(r.has_value());
  CHECK(r.error().kind == DeviceError::Kind::kDisconnected);
  CHECK(to_string(r.error()) == "disconnected");

  REQUIRE(wait_until([&] { return rec.reconnected == 1; }, 6s));
  CHECK(dev->connected());
  const auto up = rec.last(Event::Kind::kReconnected);
  REQUIRE(up.has_value());
  CHECK(up->attempts >= 1);
  CHECK(to_string(*up) == "reconnected attempts=" + std::to_string(up->attempts));
  // Sampling was requested before the outage: data flows again without user action.
  const auto frames_after_reconnect = rec.frames.load();
  REQUIRE(wait_until([&] { return rec.frames >= frames_after_reconnect + 3; }));
  const DeviceStats s = dev->stats();
  CHECK(s.disconnects == 1);
  CHECK(s.reconnects == 1);
  CHECK(rec.disconnected == 1);
  // Callbacks stayed frozen throughout (sampling is still requested).
  CHECK_FALSE(dev->on_imu([](const ImuData&) {}).has_value());
}

TEST_CASE("Reconnect: reboot() is noticed immediately and settings are replayed",
          "[sim][reconnect]") {
  Fixture f({"--reboot-silence", "0.3"});
  if (!f.sim) SKIP("simulator unavailable: " << f.err);
  Recorder rec;
  auto dev = f.open();
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 3; }));

  const auto t0 = std::chrono::steady_clock::now();
  REQUIRE(dev->reboot().has_value());
  REQUIRE(wait_until([&] { return rec.disconnected == 1; }, 200ms));  // well under push_timeout
  CHECK(std::chrono::steady_clock::now() - t0 < 400ms);
  const auto down = rec.last(Event::Kind::kDisconnected);
  REQUIRE(down.has_value());
  CHECK(down->reason == DisconnectReason::kRebootRequested);
  CHECK(to_string(*down) == "disconnected reason=reboot_requested");

  REQUIRE(wait_until([&] { return rec.reconnected == 1; }, 8s));
  // The simulator reboots into a non-sampling state; the replay brings it back.
  REQUIRE(wait_until([&] { return dev->work_state() == WorkState::kSampling; }, 5s));
  const auto frames_after = rec.frames.load();
  REQUIRE(wait_until([&] { return rec.frames >= frames_after + 3; }));
  CHECK(dev->stats().reconnects == 1);
}

TEST_CASE("Reconnect: a lost ACK alone is not a disconnect, a stale push plus timeout is",
          "[sim][reconnect]") {
  Fixture f;
  if (!f.sim) SKIP("simulator unavailable: " << f.err);
  Recorder rec;
  DeviceOptions o = Fixture::options();
  o.session.request = {.timeout = 100ms, .attempts = 1};
  o.reconnect.push_timeout = 1000ms;  // stale after 333 ms; the push timeout itself is late
  auto dev = f.open(o);
  rec.attach(*dev);

  // Pushes keep flowing: the timeout is reported to the caller and nothing else happens.
  REQUIRE(f.sim->control(R"({"cmd":"drop_ack","count":1})"));
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());
  const std::uint16_t keys[] = {static_cast<std::uint16_t>(Key::kSn)};
  const auto r = dev->inquire(keys);
  REQUIRE_FALSE(r.has_value());
  REQUIRE(r.error().session.has_value());
  CHECK(r.error().session->kind == SessionErrorKind::kTimeout);
  std::this_thread::sleep_for(200ms);
  CHECK(dev->connected());
  CHECK(rec.disconnected == 0);

  // Silence: the push is stale (> push_timeout / 3) when the command times out at ~500 ms,
  // well ahead of the 1 s push timeout, so the command timeout declares the disconnect.
  REQUIRE(f.sim->control(R"({"cmd":"silence","seconds":1.0})"));
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());
  std::this_thread::sleep_for(400ms);
  CHECK_FALSE(dev->inquire(keys).has_value());
  REQUIRE(wait_until([&] { return rec.disconnected == 1; }, 100ms));
  const auto down = rec.last(Event::Kind::kDisconnected);
  REQUIRE(down.has_value());
  CHECK(down->reason == DisconnectReason::kCommandTimeout);
  REQUIRE(wait_until([&] { return rec.reconnected == 1; }, 6s));
  CHECK(dev->connected());
}

TEST_CASE("Reconnect: disabled means detect only; reconnect() recovers by hand",
          "[sim][reconnect]") {
  Fixture f;
  if (!f.sim) SKIP("simulator unavailable: " << f.err);
  Recorder rec;
  DeviceOptions o = Fixture::options();
  o.reconnect.enabled = false;
  auto dev = f.open(o);
  rec.attach(*dev);
  CHECK(dev->reconnect().has_value());  // no-op while connected
  REQUIRE(wait_until([&] { return dev->work_state().has_value(); }));
  const WorkState before = *dev->work_state();
  REQUIRE(before != WorkState::kError);

  REQUIRE(f.sim->control(R"({"cmd":"silence","seconds":0.8})"));
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());
  // The LiDAR changes state during the outage (4 = ERROR is what a real one might show).
  REQUIRE(f.sim->control(R"({"cmd":"set_state","state":4})"));
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());
  REQUIRE(wait_until([&] { return rec.disconnected == 1; }, 2s));
  // The simulator is back, but nobody reconnects.
  std::this_thread::sleep_for(1200ms);
  CHECK_FALSE(dev->connected());
  CHECK(rec.reconnected == 0);
  // The push baseline survives the outage: the first push afterwards is reported against
  // the last state seen before it, not swallowed as after open().
  const auto changed = rec.last(Event::Kind::kStateChanged);
  REQUIRE(changed.has_value());
  CHECK(changed->old_state == before);
  CHECK(changed->new_state == WorkState::kError);
  REQUIRE(f.sim->control(R"({"cmd":"set_state","state":2})"));
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());

  REQUIRE(dev->reconnect().has_value());
  CHECK(dev->connected());
  REQUIRE(wait_until([&] { return rec.reconnected == 1; }));
  const auto up = rec.last(Event::Kind::kReconnected);
  REQUIRE(up.has_value());
  CHECK(up->attempts == 1);
  // Callbacks were never frozen (sampling not requested), so they can still be changed.
  CHECK(dev->on_imu([](const ImuData&) {}).has_value());
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec.frames >= 3; }));

  // disconnect() forces a re-discovery pass without touching the LiDAR.
  dev->disconnect();
  REQUIRE(wait_until([&] { return rec.disconnected == 2; }, 1s));
  const auto down = rec.last(Event::Kind::kDisconnected);
  REQUIRE(down.has_value());
  CHECK(down->reason == DisconnectReason::kUser);
  REQUIRE(dev->reconnect().has_value());
  REQUIRE(wait_until([&] { return rec.reconnected == 2; }));
}

TEST_CASE("Reconnect: the destructor returns promptly while an attempt is in progress",
          "[sim][reconnect]") {
  Fixture f;
  if (!f.sim) SKIP("simulator unavailable: " << f.err);
  Recorder rec;
  DeviceOptions o = Fixture::options();
  o.session.request = {.timeout = 1000ms, .attempts = 3};  // a direct attempt takes 3 s
  o.reconnect.discovery_timeout = 2000ms;
  auto dev = f.open(o);
  rec.attach(*dev);
  REQUIRE(f.sim->control(R"({"cmd":"silence","seconds":10})"));
  REQUIRE(f.sim->wait_event(R"("event":"control")").has_value());
  REQUIRE(wait_until([&] { return rec.disconnected == 1; }, 2s));
  std::this_thread::sleep_for(300ms);  // the worker is inside Session::connect now
  const auto t0 = std::chrono::steady_clock::now();
  dev.reset();
  CHECK(std::chrono::steady_clock::now() - t0 < 500ms);
}

TEST_CASE("Reconnect: invalid options", "[reconnect]") {
  auto ctx = loopback_context();
  DeviceOptions o;
  o.reconnect.max_backoff = 100ms;
  o.reconnect.initial_backoff = 200ms;
  const auto d = Device::open(*ctx, DiscoveredDevice{.serial_number = "X", .from = {}}, o);
  REQUIRE_FALSE(d.has_value());
  CHECK(d.error().kind == DeviceError::Kind::kInvalidArgument);
}

TEST_CASE("Multi-device: two LiDARs on one Context, looked up by serial", "[sim][device]") {
  // The second simulator answers from 127.0.0.2; macOS needs `ifconfig lo0 alias 127.0.0.2`.
  if (!UdpSocket::open(Endpoint{{127, 0, 0, 2}, 0}).has_value()) {
    SKIP("127.0.0.2 is not configured on the loopback interface");
  }
  Fixture a;
  if (!a.sim) SKIP("simulator unavailable: " << a.err);
  std::string err;
  auto sim_b = SimProcess::start(err, {"--bind", "127.0.0.2", "--sn", "SIM0000000000002",
                                       "--rate-multiplier", "0.25", "--push-rate", "10"});
  if (!sim_b) SKIP("second simulator unavailable: " << err);
  REQUIRE(sim_b->ip() == "127.0.0.2");

  Recorder rec_a;
  Recorder rec_b;
  auto dev_a = a.open();
  rec_a.attach(*dev_a);
  auto dev_b = Device::open(*a.context, discovered(*sim_b, {127, 0, 0, 2}), Fixture::options());
  if (!dev_b) FAIL(to_string(dev_b.error()));
  rec_b.attach(**dev_b);

  CHECK(a.context->find(a.sim->sn()) == dev_a.get());
  CHECK(a.context->find(sim_b->sn()) == dev_b->get());
  CHECK(a.context->find("nope") == nullptr);
  CHECK(a.context->devices() == std::vector<Device*>{dev_a.get(), dev_b->get()});
  CHECK((*dev_b)->info().ip == Ipv4{127, 0, 0, 2});

  // A second Device for a serial (or an IP) that is already open is refused.
  const auto dup = Device::open(*a.context, discovered(*sim_b, {127, 0, 0, 2}), Fixture::options());
  REQUIRE_FALSE(dup.has_value());
  CHECK(dup.error().kind == DeviceError::Kind::kAlreadyRegistered);

  REQUIRE(dev_a->start_sampling().has_value());
  REQUIRE((*dev_b)->start_sampling().has_value());
  REQUIRE(wait_until([&] { return rec_a.frames >= 3 && rec_b.frames >= 3; }));

  // An outage of one LiDAR does not touch the other.
  REQUIRE(sim_b->control(R"({"cmd":"silence","seconds":1.5})"));
  REQUIRE(sim_b->wait_event(R"("event":"control")").has_value());
  REQUIRE(wait_until([&] { return rec_b.disconnected == 1; }, 2s));
  CHECK(dev_a->connected());
  CHECK(rec_a.disconnected == 0);
  REQUIRE(wait_until([&] { return rec_b.reconnected == 1; }, 6s));
  CHECK(a.context->stats().unknown_source == 0);

  dev_b->reset();
  CHECK(a.context->find(sim_b->sn()) == nullptr);
  CHECK(a.context->devices() == std::vector<Device*>{dev_a.get()});
}
