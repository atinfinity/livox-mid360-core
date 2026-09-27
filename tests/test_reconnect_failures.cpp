// SPDX-License-Identifier: Apache-2.0
// Failing branches of the reconnect attempt in device.cpp (issue #97) against
// tools/livox_mid360_sim.py: a replay step the LiDAR rejects (host setup, sampling, firmware
// log), a LiDAR that comes back on an address another Device holds, and the backoff between
// attempts. The rejections are injected with the simulator's `fail_cmd` control.
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
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
#include "log_capture.hpp"
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

DiscoveredDevice discovered(const SimProcess & sim, Ipv4 ip = {127, 0, 0, 1})
{
  return DiscoveredDevice{
    .serial_number = sim.sn(),
    .ip = ip,
    .cmd_port = sim.ports().cmd,
    .dev_type = 9,
    .from = Endpoint{ip, sim.ports().cmd}};
}

/// See test_reconnect.cpp for the rates. Reconnection is manual unless a test enables it.
struct Fixture
{
  std::optional<SimProcess> sim;
  std::string err;
  std::unique_ptr<Context> context;
  std::atomic<std::uint64_t> frames{0};
  std::atomic<std::uint64_t> disconnected{0};
  std::atomic<std::uint64_t> reconnected{0};
  std::atomic<std::uint32_t> attempts{0};  ///< of the last kReconnected

  explicit Fixture(std::vector<std::string> args = {})
  {
    args.insert(args.end(), {"--rate-multiplier", "0.05", "--push-rate", "10"});
    sim = SimProcess::start(err, std::move(args));
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

  [[nodiscard]] DeviceOptions options() const
  {
    DeviceOptions o;
    o.session.host_command_port = 0;
    o.session.request = {.timeout = 200ms, .attempts = 2};
    o.reconnect.enabled = false;
    o.reconnect.push_timeout = 500ms;
    o.reconnect.initial_backoff = 100ms;
    o.reconnect.max_backoff = 400ms;
    o.reconnect.discovery_timeout = 200ms;
    o.reconnect.discovery_port = sim->ports().discovery;
    o.lidar_log_port = sim->ports().log;
    return o;
  }

  void on_event(const Event & e)
  {
    if (e.kind == Event::Kind::kDisconnected) {
      ++disconnected;
    } else if (e.kind == Event::Kind::kReconnected) {
      attempts = e.attempts;
      ++reconnected;
    }
  }

  [[nodiscard]] std::unique_ptr<Device> open(const DeviceOptions & o)
  {
    auto d = Device::open(*context, discovered(*sim), o);
    if (!d) {
      FAIL(to_string(d.error()));
    }
    REQUIRE((*d)->on_frame([this](const Frame &) { ++frames; }).has_value());
    REQUIRE((*d)->on_event([this](const Event & e) { on_event(e); }).has_value());
    return std::move(*d);
  }

  void control(const std::string & json)
  {
    REQUIRE(sim->control(json));
    REQUIRE(sim->wait_event(R"("event":"control")").has_value());
  }

  /// disconnect(), then one reconnect() that the LiDAR is expected to reject at `cmd`.
  void rejected_attempt(Device & dev, CmdId cmd)
  {
    const auto down = disconnected.load();
    dev.disconnect();
    REQUIRE(wait_until([&] { return disconnected == down + 1; }, 1s));
    const auto up = reconnected.load();
    const auto r = dev.reconnect();
    REQUIRE_FALSE(r.has_value());
    REQUIRE(r.error().kind == DeviceError::Kind::kSession);
    REQUIRE(r.error().session.has_value());
    CHECK(r.error().session->kind == SessionErrorKind::kLidarRejected);
    CHECK(r.error().session->cmd_id == static_cast<std::uint16_t>(cmd));
    CHECK(r.error().session->ret_code == static_cast<RetCode>(0x02));
    // Reported as not connected: no event, commands refused.
    CHECK_FALSE(dev.connected());
    const auto refused = dev.identity();
    REQUIRE_FALSE(refused.has_value());
    CHECK(refused.error().kind == DeviceError::Kind::kDisconnected);
    std::this_thread::sleep_for(100ms);
    CHECK(reconnected == up);
  }

  /// The next reconnect() goes through and reports both attempts.
  void recovers(Device & dev)
  {
    const auto up = reconnected.load();
    const auto r = dev.reconnect();
    if (!r) {
      FAIL(to_string(r.error()));
    }
    CHECK(dev.connected());
    REQUIRE(wait_until([&] { return reconnected == up + 1; }));
    CHECK(attempts == 2);
    CHECK(dev.identity().has_value());
  }
};

}  // namespace

TEST_CASE("Reconnect: a rejected host setup replay fails the attempt", "[sim][reconnect]")
{
  Fixture f;
  auto dev = f.open(f.options());
  f.control(R"({"cmd":"fail_cmd","cmd_id":256,"ret":2})");
  f.rejected_attempt(*dev, CmdId::kParamConfig);
  f.recovers(*dev);
}

TEST_CASE("Reconnect: a rejected sampling replay fails the attempt", "[sim][reconnect]")
{
  Fixture f;
  auto dev = f.open(f.options());
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(wait_until([&] { return f.frames >= 3; }));
  // Key 0x001A: only the work mode request matches, the host setup replay passes.
  f.control(R"({"cmd":"fail_cmd","cmd_id":256,"ret":2,"key":26})");
  f.rejected_attempt(*dev, CmdId::kParamConfig);
  f.recovers(*dev);
  REQUIRE(wait_until([&] { return dev->work_state() == WorkState::kSampling; }));
  const auto frames = f.frames.load();
  CHECK(wait_until([&] { return f.frames >= frames + 3; }));
}

TEST_CASE("Reconnect: a rejected firmware log replay fails the attempt", "[sim][reconnect]")
{
  Fixture f({"--log-chunk-interval", "0.05", "--log-chunk-bytes", "64"});
  auto dev = f.open(f.options());
  REQUIRE(dev->start_firmware_log().has_value());
  REQUIRE(wait_until([&] { return dev->stats().log_chunks >= 2; }));
  f.control(R"({"cmd":"fail_cmd","cmd_id":769,"ret":2})");
  f.rejected_attempt(*dev, CmdId::kCollectionLog);
  f.recovers(*dev);
  const auto chunks = dev->stats().log_chunks;
  CHECK(wait_until([&] { return dev->stats().log_chunks >= chunks + 2; }));
}

TEST_CASE("Reconnect: the new address belongs to another Device", "[sim][reconnect]")
{
  if (!UdpSocket::open(Endpoint{{127, 0, 0, 2}, 0}).has_value()) {
    SKIP("127.0.0.2 is not configured on the loopback interface");
  }
  Fixture f({"--reboot-silence", "0.2"});
  std::string err;
  auto sim_b = SimProcess::start(
    err, {"--bind", "127.0.0.2", "--sn", "SIM0000000000002", "--rate-multiplier", "0.05",
          "--push-rate", "10"});
  if (!sim_b) {
    SKIP("second simulator unavailable: " << err);
  }
  auto dev_a = f.open(f.options());
  auto dev_b = Device::open(*f.context, discovered(*sim_b, {127, 0, 0, 2}), f.options());
  if (!dev_b) {
    FAIL(to_string(dev_b.error()));
  }

  // A moves to the address of B with the reboot.
  const LidarIpConfig moved{.ip = {127, 0, 0, 2}, .netmask = {255, 0, 0, 0}, .gateway = {}};
  REQUIRE(dev_a->set_lidar_ip_config(moved).has_value());
  REQUIRE(dev_a->reboot().has_value());
  REQUIRE(wait_until([&] { return f.disconnected == 1; }, 1s));
  REQUIRE(f.sim->wait_event(R"("event":"rebound")").has_value());
  std::this_thread::sleep_for(300ms);  // reboot silence

  const auto r = dev_a->reconnect();
  REQUIRE_FALSE(r.has_value());
  CHECK(r.error().kind == DeviceError::Kind::kAlreadyRegistered);
  CHECK_FALSE(dev_a->connected());
  CHECK(dev_a->info().ip == Ipv4{127, 0, 0, 1});  // the registry and info() are unchanged
  CHECK(f.context->find(sim_b->sn()) == dev_b->get());
  CHECK((*dev_b)->identity().has_value());

  // Once B is closed the address is free.
  dev_b->reset();
  const auto again = dev_a->reconnect();
  if (!again) {
    FAIL(to_string(again.error()));
  }
  CHECK(dev_a->info().ip == Ipv4{127, 0, 0, 2});
  CHECK(f.context->find(f.sim->sn()) == dev_a.get());
  REQUIRE(wait_until([&] { return f.reconnected == 1; }));
  CHECK(f.attempts == 2);
}

TEST_CASE("Reconnect: the backoff doubles up to max_backoff", "[sim][reconnect]")
{
  Fixture f;
  const ScopedLogCapture log(LogLevel::kInfo);
  DeviceOptions o = f.options();
  o.reconnect.enabled = true;
  o.reconnect.discovery_targets = {Endpoint{{127, 0, 0, 1}, f.sim->ports().discovery}};
  auto dev = f.open(o);
  f.control(R"({"cmd":"silence","seconds":4.5})");
  REQUIRE(wait_until([&] { return f.disconnected == 1; }, 2s));
  REQUIRE(wait_until([&] { return f.reconnected == 1; }, 12s));
  CHECK(dev->connected());
  const auto failed = [&](int ms) {
    return log.count(LogLevel::kInfo, std::format("next in {} ms", ms));
  };
  CHECK(failed(100) == 1);
  CHECK(failed(200) == 1);
  CHECK(failed(400) >= 2);  // capped
  CHECK(failed(800) == 0);
  CHECK(f.attempts == failed(100) + failed(200) + failed(400) + 1);
}
