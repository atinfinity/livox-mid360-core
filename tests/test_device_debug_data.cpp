// SPDX-License-Identifier: Apache-2.0
// Debug raw data collection (issue #93) against tools/livox_mid360_sim.py: the Context's
// optional fifth socket, start / stop / idempotence, the error without the socket, replay
// after a reconnect and two LiDARs on one Context.
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

std::unique_ptr<Context> loopback_context(std::optional<std::uint16_t> debug_data_port)
{
  ContextOptions o;
  o.bind_address = {127, 0, 0, 1};
  o.push_port = o.point_port = o.imu_port = o.log_port = 0;
  o.debug_data_port = debug_data_port;
  auto c = Context::create(o);
  REQUIRE(c.has_value());
  return std::move(*c);
}

/// The data rate is kept very low and the debug stream at 50 datagrams/s so that sanitizer
/// builds keep up (see test_reconnect.cpp).
std::optional<SimProcess> start_sim(std::string & err, std::vector<std::string> args = {})
{
  args.insert(
    args.end(), {"--rate-multiplier", "0.05", "--push-rate", "10", "--debug-data-interval", "0.02",
                 "--debug-data-bytes", "200"});
  return SimProcess::start(err, std::move(args));
}

DeviceOptions options(const SimProcess & sim)
{
  DeviceOptions o;
  o.session.host_command_port = 0;
  o.session.request = {.timeout = 500ms, .attempts = 3};
  o.reconnect.push_timeout = 500ms;
  o.reconnect.initial_backoff = 100ms;
  o.reconnect.max_backoff = 400ms;
  o.reconnect.discovery_timeout = 200ms;
  o.lidar_debug_data_port = sim.ports().log;  // the simulator's 0x03xx socket
  return o;
}

std::unique_ptr<Device> open(Context & context, const SimProcess & sim, Ipv4 ip = {127, 0, 0, 1})
{
  const DiscoveredDevice found{
    .serial_number = sim.sn(),
    .ip = ip,
    .cmd_port = sim.ports().cmd,
    .dev_type = 9,
    .from = Endpoint{ip, sim.ports().cmd}};
  auto d = Device::open(context, found, options(sim));
  if (!d) {
    FAIL(to_string(d.error()));
  }
  return std::move(*d);
}

/// Checks the simulator's synthetic payload: seq u32, then bytes counting up from its low byte.
struct DebugRecorder
{
  std::atomic<std::uint64_t> packets{0};
  std::atomic<std::uint64_t> bytes{0};
  std::atomic<std::uint64_t> restarts{0};  ///< seq went back to 0 (the simulator rebooted)
  std::atomic<bool> ok{true};
  Ipv4 expected_ip{127, 0, 0, 1};
  std::optional<std::uint32_t> last_seq;  ///< receive thread only

  void record(const DebugDataPacket & p)
  {
    ++packets;
    bytes += p.data.size();
    if (
      p.host_receive_time_ns == 0 || p.from.ip != expected_ip || p.from.port == 0 ||
      p.data.size() != 200) {
      ok = false;
      return;
    }
    const auto at = [&](std::size_t i) { return std::to_integer<std::uint32_t>(p.data[i]); };
    const std::uint32_t seq = at(0) | (at(1) << 8) | (at(2) << 16) | (at(3) << 24);
    std::uint32_t expected = seq;
    for (const std::byte b : p.data.subspan(4)) {
      if (std::to_integer<std::uint32_t>(b) != (expected & 0xFFu)) {
        ok = false;
      }
      ++expected;
    }
    if (last_seq && seq == 0) {
      ++restarts;
    } else if (last_seq && seq <= *last_seq) {
      ok = false;
    }
    last_seq = seq;
  }

  void attach(Device & d)
  {
    REQUIRE(d.on_debug_data([this](const DebugDataPacket & p) { record(p); }).has_value());
  }
};

std::string status(SimProcess & sim)
{
  REQUIRE(sim.control(R"({"cmd":"status"})"));
  const auto line = sim.wait_event(R"("event":"status")");
  REQUIRE(line.has_value());
  return *line;
}

}  // namespace

TEST_CASE("Context: the debug data socket is opened on request only", "[device][context]")
{
  const auto without = loopback_context(std::nullopt);
  CHECK_FALSE(without->options().debug_data_port.has_value());
  CHECK(without->stats().debug_datagrams == 0);

  const auto with = loopback_context(0);
  REQUIRE(with->options().debug_data_port.has_value());
  const std::uint16_t port = *with->options().debug_data_port;
  CHECK(port != 0);  // the bound port replaces the requested 0

  // The port is taken now: a second Context asking for it fails.
  ContextOptions o;
  o.bind_address = {127, 0, 0, 1};
  o.push_port = o.point_port = o.imu_port = o.log_port = 0;
  o.debug_data_port = port;
  const auto clash = Context::create(o);
  REQUIRE_FALSE(clash.has_value());
  REQUIRE(clash.error().session.has_value());
  CHECK(clash.error().session->kind == SessionErrorKind::kTransport);
}

TEST_CASE("Device: debug data start, delivery, idempotence and stop", "[sim][device]")
{
  std::string err;
  auto sim = start_sim(err);
  if (!sim) {
    SKIP("simulator unavailable: " << err);
  }
  const auto context = loopback_context(0);
  DebugRecorder rec;
  std::atomic<std::uint64_t> frames{0};
  auto dev = open(*context, *sim);
  rec.attach(*dev);
  REQUIRE(dev->on_frame([&](const Frame &) { ++frames; }).has_value());
  CHECK(dev->stats().debug_data_packets == 0);
  CHECK(dev->stats().last_debug_data_time_ns == 0);

  // No work-state precondition: the stream starts before sampling does.
  REQUIRE(dev->start_debug_data().has_value());
  REQUIRE(wait_until([&] { return rec.packets >= 5; }));
  {
    const std::string s = status(*sim);
    CHECK(s.find("\"enabled\":true") != std::string::npos);
    CHECK(s.find(std::to_string(*context->options().debug_data_port)) != std::string::npos);
  }
  REQUIRE(dev->start_debug_data().has_value());  // idempotent
  // The point cloud keeps flowing next to the debug stream.
  REQUIRE(dev->start_sampling().has_value());
  const auto before = rec.packets.load();
  REQUIRE(wait_until([&] { return frames >= 3 && rec.packets >= before + 5; }));

  REQUIRE(dev->stop_debug_data().has_value());
  REQUIRE(dev->stop_debug_data().has_value());  // idempotent
  REQUIRE(sim->sync());
  std::this_thread::sleep_for(100ms);  // datagrams already on their way
  const auto after = rec.packets.load();
  std::this_thread::sleep_for(200ms);
  CHECK(rec.packets == after);
  CHECK(rec.ok);
  CHECK(rec.restarts == 0);

  const DeviceStats s = dev->stats();
  CHECK(s.debug_data_packets == rec.packets);
  CHECK(s.debug_data_bytes == rec.bytes);
  CHECK(s.debug_data_bytes == s.debug_data_packets * 200);
  CHECK(s.last_debug_data_time_ns > 0);
  CHECK(s.bad_packets == 0);
  CHECK(context->stats().debug_datagrams == s.debug_data_packets);
  CHECK(context->stats().unknown_source == 0);
  CHECK(status(*sim).find("\"enabled\":false") != std::string::npos);
  REQUIRE(dev->stop_sampling().has_value());
}

TEST_CASE("Device: debug data without on_debug_data is still counted", "[sim][device]")
{
  std::string err;
  auto sim = start_sim(err);
  if (!sim) {
    SKIP("simulator unavailable: " << err);
  }
  const auto context = loopback_context(0);
  auto dev = open(*context, *sim);
  REQUIRE(dev->start_debug_data().has_value());
  REQUIRE(wait_until([&] { return dev->stats().debug_data_packets >= 5; }));
  REQUIRE(dev->stop_debug_data().has_value());
}

TEST_CASE("Device: start_debug_data needs the Context's debug data socket", "[sim][device]")
{
  std::string err;
  auto sim = start_sim(err);
  if (!sim) {
    SKIP("simulator unavailable: " << err);
  }
  const auto context = loopback_context(std::nullopt);
  auto dev = open(*context, *sim);
  // Subscribing is allowed; the callback just never runs.
  std::atomic<std::uint64_t> n{0};
  REQUIRE(dev->on_debug_data([&](const DebugDataPacket &) { ++n; }).has_value());
  const auto r = dev->start_debug_data();
  REQUIRE_FALSE(r.has_value());
  CHECK(r.error().kind == DeviceError::Kind::kInvalidState);
  CHECK(status(*sim).find("\"enabled\":false") != std::string::npos);  // nothing was sent
  // A stop is still sent, e.g. to end a stream an earlier process left running.
  REQUIRE(dev->stop_debug_data().has_value());
  CHECK(n == 0);
  CHECK(dev->stats().debug_data_packets == 0);
}

TEST_CASE("Device: start_debug_data reports a rejection and a timeout", "[sim][device]")
{
  std::string err;
  auto sim = start_sim(err);
  if (!sim) {
    SKIP("simulator unavailable: " << err);
  }
  const auto context = loopback_context(0);
  DeviceOptions quiet = options(*sim);
  quiet.session.request = {.timeout = 50ms, .attempts = 2};
  quiet.lidar_debug_data_port = 1;  // no UDP listener there
  const DiscoveredDevice found{
    .serial_number = sim->sn(),
    .ip = {127, 0, 0, 1},
    .cmd_port = sim->ports().cmd,
    .dev_type = 9,
    .from = Endpoint::loopback(sim->ports().cmd)};
  auto dev = Device::open(*context, found, quiet);
  REQUIRE(dev.has_value());
  const auto r = (*dev)->start_debug_data();
  REQUIRE_FALSE(r.has_value());
  REQUIRE(r.error().session.has_value());
  CHECK(r.error().session->kind == SessionErrorKind::kTimeout);
  CHECK(r.error().session->attempts == 2);
  CHECK(r.error().session->cmd_id == static_cast<std::uint16_t>(CmdId::kDebugDataControl));
  // The failed start is not replayed and the firmware log path still works.
  CHECK((*dev)->stats().debug_data_packets == 0);
}

TEST_CASE("Reconnect: debug data is replayed until a stop succeeds", "[sim][reconnect]")
{
  std::string err;
  auto sim = start_sim(err, {"--reboot-silence", "0.3"});
  if (!sim) {
    SKIP("simulator unavailable: " << err);
  }
  const auto context = loopback_context(0);
  DebugRecorder rec;
  std::atomic<std::uint64_t> reconnected{0};
  auto dev = open(*context, *sim);
  rec.attach(*dev);
  REQUIRE(dev
            ->on_event([&](const Event & e) {
              if (e.kind == Event::Kind::kReconnected) ++reconnected;
            })
            .has_value());
  REQUIRE(dev->start_debug_data().has_value());
  REQUIRE(wait_until([&] { return rec.packets >= 5; }));

  // The simulator forgets the stream over a reboot; the replay switches it on again.
  REQUIRE(dev->reboot().has_value());
  REQUIRE(wait_until([&] { return reconnected == 1; }, 8s));
  REQUIRE(wait_until([&] { return rec.restarts == 1; }));
  const auto resumed = rec.packets.load();
  REQUIRE(wait_until([&] { return rec.packets >= resumed + 5; }));
  CHECK(rec.ok);

  // After a stop nothing is replayed.
  REQUIRE(dev->stop_debug_data().has_value());
  REQUIRE(dev->reboot().has_value());
  REQUIRE(wait_until([&] { return reconnected == 2; }, 8s));
  CHECK(status(*sim).find("\"enabled\":false") != std::string::npos);
  const auto after = rec.packets.load();
  std::this_thread::sleep_for(200ms);
  CHECK(rec.packets == after);
  CHECK(rec.restarts == 1);
}

TEST_CASE("Multi-device: debug data of two LiDARs on one Context", "[sim][device]")
{
  // The second simulator answers from 127.0.0.2; macOS needs `ifconfig lo0 alias 127.0.0.2`.
  if (!UdpSocket::open(Endpoint{{127, 0, 0, 2}, 0}).has_value()) {
    SKIP("127.0.0.2 is not configured on the loopback interface");
  }
  std::string err;
  auto sim_a = start_sim(err);
  if (!sim_a) {
    SKIP("simulator unavailable: " << err);
  }
  auto sim_b = start_sim(err, {"--bind", "127.0.0.2", "--sn", "SIM0000000000002"});
  if (!sim_b) {
    SKIP("second simulator unavailable: " << err);
  }
  const auto context = loopback_context(0);
  DebugRecorder rec_a;
  DebugRecorder rec_b;
  rec_b.expected_ip = {127, 0, 0, 2};
  auto dev_a = open(*context, *sim_a);
  auto dev_b = open(*context, *sim_b, {127, 0, 0, 2});
  rec_a.attach(*dev_a);
  rec_b.attach(*dev_b);

  REQUIRE(dev_a->start_debug_data().has_value());
  REQUIRE(wait_until([&] { return rec_a.packets >= 5; }));
  CHECK(rec_b.packets == 0);
  REQUIRE(dev_b->start_debug_data().has_value());
  REQUIRE(wait_until([&] { return rec_b.packets >= 5; }));

  // Stopping one stream leaves the other running.
  REQUIRE(dev_a->stop_debug_data().has_value());
  REQUIRE(sim_a->sync());
  std::this_thread::sleep_for(100ms);
  const auto a = rec_a.packets.load();
  const auto b = rec_b.packets.load();
  REQUIRE(wait_until([&] { return rec_b.packets >= b + 5; }));
  CHECK(rec_a.packets == a);
  REQUIRE(dev_b->stop_debug_data().has_value());
  CHECK(rec_a.ok);
  CHECK(rec_b.ok);
  CHECK(context->stats().unknown_source == 0);
  CHECK(dev_a->stats().debug_data_packets == rec_a.packets);
  CHECK(dev_b->stats().debug_data_packets == rec_b.packets);
}
