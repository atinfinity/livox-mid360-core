// SPDX-License-Identifier: Apache-2.0
// Registry and receive loop branches of context.cpp (issue #100) against
// tools/livox_mid360_sim.py: a second Device with the serial number or the address of an open
// one, and a socket that holds more datagrams than one wake-up of the receive thread drains
// (8 rounds of ContextOptions::batch_size).
#include <array>
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

std::optional<SimProcess> start_sim(std::string & err, std::vector<std::string> args = {})
{
  args.insert(args.end(), {"--rate-multiplier", "0.05", "--push-rate", "10"});
  return SimProcess::start(err, std::move(args));
}

std::unique_ptr<Context> loopback_context(std::size_t batch_size = 32)
{
  ContextOptions o;
  o.bind_address = {127, 0, 0, 1};
  o.push_port = o.point_port = o.imu_port = o.log_port = 0;
  o.debug_data_port = 0;
  o.batch_size = batch_size;
  auto c = Context::create(o);
  REQUIRE(c.has_value());
  return std::move(*c);
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

DeviceOptions options(const SimProcess & sim)
{
  DeviceOptions o;
  o.session.host_command_port = 0;
  o.session.request = {.timeout = 300ms, .attempts = 2};
  o.reconnect.enabled = false;
  o.lidar_log_port = sim.ports().log;
  o.lidar_debug_data_port = sim.ports().log;
  return o;
}

std::unique_ptr<Device> open(Context & context, const SimProcess & sim)
{
  auto d = Device::open(context, discovered(sim), options(sim));
  if (!d) {
    FAIL(to_string(d.error()));
  }
  return std::move(*d);
}

std::array<std::byte, 8> numbered(std::uint32_t n)
{
  return {
    std::byte{'C'},
    std::byte{'T'},
    std::byte{'X'},
    std::byte{'!'},
    static_cast<std::byte>(n & 0xFFu),
    static_cast<std::byte>((n >> 8u) & 0xFFu),
    static_cast<std::byte>((n >> 16u) & 0xFFu),
    static_cast<std::byte>((n >> 24u) & 0xFFu)};
}

std::optional<std::uint32_t> number_of(std::span<const std::byte> data)
{
  if (data.size() != 8 || data[0] != std::byte{'C'} || data[3] != std::byte{'!'}) {
    return std::nullopt;
  }
  return std::to_integer<std::uint32_t>(data[4]) | std::to_integer<std::uint32_t>(data[5]) << 8u |
         std::to_integer<std::uint32_t>(data[6]) << 16u |
         std::to_integer<std::uint32_t>(data[7]) << 24u;
}

}  // namespace

TEST_CASE("Context: a serial number or an address is registered once", "[sim][context]")
{
  std::string err;
  auto sim = start_sim(err);
  if (!sim) {
    SKIP("simulator unavailable: " << err);
  }
  auto context = loopback_context();
  std::atomic<std::uint64_t> frames{0};  // before the Device, which may still deliver (#253)
  auto first = open(*context, *sim);
  REQUIRE(first->on_frame([&](const Frame &) { ++frames; }).has_value());
  REQUIRE(first->start_sampling().has_value());
  REQUIRE(wait_until([&] { return frames >= 2; }));

  const auto intact = [&] {
    CHECK(context->find(sim->sn()) == first.get());
    CHECK(context->devices() == std::vector<Device *>{first.get()});
    CHECK(first->connected());
    CHECK(first->identity().has_value());
    const auto seen = frames.load();
    CHECK(wait_until([&] { return frames >= seen + 2; }));
  };

  SECTION("same address")
  {
    DiscoveredDevice d = discovered(*sim);
    d.serial_number = "SOMEONE ELSE";
    DeviceOptions o = options(*sim);
    o.session.verify_serial = false;
    const auto second = Device::open(*context, d, o);
    REQUIRE_FALSE(second.has_value());
    CHECK(second.error().kind == DeviceError::Kind::kAlreadyRegistered);
    CHECK(context->find("SOMEONE ELSE") == nullptr);
    intact();
  }
  SECTION("same serial number on another address")
  {
    if (!UdpSocket::open(Endpoint{{127, 0, 0, 2}, 0}).has_value()) {
      SKIP("127.0.0.2 is not configured on the loopback interface");
    }
    auto twin = start_sim(err, {"--bind", "127.0.0.2"});
    if (!twin) {
      SKIP("second simulator unavailable: " << err);
    }
    REQUIRE(twin->sn() == sim->sn());
    const auto second = Device::open(*context, discovered(*twin, {127, 0, 0, 2}), options(*twin));
    REQUIRE_FALSE(second.has_value());
    CHECK(second.error().kind == DeviceError::Kind::kAlreadyRegistered);
    intact();

    // The refused Device left nothing behind: the address is free for another LiDAR.
    auto other = start_sim(err, {"--bind", "127.0.0.2", "--sn", "SIM0000000000002"});
    if (!other) {
      SKIP("third simulator unavailable: " << err);
    }
    twin.reset();
    auto third = Device::open(*context, discovered(*other, {127, 0, 0, 2}), options(*other));
    if (!third) {
      FAIL(to_string(third.error()));
    }
    CHECK(context->find("SIM0000000000002") == third->get());
    CHECK(context->devices() == std::vector<Device *>{first.get(), third->get()});
    CHECK(context->find(sim->sn()) == first.get());
    CHECK(first->identity().has_value());
  }
  SECTION("the registration ends with the Device")
  {
    first.reset();
    CHECK(context->find(sim->sn()) == nullptr);
    CHECK(context->devices().empty());
    first = open(*context, *sim);
    CHECK(context->find(sim->sn()) == first.get());
    first.reset();
  }
}

TEST_CASE("Context: draining a socket spans several wake-ups", "[sim][context]")
{
  constexpr std::uint32_t kCount = 400;
  constexpr std::size_t kBatch = 4;  // 8 rounds: at most 32 datagrams per wake-up

  std::string err;
  auto sim = start_sim(err);
  if (!sim) {
    SKIP("simulator unavailable: " << err);
  }
  auto context = loopback_context(kBatch);
  // Declared before the Device, which may still deliver while the test unwinds (#253).
  std::mutex mutex;
  std::vector<std::uint32_t> seen;
  std::atomic<std::size_t> received{0};
  std::atomic<bool> hold{true};
  auto dev = open(*context, *sim);
  REQUIRE(dev
            ->on_debug_data([&](const DebugDataPacket & p) {
              const auto n = number_of(p.data);
              if (!n) {
                return;
              }
              if (*n == 0) {
                // Keep the receive thread busy while the rest queues up in the socket.
                while (hold) {
                  std::this_thread::sleep_for(1ms);
                }
              }
              const std::lock_guard lock(mutex);
              seen.push_back(*n);
              received = seen.size();
            })
            .has_value());

  auto sender = UdpSocket::open(Endpoint{{127, 0, 0, 1}, 0});
  REQUIRE(sender.has_value());
  const Endpoint to{{127, 0, 0, 1}, context->options().debug_data_port.value_or(0)};
  REQUIRE(to.port != 0);
  const auto before = context->stats();
  for (std::uint32_t i = 0; i < kCount; ++i) {
    const auto payload = numbered(i);
    REQUIRE(sender->send_to(payload, to).has_value());
  }
  std::this_thread::sleep_for(50ms);
  CHECK(received == 0);
  hold = false;

  REQUIRE(wait_until([&] { return received >= kCount; }));
  std::this_thread::sleep_for(100ms);  // anything delivered twice would show up now
  {
    const std::lock_guard lock(mutex);
    REQUIRE(seen.size() == kCount);
    bool in_order = true;
    for (std::uint32_t i = 0; i < kCount; ++i) {
      in_order = in_order && seen[i] == i;
    }
    CHECK(in_order);
  }
  const auto after = context->stats();
  CHECK(after.debug_datagrams - before.debug_datagrams == kCount);
  CHECK(after.unknown_source == before.unknown_source);

  // The other sockets were served in the meantime and still are.
  CHECK(dev->connected());
  CHECK(dev->identity().has_value());
  const auto pushes = dev->stats().pushes;
  CHECK(wait_until([&] { return dev->stats().pushes >= pushes + 2; }));
}
