// SPDX-License-Identifier: Apache-2.0
// DeviceType from the 0x0000 ACK dev_type (#58): the mapping, the discover() filter and the
// Device::open warning, against tools/livox_mid360_sim.py started with --dev-type.
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "livox/mid360/mid360.hpp"
#include "log_capture.hpp"
#include "sim_process.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{

constexpr std::uint8_t kOtherDevType = 200;

struct Fixture
{
  std::optional<SimProcess> sim;
  std::string err;

  explicit Fixture(std::vector<std::string> args = {})
  {
    args.insert(args.end(), {"--rate-multiplier", "0.05", "--push-rate", "10"});
    sim = SimProcess::start(err, std::move(args));
  }

  [[nodiscard]] DiscoveryOptions discovery_options() const
  {
    DiscoveryOptions o;
    o.targets = {Endpoint::loopback(sim->ports().discovery)};
    o.timeout = 2s;
    return o;
  }
};

std::unique_ptr<Context> loopback_context()
{
  ContextOptions o;
  o.bind_address = {127, 0, 0, 1};
  o.push_port = o.point_port = o.imu_port = o.log_port = 0;
  auto c = Context::create(o);
  REQUIRE(c.has_value());
  return std::move(*c);
}

DeviceOptions device_options(const SimProcess & sim)
{
  DeviceOptions o;
  o.session.host_command_port = 0;
  o.session.request = {.timeout = 500ms, .attempts = 3};
  o.lidar_log_port = sim.ports().log;
  return o;
}

}  // namespace

TEST_CASE("DeviceType: dev_type mapping keeps unknown values raw", "[device_type]")
{
  static_assert(to_device_type(kMid360DevType) == DeviceType::kMid360);
  static_assert(to_device_type(0) == DeviceType::kUnknown);
  static_assert(to_device_type(0xFF) == DeviceType::kUnknown);
  CHECK(kMid360DevType == 9);

  DiscoveredDevice d;
  d.dev_type = kOtherDevType;
  CHECK(d.device_type() == DeviceType::kUnknown);
  CHECK(d.dev_type == kOtherDevType);
  d.dev_type = kMid360DevType;
  CHECK(d.device_type() == DeviceType::kMid360);
}

TEST_CASE("discover: device_type filter", "[sim][session][device_type]")
{
  SECTION("a Mid-360 passes the kMid360 filter")
  {
    Fixture f;
    if (!f.sim) {
      SKIP("simulator unavailable: " << f.err);
    }
    auto o = f.discovery_options();
    o.device_type = DeviceType::kMid360;
    const auto found = discover(o);
    REQUIRE(found.has_value());
    REQUIRE(found->size() == 1);
    CHECK(found->front().dev_type == kMid360DevType);
    CHECK(found->front().device_type() == DeviceType::kMid360);
  }
  SECTION("another dev_type is kept raw and filtered out by kMid360")
  {
    Fixture f({"--dev-type", std::to_string(kOtherDevType)});
    if (!f.sim) {
      SKIP("simulator unavailable: " << f.err);
    }
    const auto all = discover(f.discovery_options());
    REQUIRE(all.has_value());
    REQUIRE(all->size() == 1);
    CHECK(all->front().dev_type == kOtherDevType);
    CHECK(all->front().device_type() == DeviceType::kUnknown);

    auto o = f.discovery_options();
    o.device_type = DeviceType::kUnknown;
    const auto unknown = discover(o);
    REQUIRE(unknown.has_value());
    CHECK(unknown->size() == 1);

    // The filtered-out answer still counts: the call returns before the timeout.
    o.device_type = DeviceType::kMid360;
    const auto start = std::chrono::steady_clock::now();
    const auto mid360 = discover(o);
    REQUIRE(mid360.has_value());
    CHECK(mid360->empty());
    CHECK(std::chrono::steady_clock::now() - start < 1500ms);
  }
}

TEST_CASE("Device::open warns about a device that is not a Mid-360", "[sim][device][device_type]")
{
  SECTION("unknown dev_type: warning, open succeeds")
  {
    Fixture f({"--dev-type", std::to_string(kOtherDevType)});
    if (!f.sim) {
      SKIP("simulator unavailable: " << f.err);
    }
    const auto found = discover(f.discovery_options());
    REQUIRE(found.has_value());
    REQUIRE(found->size() == 1);
    auto context = loopback_context();
    const ScopedLogCapture logs(LogLevel::kWarn);
    auto dev = Device::open(*context, found->front(), device_options(*f.sim));
    REQUIRE(dev.has_value());
    CHECK(logs.count(LogLevel::kWarn, "dev_type 200 is not a Mid-360") == 1);
  }
  SECTION("Mid-360: no warning")
  {
    Fixture f;
    if (!f.sim) {
      SKIP("simulator unavailable: " << f.err);
    }
    const auto found = discover(f.discovery_options());
    REQUIRE(found.has_value());
    REQUIRE(found->size() == 1);
    auto context = loopback_context();
    const ScopedLogCapture logs(LogLevel::kWarn);
    auto dev = Device::open(*context, found->front(), device_options(*f.sim));
    REQUIRE(dev.has_value());
    CHECK(logs.count(LogLevel::kWarn, "not a Mid-360") == 0);
  }
}
