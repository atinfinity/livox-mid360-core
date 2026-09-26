// SPDX-License-Identifier: Apache-2.0
// DeviceIdentity (issue #38): decode_identity() on hand-built key lists, to_string(), and
// Device::identity() against tools/livox_mid360_sim.py with the identity CLI options.
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "livox/mid360/mid360.hpp"
#include "sim_process.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{

std::vector<std::byte> bytes_of(std::string_view s, std::size_t width)
{
  std::vector<std::byte> out(width, std::byte{0});
  for (std::size_t i = 0; i < s.size() && i < width; ++i) {
    out[i] = static_cast<std::byte>(s[i]);
  }
  return out;
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

  [[nodiscard]] std::unique_ptr<Device> open() const
  {
    DeviceOptions o;
    o.session.host_command_port = 0;
    o.session.request = {.timeout = 500ms, .attempts = 3};
    auto d = Device::open(
      *context,
      DiscoveredDevice{
        .serial_number = sim->sn(),
        .ip = {127, 0, 0, 1},
        .cmd_port = sim->ports().cmd,
        .dev_type = 9,
        .from = Endpoint::loopback(sim->ports().cmd)},
      o);
    if (!d) {
      FAIL(to_string(d.error()));
    }
    return std::move(*d);
  }
};

}  // namespace

TEST_CASE("to_string(Version) is aa.bb.cc.dd", "[lidar_info]")
{
  CHECK(to_string(Version{.v = {13, 18, 0, 244}}) == "13.18.0.244");
  CHECK(to_string(Version{}) == "0.0.0.0");
}

TEST_CASE("decode_identity fills every field from a key list", "[lidar_info]")
{
  const auto sn = bytes_of("SN123", 16);
  const auto product = bytes_of("MID-360", 64);
  const std::array<std::byte, 4> app{std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}};
  const std::array<std::byte, 4> loader{std::byte{5}, std::byte{6}, std::byte{7}, std::byte{8}};
  const std::array<std::byte, 4> hw{std::byte{9}, std::byte{10}, std::byte{11}, std::byte{12}};
  const std::array<std::byte, 6> mac{std::byte{0xaa}, std::byte{0xbb}, std::byte{0xcc},
                                     std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
  const std::vector<KeyValue> kvs{
    {static_cast<std::uint16_t>(Key::kSn), sn},
    {static_cast<std::uint16_t>(Key::kProductInfo), product},
    {static_cast<std::uint16_t>(Key::kVersionApp), app},
    {static_cast<std::uint16_t>(Key::kVersionLoader), loader},
    {static_cast<std::uint16_t>(Key::kVersionHardware), hw},
    {static_cast<std::uint16_t>(Key::kMac), mac},
  };
  const auto id = decode_identity(kvs);
  CHECK(id.serial_number == "SN123");
  CHECK(id.product_info == "MID-360");
  CHECK(id.version_app.v == std::array<std::uint8_t, 4>{1, 2, 3, 4});
  CHECK(id.version_loader.v == std::array<std::uint8_t, 4>{5, 6, 7, 8});
  CHECK(id.version_hardware.v == std::array<std::uint8_t, 4>{9, 10, 11, 12});
  CHECK(id.mac == std::array<std::uint8_t, 6>{0xaa, 0xbb, 0xcc, 0x01, 0x02, 0x03});
  CHECK(
    to_string(id) ==
    "sn=SN123 product_info=MID-360 version_app=1.2.3.4 version_loader=5.6.7.8 "
    "version_hardware=9.10.11.12 mac=aa:bb:cc:01:02:03");
}

TEST_CASE("decode_identity tolerates missing and malformed keys", "[lidar_info]")
{
  const auto sn = bytes_of("ONLY", 16);
  const std::array<std::byte, 2> short_version{std::byte{1}, std::byte{2}};
  const std::vector<KeyValue> kvs{
    {static_cast<std::uint16_t>(Key::kSn), sn},
    {static_cast<std::uint16_t>(Key::kVersionApp), short_version},
  };
  const auto id = decode_identity(kvs);
  CHECK(id.serial_number == "ONLY");
  CHECK(id.product_info.empty());
  CHECK(id.version_app.v == std::array<std::uint8_t, 4>{});
  CHECK(id.mac == std::array<std::uint8_t, 6>{});
  CHECK(decode_identity({}).serial_number.empty());
}

TEST_CASE("kIdentityKeys lists 0x8000-0x8005 in order", "[lidar_info]")
{
  for (std::size_t i = 0; i < kIdentityKeys.size(); ++i) {
    CHECK(static_cast<std::uint16_t>(kIdentityKeys[i]) == 0x8000 + i);
  }
}

TEST_CASE("Device::identity reads the simulator defaults", "[lidar_info][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();
  auto id = dev->identity();
  REQUIRE(id.has_value());
  CHECK(id->serial_number == f.sim->sn());
  CHECK(id->product_info == "MID360-SIM");
  CHECK(to_string(id->version_app) == "0.0.0.1");
  CHECK(to_string(id->version_loader) == "0.0.0.1");
  CHECK(to_string(id->version_hardware) == "0.0.0.1");
  CHECK(id->mac == std::array<std::uint8_t, 6>{2, 0, 0, 0, 0, 1});
  CHECK(to_string(*id).starts_with("sn=" + f.sim->sn() + " product_info=MID360-SIM"));
}

TEST_CASE("Device::identity reflects the simulator identity options", "[lidar_info][sim]")
{
  Fixture f(
    {"--sn", "TESTSN01", "--product-info", "MID-360", "--version-app", "13.18.0.244",
     "--version-loader", "1.2.3.4", "--version-hardware", "5.6.7.8"});
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();
  auto id = dev->identity();
  REQUIRE(id.has_value());
  CHECK(id->serial_number == "TESTSN01");
  CHECK(id->product_info == "MID-360");
  CHECK(to_string(id->version_app) == "13.18.0.244");
  CHECK(to_string(id->version_loader) == "1.2.3.4");
  CHECK(to_string(id->version_hardware) == "5.6.7.8");
}

TEST_CASE("Device::identity fails when the LiDAR does not answer", "[lidar_info][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP(f.err);
  }
  auto dev = f.open();
  REQUIRE(f.sim->control(R"({"cmd":"silence","seconds":1.5})"));
  auto id = dev->identity(RequestOptions{.timeout = 50ms, .attempts = 1});
  REQUIRE_FALSE(id.has_value());
  CHECK(id.error().kind == DeviceError::Kind::kSession);
}
