// SPDX-License-Identifier: Apache-2.0
// The simulator's --pcap replay (issue #134): the committed capture tests/data/replay.pcap,
// written by tools/gen_replay_pcap.py, replayed into a Device, which must count exactly the
// frames, IMU samples and pushes it holds.
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "livox/mid360/mid360.hpp"
#include "sim_process.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{

bool wait_until(const std::function<bool()> & pred, std::chrono::milliseconds timeout = 10s)
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

// The capture's contents, as tools/gen_replay_pcap.py writes them.
constexpr std::uint64_t kPclFrames = 3;
constexpr std::uint64_t kPclPacketsPerFrame = 8;
constexpr std::uint64_t kImuPackets = 12;
constexpr std::uint64_t kPushes = 2;
constexpr std::int32_t kPushCoreTemp = 4321;

}  // namespace

TEST_CASE("pcap replay: the committed capture reaches a Device intact", "[replay][sim]")
{
  std::string err;
  const auto pcap = std::filesystem::path(LIVOX_MID360_TEST_DATA_DIR) / "replay.pcap";
  // --push-rate 0.01: the simulator's own next push is 100 s away, so the Device sees the
  // recorded pushes only.
  auto sim = SimProcess::start(err, {"--pcap", pcap.string(), "--push-rate", "0.01"});
  if (!sim) {
    SKIP("simulator unavailable: " << err);
  }
  ContextOptions co;
  co.bind_address = {127, 0, 0, 1};
  co.push_port = co.point_port = co.imu_port = co.log_port = 0;
  auto context = Context::create(co);
  REQUIRE(context.has_value());

  DeviceOptions o;
  o.session.host_command_port = 0;
  o.session.request = {.timeout = 500ms, .attempts = 3};
  o.reconnect.push_timeout = 60s;  // the capture pushes twice in 100 ms, then never again
  auto dev = Device::open(
    **context,
    DiscoveredDevice{
      .serial_number = sim->sn(),
      .ip = {127, 0, 0, 1},
      .cmd_port = sim->ports().cmd,
      .dev_type = 9,
      .from = Endpoint::loopback(sim->ports().cmd)},
    o);
  REQUIRE(dev.has_value());

  // The 0x0100 of Device::open configures the hosts, which starts the replay: 120 ms of
  // capture at the recorded spacing. No host is configured for the firmware log (key 0x0009),
  // so its one chunk is skipped.
  const auto done = sim->wait_event(R"("event":"replay_done")");
  REQUIRE(done.has_value());
  CHECK(json_int(*done, "pcl") == static_cast<std::int64_t>(kPclFrames * kPclPacketsPerFrame));
  CHECK(json_int(*done, "imu") == static_cast<std::int64_t>(kImuPackets));
  CHECK(json_int(*done, "push") == static_cast<std::int64_t>(kPushes));
  CHECK(json_int(*done, "log") == 0);
  CHECK(json_int(*done, "skipped") == 1);

  // The counters are published one by one, so wait for each of them.
  const std::uint64_t packets = kPclFrames * kPclPacketsPerFrame + kImuPackets;
  const std::uint64_t points = kPclFrames * kPclPacketsPerFrame * kPointsPerPacket;
  REQUIRE(wait_until([&] {
    const DeviceStats s = (*dev)->stats();
    return s.packets >= packets && s.points >= points && s.frames >= kPclFrames - 1 &&
           s.imu_samples >= kImuPackets && s.pushes >= kPushes;
  }));
  const DeviceStats s = (*dev)->stats();
  CHECK(s.packets == packets);
  // frame_cnt 0 and 1 are closed by the next frame's first packet; frame_cnt 2 stays open,
  // its points counted all the same.
  CHECK(s.frames == kPclFrames - 1);
  CHECK(s.points == points);
  CHECK(s.imu_samples == kImuPackets);
  CHECK(s.dropped_packets == 0);
  CHECK(s.reordered == 0);
  CHECK(s.bad_packets == 0);
  CHECK(s.pushes == kPushes);
  const auto status = (*dev)->pushed_status();
  REQUIRE(status.has_value());
  CHECK(status->core_temp == kPushCoreTemp);
  CHECK((*dev)->work_state() == WorkState::kSampling);
}
