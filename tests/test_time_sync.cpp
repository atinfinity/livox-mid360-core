// SPDX-License-Identifier: Apache-2.0
// The simulator's time model (issue #133) through a Device: a free-running clock counting from
// power-on, a step to the master time when PTP / GPS synchronisation is acquired, no step when
// it is lost, and drift. Each TimestampPolicy is checked across the event that sets it apart.
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
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

constexpr std::int64_t kSecond = 1'000'000'000;
constexpr std::int64_t kHour = 3600 * kSecond;
// Frames are compared with the host clock read in on_frame, which runs up to a frame (100 ms)
// plus the delivery latency after the frame's first point.
constexpr std::int64_t kSlack = 2 * kSecond;

std::int64_t host_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}

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

/// A delivered Frame's time type and base time, with the host time it arrived at.
struct Seen
{
  TimeType type = TimeType::kNoSync;
  std::int64_t base_ns = 0;
  std::int64_t host_ns = 0;
};

struct Rig
{
  std::optional<SimProcess> sim;
  std::string err;
  std::unique_ptr<Context> context;
  std::unique_ptr<Device> dev;
  std::mutex mutex;
  std::vector<Seen> frames;

  bool start()
  {
    // 100 point-cloud packets/s: low enough for a Debug+ASan Device on two CPUs to keep up, so
    // the host time read in on_frame is not skewed by a growing receive backlog.
    sim = SimProcess::start(err, {"--rate-multiplier", "0.05", "--push-rate", "10"});
    if (!sim) {
      return false;
    }
    ContextOptions co;
    co.bind_address = {127, 0, 0, 1};
    co.push_port = co.point_port = co.imu_port = co.log_port = 0;
    auto c = Context::create(co);
    REQUIRE(c.has_value());
    context = std::move(*c);
    return true;
  }

  void open(TimestampPolicy policy)
  {
    DeviceOptions o;
    o.session.host_command_port = 0;
    o.session.request = {.timeout = 500ms, .attempts = 3};
    o.timestamp_policy = policy;
    auto d = Device::open(
      *context,
      DiscoveredDevice{
        .serial_number = sim->sn(),
        .ip = {127, 0, 0, 1},
        .cmd_port = sim->ports().cmd,
        .dev_type = 9,
        .from = Endpoint::loopback(sim->ports().cmd)},
      o);
    REQUIRE(d.has_value());
    dev = std::move(*d);
    REQUIRE(dev
              ->on_frame([this](const Frame & f) {
                const std::lock_guard lock(mutex);
                frames.push_back(
                  {f.time_type, static_cast<std::int64_t>(f.base_time_ns), host_now_ns()});
              })
              .has_value());
  }

  /// The simulator's `time_sync` event after sending `json`.
  std::optional<std::string> time_sync(const std::string & json)
  {
    if (!sim->control(json)) {
      return std::nullopt;
    }
    return sim->wait_event(R"("event":"time_sync")");
  }

  std::size_t count()
  {
    const std::lock_guard lock(mutex);
    return frames.size();
  }

  std::vector<Seen> snapshot()
  {
    const std::lock_guard lock(mutex);
    return frames;
  }

  /// The first frame from index `from` on that satisfies `pred`, waiting for it.
  std::optional<Seen> wait_frame(std::size_t from, const std::function<bool(const Seen &)> & pred)
  {
    std::optional<Seen> found;
    wait_until([&] {
      const std::lock_guard lock(mutex);
      for (std::size_t i = from; i < frames.size(); ++i) {
        if (pred(frames[i])) {
          found = frames[i];
          return true;
        }
      }
      return false;
    });
    return found;
  }
};

bool near(std::int64_t a, std::int64_t b, std::int64_t slack = kSlack)
{
  return std::llabs(a - b) < slack;
}

}  // namespace

TEST_CASE(
  "Simulator time: power-on clock, a step to PTP time, a loss without a step", "[time][sim]")
{
  Rig r;
  if (!r.start()) {
    SKIP("simulator unavailable: " << r.err);
  }
  r.open(TimestampPolicy::kLidar);

  // Unsynchronised: the packets carry the time since power-on.
  const auto free = r.wait_frame(1, [](const Seen &) { return true; });
  REQUIRE(free.has_value());
  CHECK(free->type == TimeType::kNoSync);
  CHECK(free->base_ns < 60 * kSecond);
  const auto before = r.dev->time_sync_status();
  REQUIRE(before.has_value());
  CHECK(before->type == TimeSyncType::kNone);
  CHECK(before->last_sync_time_ns == 0);
  CHECK(static_cast<std::int64_t>(before->local_time_ns) < 60 * kSecond);

  // PTP with the master on the host's clock: kLidar follows the step at once.
  const auto ev = r.time_sync(R"({"cmd":"time_sync","type":"ptp","offset_ns":0})");
  REQUIRE(ev.has_value());
  const std::size_t synced_from = r.count();
  const auto ptp =
    r.wait_frame(synced_from, [](const Seen & s) { return s.type == TimeType::kPtp; });
  REQUIRE(ptp.has_value());
  CHECK(near(ptp->base_ns, ptp->host_ns));

  // Frame::time_type changes once, between two frames.
  const auto seen = r.snapshot();
  std::size_t first_ptp = seen.size();
  for (std::size_t i = 0; i < seen.size(); ++i) {
    if (seen[i].type == TimeType::kPtp) {
      first_ptp = std::min(first_ptp, i);
    } else {
      CHECK(i < first_ptp);  // no kNoSync frame after the first kPtp one
      CHECK(seen[i].base_ns < 60 * kSecond);
    }
  }
  CHECK(first_ptp >= 1);

  // time_sync_status() and the push agree with the packets and with the simulator.
  const auto st = r.dev->time_sync_status();
  REQUIRE(st.has_value());
  CHECK(st->type == TimeSyncType::kPtp);
  CHECK(static_cast<std::int64_t>(st->last_sync_time_ns) == json_int(*ev, "last_sync_ns"));
  CHECK(st->offset_ns == json_int(*ev, "offset_ns"));
  CHECK(st->offset_ns < -kHour);  // local (time since power-on) - source (the epoch)
  CHECK(near(static_cast<std::int64_t>(st->local_time_ns), host_now_ns()));
  REQUIRE(wait_until([&] {
    const auto p = r.dev->pushed_status();
    return p && p->time_sync_type == TimeSyncType::kPtp;
  }));
  const auto pushed = r.dev->pushed_status();
  CHECK(pushed->last_sync_time == st->last_sync_time_ns);
  CHECK(pushed->time_offset == st->offset_ns);

  // Losing the sync falls back to free running from the master time: no step back.
  REQUIRE(r.time_sync(R"({"cmd":"time_sync","type":"none"})").has_value());
  const auto lost =
    r.wait_frame(r.count(), [](const Seen & s) { return s.type == TimeType::kNoSync; });
  REQUIRE(lost.has_value());
  CHECK(lost->base_ns >= ptp->base_ns);
  CHECK(near(lost->base_ns, lost->host_ns));
  const auto after = r.dev->time_sync_status();
  REQUIRE(after.has_value());
  CHECK(after->type == TimeSyncType::kNone);
  CHECK(after->last_sync_time_ns == st->last_sync_time_ns);  // the last sync is remembered
  CHECK(after->offset_ns == st->offset_ns);
}

TEST_CASE(
  "kHostOffsetOnce passes synchronised time through and re-measures after a loss", "[time][sim]")
{
  Rig r;
  if (!r.start()) {
    SKIP("simulator unavailable: " << r.err);
  }
  r.open(TimestampPolicy::kHostOffsetOnce);

  // Free running: the offset measured at the first packet maps the LiDAR clock to the host's.
  const auto free = r.wait_frame(1, [](const Seen &) { return true; });
  REQUIRE(free.has_value());
  CHECK(free->type == TimeType::kNoSync);
  CHECK(near(free->base_ns, free->host_ns));
  const DeviceStats s0 = r.dev->stats();
  REQUIRE(s0.time_offset_valid);
  CHECK(s0.time_offset_ns > host_now_ns() - 60 * kSecond);  // host - time since power-on

  // A master an hour ahead of the host: synchronised packets are taken as they are.
  REQUIRE(r.time_sync(R"({"cmd":"time_sync","type":"ptp","offset_ns":3600000000000})").has_value());
  const auto ptp = r.wait_frame(r.count(), [](const Seen & s) { return s.type == TimeType::kPtp; });
  REQUIRE(ptp.has_value());
  CHECK(near(ptp->base_ns, ptp->host_ns + kHour));
  CHECK(r.dev->stats().time_offset_ns == s0.time_offset_ns);  // not measured while synced

  // The clock keeps the master's time when the sync is lost; the offset is measured again,
  // so the frames are back on the host's clock.
  REQUIRE(r.time_sync(R"({"cmd":"time_sync","type":"none"})").has_value());
  const auto lost =
    r.wait_frame(r.count(), [](const Seen & s) { return s.type == TimeType::kNoSync; });
  REQUIRE(lost.has_value());
  CHECK(near(lost->base_ns, lost->host_ns));
  const DeviceStats s1 = r.dev->stats();
  CHECK(s1.time_offset_valid);
  CHECK(near(s1.time_offset_ns, -kHour));
}

TEST_CASE("kHostOffsetOnce keeps its first offset while the LiDAR clock drifts", "[time][sim]")
{
  Rig r;
  if (!r.start()) {
    SKIP("simulator unavailable: " << r.err);
  }
  // 10 %: 200 ms over the 2 s watched, far above the delivery jitter.
  REQUIRE(r.time_sync(R"({"cmd":"time_sync","drift_ppm":100000})").has_value());
  r.open(TimestampPolicy::kHostOffsetOnce);

  const auto first = r.wait_frame(1, [](const Seen &) { return true; });
  REQUIRE(first.has_value());
  const std::int64_t offset = r.dev->stats().time_offset_ns;
  const auto last = r.wait_frame(
    r.count(), [&](const Seen & s) { return s.host_ns >= first->host_ns + 2 * kSecond; });
  REQUIRE(last.has_value());
  CHECK(last->type == TimeType::kNoSync);
  CHECK(r.dev->stats().time_offset_ns == offset);

  // The frames run ahead of the host clock by the drift accumulated since the offset.
  const std::int64_t host_elapsed = last->host_ns - first->host_ns;
  const std::int64_t gain = (last->base_ns - first->base_ns) - host_elapsed;
  CHECK(gain > host_elapsed / 20);      // > 5 %
  CHECK(gain < host_elapsed * 3 / 20);  // < 15 %
}
