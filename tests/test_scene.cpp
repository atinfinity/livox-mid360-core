// SPDX-License-Identifier: Apache-2.0
// The simulator's deterministic ring scene (issue #132): decoded point values in every data
// type, per-point time offsets, FOV cropping at a window's edges, the host-side extrinsic and
// lvx2 record -> read back, all checked against the scene's known geometry; and the
// simulator's --apply-attitude (issue #135) against the same scene moved by key 0x0012.
#include <algorithm>
#include <array>
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <set>
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

// ---------------------------------------------------------------------------
// The ring scene, restated from tools/livox_mid360_sim.py (ring_point) so that the simulator
// and the SDK are checked against an independent definition.
// ---------------------------------------------------------------------------
constexpr unsigned kRingPoints = 256;
constexpr std::array<int, 4> kRingPitchDeg{0, 15, -5, 45};

struct RingPoint
{
  double x, y, z;  ///< metres
  std::uint8_t tag;
};

/// Point k: azimuth k * 360 / 256 deg in whole 0.01 deg, elevation kRingPitchDeg[k % 4],
/// depth 1 m + 30 mm * k; reflectivity k; tag glue / particles / other = k, k / 3, k / 9 mod 3.
RingPoint ring_point(unsigned k)
{
  constexpr double kDegToRad = std::numbers::pi / 180.0;
  const double depth = (1000.0 + 30.0 * k) / 1000.0;
  const double pitch = kRingPitchDeg.at(k % 4) * kDegToRad;
  const unsigned phi = k * 36000 / kRingPoints;  // 0.01 deg, truncated like the simulator
  const double yaw = phi / 100.0 * kDegToRad;
  return RingPoint{
    .x = depth * std::cos(pitch) * std::cos(yaw),
    .y = depth * std::cos(pitch) * std::sin(yaw),
    .z = depth * std::sin(pitch),
    .tag = static_cast<std::uint8_t>(k % 3 | (k / 3 % 3) << 2 | (k / 9 % 3) << 4)};
}

/// `p` moved by Rz(yaw) * Ry(pitch) * Rx(roll), then the translation: the convention of
/// extrinsic_from(), restated here from the three rotations.
RingPoint moved(const InstallAttitude & a, RingPoint p)
{
  constexpr double kDegToRad = std::numbers::pi / 180.0;
  const double roll = static_cast<double>(a.roll_deg) * kDegToRad;
  const double pitch = static_cast<double>(a.pitch_deg) * kDegToRad;
  const double yaw = static_cast<double>(a.yaw_deg) * kDegToRad;
  const double y1 = std::cos(roll) * p.y - std::sin(roll) * p.z;  // Rx(roll)
  const double z1 = std::sin(roll) * p.y + std::cos(roll) * p.z;
  const double x2 = std::cos(pitch) * p.x + std::sin(pitch) * z1;  // Ry(pitch)
  const double z2 = -std::sin(pitch) * p.x + std::cos(pitch) * z1;
  const double x3 = std::cos(yaw) * x2 - std::sin(yaw) * y1;  // Rz(yaw)
  const double y3 = std::sin(yaw) * x2 + std::cos(yaw) * y1;
  return RingPoint{
    .x = x3 + a.x_mm / 1000.0, .y = y3 + a.y_mm / 1000.0, .z = z2 + a.z_mm / 1000.0, .tag = p.tag};
}

/// Quantisation of each data type in metres, plus float rounding.
double tolerance(DataType t)
{
  switch (t) {
    case DataType::kCartesian16:
      return 0.0051;  // cm
    case DataType::kCartesian32:
      return 0.0006;  // mm
    default:
      return 0.0002;  // spherical: exact angles, mm depth, float trigonometry
  }
}

/// Largest deviation of `points` from `expect(k)` for the ring point k their reflectivity
/// names; counts a wrong tag as a mismatch.
struct Deviation
{
  double max_m = 0;
  std::size_t tag_mismatches = 0;
};

Deviation deviation(
  const std::vector<Point> & points, const std::function<RingPoint(unsigned)> & expect = ring_point)
{
  Deviation d;
  for (const Point & p : points) {
    const RingPoint e = expect(p.reflectivity);
    d.max_m = std::max(
      {d.max_m, std::abs(static_cast<double>(p.x) - e.x), std::abs(static_cast<double>(p.y) - e.y),
       std::abs(static_cast<double>(p.z) - e.z)});
    if (p.tag != e.tag) {
      ++d.tag_mismatches;
    }
  }
  return d;
}

const InstallAttitude kAttitude{
  .roll_deg = 10, .pitch_deg = -20, .yaw_deg = 30, .x_mm = 100, .y_mm = -200, .z_mm = 300};
/// Cartesian32 rounding after a rotation: sqrt(3) * 0.5 mm on any axis, plus float rounding.
constexpr double kRotatedCartesian32 = 0.0009;

struct Fixture
{
  std::optional<SimProcess> sim;
  std::string err;
  std::unique_ptr<Context> context;

  explicit Fixture(std::vector<std::string> extra = {})
  {
    // 100 pkt/s (10 packets per frame): under Debug+ASan on 2 CPUs the receiver cannot keep
    // up with 500 pkt/s, and the 4 MB socket buffer then delays every frame by seconds, past
    // a format switch the tests wait for.
    extra.insert(
      extra.end(), {"--scene", "ring", "--rate-multiplier", "0.05", "--push-rate", "10"});
    sim = SimProcess::start(err, std::move(extra));
    if (sim) {
      ContextOptions o;
      o.bind_address = {127, 0, 0, 1};
      o.push_port = o.point_port = o.imu_port = o.log_port = 0;
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

/// Whole frames (points kept) while `keep` is set, and the last point-cloud time_interval.
struct Recorder
{
  std::atomic<bool> keep{false};
  std::atomic<std::uint64_t> frames{0};
  std::atomic<std::uint16_t> time_interval{0};
  std::mutex mutex;
  std::vector<Frame> kept;

  void attach(Device & d)
  {
    REQUIRE(d.on_frame([this](Frame && f) {
               ++frames;
               if (keep) {
                 const std::lock_guard lock(mutex);
                 kept.push_back(std::move(f));
               }
             })
              .has_value());
    REQUIRE(d.on_packet([this](const DataPacketView & p, const ReceiveInfo &) {
               if (p.header.data_type != DataType::kImu) {
                 time_interval = p.header.time_interval;
               }
             })
              .has_value());
  }

  /// Keeps frames until `n` of type `t` have arrived after the first one of that type, which
  /// may start mid-frame after a format switch, and returns those `n`.
  std::vector<Frame> collect(DataType t, std::size_t n)
  {
    const auto count = [&] {
      const std::lock_guard lock(mutex);
      return std::ranges::count(kept, t, &Frame::source_type);
    };
    keep = true;
    const bool enough =
      wait_until([&] { return count() >= static_cast<std::ptrdiff_t>(n + 1); }, 10s);
    keep = false;
    REQUIRE(enough);
    const std::lock_guard lock(mutex);
    std::vector<Frame> out;
    for (Frame & f : kept) {
      if (f.source_type == t) {
        out.push_back(std::move(f));
      }
    }
    kept.clear();
    out.erase(out.begin());
    out.resize(n);
    return out;
  }
};

}  // namespace

TEST_CASE("ring scene: every data type decodes to the same points", "[scene][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;  // outlives the Device: callbacks may run until the destructor returns
  auto dev = f.open();
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());

  std::size_t starts_at_zero = 0;
  for (const DataType t : {DataType::kCartesian32, DataType::kCartesian16, DataType::kSpherical}) {
    CAPTURE(to_string(t));
    REQUIRE(dev->set_point_format(t).has_value());
    const std::vector<Frame> frames = rec.collect(t, 2);
    const std::uint64_t span_ns = std::uint64_t{rec.time_interval.load()} * 100u;
    REQUIRE(span_ns > 0);
    for (const Frame & fr : frames) {
      // No FOV window: every packet carries 96 consecutive ring points.
      REQUIRE(fr.points.size() == fr.packets * kPointsPerPacket);
      const Deviation d = deviation(fr.points);
      CHECK(d.max_m <= tolerance(t));
      CHECK(d.tag_mismatches == 0);
      // A frame starts at ring index 0 unless its first packet was lost on the way.
      if (fr.points.front().reflectivity == 0) {
        ++starts_at_zero;
      }
      std::size_t order_errors = 0;
      std::size_t offset_errors = 0;
      for (std::size_t base = 0; base < fr.points.size(); base += kPointsPerPacket) {
        const Point & first = fr.points[base];
        if (first.reflectivity % 32 != 0) {  // packets start at 96 * n mod 256
          ++order_errors;
        }
        for (std::size_t i = 0; i < kPointsPerPacket; ++i) {
          const Point & p = fr.points[base + i];
          if (
            p.reflectivity != static_cast<std::uint8_t>(first.reflectivity + i) ||
            p.line != i % 4) {
            ++order_errors;
          }
          // Sample i of a packet is time_interval * i / (dot_num - 1) after its first.
          if (p.offset_ns - first.offset_ns != span_ns * i / (kPointsPerPacket - 1)) {
            ++offset_errors;
          }
        }
      }
      CHECK(order_errors == 0);
      CHECK(offset_errors == 0);
    }
  }
  CHECK(starts_at_zero >= 1);
}

TEST_CASE("ring scene: FOV cropping keeps exactly the points inside the window", "[scene][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;
  auto dev = f.open();
  rec.attach(*dev);
  // Yaw [0, 90) and pitch [0, 15]: azimuth 0 is kept and 90 is not, pitch 15 is kept and
  // -5 is not. The simulator crops on the exact angles, whatever the data type [unverified
  // firmware behaviour, #11].
  REQUIRE(dev
            ->set_fov(FovSettings{
              .fov0 =
                FovConfig{
                  .yaw_start_deg = 0,
                  .yaw_stop_deg = 90,
                  .pitch_start_deg = 0,
                  .pitch_stop_deg = 15,
                  .rsvd = 0},
              .fov1 = std::nullopt,
              .enable = FovEnable{.fov0 = true, .fov1 = false}})
            .has_value());
  REQUIRE(dev->start_sampling().has_value());
  std::set<unsigned> want;
  for (unsigned k = 0; k < 64; ++k) {
    if (k % 4 == 0 || k % 4 == 1) {  // pitch 0 or 15
      want.insert(k);
    }
  }
  // Spherical first: a slow receiver may still hold Cartesian32 frames sent before set_fov().
  for (const DataType t : {DataType::kSpherical, DataType::kCartesian32}) {
    CAPTURE(to_string(t));
    REQUIRE(dev->set_point_format(t).has_value());
    std::set<unsigned> seen;
    for (const Frame & fr : rec.collect(t, 2)) {
      const Deviation d = deviation(fr.points);
      CHECK(d.max_m <= tolerance(t));
      CHECK(d.tag_mismatches == 0);
      for (const Point & p : fr.points) {
        seen.insert(p.reflectivity);
      }
    }
    CHECK(seen == want);
  }
}

TEST_CASE("ring scene: the host-side extrinsic moves the decoded points", "[scene][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;
  auto dev = f.open();
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  Frame fr = std::move(rec.collect(DataType::kCartesian32, 1).front());
  REQUIRE_FALSE(fr.points.empty());

  const InstallAttitude a = kAttitude;
  apply(extrinsic_from(a), fr);
  const Deviation d = deviation(fr.points, [&](unsigned k) { return moved(a, ring_point(k)); });
  // The rotation mixes the per-axis 0.5 mm rounding: at most sqrt(3) * 0.5 mm on any axis.
  CHECK(d.max_m <= kRotatedCartesian32);
  CHECK(d.tag_mismatches == 0);
}

TEST_CASE(
  "ring scene: spherical packets recorded to lvx2 read back as the same points", "[scene][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  // Named after the simulator's port so that parallel runs do not share the file.
  const auto path = std::filesystem::temp_directory_path() /
                    ("mid360_scene_" + std::to_string(f.sim->ports().cmd) + ".lvx2");
  Lvx2DeviceInfo info;
  info.lidar_sn = f.sim->sn();
  info.lidar_id = 0x0100007F;
  std::mutex mutex;
  Lvx2Writer writer;
  REQUIRE(writer.open(path, std::span(&info, 1)).has_value());
  std::atomic<bool> recording{false};
  std::atomic<std::uint64_t> written{0};
  std::atomic<bool> write_failed{false};

  auto dev = f.open();
  REQUIRE(dev
            ->on_packet([&](const DataPacketView & p, const ReceiveInfo &) {
              if (!recording || p.header.data_type != DataType::kSpherical) {
                return;
              }
              const std::lock_guard lock(mutex);
              const auto r = writer.write(0, p);
              if (!r || !*r) {
                write_failed = true;
                return;
              }
              ++written;
            })
            .has_value());
  REQUIRE(dev->set_point_format(DataType::kSpherical).has_value());
  REQUIRE(dev->start_sampling().has_value());
  recording = true;
  REQUIRE(wait_until([&] { return written >= 40; }));
  recording = false;
  dev.reset();  // no callback runs after this
  CHECK_FALSE(write_failed);
  REQUIRE(writer.close().has_value());

  Lvx2Reader reader;
  REQUIRE(reader.open(path).has_value());
  std::vector<Point> points;
  std::uint64_t packages = 0;
  while (true) {
    auto pk = reader.next_packet();
    REQUIRE(pk.has_value());
    if (!*pk) {
      break;
    }
    ++packages;
    REQUIRE((*pk)->data_type == DataType::kCartesian32);
    const DataPacketView view = (*pk)->to_data_packet_view();
    REQUIRE(view.header.dot_num == kPointsPerPacket);
    for (std::size_t i = 0; i < view.header.dot_num; ++i) {
      const CartesianPoint32 c = decode_cartesian32(view, i);
      points.push_back(Point{
        .x = static_cast<float>(c.x_mm) * 0.001F,
        .y = static_cast<float>(c.y_mm) * 0.001F,
        .z = static_cast<float>(c.z_mm) * 0.001F,
        .reflectivity = c.reflectivity,
        .tag = c.tag});
    }
  }
  std::filesystem::remove(path);
  CHECK(packages == written);
  const Deviation d = deviation(points);
  CHECK(d.max_m <= tolerance(DataType::kCartesian32));
  CHECK(d.tag_mismatches == 0);
}

TEST_CASE("ring scene: --apply-attitude moves Cartesian points by key 0x0012", "[scene][sim]")
{
  Fixture f({"--apply-attitude"});
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;
  auto dev = f.open();
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  // The attitude is written before each format switch below, so every frame collected in the
  // new format left the simulator after it.
  REQUIRE(dev->set_install_attitude(kAttitude).has_value());
  const auto once = [](unsigned k) { return moved(kAttitude, ring_point(k)); };
  const auto twice = [](unsigned k) { return moved(kAttitude, moved(kAttitude, ring_point(k))); };

  SECTION("Cartesian points leave the device already moved; spherical ones do not")
  {
    for (const DataType t : {DataType::kCartesian16, DataType::kSpherical}) {
      CAPTURE(to_string(t));
      REQUIRE(dev->set_point_format(t).has_value());
      for (const Frame & fr : rec.collect(t, 2)) {
        const Deviation d =
          t == DataType::kSpherical ? deviation(fr.points) : deviation(fr.points, once);
        CHECK(d.max_m <= tolerance(t));
        CHECK(d.tag_mismatches == 0);
      }
    }
  }

  SECTION("the host-side extrinsic on top transforms the cloud twice")
  {
    // Through Cartesian16 so that no Cartesian32 frame from before the attitude is collected.
    REQUIRE(dev->set_point_format(DataType::kCartesian16).has_value());
    rec.collect(DataType::kCartesian16, 1);
    REQUIRE(dev->set_point_format(DataType::kCartesian32).has_value());
    for (Frame & fr : rec.collect(DataType::kCartesian32, 2)) {
      const Deviation device_only = deviation(fr.points, once);
      CHECK(device_only.max_m <= tolerance(DataType::kCartesian32));
      apply(extrinsic_from(kAttitude), fr);
      // The pitfall of docs/api.md: the points are now off by a whole attitude.
      CHECK(deviation(fr.points, once).max_m > 0.1);
      const Deviation d = deviation(fr.points, twice);
      CHECK(d.max_m <= kRotatedCartesian32);
      CHECK(d.tag_mismatches == 0);
    }
  }
}

TEST_CASE("ring scene: without --apply-attitude key 0x0012 leaves the points alone", "[scene][sim]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Recorder rec;
  auto dev = f.open();
  rec.attach(*dev);
  REQUIRE(dev->start_sampling().has_value());
  REQUIRE(dev->set_install_attitude(kAttitude).has_value());
  REQUIRE(dev->set_point_format(DataType::kCartesian16).has_value());
  for (const Frame & fr : rec.collect(DataType::kCartesian16, 2)) {
    const Deviation d = deviation(fr.points);
    CHECK(d.max_m <= tolerance(DataType::kCartesian16));
    CHECK(d.tag_mismatches == 0);
  }
}
