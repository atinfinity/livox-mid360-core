// SPDX-License-Identifier: Apache-2.0
// FrameAssembler (issue #6): frame splitting, drop counting, timestamp policies, conversion.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <numbers>
#include <vector>

#include "frame_assembler.hpp"
#include "generated/golden_vectors.hpp"
#include "livox/mid360/bytes.hpp"
#include "test_util.hpp"

using namespace livox::mid360;
using namespace livox::mid360::detail;
using namespace std::chrono_literals;
using Catch::Approx;

namespace
{

constexpr std::uint64_t kMs = 1'000'000;

/// Synthetic packet: header fields + zero-filled samples (owning).
struct Packet
{
  std::vector<std::byte> data;
  DataPacketView view;
};

Packet make_packet(
  std::uint16_t udp_cnt, std::uint8_t frame_cnt, std::uint64_t ts_ns, std::uint16_t dots = 4,
  DataType type = DataType::kCartesian32, std::uint16_t interval_0p1us = 3000,
  TimeType time_type = TimeType::kNoSync)
{
  Packet p;
  p.data.assign(dots * sample_size(type), std::byte{0});
  p.view.header.udp_cnt = udp_cnt;
  p.view.header.frame_cnt = frame_cnt;
  p.view.header.timestamp_ns = ts_ns;
  p.view.header.dot_num = dots;
  p.view.header.data_type = type;
  p.view.header.time_interval = interval_0p1us;
  p.view.header.time_type = time_type;
  p.view.data = p.data;
  return p;
}

FramePolicy counter_policy(std::chrono::nanoseconds window = 100ms)
{
  return {.mode = FramePolicy::Mode::kFrameCounter, .window = window};
}
FramePolicy window_policy(std::chrono::nanoseconds window = 100ms)
{
  return {.mode = FramePolicy::Mode::kTimeWindow, .window = window};
}

}  // namespace

TEST_CASE("frame assembler: golden sequence splits by frame_cnt and counts drops", "[frame]")
{
  FrameAssembler fa(counter_policy(), TimestampPolicy::kLidar);
  std::vector<Frame> frames;
  for (std::size_t i = 0; i < golden::frame_seq_count; ++i) {
    const auto bytes = span_of(
      golden::frame_seq + golden::frame_seq_offsets[i],
      golden::frame_seq_offsets[i + 1] - golden::frame_seq_offsets[i]);
    const auto pkt = parse_data_packet(bytes);
    REQUIRE(pkt.has_value());
    if (auto f = fa.push(*pkt, 0)) {
      frames.push_back(std::move(*f));
    }
  }
  REQUIRE(frames.size() == golden::frame_seq_frames - 1);
  CHECK(fa.has_partial());
  auto last = fa.flush();
  REQUIRE(last.has_value());
  frames.push_back(std::move(*last));
  CHECK_FALSE(fa.flush().has_value());

  REQUIRE(frames.size() == golden::frame_seq_frames);
  for (std::size_t i = 0; i < frames.size(); ++i) {
    CHECK(frames[i].index == i);
    CHECK(frames[i].points.size() == golden::frame_seq_points[i]);
    CHECK(frames[i].frame_cnt == i);
    CHECK(frames[i].source_type == DataType::kCartesian32);
    CHECK(frames[i].end_time_ns >= frames[i].base_time_ns);
    for (std::size_t j = 1; j < frames[i].points.size(); ++j) {
      CHECK(frames[i].points[j].offset_ns >= frames[i].points[j - 1].offset_ns);
      CHECK(frames[i].points[j].line == j % 4);
    }
  }
  CHECK(frames[0].base_time_ns == golden::frame_seq_t0_ns);
  CHECK(frames[1].base_time_ns == golden::frame_seq_t0_ns + 3ull * golden::frame_seq_step_ns);
  CHECK(frames[0].packets == 3);
  CHECK(frames[1].packets == 2);
  CHECK(frames[1].dropped_packets == golden::frame_seq_dropped);
  CHECK(frames[2].dropped_packets == 0);  // udp_cnt reset at frame start is not a drop
  // point 0 of packet 0: x = 0 mm, y = 0, z = 0; point 1 of packet 1 (frame 0): x = 1001 mm
  CHECK(frames[0].points[9].x == Approx(1.001F));
  CHECK(frames[0].points[9].y == Approx(-0.001F));
  CHECK(frames[0].points[9].z == Approx(0.010F));
  CHECK(frames[0].points[9].reflectivity == 7);
  CHECK(frames[0].points[9].tag == 1);
  // last point of packet 0 is time_interval later
  CHECK(frames[0].points[7].offset_ns == golden::frame_seq_step_ns);

  const auto & c = fa.counters();
  CHECK(c.packets == golden::frame_seq_count);
  CHECK(c.frames == golden::frame_seq_frames);
  CHECK(c.dropped_packets == golden::frame_seq_dropped);
  CHECK(c.reordered == 0);
  CHECK(c.frame_cnt_fallback == 0);
  CHECK(c.points == golden::frame_seq_count * 8);
}

TEST_CASE("frame assembler: time window closes on the first point past base + window", "[frame]")
{
  FrameAssembler fa(window_policy(100ms), TimestampPolicy::kLidar);
  std::uint16_t cnt = 0;
  std::vector<std::size_t> sizes;
  for (std::uint64_t t = 0; t < 250 * kMs; t += 10 * kMs) {
    auto p = make_packet(cnt++, 0, 1000 + t);
    if (auto f = fa.push(p.view, 0)) {
      sizes.push_back(f->points.size());
    }
  }
  // t = 0..90 -> frame, t = 100..190 -> frame, t = 200..240 partial
  CHECK(sizes == std::vector<std::size_t>{40, 40});
  auto rest = fa.flush();
  REQUIRE(rest.has_value());
  CHECK(rest->points.size() == 20);
  CHECK(rest->base_time_ns == 1000 + 200 * kMs);
  CHECK(fa.time_window_active());
}

TEST_CASE("frame assembler: constant frame_cnt falls back to the time window", "[frame]")
{
  FrameAssembler fa(counter_policy(100ms), TimestampPolicy::kLidar);
  std::uint16_t cnt = 0;
  std::size_t closed = 0;
  for (std::uint64_t t = 0; t <= 400 * kMs; t += 10 * kMs) {
    auto p = make_packet(cnt++, 0, t);
    if (fa.push(p.view, 0)) {
      ++closed;
    }
  }
  CHECK(fa.counters().frame_cnt_fallback == 1);
  CHECK(fa.time_window_active());
  // fallback at t = 200 ms closes [0, 200), then windows [200, 300), [300, 400)
  CHECK(closed == 3);
}

TEST_CASE("frame assembler: frame_cnt that changes never falls back", "[frame]")
{
  FrameAssembler fa(counter_policy(100ms), TimestampPolicy::kLidar);
  std::uint16_t cnt = 0;
  std::size_t closed = 0;
  for (std::uint64_t t = 0; t <= 400 * kMs; t += 10 * kMs) {
    auto p = make_packet(cnt++, static_cast<std::uint8_t>(t / (150 * kMs)), t);
    if (fa.push(p.view, 0)) {
      ++closed;
    }
  }
  CHECK(fa.counters().frame_cnt_fallback == 0);
  CHECK_FALSE(fa.time_window_active());
  CHECK(closed == 2);  // frame_cnt 0 -> 1 at 150 ms, 1 -> 2 at 300 ms
}

TEST_CASE("frame assembler: reordered packets are not drops", "[frame]")
{
  FrameAssembler fa(counter_policy(), TimestampPolicy::kLidar);
  for (std::uint16_t c :
       {std::uint16_t{10}, std::uint16_t{12}, std::uint16_t{11}, std::uint16_t{13}}) {
    auto p = make_packet(c, 0, 1000 + c);
    (void)fa.push(p.view, 0);
  }
  CHECK(fa.counters().dropped_packets == 1);  // 10 -> 12
  CHECK(fa.counters().reordered == 1);        // 11 after 12
  CHECK(fa.counters().packets == 4);
  auto f = fa.flush();
  REQUIRE(f.has_value());
  CHECK(f->points.size() == 16);
  CHECK(f->dropped_packets == 1);
}

TEST_CASE("frame assembler: udp_cnt wrap-around is in sequence", "[frame]")
{
  DropCounter d;
  CHECK(d.observe(0xFFFE, false).dropped == 0);
  CHECK(d.observe(0xFFFF, false).dropped == 0);
  CHECK(d.observe(0x0000, false).dropped == 0);
  CHECK(d.observe(0x0002, false).dropped == 1);
  CHECK(d.observe(0x0000, true).dropped == 0);  // reset at frame start
  CHECK(d.observe(0x0000, false).reordered);    // duplicate: treated as a reorder, not 65535 drops
  auto r = d.observe(0xFFF0, false);
  CHECK(r.reordered);
  CHECK(r.dropped == 0);
}

TEST_CASE("frame assembler: timestamp policies", "[frame]")
{
  SECTION("kHostOffsetOnce measures once and keeps the offset")
  {
    FrameAssembler fa(counter_policy(), TimestampPolicy::kHostOffsetOnce);
    auto p0 = make_packet(0, 0, 1'000'000);
    (void)fa.push(p0.view, 5'000'000);  // host is 4 ms ahead
    auto p1 = make_packet(1, 0, 2'000'000);
    (void)fa.push(p1.view, 9'000'000);  // later jitter is ignored
    REQUIRE(fa.time_mapper().offset_ns().has_value());
    CHECK(*fa.time_mapper().offset_ns() == 4'000'000);
    auto f = fa.flush();
    REQUIRE(f.has_value());
    CHECK(f->base_time_ns == 5'000'000);
    CHECK(f->points[4].offset_ns == 1'000'000);  // packet 1 first point
  }
  SECTION("kHostOffsetOnce passes PTP/GPS packets through")
  {
    FrameAssembler fa(counter_policy(), TimestampPolicy::kHostOffsetOnce);
    auto p = make_packet(0, 0, 1'000'000, 4, DataType::kCartesian32, 3000, TimeType::kPtp);
    (void)fa.push(p.view, 5'000'000);
    CHECK_FALSE(fa.time_mapper().offset_ns().has_value());
    CHECK(fa.flush()->base_time_ns == 1'000'000);
  }
  SECTION("kHostReceive uses the receive time")
  {
    FrameAssembler fa(counter_policy(), TimestampPolicy::kHostReceive);
    auto p = make_packet(0, 0, 1'000'000, 4, DataType::kCartesian32, 3000);
    (void)fa.push(p.view, 7'000'000);
    auto f = fa.flush();
    CHECK(f->base_time_ns == 7'000'000);
    CHECK(f->points[3].offset_ns == 300'000);  // interpolation is packet-relative
    CHECK(f->end_time_ns == 7'300'000);
  }
  SECTION("kLidar leaves the timestamp alone")
  {
    FrameAssembler fa(counter_policy(), TimestampPolicy::kLidar);
    auto p = make_packet(0, 0, 1'000'000);
    (void)fa.push(p.view, 7'000'000);
    CHECK(fa.flush()->base_time_ns == 1'000'000);
  }
}

TEST_CASE("frame assembler: offset_ns overflow forces a close", "[frame]")
{
  FrameAssembler fa(counter_policy(10s), TimestampPolicy::kLidar);
  auto p0 = make_packet(0, 0, 0);
  (void)fa.push(p0.view, 0);
  auto p1 = make_packet(1, 0, 5'000'000'000ull);  // 5 s later, same frame_cnt
  auto f = fa.push(p1.view, 0);
  REQUIRE(f.has_value());
  CHECK(f->points.size() == 4);
  CHECK(fa.flush()->base_time_ns == 5'000'000'000ull);
}

TEST_CASE("frame assembler: point conversion of every data type", "[frame]")
{
  Point out{};
  SECTION("cartesian16 is in 10 mm units")
  {
    auto p = make_packet(0, 0, 0, 1, DataType::kCartesian16);
    bytes::write_le<std::uint16_t>(p.data, 0, static_cast<std::uint16_t>(-150));  // x = -1.5 m
    bytes::write_le<std::uint16_t>(p.data, 2, 20);                                // y = 0.2 m
    bytes::write_le<std::uint16_t>(p.data, 4, 3);                                 // z = 0.03 m
    p.data[6] = std::byte{200};
    p.data[7] = std::byte{0x12};
    convert_point(p.view, 0, out);
    CHECK(out.x == Approx(-1.5F));
    CHECK(out.y == Approx(0.2F));
    CHECK(out.z == Approx(0.03F));
    CHECK(out.reflectivity == 200);
    CHECK(out.tag == 0x12);
  }
  SECTION("spherical uses zenith theta and azimuth phi")
  {
    auto p = make_packet(0, 0, 0, 1, DataType::kSpherical);
    bytes::write_le<std::uint32_t>(p.data, 0, 2000);  // 2 m
    bytes::write_le<std::uint16_t>(p.data, 4, 9000);  // theta 90 deg
    bytes::write_le<std::uint16_t>(p.data, 6, 3000);  // phi 30 deg
    p.data[8] = std::byte{5};
    convert_point(p.view, 0, out);
    const float phi = 30.0F * std::numbers::pi_v<float> / 180.0F;
    CHECK(out.x == Approx(2.0F * std::cos(phi)).margin(1e-4F));
    CHECK(out.y == Approx(2.0F * std::sin(phi)).margin(1e-4F));
    CHECK(out.z == Approx(0.0F).margin(1e-4F));
    CHECK(out.reflectivity == 5);
  }
  SECTION("imu packets are ignored by push")
  {
    FrameAssembler fa(counter_policy(), TimestampPolicy::kLidar);
    auto p = make_packet(0, 0, 0, 1, DataType::kImu);
    CHECK_FALSE(fa.push(p.view, 0).has_value());
    CHECK_FALSE(fa.has_partial());
    CHECK(fa.counters().packets == 0);
  }
}

TEST_CASE("frame assembler: a data_type change closes the frame with the old type", "[frame]")
{
  FrameAssembler fa(counter_policy(100ms), TimestampPolicy::kLidar);
  auto a = make_packet(0, 0, 0, 4, DataType::kCartesian32);
  auto b = make_packet(1, 0, 10 * kMs, 4, DataType::kCartesian32);
  auto c = make_packet(2, 0, 20 * kMs, 4, DataType::kCartesian16);
  CHECK_FALSE(fa.push(a.view, 0).has_value());
  CHECK_FALSE(fa.push(b.view, 0).has_value());
  const auto closed = fa.push(c.view, 0);
  REQUIRE(closed.has_value());
  CHECK(closed->source_type == DataType::kCartesian32);
  CHECK(closed->packets == 2);
  CHECK(closed->points.size() == 8);
  CHECK(fa.has_partial());
  const auto rest = fa.flush();
  REQUIRE(rest.has_value());
  CHECK(rest->source_type == DataType::kCartesian16);
  CHECK(rest->points.size() == 4);
}

TEST_CASE("frame assembler: set_policy takes effect at the next packet", "[frame]")
{
  FrameAssembler fa(counter_policy(100ms), TimestampPolicy::kLidar);
  std::uint16_t cnt = 0;
  std::size_t closed = 0;
  // constant frame_cnt: falls back at 200 ms, then windows of 100 ms
  for (std::uint64_t t = 0; t <= 250 * kMs; t += 10 * kMs) {
    auto p = make_packet(cnt++, 0, t);
    if (fa.push(p.view, 0)) {
      ++closed;
    }
  }
  CHECK(closed == 1);
  CHECK(fa.time_window_active());
  fa.set_policy(window_policy(20ms));
  CHECK(fa.policy().window == 20ms);
  CHECK(fa.time_window_active());  // by mode now, the fallback flag itself was reset
  // [200, 260): the partial frame from 200 ms is closed by the 20 ms window as soon as a
  // point past base + 20 ms arrives, then every 20 ms.
  for (std::uint64_t t = 260 * kMs; t <= 400 * kMs; t += 10 * kMs) {
    auto p = make_packet(cnt++, 0, t);
    if (fa.push(p.view, 0)) {
      ++closed;
    }
  }
  CHECK(closed >= 7);
  CHECK(fa.counters().frame_cnt_fallback == 1);
}
