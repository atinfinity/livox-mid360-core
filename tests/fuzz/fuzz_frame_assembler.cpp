// SPDX-License-Identifier: Apache-2.0
// Drives detail::FrameAssembler with synthesised packets (issue #6). Byte-level parsing is
// covered by fuzz_data_packet, so the input here is decoded as header fields directly:
//   [policy u8][records: udp_cnt u16 | frame_cnt u8 | data_type u8 | dot_num u8 |
//                        ts_delta u16 (x 100 us) | recv_delta u8 (x 1 ms)] ...
// policy: bit0 kTimeWindow, bits1-2 TimestampPolicy, bits3-4 window (10/100/1000/10000 ms).
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>

#include "frame_assembler.hpp"
#include "fuzz_check.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  using namespace livox::mid360;
  if (size < 1) return 0;
  const std::uint8_t policy = data[0];
  FramePolicy fp;
  fp.mode = (policy & 1) ? FramePolicy::Mode::kTimeWindow : FramePolicy::Mode::kFrameCounter;
  static constexpr std::array<std::int64_t, 4> kWindowsMs{10, 100, 1000, 10000};
  fp.window = std::chrono::milliseconds{kWindowsMs[(policy >> 3) & 3]};
  const auto tp = static_cast<TimestampPolicy>(std::min<unsigned>((policy >> 1) & 3, 2));
  detail::FrameAssembler fa(fp, tp);

  static std::array<std::byte, std::size_t{255} * 24> zeros{};
  std::uint64_t ts = 1'000'000'000'000ull;
  std::uint64_t recv = 2'000'000'000'000ull;
  std::uint64_t fed_points = 0;
  std::uint64_t delivered_points = 0;
  std::uint64_t last_dropped = 0;
  std::uint32_t next_index = 0;
  constexpr std::size_t kRec = 8;

  auto check_frame = [&](const Frame & f) {
    fuzz::require(!f.points.empty());
    fuzz::require(f.index == next_index++);
    fuzz::require(f.end_time_ns >= f.base_time_ns);
    fuzz::require(f.packets >= 1);
    delivered_points += f.points.size();
    fuzz::require(delivered_points <= fed_points);
  };

  for (std::size_t off = 1; off + kRec <= size; off += kRec) {
    const std::uint8_t * r = data + off;
    DataPacketView v;
    std::memcpy(&v.header.udp_cnt, r, 2);
    v.header.frame_cnt = r[2];
    v.header.data_type = static_cast<DataType>(r[3] & 3);
    v.header.dot_num = r[4];
    std::uint16_t ts_delta = 0;
    std::memcpy(&ts_delta, r + 5, 2);
    ts += static_cast<std::uint64_t>(ts_delta) * 100'000u;
    recv += static_cast<std::uint64_t>(r[7]) * 1'000'000u;
    v.header.timestamp_ns = ts;
    v.header.time_interval = ts_delta;  // 0.1 us units; keeps span <= delta
    v.header.time_type = static_cast<TimeType>((r[3] >> 2) % 3);
    const std::size_t bytes = v.header.dot_num * sample_size(v.header.data_type);
    v.data = std::span<const std::byte>(zeros).first(bytes);

    const std::uint64_t before_pkts = fa.counters().packets;
    if (auto f = fa.push(v, recv)) check_frame(*f);
    if (fa.counters().packets != before_pkts) {
      fed_points += v.header.dot_num;
      // offsets are non-decreasing within the packet just appended (unless it was reordered
      // before the frame base and clamped, which keeps them equal)
    }
    fuzz::require(fa.counters().dropped_packets >= last_dropped);
    last_dropped = fa.counters().dropped_packets;
  }
  if (auto f = fa.flush()) check_frame(*f);
  fuzz::require(!fa.has_partial());
  fuzz::require(fa.counters().points == fed_points);
  fuzz::require(fa.counters().frames == next_index);
  return 0;
}
