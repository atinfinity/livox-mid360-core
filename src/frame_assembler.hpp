// SPDX-License-Identifier: Apache-2.0
// Internal (not installed): turns parsed point-cloud packets into Frames (issue #6). No I/O,
// no threads; the Device's receive thread drives it. Design decisions are on issue #6.
#pragma once

#include <cstdint>
#include <optional>

#include "livox/mid360/export.hpp"
#include "livox/mid360/frame.hpp"
#include "livox/mid360/protocol.hpp"

// Exported (like session_detail.hpp) so the tests and fuzzers can link against it.
LIVOX_MID360_API_BEGIN
namespace livox::mid360::detail
{

/// udp_cnt gap tracking for one source port (point cloud and IMU are counted separately).
class DropCounter
{
public:
  struct Result
  {
    std::uint32_t dropped = 0;  ///< packets missing before this one
    bool reordered = false;     ///< udp_cnt went backwards; not a drop
  };
  /// `frame_changed`: the packet's frame_cnt differs from the previous packet's. Together
  /// with udp_cnt == 0 it is the wiki's "reset at frame start" and yields no gap.
  Result observe(std::uint16_t udp_cnt, bool frame_changed) noexcept;
  void reset() noexcept { have_ = false; }

private:
  bool have_ = false;
  std::uint16_t last_ = 0;
};

/// Maps packet timestamps to output time according to TimestampPolicy.
class TimeMapper
{
public:
  explicit TimeMapper(TimestampPolicy policy) noexcept : policy_(policy) {}
  /// Output time of the packet's first sample. `recv_time_ns` is the kernel receive time.
  [[nodiscard]] std::uint64_t map(const DataPacketHeader & h, std::uint64_t recv_time_ns) noexcept;
  [[nodiscard]] std::optional<std::int64_t> offset_ns() const noexcept { return offset_; }
  void reset() noexcept { offset_.reset(); }

private:
  TimestampPolicy policy_;
  std::optional<std::int64_t> offset_;
};

class FrameAssembler
{
public:
  struct Counters
  {
    std::uint64_t packets = 0;
    std::uint64_t points = 0;
    std::uint64_t frames = 0;
    std::uint64_t dropped_packets = 0;
    std::uint64_t reordered = 0;
    std::uint64_t frame_cnt_fallback = 0;
  };

  FrameAssembler(FramePolicy policy, TimestampPolicy timestamps);

  /// Feed one parsed point-cloud packet (data_type != kImu; IMU packets are ignored). Returns
  /// the frame closed by this packet, if any. The returned Frame owns its points.
  [[nodiscard]] std::optional<Frame> push(const DataPacketView & pkt, std::uint64_t recv_time_ns);
  /// Close and return the partial frame (idle close); nullopt when empty.
  [[nodiscard]] std::optional<Frame> flush();
  /// Drop the partial frame without delivering it (stop_sampling); its points are taken
  /// back out of `counters().points`. Drop / timestamp baselines are kept.
  void discard() noexcept;

  [[nodiscard]] bool has_partial() const noexcept { return !cur_.points.empty(); }
  [[nodiscard]] const Counters & counters() const noexcept { return counters_; }
  [[nodiscard]] TimeMapper & time_mapper() noexcept { return time_; }
  /// True once kFrameCounter has fallen back to the time window.
  [[nodiscard]] bool time_window_active() const noexcept;

private:
  Frame take_frame();
  void append(const DataPacketView & pkt, std::uint64_t t0);

  FramePolicy policy_;
  TimeMapper time_;
  DropCounter drops_;
  Counters counters_;
  Frame cur_;
  std::uint32_t next_index_ = 0;
  bool have_prev_ = false;
  std::uint8_t prev_frame_cnt_ = 0;
  bool frame_cnt_changed_ever_ = false;
  bool fallback_ = false;
  std::optional<std::uint64_t> first_time_;
};

/// Converts one sample of `pkt` (any point-cloud data type) to metres.
void convert_point(const DataPacketView & pkt, std::size_t i, Point & out) noexcept;

}  // namespace livox::mid360::detail
LIVOX_MID360_API_END
