// SPDX-License-Identifier: Apache-2.0
#include "frame_assembler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace livox::mid360::detail
{

// ---------------------------------------------------------------------------
// DropCounter
// ---------------------------------------------------------------------------
DropCounter::Result DropCounter::observe(std::uint16_t udp_cnt, bool frame_changed) noexcept
{
  Result r;
  if (!have_) {
    have_ = true;
    last_ = udp_cnt;
    return r;
  }
  const auto expected = static_cast<std::uint16_t>(last_ + 1u);
  const auto gap = static_cast<std::uint16_t>(udp_cnt - expected);
  if (gap == 0 || (udp_cnt == 0 && frame_changed)) {
    // in sequence, or the documented reset at a frame boundary
  } else if (gap > 0x8000u) {
    r.reordered = true;
    return r;  // keep `last_` at the highest counter seen
  } else {
    r.dropped = gap;
  }
  last_ = udp_cnt;
  return r;
}

// ---------------------------------------------------------------------------
// TimeMapper
// ---------------------------------------------------------------------------
std::uint64_t TimeMapper::map(const DataPacketHeader & h, std::uint64_t recv_time_ns) noexcept
{
  switch (policy_) {
    case TimestampPolicy::kLidar:
      return h.timestamp_ns;
    case TimestampPolicy::kHostReceive:
      return recv_time_ns;
    case TimestampPolicy::kHostOffsetOnce:
      break;
  }
  if (h.time_type != TimeType::kNoSync) {
    return h.timestamp_ns;  // synchronised: trust it
  }
  if (!offset_) {
    offset_ = static_cast<std::int64_t>(recv_time_ns) - static_cast<std::int64_t>(h.timestamp_ns);
  }
  return static_cast<std::uint64_t>(static_cast<std::int64_t>(h.timestamp_ns) + *offset_);
}

// ---------------------------------------------------------------------------
// Point conversion
// ---------------------------------------------------------------------------
void convert_point(const DataPacketView & pkt, std::size_t i, Point & out) noexcept
{
  switch (pkt.header.data_type) {
    case DataType::kCartesian32: {
      const auto p = decode_cartesian32(pkt, i);
      out.x = static_cast<float>(p.x_mm) * 0.001F;
      out.y = static_cast<float>(p.y_mm) * 0.001F;
      out.z = static_cast<float>(p.z_mm) * 0.001F;
      out.reflectivity = p.reflectivity;
      out.tag = p.tag;
      break;
    }
    case DataType::kCartesian16: {
      const auto p = decode_cartesian16(pkt, i);
      out.x = static_cast<float>(p.x_cm) * 0.01F;
      out.y = static_cast<float>(p.y_cm) * 0.01F;
      out.z = static_cast<float>(p.z_cm) * 0.01F;
      out.reflectivity = p.reflectivity;
      out.tag = p.tag;
      break;
    }
    case DataType::kSpherical: {
      const auto p = decode_spherical(pkt, i);
      constexpr float kCentidegToRad = std::numbers::pi_v<float> / 18000.0F;
      const float d = static_cast<float>(p.depth_mm) * 0.001F;
      const float theta = static_cast<float>(p.theta_centideg) * kCentidegToRad;
      const float phi = static_cast<float>(p.phi_centideg) * kCentidegToRad;
      const float st = std::sin(theta);
      out.x = d * st * std::cos(phi);
      out.y = d * st * std::sin(phi);
      out.z = d * std::cos(theta);
      out.reflectivity = p.reflectivity;
      out.tag = p.tag;
      break;
    }
    case DataType::kImu:
      out = Point{};
      break;
  }
  out.line = static_cast<std::uint8_t>(i % 4);
}

// ---------------------------------------------------------------------------
// FrameAssembler
// ---------------------------------------------------------------------------
FrameAssembler::FrameAssembler(FramePolicy policy, TimestampPolicy timestamps)
: policy_(policy), time_(timestamps)
{
}

void FrameAssembler::set_policy(FramePolicy policy) noexcept
{
  policy_ = policy;
  fallback_ = false;
  frame_cnt_changed_ever_ = false;
  first_time_.reset();
}

bool FrameAssembler::time_window_active() const noexcept
{
  return policy_.mode == FramePolicy::Mode::kTimeWindow || fallback_;
}

Frame FrameAssembler::take_frame()
{
  Frame out = std::move(cur_);
  out.index = next_index_++;
  ++counters_.frames;
  cur_ = Frame{};
  cur_.points.reserve(out.points.size());
  return out;
}

void FrameAssembler::discard() noexcept
{
  counters_.points -= cur_.points.size();
  cur_.points.clear();
  cur_.packets = 0;
  cur_.dropped_packets = 0;
}

std::optional<Frame> FrameAssembler::flush()
{
  if (cur_.points.empty()) {
    return std::nullopt;
  }
  return take_frame();
}

std::optional<Frame> FrameAssembler::push(const DataPacketView & pkt, std::uint64_t recv_time_ns)
{
  const DataPacketHeader & h = pkt.header;
  if (h.data_type == DataType::kImu || h.dot_num == 0) {
    return std::nullopt;
  }

  const std::uint64_t t0 = time_.map(h, recv_time_ns);
  const bool frame_changed = have_prev_ && h.frame_cnt != prev_frame_cnt_;
  const auto drop = drops_.observe(h.udp_cnt, frame_changed);
  counters_.dropped_packets += drop.dropped;
  counters_.reordered += drop.reordered ? 1u : 0u;

  const auto window = static_cast<std::uint64_t>(std::max<std::int64_t>(policy_.window.count(), 1));
  if (!first_time_) {
    first_time_ = t0;
  }
  if (policy_.mode == FramePolicy::Mode::kFrameCounter && !fallback_) {
    if (frame_changed) {
      frame_cnt_changed_ever_ = true;
    } else if (!frame_cnt_changed_ever_ && t0 >= *first_time_ + 2 * window) {
      fallback_ = true;
      ++counters_.frame_cnt_fallback;
    }
  }

  std::optional<Frame> out;
  if (!cur_.points.empty()) {
    bool close = time_window_active() ? t0 >= cur_.base_time_ns + window : frame_changed;
    if (h.data_type != cur_.source_type) {
      close = true;  // the point format changed (set_point_format()): one format per Frame
    }
    const std::uint64_t span_ns = static_cast<std::uint64_t>(h.time_interval) * 100u;
    if (
      t0 >= cur_.base_time_ns &&
      t0 - cur_.base_time_ns + span_ns > std::numeric_limits<std::uint32_t>::max()) {
      close = true;  // Point::offset_ns would overflow
    }
    if (close) {
      out = take_frame();
    }
  }
  if (cur_.points.empty()) {
    cur_.base_time_ns = t0;
    cur_.end_time_ns = t0;
    cur_.frame_cnt = h.frame_cnt;
    cur_.source_type = h.data_type;
    cur_.time_type = h.time_type;
  }
  append(pkt, t0);
  cur_.dropped_packets += drop.dropped;
  ++cur_.packets;
  ++counters_.packets;
  counters_.points += h.dot_num;
  have_prev_ = true;
  prev_frame_cnt_ = h.frame_cnt;
  return out;
}

void FrameAssembler::append(const DataPacketView & pkt, std::uint64_t t0)
{
  const DataPacketHeader & h = pkt.header;
  const std::size_t n = h.dot_num;
  const std::size_t first = cur_.points.size();
  cur_.points.resize(first + n);
  for (std::size_t i = 0; i < n; ++i) {
    Point & p = cur_.points[first + i];
    convert_point(pkt, i, p);
    const std::uint64_t ts = t0 + (sample_timestamp_ns(h, i) - h.timestamp_ns);
    // A reordered packet may precede the frame base; clamp rather than wrap.
    const std::uint64_t off = ts >= cur_.base_time_ns ? ts - cur_.base_time_ns : 0;
    p.offset_ns = static_cast<std::uint32_t>(std::min<std::uint64_t>(off, 0xFFFFFFFFu));
    cur_.end_time_ns = std::max(cur_.end_time_ns, ts);
  }
}

}  // namespace livox::mid360::detail
