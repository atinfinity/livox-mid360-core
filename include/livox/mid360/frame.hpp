// SPDX-License-Identifier: Apache-2.0
// Public API skeleton (issue #9): output types of the device layer. Plain structs so that the
// phase-3 C ABI can expose the same layout. Frame assembly is implemented in #6.
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

#include "livox/mid360/export.hpp"
#include "livox/mid360/protocol.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

/// One return, same semantics as livox_ros_driver2's CustomPoint. Spherical packets are
/// converted to Cartesian before they reach a Frame.
struct Point
{
  float x = 0;  ///< metres, LiDAR frame
  float y = 0;
  float z = 0;
  std::uint8_t reflectivity = 0;  ///< 0-255
  std::uint8_t tag = 0;           ///< raw tag byte; see decode_tag()
  std::uint8_t line = 0;          ///< Mid-360 has no physical lines: sample index % 4
  std::uint32_t offset_ns = 0;    ///< sample time - Frame::base_time_ns
};

/// A group of points closed by FramePolicy. Owns its storage; delivered by value.
struct Frame
{
  std::uint32_t index = 0;         ///< +1 per delivered frame, per Device
  std::uint64_t base_time_ns = 0;  ///< time of the first point (after timestamp policy)
  std::uint64_t end_time_ns = 0;   ///< time of the last point
  std::vector<Point> points;
  std::uint32_t packets = 0;          ///< data packets merged into this frame
  std::uint32_t dropped_packets = 0;  ///< udp_cnt gaps observed while assembling it
  std::uint8_t frame_cnt = 0;         ///< header frame_cnt of the first packet
  DataType source_type = DataType::kCartesian32;
  TimeType time_type = TimeType::kNoSync;
};

/// One IMU packet (the LiDAR sends one sample per packet at 200 Hz) with its time. Not part
/// of a Frame. `sample` is the protocol-layer ImuSample (gyro rad/s, acc g).
struct ImuData
{
  std::uint64_t time_ns = 0;  ///< after timestamp policy
  ImuSample sample{};
};

/// How a Device cuts the packet stream into Frames (#6).
struct FramePolicy
{
  enum class Mode : std::uint8_t
  {
    kFrameCounter,  ///< close on a change of header frame_cnt; a jump still closes one frame
    kTimeWindow,    ///< close every `window` of point time (livox_ros_driver2 publish period)
  };
  Mode mode = Mode::kFrameCounter;
  std::chrono::nanoseconds window{std::chrono::milliseconds{100}};  ///< kTimeWindow only
};

/// How point/IMU timestamps are produced from the packet timestamp (#6).
enum class TimestampPolicy : std::uint8_t
{
  kLidar,           ///< packet timestamp as is (use with PTP / GPS synchronisation)
  kHostOffsetOnce,  ///< default: LiDAR time + (host - LiDAR) measured once at the first packet
  kHostReceive,     ///< kernel receive time of the packet
};

/// Bounded single-producer / multi-consumer queue for handing Frames or ImuData from the
/// receive thread to any other thread. Overflow drops the OLDEST element (newest data wins)
/// and counts it in `dropped()`. Not part of Device: connect it yourself, e.g.
///   BoundedQueue<Frame> q;  device.on_frame([&](Frame&& f) { q.push(std::move(f)); });
template <class T>
class BoundedQueue
{
public:
  explicit BoundedQueue(std::size_t capacity = 8) : capacity_(capacity) {}

  /// Producer side (receive thread). Never blocks.
  void push(T && item)
  {
    {
      std::lock_guard lock(mutex_);
      if (closed_) {
        return;
      }
      if (items_.size() >= capacity_) {
        items_.pop_front();
        ++dropped_;
      }
      items_.push_back(std::move(item));
    }
    cv_.notify_one();
  }

  /// Consumer side. Empty after `timeout` or once closed and drained.
  [[nodiscard]] std::optional<T> pop(std::chrono::nanoseconds timeout)
  {
    std::unique_lock lock(mutex_);
    cv_.wait_for(lock, timeout, [this] { return closed_ || !items_.empty(); });
    return take();
  }

  [[nodiscard]] std::optional<T> try_pop()
  {
    std::lock_guard lock(mutex_);
    return take();
  }

  /// Wakes every waiting pop(); later pushes are ignored, queued items can still be popped.
  void close()
  {
    {
      std::lock_guard lock(mutex_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  [[nodiscard]] std::uint64_t dropped() const
  {
    std::lock_guard lock(mutex_);
    return dropped_;
  }
  [[nodiscard]] std::size_t size() const
  {
    std::lock_guard lock(mutex_);
    return items_.size();
  }
  [[nodiscard]] std::size_t capacity() const noexcept { return capacity_; }

private:
  std::optional<T> take()
  {
    if (items_.empty()) {
      return std::nullopt;
    }
    std::optional<T> out{std::move(items_.front())};
    items_.pop_front();
    return out;
  }

  std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<T> items_;
  std::uint64_t dropped_ = 0;
  bool closed_ = false;
};

}  // namespace livox::mid360
LIVOX_MID360_API_END
