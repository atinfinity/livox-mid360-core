// SPDX-License-Identifier: Apache-2.0
// Public API (issue #9): one LiDAR. Wraps a Session (commands, caller's thread) and
// receives its data through a Context (receive thread, callbacks). Threading rules:
//   - callbacks run on the Context's receive thread, must return quickly and never block;
//   - commands are serialised by an internal mutex and may be called from any user thread,
//     but NEVER from a callback (deadlock; asserted in debug builds) - react to Events from
//     your own thread instead;
//   - an exception escaping a callback terminates the process;
//   - a Device must not be destroyed from inside its own callbacks.
// Data path implemented in #6; push / state / HMS are #7 and reconnection is #8.
#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <span>

#include "livox/mid360/config.hpp"
#include "livox/mid360/context.hpp"
#include "livox/mid360/event.hpp"
#include "livox/mid360/export.hpp"
#include "livox/mid360/frame.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/session.hpp"
#include "livox/mid360/transport.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360 {

struct DeviceOptions {
  /// Ports, data type and IMU enable. `ip` and the ports are taken from the Context;
  /// `work_tgt_mode` is ignored (use start_sampling()).
  HostSetup host_setup;
  TimestampPolicy timestamp_policy = TimestampPolicy::kHostOffsetOnce;
  FramePolicy frame_policy;
  SessionOptions session;  ///< command socket; `bind_address` defaults to the Context's
  /// Verify the CRC32 of every data packet; failures count in DeviceStats::bad_packets.
  bool verify_crc = true;
  /// Period of Event::Kind::kStats; 0 disables it.
  std::chrono::milliseconds stats_interval{1000};
};

/// Per-packet metadata handed to on_packet together with the non-owning DataPacketView.
struct ReceiveInfo {
  std::uint64_t host_time_ns = 0;  ///< kernel receive timestamp
  Endpoint source;
};

using PacketCallback = std::function<void(const DataPacketView&, const ReceiveInfo&)>;
using FrameCallback = std::function<void(Frame&&)>;
using ImuCallback = std::function<void(const ImuData&)>;
using EventCallback = std::function<void(const Event&)>;

class Device {
 public:
  /// Registers with the Context (kAlreadyRegistered when another Device has the same IP),
  /// connects the Session and applies `opts.host_setup` pointed at the command socket's
  /// local address (or `host_setup.ip`) and the Context's ports. Does not change the work mode.
  [[nodiscard]] static std::expected<std::unique_ptr<Device>, DeviceError> open(
      Context& context, const DiscoveredDevice& device, const DeviceOptions& opts = {});

  /// Unregisters from the Context (waits for an in-flight callback to return) and drops the
  /// Session. The LiDAR keeps streaming; call stop_sampling() first if that matters.
  ~Device();
  Device(const Device&) = delete;
  Device& operator=(const Device&) = delete;
  Device(Device&&) = delete;
  Device& operator=(Device&&) = delete;

  // --- callbacks: one per kind, settable while sampling has not been requested (before
  // start_sampling() or after stop_sampling()); otherwise kInvalidState. Pass an empty
  // function to clear. Packets arriving while no callback is set are dropped silently.
  std::expected<void, DeviceError> on_packet(PacketCallback cb);
  std::expected<void, DeviceError> on_frame(FrameCallback cb);
  std::expected<void, DeviceError> on_imu(ImuCallback cb);
  std::expected<void, DeviceError> on_event(EventCallback cb);

  // --- commands (caller's thread, serialised, blocking; see Session for the semantics)
  /// work_tgt_mode = SAMPLING, then wait for cur_work_state (host_setup.wait_timeout).
  /// Idempotent. Callbacks are frozen from the first successful call on.
  std::expected<void, DeviceError> start_sampling(
      std::optional<RequestOptions> opts = std::nullopt);
  /// work_tgt_mode = IDLE, then wait. A partial frame is discarded, not delivered.
  std::expected<void, DeviceError> stop_sampling(std::optional<RequestOptions> opts = std::nullopt);
  std::expected<ParamConfigAck, DeviceError> configure(
      std::span<const KeyValue> values, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<InquireResult, DeviceError> inquire(
      std::span<const std::uint16_t> keys, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<void, DeviceError> reboot(std::optional<RequestOptions> opts = std::nullopt);
  /// Interrupts a blocking command from another thread (not serialised).
  void cancel() noexcept;

  // --- observation (thread-safe snapshots)
  [[nodiscard]] const DiscoveredDevice& info() const noexcept;
  [[nodiscard]] std::optional<WorkState> work_state()
      const;  ///< last pushed 0x8006 (#7; nullopt until then)
  [[nodiscard]] DeviceStats stats() const;
  [[nodiscard]] SessionStats session_stats() const;

 private:
  struct Impl;
  explicit Device(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace livox::mid360
LIVOX_MID360_API_END
