// SPDX-License-Identifier: Apache-2.0
// Public API (issue #9): one LiDAR. Wraps a Session (commands, caller's thread) and
// receives its data through a Context (receive thread, callbacks). Threading rules:
//   - callbacks run on the Context's receive thread, must return quickly and never block;
//   - commands are serialised by an internal mutex and may be called from any user thread,
//     but NEVER from a callback (deadlock; asserted in debug builds) - react to Events from
//     your own thread instead;
//   - an exception escaping a callback terminates the process;
//   - a Device must not be destroyed from inside its own callbacks.
// Data path (#6), push / state / HMS (#7) and reconnection (#8) are implemented.
// Reconnection: a Device is *disconnected* when no 0x0102 push arrived for
// ReconnectOptions::push_timeout (or a command timed out while the push was already stale,
// or reboot() was acknowledged). Event::kDisconnected is raised, commands fail with
// DeviceError::Kind::kDisconnected, and - when ReconnectOptions::enabled - a worker thread
// owned by the Device retries with exponential backoff: the last known command endpoint first
// (serial verified), then discovery filtered by serial. On success the host setup is applied
// again, sampling is resumed if it had been requested, and Event::kReconnected is raised.
// Callbacks stay frozen for the whole period when sampling was requested.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

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
namespace livox::mid360
{

/// Disconnect detection and automatic recovery (issue #8).
struct ReconnectOptions
{
  bool enabled = true;  ///< false: only detect; recover with Device::reconnect()
  /// No accepted 0x0102 push for this long → kDisconnected. The LiDAR pushes about once per
  /// second. A command timeout counts as a disconnect only when the last push is older than
  /// a third of this value (one nominal push period).
  std::chrono::milliseconds push_timeout{3000};
  std::chrono::milliseconds initial_backoff{500};  ///< delay after the first failed attempt
  std::chrono::milliseconds max_backoff{8000};     ///< doubling stops here
  /// Timeout of the discovery fallback per attempt (the direct attempt uses `session.request`).
  std::chrono::milliseconds discovery_timeout{1000};
  /// Unicast discovery targets for the fallback; empty = broadcast (see DiscoveryOptions).
  std::vector<Endpoint> discovery_targets;
};

struct DeviceOptions
{
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
  ReconnectOptions reconnect;
};

/// Per-packet metadata handed to on_packet together with the non-owning DataPacketView.
struct ReceiveInfo
{
  std::uint64_t host_time_ns = 0;  ///< kernel receive timestamp
  Endpoint source;
};

using PacketCallback = std::function<void(const DataPacketView &, const ReceiveInfo &)>;
using FrameCallback = std::function<void(Frame &&)>;
using ImuCallback = std::function<void(const ImuData &)>;
using EventCallback = std::function<void(const Event &)>;

/// Result of Device::set<K>() / set_many<>(): the LiDAR accepted the values.
struct SetResult
{
  /// ret_code 0x21: the new value takes effect after the next reboot (e.g. kLidarIpCfg).
  bool reboot_required = false;
};

class Device
{
public:
  /// Registers with the Context (kAlreadyRegistered when another Device has the same IP),
  /// connects the Session and applies `opts.host_setup` pointed at the command socket's
  /// local address (or `host_setup.ip`) and the Context's ports. Does not change the work mode.
  [[nodiscard]] static std::expected<std::unique_ptr<Device>, DeviceError> open(
    Context & context, const DiscoveredDevice & device, const DeviceOptions & opts = {});

  /// Unregisters from the Context (waits for an in-flight callback to return) and drops the
  /// Session. The LiDAR keeps streaming; call stop_sampling() first if that matters.
  ~Device();
  Device(const Device &) = delete;
  Device & operator=(const Device &) = delete;
  Device(Device &&) = delete;
  Device & operator=(Device &&) = delete;

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
  std::expected<InquireResult, DeviceError> inquire(
    std::span<const Key> keys, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<void, DeviceError> reboot(std::optional<RequestOptions> opts = std::nullopt);

  // --- typed key access (issue #57): key_traits<K> in keys.hpp gives each key its C++ type.
  /// One 0x0100 with the encoded value. Only the ACK is awaited: set<Key::kWorkTgtMode>()
  /// does not wait for the state change (start_sampling() / stop_sampling() do). Read-only
  /// keys and the unmodelled Mid-360S / 360L keys do not compile.
  template <Key K>
    requires writable_key<K>
  std::expected<SetResult, DeviceError> set(
    const key_value_t<K> & value, std::optional<RequestOptions> opts = std::nullopt)
  {
    return set_many<K>(opts, value);
  }
  /// Several keys in one 0x0100 (the LiDAR applies all or rejects the whole request).
  template <Key... Ks>
    requires(sizeof...(Ks) > 0 && (writable_key<Ks> && ...))
  std::expected<SetResult, DeviceError> set_many(const key_value_t<Ks> &... values)
  {
    return set_many<Ks...>(std::nullopt, values...);
  }
  template <Key... Ks>
    requires(sizeof...(Ks) > 0 && (writable_key<Ks> && ...))
  std::expected<SetResult, DeviceError> set_many(
    std::optional<RequestOptions> opts, const key_value_t<Ks> &... values)
  {
    auto encoded = std::make_tuple(key_traits<Ks>::encode(values)...);
    const auto kvs = std::apply(
      [](const auto &... e) {
        return std::array<KeyValue, sizeof...(Ks)>{
          KeyValue{static_cast<std::uint16_t>(Ks), std::span<const std::byte>(e)}...};
      },
      encoded);
    auto ack = configure(kvs, opts);
    if (!ack) {
      return std::unexpected(ack.error());
    }
    return SetResult{.reboot_required = ack->ret_code == RetCode::kParamRebootEffect};
  }
  /// One 0x0101 for `K`, decoded. kDecodeFailed (with `key`) when the ACK lacks the key or
  /// its value has the wrong length / an out-of-range value.
  template <Key K>
    requires typed_key<K>
  std::expected<key_value_t<K>, DeviceError> get(std::optional<RequestOptions> opts = std::nullopt)
  {
    auto r = get_many<K>(opts);
    if (!r) {
      return std::unexpected(r.error());
    }
    return std::get<0>(std::move(*r));
  }
  /// Several keys in one 0x0101, decoded into a tuple in the order of `Ks`.
  template <Key... Ks>
    requires(sizeof...(Ks) > 0 && (typed_key<Ks> && ...))
  std::expected<std::tuple<key_value_t<Ks>...>, DeviceError> get_many(
    std::optional<RequestOptions> opts = std::nullopt)
  {
    const std::array<Key, sizeof...(Ks)> keys{Ks...};
    auto res = inquire(keys, opts);
    if (!res) {
      return std::unexpected(res.error());
    }
    std::optional<DeviceError> failed;
    std::tuple<key_value_t<Ks>...> out{decode_one<Ks>(*res, failed)...};
    if (failed) {
      return std::unexpected(*failed);
    }
    return out;
  }
  /// Interrupts a blocking command from another thread (not serialised).
  void cancel() noexcept;

  // --- connection (issue #8)
  /// False between kDisconnected and kReconnected.
  [[nodiscard]] bool connected() const noexcept;
  /// One recovery attempt on the caller's thread (serialised with the commands): direct
  /// endpoint, then discovery; host setup and sampling replayed. No-op when connected.
  /// The automatic worker uses the same path. kSession on failure.
  std::expected<void, DeviceError> reconnect();
  /// Declares the Device disconnected (reason kUser) without touching the LiDAR; the
  /// automatic worker, when enabled, reconnects. Useful for tests and for forcing a
  /// re-discovery after a known network change.
  void disconnect();

  // --- observation (thread-safe snapshots)
  /// Discovery record; `ip` / `cmd_port` / `from` follow a reconnect to a new address.
  [[nodiscard]] DiscoveredDevice info() const;
  /// cur_work_state from the last 0x0102 push; nullopt before the first push.
  [[nodiscard]] std::optional<WorkState> work_state() const;
  /// hms_code slots from the last 0x0102 push (all inactive before the first push).
  [[nodiscard]] std::array<HmsCode, 8> hms() const;
  [[nodiscard]] DeviceStats stats() const;
  [[nodiscard]] SessionStats session_stats() const;

private:
  struct Impl;
  explicit Device(std::unique_ptr<Impl> impl);

  template <Key K>
  static key_value_t<K> decode_one(const InquireResult & res, std::optional<DeviceError> & failed)
  {
    if (const auto raw = res.get(K)) {
      if (auto v = key_traits<K>::decode(*raw)) {
        return std::move(*v);
      }
    }
    if (!failed) {
      failed =
        DeviceError{.kind = DeviceError::Kind::kDecodeFailed, .session = std::nullopt, .key = K};
    }
    return {};
  }
  std::unique_ptr<Impl> impl_;
};

}  // namespace livox::mid360
LIVOX_MID360_API_END
