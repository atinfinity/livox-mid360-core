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
#include "livox/mid360/firmware_log.hpp"
#include "livox/mid360/frame.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/lidar_info.hpp"
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
  /// Port of the extra unicast target `{ip configured by set_lidar_ip_config(), port}` that
  /// the fallback adds after a network-config change (#50). Tests point it at the simulator.
  std::uint16_t discovery_port = kDiscoveryPort;
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
  /// LiDAR-side port that 0x0301 is sent to (#44). Tests point it at the simulator.
  std::uint16_t lidar_log_port = kLogPort;
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
/// Every successfully parsed 0x0102 push, as the snapshot pushed_status() now returns (#56).
using PushCallback = std::function<void(const LidarStatus &)>;
/// One 0x0300 firmware log push (#44); `data` is valid only during the call.
using FirmwareLogCallback = std::function<void(const FirmwareLogChunk &)>;

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
  /// Receive thread, once per parsed push, after the push's kStateChanged / kHms /
  /// kDiagChanged events; the argument is the merged snapshot (missing keys carried over).
  std::expected<void, DeviceError> on_push(PushCallback cb);
  /// Receive thread, once per 0x0300 push including begin / end packets (#44).
  std::expected<void, DeviceError> on_firmware_log(FirmwareLogCallback cb);

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

  // --- aggregated read-back (issues #38 / #41): one 0x0101 each, not cached.
  /// Keys 0x8000-0x8005. Missing keys leave their field empty; see decode_identity().
  std::expected<DeviceIdentity, DeviceError> identity(
    std::optional<RequestOptions> opts = std::nullopt);
  /// The 16 modelled writable keys as stored by the LiDAR; see decode_settings(). A key the
  /// firmware rejects with kParamNotSupport (0x002B on older firmware) is dropped and the
  /// inquire repeated, so its optional is empty rather than the whole call failing.
  std::expected<LidarSettings, DeviceError> settings(
    std::optional<RequestOptions> opts = std::nullopt);
  /// Keys 0x8006-0x8011 by inquire; see decode_status(). pushed_status() has the same data
  /// from the last 0x0102 push without a round trip.
  std::expected<LidarStatus, DeviceError> status(std::optional<RequestOptions> opts = std::nullopt);

  // --- point stream (issue #40): keys 0x0000 / 0x0001 and the host-side frame policy.
  /// Key 0x0000. kImu → kInvalidArgument with `key` before any I/O. The receive side needs no
  /// help: the frame being assembled is closed and delivered when the first packet in the
  /// new format arrives, so every Frame has one `source_type`.
  std::expected<SetResult, DeviceError> set_point_format(
    DataType format, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<DataType, DeviceError> point_format(
    std::optional<RequestOptions> opts = std::nullopt);
  /// Key 0x0001. Not pre-checked: the wiki says only kNonRepetitive works on the base
  /// Mid-360, and the LiDAR's ACK (kSession / kLidarRejected) is the answer for the others.
  std::expected<SetResult, DeviceError> set_scan_pattern(
    ScanPattern pattern, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<ScanPattern, DeviceError> scan_pattern(
    std::optional<RequestOptions> opts = std::nullopt);
  /// Replace DeviceOptions::frame_policy at run time (`window` must be > 0 →
  /// kInvalidArgument). Takes effect at the next data packet; the frame in progress is closed
  /// by whichever policy is active then. The base Mid-360 has no frame-rate key: the frame
  /// rate is this policy's `window` (or the LiDAR's frame_cnt period).
  std::expected<void, DeviceError> set_frame_policy(const FramePolicy & policy);
  /// The policy last requested (DeviceOptions::frame_policy until set_frame_policy()).
  [[nodiscard]] FramePolicy frame_policy() const;

  // --- FOV (issue #39): keys 0x0015 / 0x0016 / 0x0017.
  /// One 0x0100 with the present fields of `fov` (the LiDAR applies all or none). Validated
  /// before any I/O: no field → kInvalidArgument without `key`; a window outside
  /// fov_in_range() → kInvalidArgument with `key` = 0x0015 / 0x0016. HostSetup::fov does the
  /// same at open() and after a reconnect.
  std::expected<SetResult, DeviceError> set_fov(
    const FovSettings & fov, std::optional<RequestOptions> opts = std::nullopt);
  /// One 0x0101 for the three keys. A key the ACK lacks leaves its field empty.
  std::expected<FovSettings, DeviceError> fov(std::optional<RequestOptions> opts = std::nullopt);

  // --- install attitude (issue #51): key 0x0012.
  /// install_attitude_valid() → else kInvalidArgument with `key` before any I/O. Only stores
  /// the value on the LiDAR; whether the firmware applies it to the emitted points is
  /// unverified (#11). To transform on the host use extrinsic_from() / apply() in frame.hpp.
  std::expected<SetResult, DeviceError> set_install_attitude(
    const InstallAttitude & a, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<InstallAttitude, DeviceError> install_attitude(
    std::optional<RequestOptions> opts = std::nullopt);

  // --- function IO (issue #52): key 0x0019, PPS / GPS inputs and the two outputs.
  /// func_io_config_valid() → else kInvalidArgument with `key` before any I/O. Persisted on
  /// the LiDAR, not replayed on reconnect. IN1 = GPS is the input set_gps_time() (0x0202)
  /// complements; time_sync_status() reports the resulting synchronisation (#53).
  std::expected<SetResult, DeviceError> set_func_io_config(
    const FuncIoConfig & c, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<FuncIoConfig, DeviceError> func_io_config(
    std::optional<RequestOptions> opts = std::nullopt);

  // --- stored settings (issues #46 / #47 / #54): keys 0x0018, 0x001C, 0x002B, 0x0026.
  /// Thin wrappers over set<K>() / get<K>(). The setters return the LiDAR's answer; a value
  /// outside its enum (DetectMode > 1, an ImuSensorConfig field past its last enumerator) is
  /// kInvalidArgument with `key` before any I/O. Every accepted write is folded into the
  /// replayed HostSetup, so a reconnect restores it (see set_point_format()).
  std::expected<SetResult, DeviceError> set_detect_mode(
    DetectMode mode, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<DetectMode, DeviceError> detect_mode(
    std::optional<RequestOptions> opts = std::nullopt);
  // --- time synchronisation (issue #53): keys 0x8009–0x800C and command 0x0202.
  /// One 0x0101 for the four keys; a key missing or undecodable → kDecodeFailed with `key`.
  /// The pushed copy is in pushed_status(). Diagnostics only: the data path acts on each
  /// packet's own time_type (see TimestampPolicy), never on these keys.
  std::expected<TimeSyncStatus, DeviceError> time_sync_status(
    std::optional<RequestOptions> opts = std::nullopt);
  /// 0x0202: hands the host's GPS time of the last PPS edge to the LiDAR. Not validated;
  /// read time_sync_status() back to see whether the LiDAR took it (type becomes kGps).
  std::expected<void, DeviceError> set_gps_time(
    std::uint64_t pps_time_ns, std::optional<RequestOptions> opts = std::nullopt);

  // --- firmware log collection (issue #44): 0x0301 on the LiDAR's log port, pushes on
  // the Context's log socket. Layouts follow SDK2 and are unverified on hardware (#11).
  /// Writes key 0x0009 (this host, the Context's log port) through the session, then sends
  /// 0x0301 enable from the log socket and waits for its ACK (`opts`: timeout / attempts,
  /// the session defaults otherwise). Idempotent; replayed after a reconnect until a
  /// successful stop_firmware_log(). ret_code != 0 → kSession / kLidarRejected.
  std::expected<void, DeviceError> start_firmware_log(
    FirmwareLogType type = FirmwareLogType::kRealTime,
    std::optional<RequestOptions> opts = std::nullopt);
  /// 0x0301 disable. The destructor does not send it: the LiDAR keeps pushing.
  std::expected<void, DeviceError> stop_firmware_log(
    FirmwareLogType type = FirmwareLogType::kRealTime,
    std::optional<RequestOptions> opts = std::nullopt);

  /// Key 0x800E by inquire (#55); the pushed value is pushed_status()->lidar_diag_status
  /// and changes raise Event::kDiagChanged.
  std::expected<DiagStatus, DeviceError> diag_status(
    std::optional<RequestOptions> opts = std::nullopt);
  std::expected<SetResult, DeviceError> set_imu_enabled(
    bool on, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<bool, DeviceError> imu_enabled(std::optional<RequestOptions> opts = std::nullopt);
  /// Key 0x002B is absent on older firmware: the LiDAR then answers kLidarRejected with
  /// ret_code kParamNotSupport and error_key 0x002B (no distinct Kind).
  std::expected<SetResult, DeviceError> set_imu_sensor_config(
    const ImuSensorConfig & cfg, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<ImuSensorConfig, DeviceError> imu_sensor_config(
    std::optional<RequestOptions> opts = std::nullopt);
  /// Key 0x0026: with 1 the LiDAR keeps streaming through a GPS time rollback (wiki).
  std::expected<SetResult, DeviceError> set_time_filter(
    bool on, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<bool, DeviceError> time_filter(std::optional<RequestOptions> opts = std::nullopt);

  // --- LiDAR network config (issue #50): key 0x0004.
  /// lidar_ip_config_valid() → else kInvalidArgument with `key` before any I/O. The LiDAR
  /// answers a change with ret_code 0x21 (`reboot_required`, passed through as is): the new
  /// address is used after reboot(). Reconnection then finds the LiDAR by serial through
  /// discovery, which also unicasts to `{cfg.ip, reconnect.discovery_port}`, so a subnet
  /// change that broadcast does not reach is still recovered. Not rebooted automatically.
  std::expected<SetResult, DeviceError> set_lidar_ip_config(
    const LidarIpConfig & cfg, std::optional<RequestOptions> opts = std::nullopt);
  std::expected<LidarIpConfig, DeviceError> lidar_ip_config(
    std::optional<RequestOptions> opts = std::nullopt);

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
  /// The latest value of every status key seen in a 0x0102 push, `time_ns` = receive time
  /// of the last push; nullopt before the first push. A key a push omits keeps the value an
  /// earlier push carried, so a field is empty only if no push has carried it yet (#56).
  [[nodiscard]] std::optional<LidarStatus> pushed_status() const;
  /// cur_work_state from pushed_status(); nullopt before the first push.
  [[nodiscard]] std::optional<WorkState> work_state() const;
  /// hms_code slots from pushed_status() (all inactive before the first push).
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
