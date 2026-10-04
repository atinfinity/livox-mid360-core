// SPDX-License-Identifier: Apache-2.0
/// @file
/// Session layer: discovery and synchronous command round-trips with one Mid-360.
///
/// Design (see issue #4):
///   * No threads. Every call blocks on the calling thread and drives Poller::wait.
///     A receive thread / Device abstraction is layered on top in phase 2 (#9).
///   * Requests are matched by seq_num + cmd_id + source endpoint; retries reuse the
///     same seq_num so that a delayed ACK to an earlier attempt still matches.
///   * Errors are std::expected<T, SessionError>; a LiDAR ret_code != 0 is final
///     (never retried), malformed datagrams count as "no response".
///   * cancel() may be called from any thread to abort a blocking call.
///
/// API stability: this header is expected to change when #9 settles the public API.
#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

#include "livox/mid360/export.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/transport.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Category of a SessionError.
enum class SessionErrorKind : std::uint8_t
{
  kTransport,        ///< socket error; see `transport`
  kTimeout,          ///< no matching ACK within all attempts
  kBadResponse,      ///< ACK arrived but its payload did not parse / serial mismatch
  kLidarRejected,    ///< ACK with ret_code != success; see `ret_code` / `error_key`
  kUnexpectedState,  ///< wait_for_state observed ERROR or UPGRADE; see `work_state`
  kCancelled,        ///< cancel() was called
  kInvalidArgument,  ///< rejected before sending; see `error_key` for the offending key
};

/// Short snake_case name, e.g. "timeout".
[[nodiscard]] std::string_view to_string(SessionErrorKind kind) noexcept;

/// Error of a Session call or of discover(). Only the fields named by `kind` are meaningful.
struct SessionError
{
  SessionErrorKind kind = SessionErrorKind::kTransport;  ///< what went wrong
  std::uint16_t cmd_id = 0;                 ///< command in flight, 0 when not applicable
  std::uint32_t attempts = 0;               ///< datagrams sent for this request
  std::optional<TransportError> transport;  ///< kTransport only
  std::optional<ParseError> parse;          ///< kBadResponse when the ACK payload did not parse
  RetCode ret_code = RetCode::kSuccess;     ///< kLidarRejected only
  /// kLidarRejected on 0x0100 (from the ACK) and 0x0101 (for kParamNotSupport the first
  /// requested key the ACK leaves out), kInvalidArgument
  std::uint16_t error_key = 0;
  std::optional<WorkState> work_state;      ///< kUnexpectedState only
};

/// Human readable one-line description.
[[nodiscard]] std::string to_string(const SessionError & err);

// ---------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------

/// One LiDAR that answered the 0x0000 discovery broadcast.
struct DiscoveredDevice
{
  std::string serial_number;   ///< from the ACK, NUL padding removed
  Ipv4 ip{};                   ///< lidar_ip from the ACK
  std::uint16_t cmd_port = 0;  ///< command port from the ACK
  std::uint8_t dev_type = 0;   ///< device type code from the ACK
  Endpoint from;               ///< where the ACK actually came from
};

/// Parameters of discover().
struct DiscoveryOptions
{
  /// Unicast targets (ip:port). Empty → broadcast to 255.255.255.255:kDiscoveryPort and
  /// wait for the whole timeout. With targets, returns as soon as all have answered.
  std::vector<Endpoint> targets;
  std::chrono::milliseconds timeout{1000};  ///< total time to wait for ACKs
  /// Local address to bind the discovery socket to (selects the interface for broadcast).
  Ipv4 bind_address{0, 0, 0, 0};
  /// Optional stop token: when a stop is requested the call returns kCancelled within about
  /// 100 ms (issue #8: the Device's reconnect thread must be interruptible).
  std::stop_token stop;
};

/// Send 0x0000 and collect ACKs. Duplicates (same serial number) are collapsed, first wins.
/// An empty result is not an error.
[[nodiscard]] std::expected<std::vector<DiscoveredDevice>, SessionError> discover(
  const DiscoveryOptions & options = {});

// ---------------------------------------------------------------------------
// Session
// ---------------------------------------------------------------------------

/// Timeout and retry policy of one request.
struct RequestOptions
{
  std::chrono::milliseconds timeout{500};  ///< per attempt
  std::uint32_t attempts = 3;              ///< total datagrams sent (1 = no retry)
};

/// Parameters of Session::connect().
struct SessionOptions
{
  std::uint16_t host_command_port = kDefaultHostCommandPort;  ///< 0 = ephemeral
  Ipv4 bind_address{0, 0, 0, 0};  ///< local address of the command socket
  /// On connect, read key 0x8000 and compare with the discovered serial number.
  bool verify_serial = true;
  RequestOptions request;  ///< default for calls that pass no RequestOptions
  std::chrono::milliseconds state_poll_interval{100};  ///< wait_for_state() poll period
  /// Optional stop token checked by every blocking call (like cancel(), but level-triggered
  /// and shared by connect() as well). A stop makes the call return kCancelled within about
  /// 100 ms.
  std::stop_token stop;
};

/// Counters of one Session, see Session::stats().
struct SessionStats
{
  std::uint64_t requests = 0;    ///< request() calls
  std::uint64_t retries = 0;     ///< datagrams sent beyond the first per request
  std::uint64_t timeouts = 0;    ///< requests that exhausted all attempts
  std::uint64_t late_acks = 0;   ///< ACKs that matched no pending request
  std::uint64_t bad_frames = 0;  ///< datagrams that failed parse_command_frame
};

/// Result of a raw request: the ACK payload, owned.
struct RawAck
{
  std::uint16_t cmd_id = 0;     ///< command the ACK answers
  std::uint32_t seq_num = 0;    ///< sequence number shared by the request and the ACK
  std::vector<std::byte> data;  ///< ACK payload (the frame's data field)
};

/// Result of 0x0101: owns the ACK bytes so that `values` stays valid after moves.
struct InquireResult
{
  RetCode ret_code = RetCode::kSuccess;  ///< ret_code of the ACK
  std::vector<KeyValue> values;          ///< views into `raw`
  std::vector<std::byte> raw;            ///< owned copy of the ACK payload

  InquireResult() = default;
  /// Moves keep `values` valid: the views follow the moved `raw` buffer.
  InquireResult(InquireResult &&) noexcept = default;
  /// See the move constructor.
  InquireResult & operator=(InquireResult &&) noexcept = default;
  InquireResult(const InquireResult &) = delete;
  InquireResult & operator=(const InquireResult &) = delete;
  ~InquireResult() = default;

  /// Value bytes of `key`, or nullopt when the ACK did not carry it.
  [[nodiscard]] std::optional<std::span<const std::byte>> get(Key key) const noexcept
  {
    return find_key(values, key);
  }
};

namespace detail
{
struct SessionAccess;  // src/session_detail.hpp: Device's access to the cancel flag
}  // namespace detail

/// Synchronous command channel to one LiDAR. Move-only.
class Session
{
public:
  /// Connect using a discovery result (cmd endpoint = ip:cmd_port).
  [[nodiscard]] static std::expected<Session, SessionError> connect(
    const DiscoveredDevice & device, const SessionOptions & options = {});
  /// Connect to a known command endpoint; the serial number is read from the device.
  [[nodiscard]] static std::expected<Session, SessionError> connect(
    Endpoint cmd_endpoint, const SessionOptions & options = {});

  /// Takes over the socket, the counters and a pending cancel request of `other`.
  Session(Session && other) noexcept;
  /// See the move constructor.
  Session & operator=(Session && other) noexcept;
  Session(const Session &) = delete;
  Session & operator=(const Session &) = delete;
  ~Session();

  /// Serial number of the connected LiDAR.
  [[nodiscard]] const std::string & serial_number() const noexcept { return serial_; }
  /// Command endpoint of the LiDAR (ip:cmd_port).
  [[nodiscard]] Endpoint lidar_endpoint() const noexcept { return lidar_; }
  /// Address the command socket is bound to.
  [[nodiscard]] Endpoint local_endpoint() const noexcept { return socket_.local_endpoint(); }
  /// Options passed to connect().
  [[nodiscard]] const SessionOptions & options() const noexcept { return options_; }
  /// Request counters since connect().
  [[nodiscard]] const SessionStats & stats() const noexcept { return stats_; }

  /// @name Raw request
  ///@{

  /// Send `cmd_id` with `data` and wait for the matching ACK.
  [[nodiscard]] std::expected<RawAck, SessionError> request(
    std::uint16_t cmd_id, std::span<const std::byte> data,
    std::optional<RequestOptions> opts = std::nullopt);

  ///@}

  /// @name Typed commands
  ///@{

  /// 0x0000 sent unicast to this LiDAR.
  [[nodiscard]] std::expected<DiscoveryAck, SessionError> discovery_ack(
    std::optional<RequestOptions> opts = std::nullopt);
  /// 0x0100. ret_code 0x21 (effective after reboot) is returned as success.
  [[nodiscard]] std::expected<ParamConfigAck, SessionError> configure(
    std::span<const KeyValue> kvs, std::optional<RequestOptions> opts = std::nullopt);
  /// 0x0101.
  [[nodiscard]] std::expected<InquireResult, SessionError> inquire(
    std::span<const std::uint16_t> keys, std::optional<RequestOptions> opts = std::nullopt);
  /// 0x0101 with typed keys.
  [[nodiscard]] std::expected<InquireResult, SessionError> inquire(
    std::span<const Key> keys, std::optional<RequestOptions> opts = std::nullopt);
  /// 0x0200. The LiDAR goes silent after the ACK.
  [[nodiscard]] std::expected<SimpleAck, SessionError> reboot(
    std::uint16_t timeout_ms = 100, std::optional<RequestOptions> opts = std::nullopt);
  /// 0x0201.
  [[nodiscard]] std::expected<SimpleAck, SessionError> factory_reset(
    std::optional<RequestOptions> opts = std::nullopt);
  /// 0x0202.
  [[nodiscard]] std::expected<SimpleAck, SessionError> set_gps_time(
    std::uint64_t pps_time_ns, std::optional<RequestOptions> opts = std::nullopt);

  ///@}

  /// @name Work state
  ///@{

  /// Read key 0x8006.
  [[nodiscard]] std::expected<WorkState, SessionError> work_state(
    std::optional<RequestOptions> opts = std::nullopt);
  /// Poll 0x8006 until `target` is observed. ERROR / UPGRADE end the wait with
  /// kUnexpectedState; other states are treated as transitional.
  [[nodiscard]] std::expected<void, SessionError> wait_for_state(
    WorkState target, std::chrono::milliseconds timeout);

  ///@}

  /// Abort a blocking call from another thread; it returns kCancelled. The flag is
  /// consumed by the aborted call (or by the next call if none is in progress).
  void cancel() noexcept;

private:
  friend struct detail::SessionAccess;

  Session() = default;
  [[nodiscard]] std::expected<void, SessionError> open(
    const SessionOptions & options, Endpoint lidar);
  [[nodiscard]] std::expected<void, SessionError> read_serial();

  UdpSocket socket_;
  Poller poller_;
  Endpoint lidar_;
  SessionOptions options_;
  SessionStats stats_;
  std::string serial_;
  std::uint32_t next_seq_ = 1;
  std::atomic<bool> cancel_{false};
};

}  // namespace livox::mid360
LIVOX_MID360_API_END
