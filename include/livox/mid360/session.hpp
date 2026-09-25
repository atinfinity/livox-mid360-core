// SPDX-License-Identifier: Apache-2.0
// Session layer: discovery and synchronous command round-trips with one Mid-360.
//
// Design (see issue #4):
//   * No threads. Every call blocks on the calling thread and drives Poller::wait.
//     A receive thread / Device abstraction is layered on top in phase 2 (#9).
//   * Requests are matched by seq_num + cmd_id + source endpoint; retries reuse the
//     same seq_num so that a delayed ACK to an earlier attempt still matches.
//   * Errors are std::expected<T, SessionError>; a LiDAR ret_code != 0 is final
//     (never retried), malformed datagrams count as "no response".
//   * cancel() may be called from any thread to abort a blocking call.
//
// API stability: this header is expected to change when #9 settles the public API.
#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "livox/mid360/export.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/transport.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360 {

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

enum class SessionErrorKind : std::uint8_t {
  kTransport,        ///< socket error; see `transport`
  kTimeout,          ///< no matching ACK within all attempts
  kBadResponse,      ///< ACK arrived but its payload did not parse / serial mismatch
  kLidarRejected,    ///< ACK with ret_code != success; see `ret_code` / `error_key`
  kUnexpectedState,  ///< wait_for_state observed ERROR or UPGRADE; see `work_state`
  kCancelled,        ///< cancel() was called
};

[[nodiscard]] std::string_view to_string(SessionErrorKind kind) noexcept;

struct SessionError {
  SessionErrorKind kind = SessionErrorKind::kTransport;
  std::uint16_t cmd_id = 0;    ///< command in flight, 0 when not applicable
  std::uint32_t attempts = 0;  ///< datagrams sent for this request
  std::optional<TransportError> transport;
  std::optional<ParseError> parse;
  RetCode ret_code = RetCode::kSuccess;  ///< kLidarRejected only
  std::uint16_t error_key = 0;           ///< kLidarRejected on 0x0100 / 0x0101
  std::optional<WorkState> work_state;   ///< kUnexpectedState only
};

/// Human readable one-line description.
[[nodiscard]] std::string to_string(const SessionError& err);

// ---------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------

struct DiscoveredDevice {
  std::string serial_number;
  Ipv4 ip{};                   ///< lidar_ip from the ACK
  std::uint16_t cmd_port = 0;  ///< command port from the ACK
  std::uint8_t dev_type = 0;
  Endpoint from;  ///< where the ACK actually came from
};

struct DiscoveryOptions {
  /// Unicast targets (ip:port). Empty → broadcast to 255.255.255.255:kDiscoveryPort and
  /// wait for the whole timeout. With targets, returns as soon as all have answered.
  std::vector<Endpoint> targets;
  std::chrono::milliseconds timeout{1000};
  /// Local address to bind the discovery socket to (selects the interface for broadcast).
  Ipv4 bind_address{0, 0, 0, 0};
};

/// Send 0x0000 and collect ACKs. Duplicates (same serial number) are collapsed, first wins.
/// An empty result is not an error.
[[nodiscard]] std::expected<std::vector<DiscoveredDevice>, SessionError> discover(
    const DiscoveryOptions& options = {});

// ---------------------------------------------------------------------------
// Session
// ---------------------------------------------------------------------------

struct RequestOptions {
  std::chrono::milliseconds timeout{500};  ///< per attempt
  std::uint32_t attempts = 3;              ///< total datagrams sent (1 = no retry)
};

struct SessionOptions {
  std::uint16_t host_command_port = kDefaultHostCommandPort;  ///< 0 = ephemeral
  Ipv4 bind_address{0, 0, 0, 0};
  /// On connect, read key 0x8000 and compare with the discovered serial number.
  bool verify_serial = true;
  RequestOptions request;
  std::chrono::milliseconds state_poll_interval{100};
};

struct SessionStats {
  std::uint64_t requests = 0;    ///< request() calls
  std::uint64_t retries = 0;     ///< datagrams sent beyond the first per request
  std::uint64_t timeouts = 0;    ///< requests that exhausted all attempts
  std::uint64_t late_acks = 0;   ///< ACKs that matched no pending request
  std::uint64_t bad_frames = 0;  ///< datagrams that failed parse_command_frame
};

/// Result of a raw request: the ACK payload, owned.
struct RawAck {
  std::uint16_t cmd_id = 0;
  std::uint32_t seq_num = 0;
  std::vector<std::byte> data;
};

/// Result of 0x0101: owns the ACK bytes so that `values` stays valid after moves.
struct InquireResult {
  RetCode ret_code = RetCode::kSuccess;
  std::vector<KeyValue> values;  ///< views into `raw`
  std::vector<std::byte> raw;

  InquireResult() = default;
  InquireResult(InquireResult&&) noexcept = default;
  InquireResult& operator=(InquireResult&&) noexcept = default;
  InquireResult(const InquireResult&) = delete;
  InquireResult& operator=(const InquireResult&) = delete;
  ~InquireResult() = default;

  [[nodiscard]] std::optional<std::span<const std::byte>> get(Key key) const noexcept {
    return find_key(values, key);
  }
};

/// Synchronous command channel to one LiDAR. Move-only.
class Session {
 public:
  /// Connect using a discovery result (cmd endpoint = ip:cmd_port).
  [[nodiscard]] static std::expected<Session, SessionError> connect(
      const DiscoveredDevice& device, const SessionOptions& options = {});
  /// Connect to a known command endpoint; the serial number is read from the device.
  [[nodiscard]] static std::expected<Session, SessionError> connect(
      Endpoint cmd_endpoint, const SessionOptions& options = {});

  Session(Session&& other) noexcept;
  Session& operator=(Session&& other) noexcept;
  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;
  ~Session();

  [[nodiscard]] const std::string& serial_number() const noexcept { return serial_; }
  [[nodiscard]] Endpoint lidar_endpoint() const noexcept { return lidar_; }
  [[nodiscard]] Endpoint local_endpoint() const noexcept { return socket_.local_endpoint(); }
  [[nodiscard]] const SessionOptions& options() const noexcept { return options_; }
  [[nodiscard]] const SessionStats& stats() const noexcept { return stats_; }

  // -- raw ---------------------------------------------------------------
  /// Send `cmd_id` with `data` and wait for the matching ACK.
  [[nodiscard]] std::expected<RawAck, SessionError> request(
      std::uint16_t cmd_id, std::span<const std::byte> data,
      std::optional<RequestOptions> opts = std::nullopt);

  // -- typed -------------------------------------------------------------
  [[nodiscard]] std::expected<DiscoveryAck, SessionError> discovery_ack(
      std::optional<RequestOptions> opts = std::nullopt);
  /// 0x0100. ret_code 0x21 (effective after reboot) is returned as success.
  [[nodiscard]] std::expected<ParamConfigAck, SessionError> configure(
      std::span<const KeyValue> kvs, std::optional<RequestOptions> opts = std::nullopt);
  /// 0x0101.
  [[nodiscard]] std::expected<InquireResult, SessionError> inquire(
      std::span<const std::uint16_t> keys, std::optional<RequestOptions> opts = std::nullopt);
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

  // -- state -------------------------------------------------------------
  /// Read key 0x8006.
  [[nodiscard]] std::expected<WorkState, SessionError> work_state(
      std::optional<RequestOptions> opts = std::nullopt);
  /// Poll 0x8006 until `target` is observed. ERROR / UPGRADE end the wait with
  /// kUnexpectedState; other states are treated as transitional.
  [[nodiscard]] std::expected<void, SessionError> wait_for_state(WorkState target,
                                                                 std::chrono::milliseconds timeout);

  /// Abort a blocking call from another thread; it returns kCancelled. The flag is
  /// consumed by the aborted call (or by the next call if none is in progress).
  void cancel() noexcept;

 private:
  Session() = default;
  [[nodiscard]] std::expected<void, SessionError> open(const SessionOptions& options,
                                                       Endpoint lidar);
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
