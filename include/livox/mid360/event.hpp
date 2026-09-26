// SPDX-License-Identifier: Apache-2.0
// Public API (issue #9): notifications and errors of the device layer. Plain structs
// (enum + fields, strings only through to_string) so the phase-3 C ABI can mirror them.
// State/HMS events come from 0x0102 pushes (#7), disconnect/reconnect from #8.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "livox/mid360/export.hpp"
#include "livox/mid360/hms.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/session.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

/// Counters of one Device, snapshot via Device::stats(). Monotonic since open().
struct DeviceStats
{
  std::uint64_t packets = 0;              ///< data packets accepted (point cloud + IMU)
  std::uint64_t points = 0;               ///< samples delivered in Frames
  std::uint64_t frames = 0;               ///< Frames delivered
  std::uint64_t imu_samples = 0;          ///< ImuSamples delivered
  std::uint64_t bad_packets = 0;          ///< parse_data_packet failures
  std::uint64_t dropped_packets = 0;      ///< udp_cnt gaps (per source port)
  std::uint64_t reordered = 0;            ///< udp_cnt went backwards (duplicate / reordered)
  std::uint64_t queue_drops = 0;          ///< reserved for internal queues
  std::uint64_t frame_cnt_fallback = 0;   ///< times frame_cnt mode fell back to the time window
  std::uint64_t last_packet_time_ns = 0;  ///< host receive time of the last packet, 0 = none
  std::uint64_t pushes = 0;               ///< 0x0102 pushes accepted
  std::uint64_t last_push_time_ns = 0;    ///< host receive time of the last push, 0 = none
  std::uint64_t disconnects = 0;          ///< kDisconnected events raised
  std::uint64_t reconnects = 0;           ///< kReconnected events raised
  std::int64_t time_offset_ns = 0;        ///< kHostOffsetOnce: host - LiDAR, once measured
  bool time_offset_valid = false;
};

/// Counters of the shared receive side, snapshot via Context::stats().
struct ContextStats
{
  std::uint64_t datagrams = 0;       ///< received on the three sockets
  std::uint64_t unknown_source = 0;  ///< dropped: source IP not registered by any Device
};

/// Why a Device left the connected state (Event::reason, issue #8).
enum class DisconnectReason : std::uint8_t
{
  kNone = 0,
  kPushTimeout,      ///< no 0x0102 push for ReconnectOptions::push_timeout
  kCommandTimeout,   ///< a command timed out while the push was already stale
  kRebootRequested,  ///< Device::reboot() was acknowledged
  kUser,             ///< Device::disconnect()
};

/// One notification. `kind` selects which fields are meaningful; the rest are default.
struct Event
{
  enum class Kind : std::uint8_t
  {
    kStateChanged,  ///< `old_state` -> `new_state` seen in a 0x0102 push
    kHms,           ///< the set of active `hms` codes changed; see `hms_level`
    kDisconnected,  ///< see `reason` (#8); commands fail with kDisconnected until kReconnected
    kReconnected,   ///< session, host setup and sampling re-established after `attempts`
    kStats,         ///< periodic `stats` snapshot (#6)
  };
  Kind kind = Kind::kStats;
  std::uint64_t time_ns = 0;                          ///< host time of the observation
  DisconnectReason reason = DisconnectReason::kNone;  ///< kDisconnected / kReconnected
  std::uint32_t attempts = 0;              ///< kReconnected: attempts including the successful one
  WorkState old_state = WorkState::kIdle;  ///< kStateChanged
  WorkState new_state = WorkState::kIdle;  ///< kStateChanged
  std::array<HmsCode, 8> hms{};            ///< kHms: key 0x8011 slots, inactive ones raw == 0
  HmsLevel hms_level = HmsLevel::kNone;    ///< kHms: highest level among the active slots
  DeviceStats stats;                       ///< kStats
};

[[nodiscard]] std::string_view to_string(Event::Kind kind) noexcept;
[[nodiscard]] std::string_view to_string(DisconnectReason reason) noexcept;
[[nodiscard]] std::string to_string(const Event & event);

/// Errors of Context / Device. Session-level failures are wrapped, not re-encoded.
struct DeviceError
{
  enum class Kind : std::uint8_t
  {
    kSession,            ///< see `session`
    kInvalidArgument,    ///< option rejected before any I/O
    kInvalidState,       ///< e.g. callback set while running, command from a callback
    kAlreadyRegistered,  ///< another Device with the same IP is open on this Context
    kNotOpen,            ///< Device was closed (or never opened)
    kDisconnected,       ///< command refused: the Device is between kDisconnected and kReconnected
    kDecodeFailed,       ///< get<K>(): the ACK lacked `key` or its value did not decode (#57)
  };
  Kind kind = Kind::kSession;
  std::optional<SessionError> session;  ///< kSession only
  std::optional<Key> key;               ///< kDecodeFailed only
};

[[nodiscard]] std::string_view to_string(DeviceError::Kind kind) noexcept;
[[nodiscard]] std::string to_string(const DeviceError & err);

}  // namespace livox::mid360
LIVOX_MID360_API_END
