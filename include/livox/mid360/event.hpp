// SPDX-License-Identifier: Apache-2.0
// Public API skeleton (issue #9): notifications and errors of the device layer. Plain structs
// (enum + fields, strings only through to_string) so the phase-3 C ABI can mirror them.
// State/HMS events are produced by #7, disconnect/reconnect by #8.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "livox/mid360/export.hpp"
#include "livox/mid360/hms.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/session.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360 {

/// Counters of one Device, snapshot via Device::stats(). Monotonic since open().
struct DeviceStats {
  std::uint64_t packets = 0;              ///< data packets accepted (point cloud + IMU)
  std::uint64_t points = 0;               ///< samples delivered in Frames
  std::uint64_t frames = 0;               ///< Frames delivered
  std::uint64_t imu_samples = 0;          ///< ImuSamples delivered
  std::uint64_t bad_packets = 0;          ///< parse_data_packet failures
  std::uint64_t dropped_packets = 0;      ///< udp_cnt gaps (per source port)
  std::uint64_t queue_drops = 0;          ///< reserved for internal queues (#6)
  std::uint64_t last_packet_time_ns = 0;  ///< host receive time of the last packet, 0 = none
};

/// Counters of the shared receive side, snapshot via Context::stats().
struct ContextStats {
  std::uint64_t datagrams = 0;       ///< received on the three sockets
  std::uint64_t unknown_source = 0;  ///< dropped: source IP not registered by any Device
};

/// One notification. `kind` selects which fields are meaningful; the rest are default.
struct Event {
  enum class Kind : std::uint8_t {
    kStateChanged,  ///< `old_state` -> `new_state` seen in a 0x0102 push (#7)
    kHms,           ///< `hms` changed (#7)
    kDisconnected,  ///< no push for the configured interval / command failures (#8)
    kReconnected,   ///< session and host setup re-established (#8)
    kStats,         ///< periodic `stats` snapshot (#6)
  };
  Kind kind = Kind::kStats;
  std::uint64_t time_ns = 0;               ///< host time of the observation
  WorkState old_state = WorkState::kIdle;  ///< kStateChanged
  WorkState new_state = WorkState::kIdle;  ///< kStateChanged
  std::array<HmsCode, 8> hms{};            ///< kHms: key 0x8011 slots, inactive ones raw == 0
  DeviceStats stats;                       ///< kStats
};

[[nodiscard]] std::string_view to_string(Event::Kind kind) noexcept;
[[nodiscard]] std::string to_string(const Event& event);

/// Errors of Context / Device. Session-level failures are wrapped, not re-encoded.
struct DeviceError {
  enum class Kind : std::uint8_t {
    kSession,            ///< see `session`
    kInvalidArgument,    ///< option rejected before any I/O
    kInvalidState,       ///< e.g. callback set while running, command from a callback
    kAlreadyRegistered,  ///< another Device with the same IP is open on this Context
    kNotOpen,            ///< Device was closed (or never opened)
  };
  Kind kind = Kind::kSession;
  std::optional<SessionError> session;  ///< kSession only
};

[[nodiscard]] std::string_view to_string(DeviceError::Kind kind) noexcept;
[[nodiscard]] std::string to_string(const DeviceError& err);

}  // namespace livox::mid360
LIVOX_MID360_API_END
