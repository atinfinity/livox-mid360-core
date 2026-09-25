// SPDX-License-Identifier: Apache-2.0
// Internal helpers behind Session / discover(): the pure "datagram -> result" steps, kept
// separate so that the fuzz targets and unit tests can drive them without a socket.
//
// NOT a stable API. This header is not installed; only src/, tests/ and tests/fuzz/ include
// it. The symbols are exported so the fuzzers can link against the shared library.
#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

#include "livox/mid360/export.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/session.hpp"
#include "livox/mid360/transport.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360::detail {

/// Why a datagram is not the ACK Session::request() is waiting for.
enum class AckMismatch : std::uint8_t {
  kBadFrame,  ///< parse_command_frame rejected it (counts as SessionStats::bad_frames)
  kNotAck,    ///< a valid frame that is not an ACK (push / request); ignored silently
  kLate,      ///< an ACK with another seq / cmd_id / source IP (counts as late_acks)
};

/// Classify one received datagram against the request in flight.
[[nodiscard]] std::expected<CommandFrameView, AckMismatch> match_ack(
    std::span<const std::byte> datagram, const Endpoint& from, std::uint32_t seq,
    std::uint16_t cmd_id, const Ipv4& lidar_ip) noexcept;

/// discover(): datagram -> device, nullopt for anything that is not a successful 0x0000 ACK.
[[nodiscard]] std::optional<DiscoveredDevice> parse_discovered_device(
    std::span<const std::byte> datagram, const Endpoint& from);

// Typed post-processing of a matched ACK. ret_code != success maps to kLidarRejected,
// except 0x21 (effective after reboot) on 0x0100, which is success.
[[nodiscard]] std::expected<DiscoveryAck, SessionError> to_discovery_ack(const RawAck& ack);
[[nodiscard]] std::expected<ParamConfigAck, SessionError> to_config_ack(const RawAck& ack);
/// Takes the ACK by value; the returned views point into InquireResult::raw (moved from ack.data).
[[nodiscard]] std::expected<InquireResult, SessionError> to_inquire_result(RawAck ack);
[[nodiscard]] std::expected<SimpleAck, SessionError> to_simple_ack(const RawAck& ack);
/// Reads key 0x8006 out of an inquire result.
[[nodiscard]] std::expected<WorkState, SessionError> to_work_state(const InquireResult& result);

}  // namespace livox::mid360::detail
LIVOX_MID360_API_END
