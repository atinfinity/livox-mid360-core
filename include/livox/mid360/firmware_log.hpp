// SPDX-License-Identifier: Apache-2.0
// Firmware log collection (issue #44): the 0x03xx command group on the LiDAR's log port
// (56500). The layouts follow Livox-SDK2 (sdk_core/comm/define.h) and are unverified on
// hardware (#11). Unrelated to the SDK's own diagnostic logging (log.hpp).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "livox/mid360/export.hpp"
#include "livox/mid360/protocol.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

/// `log_type` of 0x0300 / 0x0301. SDK2 states that only the real-time log is supported.
enum class FirmwareLogType : std::uint8_t
{
  kRealTime = 0,
  kException = 1,  ///< [unverified] accepted by the API, untested on hardware (#11)
};

/// `flag` bits of a 0x0300 push (SDK2 logger_manager.cpp).
struct FirmwareLogFlags
{
  std::uint8_t raw = 0;
  [[nodiscard]] constexpr bool ack_requested() const noexcept { return (raw & 0x01) != 0; }
  [[nodiscard]] constexpr bool file_begin() const noexcept { return (raw & 0x02) != 0; }
  [[nodiscard]] constexpr bool file_end() const noexcept { return (raw & 0x04) != 0; }
};

inline constexpr std::uint8_t kFirmwareLogFlagAck = 0x01;
inline constexpr std::uint8_t kFirmwareLogFlagBegin = 0x02;
inline constexpr std::uint8_t kFirmwareLogFlagEnd = 0x04;

inline constexpr std::size_t kFirmwareLogPushHeaderSize = 16;
inline constexpr std::size_t kFirmwareLogPushDataMaxSize =
  kCommandDataMaxSize - kFirmwareLogPushHeaderSize;

/// 0x0301 request: start (`enable`) or stop collecting `log_type`.
struct FirmwareLogControlRequest
{
  FirmwareLogType log_type = FirmwareLogType::kRealTime;
  bool enable = true;
};

/// 0x0300 push header (16 bytes, little-endian) followed by `data_length` bytes.
struct FirmwareLogPushHeader
{
  FirmwareLogType log_type = FirmwareLogType::kRealTime;
  std::uint8_t file_index = 0;  ///< which file of the transfer; a begin flag opens a new one
  std::uint8_t file_num = 0;    ///< [unverified] total files; SDK2 never reads it
  FirmwareLogFlags flags;
  std::uint32_t timestamp = 0;  ///< [unverified] raw, unit unknown; SDK2 never reads it
  std::uint16_t rsvd = 0;
  std::uint32_t trans_index = 0;  ///< +1 per packet inside one file
  std::uint16_t data_length = 0;
};

/// A validated 0x0300 push: header plus a view over its data.
struct FirmwareLogPushView
{
  FirmwareLogPushHeader header;
  std::span<const std::byte> data;
};

/// Host → LiDAR answer to a push with `ack_requested()`, sent as a cmd-type 0x0300 frame.
struct FirmwareLogPushAck
{
  RetCode ret_code = RetCode::kSuccess;
  FirmwareLogType log_type = FirmwareLogType::kRealTime;
  std::uint8_t file_index = 0;
  std::uint32_t trans_index = 0;
};

inline constexpr std::size_t kFirmwareLogPushAckSize = 7;

/// What Device hands to a FirmwareLogCallback (receive thread; `data` is valid only during
/// the callback, copy it to keep it). Begin / end packets are delivered too, usually with
/// empty data, so the application can split files.
struct FirmwareLogChunk
{
  FirmwareLogPushHeader header;
  std::uint64_t host_receive_time_ns = 0;
  std::span<const std::byte> data;
};

[[nodiscard]] std::string_view to_string(FirmwareLogType t) noexcept;
[[nodiscard]] std::string to_string(const FirmwareLogPushHeader & h);

/// 0x0301 payload: `{log_type, enable}`.
[[nodiscard]] std::array<std::byte, 2> encode_firmware_log_control(
  const FirmwareLogControlRequest & req) noexcept;
[[nodiscard]] std::expected<FirmwareLogControlRequest, ParseError> parse_firmware_log_control(
  std::span<const std::byte> data) noexcept;

/// 0x0300 payload: header + data. `data_length` must match the bytes present (kTruncated /
/// kLengthMismatch otherwise); `log_type` is not range-checked (unknown types pass through).
[[nodiscard]] std::expected<FirmwareLogPushView, ParseError> parse_firmware_log_push(
  std::span<const std::byte> data) noexcept;
/// Serialises a push payload (header + data); nullopt-free: `data` above the frame limit is
/// an EncodeError::kDataTooLarge.
[[nodiscard]] std::expected<std::vector<std::byte>, EncodeError> encode_firmware_log_push(
  const FirmwareLogPushHeader & header, std::span<const std::byte> data);

[[nodiscard]] std::array<std::byte, kFirmwareLogPushAckSize> encode_firmware_log_push_ack(
  const FirmwareLogPushAck & ack) noexcept;
[[nodiscard]] std::expected<FirmwareLogPushAck, ParseError> parse_firmware_log_push_ack(
  std::span<const std::byte> data) noexcept;

}  // namespace livox::mid360
LIVOX_MID360_API_END
