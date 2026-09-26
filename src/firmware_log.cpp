// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/firmware_log.hpp"

#include <algorithm>
#include <format>

#include "livox/mid360/bytes.hpp"

namespace livox::mid360
{

using bytes::read_le;
using bytes::write_le;

std::string_view to_string(FirmwareLogType t) noexcept
{
  switch (t) {
    case FirmwareLogType::kRealTime:
      return "realtime";
    case FirmwareLogType::kException:
      return "exception";
  }
  return "unknown";
}

std::string to_string(const FirmwareLogPushHeader & h)
{
  return std::format(
    "type={} file={}/{} flags={:#04x}{}{}{} trans={} len={}", to_string(h.log_type), h.file_index,
    h.file_num, h.flags.raw, h.flags.ack_requested() ? " ack" : "",
    h.flags.file_begin() ? " begin" : "", h.flags.file_end() ? " end" : "", h.trans_index,
    h.data_length);
}

std::array<std::byte, 2> encode_firmware_log_control(const FirmwareLogControlRequest & req) noexcept
{
  return {
    std::byte{static_cast<std::uint8_t>(req.log_type)}, static_cast<std::byte>(req.enable ? 1 : 0)};
}

std::expected<FirmwareLogControlRequest, ParseError> parse_firmware_log_control(
  std::span<const std::byte> data) noexcept
{
  if (data.size() < 2) {
    return std::unexpected(ParseError::kTooShort);
  }
  return FirmwareLogControlRequest{
    .log_type = static_cast<FirmwareLogType>(read_le<std::uint8_t>(data, 0)),
    .enable = read_le<std::uint8_t>(data, 1) != 0};
}

std::expected<FirmwareLogPushView, ParseError> parse_firmware_log_push(
  std::span<const std::byte> data) noexcept
{
  if (data.size() < kFirmwareLogPushHeaderSize) {
    return std::unexpected(ParseError::kTooShort);
  }
  FirmwareLogPushHeader h;
  h.log_type = static_cast<FirmwareLogType>(read_le<std::uint8_t>(data, 0));
  h.file_index = read_le<std::uint8_t>(data, 1);
  h.file_num = read_le<std::uint8_t>(data, 2);
  h.flags.raw = read_le<std::uint8_t>(data, 3);
  h.timestamp = read_le<std::uint32_t>(data, 4);
  h.rsvd = read_le<std::uint16_t>(data, 8);
  h.trans_index = read_le<std::uint32_t>(data, 10);
  h.data_length = read_le<std::uint16_t>(data, 14);
  const std::size_t present = data.size() - kFirmwareLogPushHeaderSize;
  if (h.data_length > present) {
    return std::unexpected(ParseError::kTruncated);
  }
  if (h.data_length < present) {
    return std::unexpected(ParseError::kLengthMismatch);
  }
  return FirmwareLogPushView{h, data.subspan(kFirmwareLogPushHeaderSize, h.data_length)};
}

std::expected<std::vector<std::byte>, EncodeError> encode_firmware_log_push(
  const FirmwareLogPushHeader & header, std::span<const std::byte> data)
{
  if (data.size() > kFirmwareLogPushDataMaxSize) {
    return std::unexpected(EncodeError::kDataTooLarge);
  }
  std::vector<std::byte> out(kFirmwareLogPushHeaderSize + data.size());
  write_le<std::uint8_t>(out, 0, static_cast<std::uint8_t>(header.log_type));
  write_le<std::uint8_t>(out, 1, header.file_index);
  write_le<std::uint8_t>(out, 2, header.file_num);
  write_le<std::uint8_t>(out, 3, header.flags.raw);
  write_le<std::uint32_t>(out, 4, header.timestamp);
  write_le<std::uint16_t>(out, 8, header.rsvd);
  write_le<std::uint32_t>(out, 10, header.trans_index);
  write_le<std::uint16_t>(out, 14, static_cast<std::uint16_t>(data.size()));
  std::ranges::copy(data, out.begin() + static_cast<std::ptrdiff_t>(kFirmwareLogPushHeaderSize));
  return out;
}

std::array<std::byte, kFirmwareLogPushAckSize> encode_firmware_log_push_ack(
  const FirmwareLogPushAck & ack) noexcept
{
  std::array<std::byte, kFirmwareLogPushAckSize> out{};
  write_le<std::uint8_t>(out, 0, static_cast<std::uint8_t>(ack.ret_code));
  write_le<std::uint8_t>(out, 1, static_cast<std::uint8_t>(ack.log_type));
  write_le<std::uint8_t>(out, 2, ack.file_index);
  write_le<std::uint32_t>(out, 3, ack.trans_index);
  return out;
}

std::expected<FirmwareLogPushAck, ParseError> parse_firmware_log_push_ack(
  std::span<const std::byte> data) noexcept
{
  if (data.size() < kFirmwareLogPushAckSize) {
    return std::unexpected(ParseError::kTooShort);
  }
  return FirmwareLogPushAck{
    .ret_code = static_cast<RetCode>(read_le<std::uint8_t>(data, 0)),
    .log_type = static_cast<FirmwareLogType>(read_le<std::uint8_t>(data, 1)),
    .file_index = read_le<std::uint8_t>(data, 2),
    .trans_index = read_le<std::uint32_t>(data, 3)};
}

}  // namespace livox::mid360
