// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/protocol.hpp"

#include <algorithm>
#include <cstring>

#include "livox/mid360/bytes.hpp"
#include "livox/mid360/crc.hpp"

namespace livox::mid360
{

using bytes::read_le;
using bytes::write_le;

// ---------------------------------------------------------------------------
// to_string
// ---------------------------------------------------------------------------
std::string_view to_string(RetCode c) noexcept
{
  switch (c) {
    case RetCode::kSuccess:
      return "SUCCESS";
    case RetCode::kFailure:
      return "FAILURE";
    case RetCode::kNotPermitNow:
      return "NOT_PERMIT_NOW";
    case RetCode::kOutOfRange:
      return "OUT_OF_RANGE";
    case RetCode::kParamNotSupport:
      return "PARAM_NOTSUPPORT";
    case RetCode::kParamRebootEffect:
      return "PARAM_REBOOT_EFFECT";
    case RetCode::kParamReadOnly:
      return "PARAM_RD_ONLY";
    case RetCode::kParamInvalidLen:
      return "PARAM_INVALID_LEN";
    case RetCode::kParamKeyNumErr:
      return "PARAM_KEY_NUM_ERR";
    case RetCode::kUpgradePubKeyError:
      return "UPGRADE_PUB_KEY_ERROR";
    case RetCode::kUpgradeDigestError:
      return "UPGRADE_DIGEST_ERROR";
    case RetCode::kUpgradeFwTypeError:
      return "UPGRADE_FW_TYPE_ERROR";
    case RetCode::kUpgradeFwOutOfRange:
      return "UPGRADE_FW_OUT_OF_RANGE";
    case RetCode::kUpgradeFwErasing:
      return "UPGRADE_FW_ERASING";
  }
  return "UNKNOWN";
}

std::string_view to_string(WorkState s) noexcept
{
  switch (s) {
    case WorkState::kSampling:
      return "SAMPLING";
    case WorkState::kIdle:
      return "IDLE";
    case WorkState::kError:
      return "ERROR";
    case WorkState::kSelfCheck:
      return "SELFCHECK";
    case WorkState::kMotorStartup:
      return "MOTORSTARTUP";
    case WorkState::kUpgrade:
      return "UPGRADE";
    case WorkState::kReady:
      return "READY";
  }
  return "UNKNOWN";
}

std::string_view to_string(DataType t) noexcept
{
  switch (t) {
    case DataType::kImu:
      return "IMU";
    case DataType::kCartesian32:
      return "CARTESIAN32";
    case DataType::kCartesian16:
      return "CARTESIAN16";
    case DataType::kSpherical:
      return "SPHERICAL";
  }
  return "UNKNOWN";
}

std::string_view to_string(TagConfidence c) noexcept
{
  switch (c) {
    case TagConfidence::kHigh:
      return "high";
    case TagConfidence::kMedium:
      return "medium";
    case TagConfidence::kLow:
      return "low";
    case TagConfidence::kReserved:
      return "reserved";
  }
  return "unknown";
}

std::string to_string(const TagInfo & t)
{
  std::string out = "glue=" + std::string(to_string(t.adjacent_glue)) +
                    " particles=" + std::string(to_string(t.particles)) +
                    " other=" + std::string(to_string(t.other));
  if (t.reserved != TagConfidence::kHigh) {
    out += " reserved=" + std::string(to_string(t.reserved));
  }
  return out;
}

std::string_view to_string(TimeType t) noexcept
{
  switch (t) {
    case TimeType::kNoSync:
      return "NO_SYNC";
    case TimeType::kPtp:
      return "PTP";
    case TimeType::kGps:
      return "GPS";
  }
  return "UNKNOWN";
}

std::string_view to_string(ParseError e) noexcept
{
  switch (e) {
    case ParseError::kTooShort:
      return "too short";
    case ParseError::kBadSof:
      return "bad SOF";
    case ParseError::kBadVersion:
      return "bad version";
    case ParseError::kLengthMismatch:
      return "length mismatch";
    case ParseError::kBadCrc16:
      return "bad CRC16";
    case ParseError::kBadCrc32:
      return "bad CRC32";
    case ParseError::kUnknownDataType:
      return "unknown data_type";
    case ParseError::kBadDotNum:
      return "bad dot_num";
    case ParseError::kTruncated:
      return "truncated";
    case ParseError::kKeyNumMismatch:
      return "key_num mismatch";
  }
  return "unknown";
}

std::string_view to_string(CmdId id) noexcept
{
  switch (id) {
    case CmdId::kDiscovery:
      return "DISCOVERY";
    case CmdId::kParamConfig:
      return "PARAM_CONFIG";
    case CmdId::kParamInquire:
      return "PARAM_INQUIRE";
    case CmdId::kInfoPush:
      return "INFO_PUSH";
    case CmdId::kReboot:
      return "REBOOT";
    case CmdId::kFactoryReset:
      return "FACTORY_RESET";
    case CmdId::kSetGpsTimestamp:
      return "SET_GPS_TIMESTAMP";
    case CmdId::kPushLog:
      return "PUSH_LOG";
    case CmdId::kCollectionLog:
      return "COLLECTION_LOG";
  }
  return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// Command frame
// ---------------------------------------------------------------------------
std::expected<CommandFrameView, ParseError> parse_command_frame(
  std::span<const std::byte> frame) noexcept
{
  if (frame.size() < kCommandHeaderSize) {
    return std::unexpected(ParseError::kTooShort);
  }
  if (read_le<std::uint8_t>(frame, 0) != kCommandSof) {
    return std::unexpected(ParseError::kBadSof);
  }
  if (read_le<std::uint8_t>(frame, 1) != kProtocolVersion) {
    return std::unexpected(ParseError::kBadVersion);
  }
  CommandHeader h;
  h.length = read_le<std::uint16_t>(frame, 2);
  if (h.length < kCommandHeaderSize || h.length > kCommandFrameMaxSize || h.length > frame.size()) {
    return std::unexpected(ParseError::kLengthMismatch);
  }
  h.seq_num = read_le<std::uint32_t>(frame, 4);
  h.cmd_id = read_le<std::uint16_t>(frame, 8);
  h.cmd_type = static_cast<CmdType>(read_le<std::uint8_t>(frame, 10));
  h.sender_type = static_cast<SenderType>(read_le<std::uint8_t>(frame, 11));
  std::memcpy(h.resv.data(), frame.data() + 12, 6);
  h.crc16 = read_le<std::uint16_t>(frame, 18);
  h.crc32 = read_le<std::uint32_t>(frame, 20);

  if (crc::crc16_ccitt_false(frame.first(18)) != h.crc16) {
    return std::unexpected(ParseError::kBadCrc16);
  }
  const auto data = frame.subspan(kCommandHeaderSize, h.length - kCommandHeaderSize);
  const std::uint32_t expected_crc32 = data.empty() ? 0u : crc::crc32(data);
  if (expected_crc32 != h.crc32) {
    return std::unexpected(ParseError::kBadCrc32);
  }
  return CommandFrameView{h, data};
}

std::expected<std::size_t, EncodeError> encode_command_frame(
  std::span<std::byte> out, const CommandFrameSpec & spec) noexcept
{
  if (spec.data.size() > kCommandDataMaxSize) {
    return std::unexpected(EncodeError::kDataTooLarge);
  }
  const std::size_t total = command_frame_size(spec.data.size());
  if (out.size() < total) {
    return std::unexpected(EncodeError::kBufferTooSmall);
  }

  write_le<std::uint8_t>(out, 0, kCommandSof);
  write_le<std::uint8_t>(out, 1, kProtocolVersion);
  write_le<std::uint16_t>(out, 2, static_cast<std::uint16_t>(total));
  write_le<std::uint32_t>(out, 4, spec.seq_num);
  write_le<std::uint16_t>(out, 8, spec.cmd_id);
  write_le<std::uint8_t>(out, 10, static_cast<std::uint8_t>(spec.cmd_type));
  write_le<std::uint8_t>(out, 11, static_cast<std::uint8_t>(spec.sender_type));
  std::fill_n(out.data() + 12, 6, std::byte{0});
  write_le<std::uint16_t>(out, 18, crc::crc16_ccitt_false(out.first(18)));
  if (!spec.data.empty()) {
    std::memcpy(out.data() + kCommandHeaderSize, spec.data.data(), spec.data.size());
    write_le<std::uint32_t>(out, 20, crc::crc32(spec.data));
  } else {
    write_le<std::uint32_t>(out, 20, 0u);
  }
  return total;
}

std::expected<std::vector<std::byte>, EncodeError> build_command_frame(
  const CommandFrameSpec & spec)
{
  std::vector<std::byte> buf(command_frame_size(spec.data.size()));
  auto r = encode_command_frame(buf, spec);
  if (!r) {
    return std::unexpected(r.error());
  }
  return buf;
}

// ---------------------------------------------------------------------------
// Data packet
// ---------------------------------------------------------------------------
std::expected<DataPacketView, ParseError> parse_data_packet(
  std::span<const std::byte> packet, bool verify_crc) noexcept
{
  if (packet.size() < kDataPacketHeaderSize) {
    return std::unexpected(ParseError::kTooShort);
  }
  DataPacketHeader h;
  h.version = read_le<std::uint8_t>(packet, 0);
  if (h.version != kProtocolVersion) {
    return std::unexpected(ParseError::kBadVersion);
  }
  h.length = read_le<std::uint16_t>(packet, 1);
  if (h.length < kDataPacketHeaderSize || h.length > packet.size()) {
    return std::unexpected(ParseError::kLengthMismatch);
  }
  h.time_interval = read_le<std::uint16_t>(packet, 3);
  h.dot_num = read_le<std::uint16_t>(packet, 5);
  h.udp_cnt = read_le<std::uint16_t>(packet, 7);
  h.frame_cnt = read_le<std::uint8_t>(packet, 9);
  const auto raw_type = read_le<std::uint8_t>(packet, 10);
  if (raw_type > 3) {
    return std::unexpected(ParseError::kUnknownDataType);
  }
  h.data_type = static_cast<DataType>(raw_type);
  h.time_type = static_cast<TimeType>(read_le<std::uint8_t>(packet, 11));
  std::memcpy(h.reserved.data(), packet.data() + 12, 12);
  h.crc32 = read_le<std::uint32_t>(packet, 24);
  h.timestamp_ns = read_le<std::uint64_t>(packet, 28);

  const auto data = packet.subspan(kDataPacketHeaderSize, h.length - kDataPacketHeaderSize);
  if (data.size() != static_cast<std::size_t>(h.dot_num) * sample_size(h.data_type)) {
    return std::unexpected(ParseError::kBadDotNum);
  }
  if (verify_crc) {
    // CRC covers timestamp (8 bytes at offset 28) followed by data: contiguous on the wire.
    const auto covered = packet.subspan(28, 8 + data.size());
    if (crc::crc32(covered) != h.crc32) {
      return std::unexpected(ParseError::kBadCrc32);
    }
  }
  return DataPacketView{h, data};
}

CartesianPoint32 decode_cartesian32(const DataPacketView & p, std::size_t i) noexcept
{
  const std::size_t o = i * 14;
  return {
    read_le<std::int32_t>(p.data, o), read_le<std::int32_t>(p.data, o + 4),
    read_le<std::int32_t>(p.data, o + 8), read_le<std::uint8_t>(p.data, o + 12),
    read_le<std::uint8_t>(p.data, o + 13)};
}

CartesianPoint16 decode_cartesian16(const DataPacketView & p, std::size_t i) noexcept
{
  const std::size_t o = i * 8;
  return {
    read_le<std::int16_t>(p.data, o), read_le<std::int16_t>(p.data, o + 2),
    read_le<std::int16_t>(p.data, o + 4), read_le<std::uint8_t>(p.data, o + 6),
    read_le<std::uint8_t>(p.data, o + 7)};
}

SphericalPoint decode_spherical(const DataPacketView & p, std::size_t i) noexcept
{
  const std::size_t o = i * 10;
  return {
    read_le<std::uint32_t>(p.data, o), read_le<std::uint16_t>(p.data, o + 4),
    read_le<std::uint16_t>(p.data, o + 6), read_le<std::uint8_t>(p.data, o + 8),
    read_le<std::uint8_t>(p.data, o + 9)};
}

ImuSample decode_imu(const DataPacketView & p, std::size_t i) noexcept
{
  const std::size_t o = i * 24;
  return {read_le<float>(p.data, o),      read_le<float>(p.data, o + 4),
          read_le<float>(p.data, o + 8),  read_le<float>(p.data, o + 12),
          read_le<float>(p.data, o + 16), read_le<float>(p.data, o + 20)};
}

namespace
{
template <typename T, typename F>
std::vector<T> decode_all(const DataPacketView & p, DataType expect, const F & f)
{
  std::vector<T> out;
  if (p.header.data_type != expect) {
    return out;
  }
  out.reserve(p.header.dot_num);
  for (std::size_t i = 0; i < p.header.dot_num; ++i) {
    out.push_back(f(p, i));
  }
  return out;
}
}  // namespace

std::vector<CartesianPoint32> decode_all_cartesian32(const DataPacketView & p)
{
  return decode_all<CartesianPoint32>(p, DataType::kCartesian32, decode_cartesian32);
}
std::vector<CartesianPoint16> decode_all_cartesian16(const DataPacketView & p)
{
  return decode_all<CartesianPoint16>(p, DataType::kCartesian16, decode_cartesian16);
}
std::vector<SphericalPoint> decode_all_spherical(const DataPacketView & p)
{
  return decode_all<SphericalPoint>(p, DataType::kSpherical, decode_spherical);
}
std::vector<ImuSample> decode_all_imu(const DataPacketView & p)
{
  return decode_all<ImuSample>(p, DataType::kImu, decode_imu);
}

std::uint64_t sample_timestamp_ns(const DataPacketHeader & h, std::size_t i) noexcept
{
  if (h.dot_num <= 1 || i == 0) {
    return h.timestamp_ns;
  }
  const std::uint64_t span_ns = static_cast<std::uint64_t>(h.time_interval) * 100u;
  return h.timestamp_ns + span_ns * i / (static_cast<std::uint64_t>(h.dot_num) - 1u);
}

// ---------------------------------------------------------------------------
// Key-value lists
// ---------------------------------------------------------------------------
std::expected<std::vector<KeyValue>, ParseError> parse_key_value_list(
  std::span<const std::byte> in, std::size_t key_num) noexcept
{
  std::vector<KeyValue> out;
  out.reserve(key_num);
  std::size_t off = 0;
  for (std::size_t n = 0; n < key_num; ++n) {
    if (in.size() - off < 4) {
      return std::unexpected(ParseError::kTruncated);
    }
    const auto key = read_le<std::uint16_t>(in, off);
    const auto len = read_le<std::uint16_t>(in, off + 2);
    off += 4;
    if (in.size() - off < len) {
      return std::unexpected(ParseError::kTruncated);
    }
    out.push_back({key, in.subspan(off, len)});
    off += len;
  }
  if (off != in.size()) {
    return std::unexpected(ParseError::kKeyNumMismatch);
  }
  return out;
}

void append_key_value_list(std::vector<std::byte> & out, std::span<const KeyValue> kvs)
{
  for (const auto & kv : kvs) {
    const std::size_t o = out.size();
    out.resize(o + 4 + kv.value.size());
    write_le<std::uint16_t>(out, o, kv.key);
    write_le<std::uint16_t>(out, o + 2, static_cast<std::uint16_t>(kv.value.size()));
    std::copy(kv.value.begin(), kv.value.end(), out.begin() + static_cast<std::ptrdiff_t>(o + 4));
  }
}

std::vector<std::byte> encode_param_config_request(std::span<const KeyValue> kvs)
{
  std::vector<std::byte> out(4);
  write_le<std::uint16_t>(out, 0, static_cast<std::uint16_t>(kvs.size()));
  write_le<std::uint16_t>(out, 2, 0);
  append_key_value_list(out, kvs);
  return out;
}

std::vector<std::byte> encode_param_inquire_request(std::span<const std::uint16_t> keys)
{
  std::vector<std::byte> out(4 + 2 * keys.size());
  write_le<std::uint16_t>(out, 0, static_cast<std::uint16_t>(keys.size()));
  write_le<std::uint16_t>(out, 2, 0);
  for (std::size_t i = 0; i < keys.size(); ++i) {
    write_le<std::uint16_t>(out, 4 + 2 * i, keys[i]);
  }
  return out;
}

std::vector<std::byte> encode_reboot_request(std::uint16_t timeout_ms)
{
  std::vector<std::byte> out(2);
  write_le<std::uint16_t>(out, 0, timeout_ms);
  return out;
}

std::vector<std::byte> encode_factory_reset_request() { return std::vector<std::byte>(16); }

std::vector<std::byte> encode_set_gps_timestamp_request(std::uint64_t pps_time_ns)
{
  std::vector<std::byte> out(9);
  write_le<std::uint8_t>(out, 0, 2);
  write_le<std::uint64_t>(out, 1, pps_time_ns);
  return out;
}

std::string_view DiscoveryAck::serial_number_view() const noexcept
{
  const auto * const end = std::find(serial_number.begin(), serial_number.end(), '\0');
  return {serial_number.data(), static_cast<std::size_t>(end - serial_number.begin())};
}

std::expected<DiscoveryAck, ParseError> parse_discovery_ack(
  std::span<const std::byte> data) noexcept
{
  if (data.size() < 24) {
    return std::unexpected(ParseError::kTruncated);
  }
  DiscoveryAck a{};
  a.ret_code = static_cast<RetCode>(read_le<std::uint8_t>(data, 0));
  a.dev_type = read_le<std::uint8_t>(data, 1);
  std::memcpy(a.serial_number.data(), data.data() + 2, 16);
  std::memcpy(a.lidar_ip.data(), data.data() + 18, 4);
  a.cmd_port = read_le<std::uint16_t>(data, 22);
  return a;
}

std::expected<ParamConfigAck, ParseError> parse_param_config_ack(
  std::span<const std::byte> data) noexcept
{
  if (data.size() < 3) {
    return std::unexpected(ParseError::kTruncated);
  }
  return ParamConfigAck{
    static_cast<RetCode>(read_le<std::uint8_t>(data, 0)), read_le<std::uint16_t>(data, 1)};
}

std::expected<ParamInquireAck, ParseError> parse_param_inquire_ack(
  std::span<const std::byte> data) noexcept
{
  if (data.size() < 3) {
    return std::unexpected(ParseError::kTruncated);
  }
  ParamInquireAck a;
  a.ret_code = static_cast<RetCode>(read_le<std::uint8_t>(data, 0));
  const auto key_num = read_le<std::uint16_t>(data, 1);
  auto kvs = parse_key_value_list(data.subspan(3), key_num);
  if (!kvs) {
    return std::unexpected(kvs.error());
  }
  a.values = std::move(*kvs);
  return a;
}

std::expected<InfoPush, ParseError> parse_info_push(std::span<const std::byte> data) noexcept
{
  if (data.size() < 4) {
    return std::unexpected(ParseError::kTruncated);
  }
  const auto key_num = read_le<std::uint16_t>(data, 0);
  auto kvs = parse_key_value_list(data.subspan(4), key_num);
  if (!kvs) {
    return std::unexpected(kvs.error());
  }
  return InfoPush{std::move(*kvs)};
}

std::expected<SimpleAck, ParseError> parse_simple_ack(std::span<const std::byte> data) noexcept
{
  if (data.empty()) {
    return std::unexpected(ParseError::kTruncated);
  }
  return SimpleAck{static_cast<RetCode>(read_le<std::uint8_t>(data, 0))};
}

}  // namespace livox::mid360
