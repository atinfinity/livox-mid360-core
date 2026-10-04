// SPDX-License-Identifier: Apache-2.0
/// @file
/// Pure-function protocol layer for the Livox Mid-360: framing, parsing and serialisation of
/// control-command frames, point-cloud / IMU data packets and key-value lists.
///
/// Reference: "Livox LiDAR Communication Protocol - Mid360", wiki revision v1.4.12 (2026-09-21).
/// Everything is UDP, little-endian. No I/O happens in this header.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string_view>
#include <vector>

#include "livox/mid360/export.hpp"
#include "livox/mid360/tag.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

/// @name Constants
///@{

/// LiDAR discovery port (`0x0000`). discover() broadcasts to it by default; a unicast to it
/// (DiscoveryOptions::targets, the Device reconnect path) is answered by the simulator and
/// is still to be confirmed on hardware (#11).
inline constexpr std::uint16_t kDiscoveryPort = 56000;
inline constexpr std::uint16_t kCommandPort = 56100;     ///< LiDAR control command port
inline constexpr std::uint16_t kPushPort = 56200;        ///< LiDAR-side source port for 0x0102 push
inline constexpr std::uint16_t kPointCloudPort = 56300;  ///< LiDAR-side source port for point cloud
inline constexpr std::uint16_t kImuPort = 56400;         ///< LiDAR-side source port for IMU
inline constexpr std::uint16_t kLogPort = 56500;         ///< LiDAR log port (0x03xx, #44)
/// LiDAR-side source port of the debug raw data stream (#93, verified in #106)
inline constexpr std::uint16_t kDebugDataPort = 60301;

inline constexpr std::uint16_t kDefaultHostCommandPort = 56101;     ///< host port for commands
inline constexpr std::uint16_t kDefaultHostPushPort = 56201;        ///< host port for 0x0102 push
inline constexpr std::uint16_t kDefaultHostPointCloudPort = 56301;  ///< host port for point cloud
inline constexpr std::uint16_t kDefaultHostImuPort = 56401;         ///< host port for IMU
inline constexpr std::uint16_t kDefaultHostLogPort = 56501;         ///< firmware log (#44)
inline constexpr std::uint16_t kDefaultHostDebugDataPort = 44332;   ///< as SDK2 (#93)

inline constexpr std::uint8_t kCommandSof = 0xAA;      ///< first byte of every command frame
inline constexpr std::uint8_t kProtocolVersion = 0;    ///< version of command frames and packets
inline constexpr std::size_t kCommandHeaderSize = 24;  ///< command frame header, sof .. crc32
inline constexpr std::size_t kCommandFrameMaxSize = 1400;  ///< whole command frame
/// Largest command payload that fits a frame.
inline constexpr std::size_t kCommandDataMaxSize = kCommandFrameMaxSize - kCommandHeaderSize;

inline constexpr std::size_t kDataPacketHeaderSize = 36;  ///< version .. timestamp
inline constexpr std::size_t kPointsPerPacket = 96;       ///< N for data_type 1, 2 and 3

///@}

/// @name Enumerations
///@{

/// Command IDs (`cmd_id` of the command frame header).
enum class CmdId : std::uint16_t
{
  kDiscovery = 0x0000,        ///< discovery request; the ACK is a DiscoveryAck
  kParamConfig = 0x0100,      ///< write a key-value list
  kParamInquire = 0x0101,     ///< read a list of keys
  kInfoPush = 0x0102,         ///< LiDAR → host state push (key-value list)
  kReboot = 0x0200,           ///< reboot after a timeout
  kFactoryReset = 0x0201,     ///< restore factory settings
  kSetGpsTimestamp = 0x0202,  ///< set the time of the last PPS edge
  kPushLog = 0x0300,          ///< LiDAR → host on the log port; see firmware_log.hpp (#44)
  kCollectionLog = 0x0301,    ///< host → LiDAR port 56500: start / stop a log type
  // 0x0302 (log time sync) is defined by SDK2 but never used; 0x04xx (upgrade) is not modelled.
  kDebugDataControl = 0x0303,  ///< host → LiDAR: start / stop debug raw data; debug_data.hpp (#93)
};

/// Request or acknowledgement (`cmd_type` of the command frame header).
enum class CmdType : std::uint8_t
{
  kReq = 0x00,  ///< request
  kAck = 0x01   ///< acknowledgement of a request
};
/// Originator of a command frame (`sender_type` of the header).
enum class SenderType : std::uint8_t
{
  kHost = 0x00,  ///< the host computer
  kLidar = 0x01  ///< the LiDAR
};

/// Result code carried as the first byte of every ACK.
enum class RetCode : std::uint8_t
{
  kSuccess = 0x00,
  kFailure = 0x01,
  kNotPermitNow = 0x02,
  kOutOfRange = 0x03,
  kParamNotSupport = 0x20,
  kParamRebootEffect = 0x21,
  kParamReadOnly = 0x22,
  kParamInvalidLen = 0x23,
  kParamKeyNumErr = 0x24,
  kUpgradePubKeyError = 0x30,
  kUpgradeDigestError = 0x31,
  kUpgradeFwTypeError = 0x32,
  kUpgradeFwOutOfRange = 0x33,
  kUpgradeFwErasing = 0x34,
};

/// LiDAR working state (key 0x8006 cur_work_state / 0x001A work_tgt_mode).
/// Only kSampling, kIdle and kReady may be requested via work_tgt_mode.
enum class WorkState : std::uint8_t
{
  kSampling = 0x01,
  kIdle = 0x02,
  kError = 0x04,
  kSelfCheck = 0x05,
  kMotorStartup = 0x06,
  kUpgrade = 0x08,
  kReady = 0x09,
};

/// Sample format of a data packet (`data_type` of the header).
enum class DataType : std::uint8_t
{
  kImu = 0,          ///< ImuSample, 24 bytes, one per packet
  kCartesian32 = 1,  ///< int32 mm, 14 bytes/point, default
  kCartesian16 = 2,  ///< int16 10 mm, 8 bytes/point
  kSpherical = 3,    ///< depth mm + theta/phi 0.01 deg, 10 bytes/point
};

/// Clock source of a data packet timestamp (`time_type` of the header).
enum class TimeType : std::uint8_t
{
  kNoSync = 0,  ///< timestamp counts from LiDAR power-on
  kPtp = 1,     ///< IEEE 1588v2.0 or gPTP master time
  kGps = 2,     ///< GPS (PPS + GPRMC)
};

/// Why a frame, packet or payload was rejected by the parsers in this header.
enum class ParseError : std::uint8_t
{
  kTooShort,         ///< buffer shorter than the minimal header
  kBadSof,           ///< command frame does not start with 0xAA
  kBadVersion,       ///< protocol version != 0
  kLengthMismatch,   ///< length field disagrees with the buffer size or exceeds 1400
  kBadCrc16,         ///< command header CRC mismatch
  kBadCrc32,         ///< payload CRC mismatch
  kUnknownDataType,  ///< data_type not in 0..3
  kBadDotNum,        ///< dot_num inconsistent with payload size / data_type
  kTruncated,        ///< a key-value list or fixed-layout payload ended early
  kKeyNumMismatch,   ///< key_num does not match the number of entries present
};

/// Short name of a return code, for logs.
[[nodiscard]] std::string_view to_string(RetCode c) noexcept;
/// Short name of a working state, for logs.
[[nodiscard]] std::string_view to_string(WorkState s) noexcept;
/// Short name of a data type, for logs.
[[nodiscard]] std::string_view to_string(DataType t) noexcept;
/// Short name of a time type, for logs.
[[nodiscard]] std::string_view to_string(TimeType t) noexcept;
/// Short description of a parse error, for logs.
[[nodiscard]] std::string_view to_string(ParseError e) noexcept;
/// Short name of a command ID, for logs.
[[nodiscard]] std::string_view to_string(CmdId id) noexcept;

///@}

/// @name Control command frame
/// 24-byte header + data, max 1400 bytes.
///@{

/// The 24-byte command frame header as decoded by parse_command_frame().
struct CommandHeader
{
  std::uint16_t length = 0;                    ///< sof .. end of data
  std::uint32_t seq_num = 0;                   ///< the ACK echoes the request's value
  std::uint16_t cmd_id = 0;                    ///< raw; compare against CmdId
  CmdType cmd_type = CmdType::kReq;            ///< request or ACK
  SenderType sender_type = SenderType::kHost;  ///< host or LiDAR
  std::array<std::uint8_t, 6> resv{};          ///< reserved bytes, as carried on the wire
  std::uint16_t crc16 = 0;                     ///< header CRC as carried on the wire
  std::uint32_t crc32 = 0;  ///< data CRC as carried on the wire (0 when data is empty)
};

/// A validated, non-owning view over a command frame.
struct CommandFrameView
{
  CommandHeader header;             ///< the validated header
  std::span<const std::byte> data;  ///< payload after the 24-byte header
};

/// Parses and fully validates (SOF, version, length, CRC16, CRC32) one command frame.
[[nodiscard]] std::expected<CommandFrameView, ParseError> parse_command_frame(
  std::span<const std::byte> frame) noexcept;

/// What encode_command_frame() writes; length and both CRCs are computed from it.
struct CommandFrameSpec
{
  std::uint32_t seq_num = 0;                   ///< echoed by the ACK
  std::uint16_t cmd_id = 0;                    ///< raw command ID, see CmdId
  CmdType cmd_type = CmdType::kReq;            ///< request or ACK
  SenderType sender_type = SenderType::kHost;  ///< host or LiDAR
  // The explicit {} keeps GCC -Wmissing-field-initializers quiet for designated initializers.
  /// Payload, at most kCommandDataMaxSize bytes.
  std::span<const std::byte> data{};  // NOLINT(readability-redundant-member-init)
};

/// Why encode_command_frame() or build_command_frame() failed.
enum class EncodeError : std::uint8_t
{
  kDataTooLarge,    ///< data exceeds kCommandDataMaxSize
  kBufferTooSmall,  ///< output span cannot hold the frame
};

/// Total wire size of a frame carrying `data_size` payload bytes.
[[nodiscard]] constexpr std::size_t command_frame_size(std::size_t data_size) noexcept
{
  return kCommandHeaderSize + data_size;
}

/// Serialises a frame into `out`, computing both CRCs. Returns bytes written.
[[nodiscard]] std::expected<std::size_t, EncodeError> encode_command_frame(
  std::span<std::byte> out, const CommandFrameSpec & spec) noexcept;

/// Convenience: allocate and serialise.
[[nodiscard]] std::expected<std::vector<std::byte>, EncodeError> build_command_frame(
  const CommandFrameSpec & spec);

///@}

/// @name Point cloud / IMU data packet
/// 36-byte header + data.
///@{

/// The 36-byte data packet header as decoded by parse_data_packet().
struct DataPacketHeader
{
  std::uint8_t version = 0;                     ///< kProtocolVersion
  std::uint16_t length = 0;                     ///< whole UDP payload, from `version`
  std::uint16_t time_interval = 0;              ///< unit 0.1 us; last point time - first point time
  std::uint16_t dot_num = 0;                    ///< number of samples in `data`
  std::uint16_t udp_cnt = 0;                    ///< +1 per packet
  std::uint8_t frame_cnt = 0;                   ///< invalid for non-repetitive scan
  DataType data_type = DataType::kCartesian32;  ///< format of the samples in `data`
  TimeType time_type = TimeType::kNoSync;       ///< clock source of `timestamp_ns`
  std::array<std::uint8_t, 12>
    reserved{};                    ///< wiki: "reserved"; diagram labels part of it pack_info
  std::uint32_t crc32 = 0;         ///< over timestamp + data
  std::uint64_t timestamp_ns = 0;  ///< time of the first sample
};

/// A validated, non-owning view over a data packet.
struct DataPacketView
{
  DataPacketHeader header;          ///< the validated header
  std::span<const std::byte> data;  ///< payload after the 36-byte header
};

/// Parses and validates a data packet (version, length, CRC32, data_type/dot_num consistency).
/// A `crc32` of 0 is not checked: a Mid-360 leaves it at 0 in point cloud packets (#219).
/// Set `verify_crc=false` to skip the CRC on hot paths after you have trusted the source.
[[nodiscard]] std::expected<DataPacketView, ParseError> parse_data_packet(
  std::span<const std::byte> packet, bool verify_crc = true) noexcept;

/// Bytes per sample for a data type (0 for unknown).
[[nodiscard]] constexpr std::size_t sample_size(DataType t) noexcept
{
  switch (t) {
    case DataType::kImu:
      return 24;
    case DataType::kCartesian32:
      return 14;
    case DataType::kCartesian16:
      return 8;
    case DataType::kSpherical:
      return 10;
  }
  return 0;
}

/// One sample of data type 1 (kCartesian32).
struct CartesianPoint32
{
  std::int32_t x_mm;          ///< mm
  std::int32_t y_mm;          ///< mm
  std::int32_t z_mm;          ///< mm
  std::uint8_t reflectivity;  ///< 0-255
  std::uint8_t tag;           ///< raw tag byte
  /// The decoded tag byte.
  [[nodiscard]] constexpr TagInfo tag_info() const noexcept { return decode_tag(tag); }
};

/// One sample of data type 2 (kCartesian16).
struct CartesianPoint16
{
  std::int16_t x_cm;          ///< unit 10 mm
  std::int16_t y_cm;          ///< unit 10 mm
  std::int16_t z_cm;          ///< unit 10 mm
  std::uint8_t reflectivity;  ///< 0-255
  std::uint8_t tag;           ///< raw tag byte
  /// The decoded tag byte.
  [[nodiscard]] constexpr TagInfo tag_info() const noexcept { return decode_tag(tag); }
};

/// One sample of data type 3 (kSpherical).
struct SphericalPoint
{
  std::uint32_t depth_mm;        ///< distance, mm
  std::uint16_t theta_centideg;  ///< zenith, [0, 18000], unit 0.01 deg
  std::uint16_t phi_centideg;    ///< azimuth, [0, 36000], unit 0.01 deg
  std::uint8_t reflectivity;     ///< 0-255
  std::uint8_t tag;              ///< raw tag byte
  /// The decoded tag byte.
  [[nodiscard]] constexpr TagInfo tag_info() const noexcept { return decode_tag(tag); }
};

/// The sample of an IMU packet (data type 0).
struct ImuSample
{
  float gyro_x;  ///< rad/s
  float gyro_y;  ///< rad/s
  float gyro_z;  ///< rad/s
  float acc_x;   ///< g
  float acc_y;   ///< g
  float acc_z;   ///< g
};

/// Decodes the i-th sample. The caller guarantees `data_type` matches and `i < dot_num`.
[[nodiscard]] CartesianPoint32 decode_cartesian32(const DataPacketView & p, std::size_t i) noexcept;
/// Decodes the i-th sample, as decode_cartesian32() does for data type 2.
[[nodiscard]] CartesianPoint16 decode_cartesian16(const DataPacketView & p, std::size_t i) noexcept;
/// Decodes the i-th sample, as decode_cartesian32() does for data type 3.
[[nodiscard]] SphericalPoint decode_spherical(const DataPacketView & p, std::size_t i) noexcept;
/// Decodes the i-th sample, as decode_cartesian32() does for data type 0.
[[nodiscard]] ImuSample decode_imu(const DataPacketView & p, std::size_t i) noexcept;

/// Whole-packet decoders (allocate).
[[nodiscard]] std::vector<CartesianPoint32> decode_all_cartesian32(const DataPacketView & p);
/// Whole-packet decoder for data type 2 (allocates).
[[nodiscard]] std::vector<CartesianPoint16> decode_all_cartesian16(const DataPacketView & p);
/// Whole-packet decoder for data type 3 (allocates).
[[nodiscard]] std::vector<SphericalPoint> decode_all_spherical(const DataPacketView & p);
/// Whole-packet decoder for data type 0 (allocates).
[[nodiscard]] std::vector<ImuSample> decode_all_imu(const DataPacketView & p);

/// Timestamp of the i-th sample: timestamp + i * time_interval / (dot_num - 1), in ns.
/// time_interval is in 0.1 us (= 100 ns) units. Returns `timestamp` when dot_num <= 1.
[[nodiscard]] std::uint64_t sample_timestamp_ns(const DataPacketHeader & h, std::size_t i) noexcept;

///@}

/// @name Key-value lists
/// 0x0100 / 0x0101 / 0x0102 payloads.
///@{

/// One entry of a key-value list. Non-owning: `value` points into the parsed buffer.
struct KeyValue
{
  std::uint16_t key;                 ///< raw key; see keys.hpp
  std::span<const std::byte> value;  ///< raw little-endian value bytes
};

/// Parses `key_num` consecutive {key u16, length u16, value[length]} entries.
[[nodiscard]] std::expected<std::vector<KeyValue>, ParseError> parse_key_value_list(
  std::span<const std::byte> in, std::size_t key_num) noexcept;

/// Encodes {key,length,value}* into `out` (appends).
void append_key_value_list(std::vector<std::byte> & out, std::span<const KeyValue> kvs);

/// 0x0100 REQ data: key_num u16, rsvd u16, key_value_list.
[[nodiscard]] std::vector<std::byte> encode_param_config_request(std::span<const KeyValue> kvs);

/// 0x0101 REQ data: key_num u16, rsvd u16, key u16[key_num].
[[nodiscard]] std::vector<std::byte> encode_param_inquire_request(
  std::span<const std::uint16_t> keys);

/// 0x0200 REQ data: timeout u16 (ms).
[[nodiscard]] std::vector<std::byte> encode_reboot_request(std::uint16_t timeout_ms);

/// 0x0201 REQ data: 16 reserved bytes (documented as "SN, reserved"). Zero-filled.
[[nodiscard]] std::vector<std::byte> encode_factory_reset_request();

/// 0x0202 REQ data: type u8 (2 = GPS), time_set u64 (ns of last PPS rising edge).
[[nodiscard]] std::vector<std::byte> encode_set_gps_timestamp_request(std::uint64_t pps_time_ns);

/// Data of the 0x0000 discovery ACK.
struct DiscoveryAck
{
  RetCode ret_code;                      ///< result of the request
  std::uint8_t dev_type;                 ///< raw device type; see DeviceType in session.hpp
  std::array<char, 16> serial_number;    ///< NUL-padded
  std::array<std::uint8_t, 4> lidar_ip;  ///< IPv4 address, first octet first
  std::uint16_t cmd_port;                ///< LiDAR command port

  /// `serial_number` up to the first NUL.
  [[nodiscard]] std::string_view serial_number_view() const noexcept;
};

/// Data of the 0x0100 parameter configuration ACK.
struct ParamConfigAck
{
  RetCode ret_code;         ///< result of the request
  std::uint16_t error_key;  ///< only meaningful when ret_code != kSuccess
};

/// Data of the 0x0101 parameter inquire ACK.
struct ParamInquireAck
{
  RetCode ret_code = RetCode::kSuccess;  ///< result of the request
  std::vector<KeyValue> values;          ///< spans point into the input buffer
};

/// Data of the 0x0102 information push.
struct InfoPush
{
  std::vector<KeyValue> values;  ///< spans point into the input buffer
};

/// Data of an ACK that carries only a return code (0x0200, 0x0201, 0x0202, 0x0301, 0x0303).
struct SimpleAck
{
  RetCode ret_code;  ///< result of the request
};

/// Parses the data of a discovery ACK (24 bytes).
[[nodiscard]] std::expected<DiscoveryAck, ParseError> parse_discovery_ack(
  std::span<const std::byte> data) noexcept;
/// Parses the data of a parameter configuration ACK (3 bytes).
[[nodiscard]] std::expected<ParamConfigAck, ParseError> parse_param_config_ack(
  std::span<const std::byte> data) noexcept;
/// Parses the data of a parameter inquire ACK: ret_code u8, key_num u16, key_value_list.
[[nodiscard]] std::expected<ParamInquireAck, ParseError> parse_param_inquire_ack(
  std::span<const std::byte> data) noexcept;
/// Parses the data of a 0x0102 push: key_num u16, rsvd u16, key_value_list.
[[nodiscard]] std::expected<InfoPush, ParseError> parse_info_push(
  std::span<const std::byte> data) noexcept;
/// Parses the data of an ACK whose first byte is the return code.
[[nodiscard]] std::expected<SimpleAck, ParseError> parse_simple_ack(
  std::span<const std::byte> data) noexcept;

///@}

}  // namespace livox::mid360
LIVOX_MID360_API_END
