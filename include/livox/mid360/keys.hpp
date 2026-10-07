// SPDX-License-Identifier: Apache-2.0
/// @file
/// Key definitions and typed encoders/decoders for the Mid-360 key-value list
/// (section "0x0102 LiDAR Information Push", table of keys).
///
/// Scope (v1): the base Mid-360 only. Keys that exist solely for Mid-360S/360L
/// (0x0021 speed_mode, 0x0029 pc_freq_mod) are listed for completeness but not modelled.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "livox/mid360/export.hpp"
#include "livox/mid360/protocol.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

/// Keys of the key-value list (0x0100 configure, 0x0101 inquire, 0x0102 push). Keys with bit
/// 15 set are read-only.
enum class Key : std::uint16_t
{
  // ---- writable (0x0100) -------------------------------------------------
  kPclDataType = 0x0000,          ///< u8, DataType 1/2/3
  kPatternMode = 0x0001,          ///< u8 ScanPattern; a change restarts the motor
  kLidarIpCfg = 0x0004,           ///< u8[12] ip, mask, gateway
  kStateInfoHostIpCfg = 0x0005,   ///< u8[8] ip, dst port, src port
  kPointCloudHostIpCfg = 0x0006,  ///< u8[8]
  kImuHostIpCfg = 0x0007,         ///< u8[8]
  kLogHostIpCfg = 0x0009,         ///< u8[8] firmware log host (#44); SDK2 only reads it
  kInstallAttitude = 0x0012,      ///< 24 bytes: 3 float deg + 3 int mm
  kFovCfg0 = 0x0015,              ///< 20 bytes
  kFovCfg1 = 0x0016,              ///< 20 bytes
  kFovCfgEn = 0x0017,             ///< u8 bitmask
  kDetectMode = 0x0018,           ///< u8 0 normal / 1 sensitive
  kFuncIoCfg = 0x0019,            ///< u8[4] IN0, IN1, OUT0, OUT1
  kWorkTgtMode = 0x001A,          ///< u8 WorkState (1 SAMPLING, 2 IDLE, 9 READY)
  kImuDataEn = 0x001C,            ///< u8 0/1
  kSpeedMode = 0x0021,            ///< u8, 360S/360L only (not modelled)
  kTimeFilter = 0x0026,           ///< u8 0/1
  kPcFreqMod = 0x0029,            ///< u8, 360L only (not modelled)
  kImuSensorCfg = 0x002B,         ///< u8[3] rate, accel range, gyro range
  // ---- read-only -----------------------------------------------------------
  kSn = 0x8000,               ///< char[16]
  kProductInfo = 0x8001,      ///< char[64]
  kVersionApp = 0x8002,       ///< u8[4]
  kVersionLoader = 0x8003,    ///< u8[4]
  kVersionHardware = 0x8004,  ///< u8[4]
  kMac = 0x8005,              ///< u8[6]
  kCurWorkState = 0x8006,     ///< u8 WorkState
  kCoreTemp = 0x8007,         ///< i32, 0.01 degC
  kPowerupCnt = 0x8008,       ///< u32
  kLocalTimeNow = 0x8009,     ///< u64 ns
  kLastSyncTime = 0x800A,     ///< u64 ns
  kTimeOffset = 0x800B,       ///< i64 ns, local - source
  kTimeSyncType = 0x800C,     ///< u8 0 none / 1 PTP / 2 GPS
  kLidarDiagStatus = 0x800E,  ///< u16 bitfield
  kFwType = 0x8010,           ///< u8 0 loader / 1 app
  kHmsCode = 0x8011,          ///< u32[8]
};

/// Wire name of the key, e.g. "pcl_data_type".
[[nodiscard]] std::string_view to_string(Key k) noexcept;
/// True for the 0x8000 range (inquire / push only).
[[nodiscard]] constexpr bool is_read_only(Key k) noexcept
{
  return (static_cast<std::uint16_t>(k) & 0x8000u) != 0;
}
/// Fixed value length documented for the key, or nullopt if variable/unknown.
[[nodiscard]] std::optional<std::size_t> key_value_length(Key k) noexcept;

/// Why a key value did not decode.
enum class KeyError : std::uint8_t
{
  kWrongLength,  ///< the value's length differs from the key's fixed length
  kOutOfRange    ///< a field is outside its enum or documented range
};

/// @name Typed values
///@{

/// IPv4 address in network order, e.g. {192, 168, 1, 12}.
using Ipv4 = std::array<std::uint8_t, 4>;

/// Keys 0x0005 / 0x0006 / 0x0007 / 0x0009: where the LiDAR sends one kind of data.
struct HostIpConfig
{
  Ipv4 ip{};                   ///< host address
  std::uint16_t dst_port = 0;  ///< host-side listening port
  std::uint16_t src_port = 0;  ///< LiDAR-side source port
};

/// Key 0x0004: the LiDAR's own network configuration.
struct LidarIpConfig
{
  Ipv4 ip{};       ///< LiDAR address
  Ipv4 netmask{};  ///< subnet mask
  Ipv4 gateway{};  ///< gateway; 0.0.0.0 = none
};

/// Key 0x0012: mounting rotation (degrees) and offset (millimetres) of the LiDAR.
struct InstallAttitude
{
  float roll_deg = 0;     ///< rotation about x
  float pitch_deg = 0;    ///< rotation about y
  float yaw_deg = 0;      ///< rotation about z
  std::int32_t x_mm = 0;  ///< offset along x
  std::int32_t y_mm = 0;  ///< offset along y
  std::int32_t z_mm = 0;  ///< offset along z
};

/// Keys 0x0015 / 0x0016: one FOV window in degrees.
struct FovConfig
{
  std::int32_t yaw_start_deg = 0;    ///< [0, 360]
  std::int32_t yaw_stop_deg = 0;     ///< [0, 360]
  std::int32_t pitch_start_deg = 0;  ///< [-10, 60]
  std::int32_t pitch_stop_deg = 0;   ///< [-10, 60]
  std::uint32_t rsvd = 0;            ///< reserved; encoded as given
};

/// key 0x0001. The wiki says only kNonRepetitive is effective on the base Mid-360, but the
/// LiDAR accepts all three; a change restarts the motor (#11, #269).
enum class ScanPattern : std::uint8_t
{
  kNonRepetitive = 0,
  kRepetitive = 1,
  kLowRateRepetitive = 2
};

/// Key 0x0018: detection sensitivity.
enum class DetectMode : std::uint8_t
{
  kNormal = 0,
  kSensitive = 1
};

/// Pin functions of key 0x0019 (issue #52). The wiki defines a single input function per
/// pin; the enums exist so a value outside the table is caught before any I/O.
enum class FuncIn0 : std::uint8_t
{
  kPps = 0,  ///< PPS input (M12 pin 8)
};
/// Function of pin IN1 (key 0x0019).
enum class FuncIn1 : std::uint8_t
{
  kGps = 0,  ///< GPS input (M12 pin 10)
};
/// Function of pins OUT0 / OUT1 (key 0x0019).
enum class FuncOut : std::uint8_t
{
  kNone = 0,         ///< output unused
  kFollowInput = 1,  ///< OUT0 follows IN0, OUT1 follows IN1
  kSafetyZone = 2,   ///< safety zone output 0 / 1
};

/// Key 0x0019; M12 pins 8, 10, 12, 11. Same layout as the 4 raw bytes.
struct FuncIoConfig
{
  FuncIn0 in0 = FuncIn0::kPps;    ///< IN0, M12 pin 8
  FuncIn1 in1 = FuncIn1::kGps;    ///< IN1, M12 pin 10
  FuncOut out0 = FuncOut::kNone;  ///< OUT0, M12 pin 12
  FuncOut out1 = FuncOut::kNone;  ///< OUT1, M12 pin 11
};

/// IMU output rate (key 0x002B byte 0).
enum class ImuOutputRate : std::uint8_t
{
  k200Hz = 0,
  k500Hz = 1,
  k100Hz = 2,
  k50Hz = 3
};
/// Accelerometer full-scale range (key 0x002B byte 1).
enum class ImuAccelRange : std::uint8_t
{
  k4g = 0,
  k8g = 1,
  k16g = 2,
  k32g = 3
};
/// Gyroscope full-scale range in degrees per second (key 0x002B byte 2).
enum class ImuGyroRange : std::uint8_t
{
  k2000dps = 0,
  k1000dps = 1,
  k500dps = 2,
  k250dps = 3,
  k125dps = 4,
  k62_5dps = 5,
  k31_25dps = 6,
  k15_625dps = 7,
};
/// Key 0x002B: IMU output rate and ranges.
struct ImuSensorConfig
{
  ImuOutputRate output_rate = ImuOutputRate::k200Hz;  ///< byte 0
  ImuAccelRange accel_range = ImuAccelRange::k4g;     ///< byte 1
  ImuGyroRange gyro_range = ImuGyroRange::k2000dps;   ///< byte 2
};

/// Keys 0x8002 / 0x8003 / 0x8004, "aa.bb.cc.dd".
struct Version
{
  std::array<std::uint8_t, 4> v{};  ///< aa, bb, cc, dd
};

/// Key 0x800C: the time source the LiDAR is synchronised to.
enum class TimeSyncType : std::uint8_t
{
  kNone = 0,
  kPtp = 1,
  kGps = 2
};

/// key 0x800E: per-module abnormality level 0 normal / 1 warning / 2 error / 3 safety_err.
/// One nibble of key 0x800E [unverified on hardware, #11]: 0 normal .. 3 safety error.
enum class DiagLevel : std::uint8_t
{
  kNormal = 0,
  kWarning = 1,
  kError = 2,
  kSafetyError = 3
};

/// Key 0x800E, u16 bitfield split into four subsystem levels (issue #55). Values above 3 in a
/// nibble are rejected by decode_diag_status() with kOutOfRange.
struct DiagStatus
{
  DiagLevel system = DiagLevel::kNormal;         ///< bit 0-3
  DiagLevel scan = DiagLevel::kNormal;           ///< bit 4-7
  DiagLevel ranging = DiagLevel::kNormal;        ///< bit 8-11
  DiagLevel communication = DiagLevel::kNormal;  ///< bit 12-15

  /// Field-wise equality.
  [[nodiscard]] constexpr bool operator==(const DiagStatus &) const noexcept = default;
  /// Highest level among the four subsystems.
  [[nodiscard]] constexpr DiagLevel worst() const noexcept
  {
    return std::max({system, scan, ranging, communication});
  }
  /// worst() == kNormal.
  [[nodiscard]] constexpr bool normal() const noexcept { return worst() == DiagLevel::kNormal; }
};

/// Key 0x8010: which firmware image is running.
enum class FwType : std::uint8_t
{
  kLoader = 0,
  kApp = 1
};

/// Key 0x0017 bitmask: bit 0 enables fov_cfg0, bit 1 enables fov_cfg1.
struct FovEnable
{
  bool fov0 = false;  ///< bit 0: window 0x0015 crops the point cloud
  bool fov1 = false;  ///< bit 1: window 0x0016 crops the point cloud
};

/// The three FOV keys together (issue #39): Device::set_fov() sends the present ones in one
/// 0x0100, Device::fov() reads all three. Both windows stay stored while `enable` says which
/// of them crop the point cloud.
struct FovSettings
{
  std::optional<FovConfig> fov0;    ///< key 0x0015
  std::optional<FovConfig> fov1;    ///< key 0x0016
  std::optional<FovEnable> enable;  ///< key 0x0017
};

/// What a Mid-360 accepts for a FOV window (#268): yaw in [0, 360] and pitch in [-10, 60],
/// both ends included, and start <= stop for both. A reversed window is rejected (no
/// wrap-around); an equal yaw start / stop is accepted and crops every point. The codecs do
/// not check this; Device::set_fov() and HostSetup do.
[[nodiscard]] bool fov_in_range(const FovConfig & f) noexcept;
/// Key 0x0004 sanity (issue #50): `ip` neither 0.0.0.0 nor 255.255.255.255 nor the subnet's
/// network / broadcast address, `netmask` a contiguous prefix of 1 to 30 bits, `gateway`
/// either 0.0.0.0 (none) or inside the subnet and different from `ip`. The codec does not
/// check this; Device::set_lidar_ip_config() does.
[[nodiscard]] bool lidar_ip_config_valid(const LidarIpConfig & c) noexcept;
/// Key 0x0012 sanity (issue #51): the three angles finite and within [-180, 180] degrees.
/// The offsets are int32 mm and always encodable. The codec does not check this;
/// Device::set_install_attitude() does.
[[nodiscard]] bool install_attitude_valid(const InstallAttitude & a) noexcept;
/// Key 0x0019 sanity (issue #52): every field within its enum. decode_func_io_config()
/// rejects the same values with KeyError::kOutOfRange; Device::set_func_io_config() checks
/// this before any I/O.
[[nodiscard]] bool func_io_config_valid(const FuncIoConfig & c) noexcept;
///@}

/// @name Encoders
/// Encoders: produce the raw value bytes for a key (to be wrapped in a KeyValue).
///@{

/// One byte.
[[nodiscard]] std::array<std::byte, 1> encode_u8(std::uint8_t v) noexcept;
/// u8 0 / 1.
[[nodiscard]] std::array<std::byte, 1> encode_bool(bool v) noexcept;
/// Key 0x0017 bitmask.
[[nodiscard]] std::array<std::byte, 1> encode_fov_enable(FovEnable e) noexcept;
/// u8 enums (DataType, DetectMode, WorkState, ...).
template <typename E>
  requires std::is_enum_v<E> && std::is_same_v<std::underlying_type_t<E>, std::uint8_t>
[[nodiscard]] std::array<std::byte, 1> encode_enum_u8(E v) noexcept
{
  return encode_u8(static_cast<std::uint8_t>(v));
}
/// Keys 0x0005 / 0x0006 / 0x0007 / 0x0009: ip, dst port, src port.
[[nodiscard]] std::array<std::byte, 8> encode_host_ip_config(const HostIpConfig & c) noexcept;
/// Key 0x0004: ip, netmask, gateway.
[[nodiscard]] std::array<std::byte, 12> encode_lidar_ip_config(const LidarIpConfig & c) noexcept;
/// Key 0x0012: three float angles, then three int32 offsets.
[[nodiscard]] std::array<std::byte, 24> encode_install_attitude(const InstallAttitude & a) noexcept;
/// Keys 0x0015 / 0x0016.
[[nodiscard]] std::array<std::byte, 20> encode_fov_config(const FovConfig & f) noexcept;
/// Key 0x0019: in0, in1, out0, out1.
[[nodiscard]] std::array<std::byte, 4> encode_func_io_config(const FuncIoConfig & c) noexcept;
/// Key 0x002B: rate, accel range, gyro range.
[[nodiscard]] std::array<std::byte, 3> encode_imu_sensor_config(const ImuSensorConfig & c) noexcept;
///@}

/// @name Decoders
/// Decoders: parse a key's raw value bytes. Length is validated.
///@{

/// One byte.
[[nodiscard]] std::expected<std::uint8_t, KeyError> decode_u8(
  std::span<const std::byte> v) noexcept;
/// Little-endian u16.
[[nodiscard]] std::expected<std::uint16_t, KeyError> decode_u16(
  std::span<const std::byte> v) noexcept;
/// Little-endian u32.
[[nodiscard]] std::expected<std::uint32_t, KeyError> decode_u32(
  std::span<const std::byte> v) noexcept;
/// Little-endian i32.
[[nodiscard]] std::expected<std::int32_t, KeyError> decode_i32(
  std::span<const std::byte> v) noexcept;
/// Little-endian u64.
[[nodiscard]] std::expected<std::uint64_t, KeyError> decode_u64(
  std::span<const std::byte> v) noexcept;
/// Little-endian i64.
[[nodiscard]] std::expected<std::int64_t, KeyError> decode_i64(
  std::span<const std::byte> v) noexcept;
/// Keys 0x0005 / 0x0006 / 0x0007 / 0x0009.
[[nodiscard]] std::expected<HostIpConfig, KeyError> decode_host_ip_config(
  std::span<const std::byte> v) noexcept;
/// Key 0x0004. Not checked with lidar_ip_config_valid().
[[nodiscard]] std::expected<LidarIpConfig, KeyError> decode_lidar_ip_config(
  std::span<const std::byte> v) noexcept;
/// Key 0x0012. Not checked with install_attitude_valid().
[[nodiscard]] std::expected<InstallAttitude, KeyError> decode_install_attitude(
  std::span<const std::byte> v) noexcept;
/// Keys 0x0015 / 0x0016. Not checked with fov_in_range().
[[nodiscard]] std::expected<FovConfig, KeyError> decode_fov_config(
  std::span<const std::byte> v) noexcept;
/// Key 0x0019; a field outside its enum is kOutOfRange.
[[nodiscard]] std::expected<FuncIoConfig, KeyError> decode_func_io_config(
  std::span<const std::byte> v) noexcept;
/// Key 0x002B; a field outside its enum is kOutOfRange.
[[nodiscard]] std::expected<ImuSensorConfig, KeyError> decode_imu_sensor_config(
  std::span<const std::byte> v) noexcept;
/// Keys 0x8002 / 0x8003 / 0x8004.
[[nodiscard]] std::expected<Version, KeyError> decode_version(
  std::span<const std::byte> v) noexcept;
/// Key 0x8005.
[[nodiscard]] std::expected<std::array<std::uint8_t, 6>, KeyError> decode_mac(
  std::span<const std::byte> v) noexcept;
/// Keys 0x001A / 0x8006; a value that is not a WorkState enumerator is kOutOfRange.
[[nodiscard]] std::expected<WorkState, KeyError> decode_work_state(
  std::span<const std::byte> v) noexcept;
/// u8 0 / 1; anything else is kOutOfRange.
[[nodiscard]] std::expected<bool, KeyError> decode_bool(std::span<const std::byte> v) noexcept;
/// key 0x0000: 1, 2 or 3 (0 = IMU is not a point-cloud data type).
[[nodiscard]] std::expected<DataType, KeyError> decode_data_type(
  std::span<const std::byte> v) noexcept;
/// Key 0x0001: 0, 1 or 2.
[[nodiscard]] std::expected<ScanPattern, KeyError> decode_scan_pattern(
  std::span<const std::byte> v) noexcept;
/// Key 0x0018: 0 or 1.
[[nodiscard]] std::expected<DetectMode, KeyError> decode_detect_mode(
  std::span<const std::byte> v) noexcept;
/// Key 0x800C: 0, 1 or 2.
[[nodiscard]] std::expected<TimeSyncType, KeyError> decode_time_sync_type(
  std::span<const std::byte> v) noexcept;
/// Key 0x8010: 0 or 1.
[[nodiscard]] std::expected<FwType, KeyError> decode_fw_type(std::span<const std::byte> v) noexcept;
/// key 0x0017; bits above bit 1 are kOutOfRange.
[[nodiscard]] std::expected<FovEnable, KeyError> decode_fov_enable(
  std::span<const std::byte> v) noexcept;
/// Key 0x800E; a nibble above 3 is kOutOfRange.
[[nodiscard]] std::expected<DiagStatus, KeyError> decode_diag_status(
  std::span<const std::byte> v) noexcept;
/// Key 0x8011: the eight raw slots; see decode_hms().
[[nodiscard]] std::expected<std::array<std::uint32_t, 8>, KeyError> decode_hms_codes(
  std::span<const std::byte> v) noexcept;
/// NUL-padded string keys (0x8000 sn, 0x8001 product_info). Returns the text before the first NUL.
[[nodiscard]] std::string_view decode_string(std::span<const std::byte> v) noexcept;
///@}

/// Finds the first entry with `key` in a parsed list.
[[nodiscard]] std::optional<std::span<const std::byte>> find_key(
  std::span<const KeyValue> kvs, Key key) noexcept;

/// @name Key traits
/// `key_traits<K>` (issue #57): the C++ type of each key and its codec, used by
/// `Device::set<K>()` / `get<K>()`. Writable keys have `encode` (bytes for a KeyValue) and
/// `decode`; read-only keys only `decode`. Keys of the Mid-360S / 360L (kSpeedMode,
/// kPcFreqMod) have no traits on purpose (project decision, docs/roadmap.md).
///@{

template <Key K>
struct key_traits;  // primary template: undefined for keys without a typed mapping

/// Satisfied by every key that has key_traits.
template <Key K>
concept typed_key = requires { typename key_traits<K>::value_type; };
/// Satisfied by keys that may be written with Device::set<K>().
template <Key K>
concept writable_key = typed_key<K> && !is_read_only(K);

/// The C++ type of key `K`, e.g. `key_value_t<Key::kPclDataType>` is DataType.
template <Key K>
using key_value_t = typename key_traits<K>::value_type;
///@}

namespace detail
{
template <typename T, auto Enc, auto Dec>
struct WritableTraits
{
  using value_type = T;
  static constexpr bool writable = true;
  [[nodiscard]] static auto encode(const T & v) noexcept { return Enc(v); }
  [[nodiscard]] static std::expected<T, KeyError> decode(std::span<const std::byte> v) noexcept
  {
    return Dec(v);
  }
};
template <typename T, auto Dec>
struct ReadOnlyTraits
{
  using value_type = T;
  static constexpr bool writable = false;
  [[nodiscard]] static std::expected<T, KeyError> decode(std::span<const std::byte> v) noexcept
  {
    return Dec(v);
  }
};
[[nodiscard]] inline std::expected<std::string, KeyError> decode_string_owned(
  std::span<const std::byte> v) noexcept
{
  return std::string(decode_string(v));
}
}  // namespace detail

// clang-format off
template <> struct key_traits<Key::kPclDataType>
: detail::WritableTraits<DataType, encode_enum_u8<DataType>, decode_data_type> {};
template <> struct key_traits<Key::kPatternMode>
: detail::WritableTraits<ScanPattern, encode_enum_u8<ScanPattern>, decode_scan_pattern> {};
template <> struct key_traits<Key::kLidarIpCfg>
: detail::WritableTraits<LidarIpConfig, encode_lidar_ip_config, decode_lidar_ip_config> {};
template <> struct key_traits<Key::kStateInfoHostIpCfg>
: detail::WritableTraits<HostIpConfig, encode_host_ip_config, decode_host_ip_config> {};
template <> struct key_traits<Key::kPointCloudHostIpCfg>
: detail::WritableTraits<HostIpConfig, encode_host_ip_config, decode_host_ip_config> {};
template <> struct key_traits<Key::kImuHostIpCfg>
: detail::WritableTraits<HostIpConfig, encode_host_ip_config, decode_host_ip_config> {};
template <> struct key_traits<Key::kLogHostIpCfg>
: detail::WritableTraits<HostIpConfig, encode_host_ip_config, decode_host_ip_config> {};
template <> struct key_traits<Key::kInstallAttitude>
: detail::WritableTraits<InstallAttitude, encode_install_attitude, decode_install_attitude> {};
template <> struct key_traits<Key::kFovCfg0>
: detail::WritableTraits<FovConfig, encode_fov_config, decode_fov_config> {};
template <> struct key_traits<Key::kFovCfg1>
: detail::WritableTraits<FovConfig, encode_fov_config, decode_fov_config> {};
template <> struct key_traits<Key::kFovCfgEn>
: detail::WritableTraits<FovEnable, encode_fov_enable, decode_fov_enable> {};
template <> struct key_traits<Key::kDetectMode>
: detail::WritableTraits<DetectMode, encode_enum_u8<DetectMode>, decode_detect_mode> {};
template <> struct key_traits<Key::kFuncIoCfg>
: detail::WritableTraits<FuncIoConfig, encode_func_io_config, decode_func_io_config> {};
template <> struct key_traits<Key::kWorkTgtMode>
: detail::WritableTraits<WorkState, encode_enum_u8<WorkState>, decode_work_state> {};
template <> struct key_traits<Key::kImuDataEn>
: detail::WritableTraits<bool, encode_bool, decode_bool> {};
template <> struct key_traits<Key::kTimeFilter>
: detail::WritableTraits<bool, encode_bool, decode_bool> {};
template <> struct key_traits<Key::kImuSensorCfg>
: detail::WritableTraits<ImuSensorConfig, encode_imu_sensor_config, decode_imu_sensor_config> {};
template <> struct key_traits<Key::kSn>
: detail::ReadOnlyTraits<std::string, detail::decode_string_owned> {};
template <> struct key_traits<Key::kProductInfo>
: detail::ReadOnlyTraits<std::string, detail::decode_string_owned> {};
template <> struct key_traits<Key::kVersionApp>
: detail::ReadOnlyTraits<Version, decode_version> {};
template <> struct key_traits<Key::kVersionLoader>
: detail::ReadOnlyTraits<Version, decode_version> {};
template <> struct key_traits<Key::kVersionHardware>
: detail::ReadOnlyTraits<Version, decode_version> {};
template <> struct key_traits<Key::kMac>
: detail::ReadOnlyTraits<std::array<std::uint8_t, 6>, decode_mac> {};
template <> struct key_traits<Key::kCurWorkState>
: detail::ReadOnlyTraits<WorkState, decode_work_state> {};
template <> struct key_traits<Key::kCoreTemp>  // raw 0.01 degC units, not converted
: detail::ReadOnlyTraits<std::int32_t, decode_i32> {};
template <> struct key_traits<Key::kPowerupCnt>
: detail::ReadOnlyTraits<std::uint32_t, decode_u32> {};
template <> struct key_traits<Key::kLocalTimeNow>
: detail::ReadOnlyTraits<std::uint64_t, decode_u64> {};
template <> struct key_traits<Key::kLastSyncTime>
: detail::ReadOnlyTraits<std::uint64_t, decode_u64> {};
template <> struct key_traits<Key::kTimeOffset>
: detail::ReadOnlyTraits<std::int64_t, decode_i64> {};
template <> struct key_traits<Key::kTimeSyncType>
: detail::ReadOnlyTraits<TimeSyncType, decode_time_sync_type> {};
template <> struct key_traits<Key::kLidarDiagStatus>
: detail::ReadOnlyTraits<DiagStatus, decode_diag_status> {};
template <> struct key_traits<Key::kFwType>
: detail::ReadOnlyTraits<FwType, decode_fw_type> {};
template <> struct key_traits<Key::kHmsCode>
: detail::ReadOnlyTraits<std::array<std::uint32_t, 8>, decode_hms_codes> {};
// clang-format on

}  // namespace livox::mid360
LIVOX_MID360_API_END
