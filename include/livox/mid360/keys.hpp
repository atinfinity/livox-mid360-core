// SPDX-License-Identifier: Apache-2.0
// Key definitions and typed encoders/decoders for the Mid-360 key-value list
// (section "0x0102 LiDAR Information Push", table of keys).
//
// Scope (v1): the base Mid-360 only. Keys that exist solely for Mid-360S/360L
// (0x0021 speed_mode, 0x0029 pc_freq_mod) are listed for completeness but not modelled.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include "livox/mid360/export.hpp"
#include "livox/mid360/protocol.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360 {

enum class Key : std::uint16_t {
  // ---- writable (0x0100) -------------------------------------------------
  kPclDataType = 0x0000,          ///< u8, DataType 1/2/3
  kPatternMode = 0x0001,          ///< u8, only 0 (non-repetitive) is effective
  kLidarIpCfg = 0x0004,           ///< u8[12] ip, mask, gateway
  kStateInfoHostIpCfg = 0x0005,   ///< u8[8] ip, dst port, src port
  kPointCloudHostIpCfg = 0x0006,  ///< u8[8]
  kImuHostIpCfg = 0x0007,         ///< u8[8]
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

[[nodiscard]] std::string_view to_string(Key k) noexcept;
[[nodiscard]] constexpr bool is_read_only(Key k) noexcept {
  return (static_cast<std::uint16_t>(k) & 0x8000u) != 0;
}
/// Fixed value length documented for the key, or nullopt if variable/unknown.
[[nodiscard]] std::optional<std::size_t> key_value_length(Key k) noexcept;

enum class KeyError : std::uint8_t { kWrongLength, kOutOfRange };

// ---------------------------------------------------------------------------
// Typed values
// ---------------------------------------------------------------------------
using Ipv4 = std::array<std::uint8_t, 4>;

struct HostIpConfig {  ///< keys 0x0005 / 0x0006 / 0x0007
  Ipv4 ip{};
  std::uint16_t dst_port = 0;  ///< host-side listening port
  std::uint16_t src_port = 0;  ///< LiDAR-side source port
};

struct LidarIpConfig {  ///< key 0x0004
  Ipv4 ip{};
  Ipv4 netmask{};
  Ipv4 gateway{};
};

struct InstallAttitude {  ///< key 0x0012
  float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
  std::int32_t x_mm = 0, y_mm = 0, z_mm = 0;
};

struct FovConfig {                   ///< keys 0x0015 / 0x0016
  std::int32_t yaw_start_deg = 0;    ///< [0, 360)
  std::int32_t yaw_stop_deg = 0;     ///< [0, 360)
  std::int32_t pitch_start_deg = 0;  ///< (-10, 60)
  std::int32_t pitch_stop_deg = 0;   ///< (-10, 60)
  std::uint32_t rsvd = 0;
};

enum class DetectMode : std::uint8_t { kNormal = 0, kSensitive = 1 };

struct FuncIoConfig {     ///< key 0x0019; M12 pins 8, 10, 12, 11
  std::uint8_t in0 = 0;   ///< 0 = PPS input
  std::uint8_t in1 = 0;   ///< 0 = GPS input
  std::uint8_t out0 = 0;  ///< 0 none / 1 follow IN0 / 2 safety zone out 0
  std::uint8_t out1 = 0;  ///< 0 none / 1 follow IN1 / 2 safety zone out 1
};

enum class ImuOutputRate : std::uint8_t { k200Hz = 0, k500Hz = 1, k100Hz = 2, k50Hz = 3 };
enum class ImuAccelRange : std::uint8_t { k4g = 0, k8g = 1, k16g = 2, k32g = 3 };
enum class ImuGyroRange : std::uint8_t {
  k2000dps = 0,
  k1000dps = 1,
  k500dps = 2,
  k250dps = 3,
  k125dps = 4,
  k62_5dps = 5,
  k31_25dps = 6,
  k15_625dps = 7,
};
struct ImuSensorConfig {  ///< key 0x002B
  ImuOutputRate output_rate = ImuOutputRate::k200Hz;
  ImuAccelRange accel_range = ImuAccelRange::k4g;
  ImuGyroRange gyro_range = ImuGyroRange::k2000dps;
};

struct Version {  ///< keys 0x8002 / 0x8003 / 0x8004, "aa.bb.cc.dd"
  std::array<std::uint8_t, 4> v{};
};

enum class TimeSyncType : std::uint8_t { kNone = 0, kPtp = 1, kGps = 2 };

/// key 0x800E: per-module abnormality level 0 normal / 1 warning / 2 error / 3 safety_err.
struct DiagStatus {
  std::uint8_t system;         ///< bit 0-3
  std::uint8_t scan;           ///< bit 4-7
  std::uint8_t ranging;        ///< bit 8-11
  std::uint8_t communication;  ///< bit 12-15
};

// ---------------------------------------------------------------------------
// Encoders: produce the raw value bytes for a key (to be wrapped in a KeyValue).
// ---------------------------------------------------------------------------
[[nodiscard]] std::array<std::byte, 1> encode_u8(std::uint8_t v) noexcept;
[[nodiscard]] std::array<std::byte, 8> encode_host_ip_config(const HostIpConfig& c) noexcept;
[[nodiscard]] std::array<std::byte, 12> encode_lidar_ip_config(const LidarIpConfig& c) noexcept;
[[nodiscard]] std::array<std::byte, 24> encode_install_attitude(const InstallAttitude& a) noexcept;
[[nodiscard]] std::array<std::byte, 20> encode_fov_config(const FovConfig& f) noexcept;
[[nodiscard]] std::array<std::byte, 4> encode_func_io_config(const FuncIoConfig& c) noexcept;
[[nodiscard]] std::array<std::byte, 3> encode_imu_sensor_config(const ImuSensorConfig& c) noexcept;

// ---------------------------------------------------------------------------
// Decoders: parse a key's raw value bytes. Length is validated.
// ---------------------------------------------------------------------------
[[nodiscard]] std::expected<std::uint8_t, KeyError> decode_u8(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<std::uint16_t, KeyError> decode_u16(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<std::uint32_t, KeyError> decode_u32(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<std::int32_t, KeyError> decode_i32(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<std::uint64_t, KeyError> decode_u64(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<std::int64_t, KeyError> decode_i64(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<HostIpConfig, KeyError> decode_host_ip_config(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<LidarIpConfig, KeyError> decode_lidar_ip_config(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<InstallAttitude, KeyError> decode_install_attitude(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<FovConfig, KeyError> decode_fov_config(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<FuncIoConfig, KeyError> decode_func_io_config(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<ImuSensorConfig, KeyError> decode_imu_sensor_config(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<Version, KeyError> decode_version(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<std::array<std::uint8_t, 6>, KeyError> decode_mac(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<WorkState, KeyError> decode_work_state(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<DiagStatus, KeyError> decode_diag_status(
    std::span<const std::byte> v) noexcept;
[[nodiscard]] std::expected<std::array<std::uint32_t, 8>, KeyError> decode_hms_codes(
    std::span<const std::byte> v) noexcept;
/// NUL-padded string keys (0x8000 sn, 0x8001 product_info). Returns the text before the first NUL.
[[nodiscard]] std::string_view decode_string(std::span<const std::byte> v) noexcept;

/// Finds the first entry with `key` in a parsed list.
[[nodiscard]] std::optional<std::span<const std::byte>> find_key(std::span<const KeyValue> kvs,
                                                                 Key key) noexcept;

}  // namespace livox::mid360
LIVOX_MID360_API_END
