// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/keys.hpp"

#include <algorithm>
#include <cstring>

#include "livox/mid360/bytes.hpp"

namespace livox::mid360
{

using bytes::read_le;
using bytes::write_le;

std::string_view to_string(Key k) noexcept
{
  switch (k) {
    case Key::kPclDataType:
      return "pcl_data_type";
    case Key::kPatternMode:
      return "pattern_mode";
    case Key::kLidarIpCfg:
      return "lidar_ipcfg";
    case Key::kStateInfoHostIpCfg:
      return "state_info_host_ipcfg";
    case Key::kPointCloudHostIpCfg:
      return "pointcloud_host_ipcfg";
    case Key::kImuHostIpCfg:
      return "imu_host_ipcfg";
    case Key::kInstallAttitude:
      return "install_attitude";
    case Key::kFovCfg0:
      return "fov_cfg0";
    case Key::kFovCfg1:
      return "fov_cfg1";
    case Key::kFovCfgEn:
      return "fov_cfg_en";
    case Key::kDetectMode:
      return "detect_mode";
    case Key::kFuncIoCfg:
      return "func_io_cfg";
    case Key::kWorkTgtMode:
      return "work_tgt_mode";
    case Key::kImuDataEn:
      return "imu_data_en";
    case Key::kSpeedMode:
      return "speed_mode";
    case Key::kTimeFilter:
      return "time_filter";
    case Key::kPcFreqMod:
      return "pc_freq_mod";
    case Key::kImuSensorCfg:
      return "imu_sensor_cfg";
    case Key::kSn:
      return "sn";
    case Key::kProductInfo:
      return "product_info";
    case Key::kVersionApp:
      return "version_app";
    case Key::kVersionLoader:
      return "version_loader";
    case Key::kVersionHardware:
      return "version_hardware";
    case Key::kMac:
      return "mac";
    case Key::kCurWorkState:
      return "cur_work_state";
    case Key::kCoreTemp:
      return "core_temp";
    case Key::kPowerupCnt:
      return "powerup_cnt";
    case Key::kLocalTimeNow:
      return "local_time_now";
    case Key::kLastSyncTime:
      return "last_sync_time";
    case Key::kTimeOffset:
      return "time_offset";
    case Key::kTimeSyncType:
      return "time_sync_type";
    case Key::kLidarDiagStatus:
      return "lidar_diag_status";
    case Key::kFwType:
      return "FW_TYPE";
    case Key::kHmsCode:
      return "hms_code";
  }
  return "unknown";
}

std::optional<std::size_t> key_value_length(Key k) noexcept
{
  switch (k) {
    case Key::kPclDataType:
    case Key::kPatternMode:
    case Key::kFovCfgEn:
    case Key::kDetectMode:
    case Key::kWorkTgtMode:
    case Key::kImuDataEn:
    case Key::kSpeedMode:
    case Key::kTimeFilter:
    case Key::kPcFreqMod:
    case Key::kCurWorkState:
    case Key::kTimeSyncType:
    case Key::kFwType:
      return 1;
    case Key::kLidarDiagStatus:
      return 2;
    case Key::kImuSensorCfg:
      return 3;
    case Key::kFuncIoCfg:
    case Key::kVersionApp:
    case Key::kVersionLoader:
    case Key::kVersionHardware:
    case Key::kCoreTemp:
    case Key::kPowerupCnt:
      return 4;
    case Key::kMac:
      return 6;
    case Key::kStateInfoHostIpCfg:
    case Key::kPointCloudHostIpCfg:
    case Key::kImuHostIpCfg:
    case Key::kLocalTimeNow:
    case Key::kLastSyncTime:
    case Key::kTimeOffset:
      return 8;
    case Key::kLidarIpCfg:
      return 12;
    case Key::kSn:
      return 16;
    case Key::kFovCfg0:
    case Key::kFovCfg1:
      return 20;
    case Key::kInstallAttitude:
      return 24;
    case Key::kHmsCode:
      return 32;
    case Key::kProductInfo:
      return 64;
  }
  return std::nullopt;
}

// ---- encoders --------------------------------------------------------------
std::array<std::byte, 1> encode_u8(std::uint8_t v) noexcept { return {std::byte{v}}; }

std::array<std::byte, 8> encode_host_ip_config(const HostIpConfig & c) noexcept
{
  std::array<std::byte, 8> o{};
  std::memcpy(o.data(), c.ip.data(), 4);
  write_le<std::uint16_t>(o, 4, c.dst_port);
  write_le<std::uint16_t>(o, 6, c.src_port);
  return o;
}

std::array<std::byte, 12> encode_lidar_ip_config(const LidarIpConfig & c) noexcept
{
  std::array<std::byte, 12> o{};
  std::memcpy(o.data(), c.ip.data(), 4);
  std::memcpy(o.data() + 4, c.netmask.data(), 4);
  std::memcpy(o.data() + 8, c.gateway.data(), 4);
  return o;
}

std::array<std::byte, 24> encode_install_attitude(const InstallAttitude & a) noexcept
{
  std::array<std::byte, 24> o{};
  write_le<float>(o, 0, a.roll_deg);
  write_le<float>(o, 4, a.pitch_deg);
  write_le<float>(o, 8, a.yaw_deg);
  write_le<std::int32_t>(o, 12, a.x_mm);
  write_le<std::int32_t>(o, 16, a.y_mm);
  write_le<std::int32_t>(o, 20, a.z_mm);
  return o;
}

std::array<std::byte, 20> encode_fov_config(const FovConfig & f) noexcept
{
  std::array<std::byte, 20> o{};
  write_le<std::int32_t>(o, 0, f.yaw_start_deg);
  write_le<std::int32_t>(o, 4, f.yaw_stop_deg);
  write_le<std::int32_t>(o, 8, f.pitch_start_deg);
  write_le<std::int32_t>(o, 12, f.pitch_stop_deg);
  write_le<std::uint32_t>(o, 16, f.rsvd);
  return o;
}

std::array<std::byte, 4> encode_func_io_config(const FuncIoConfig & c) noexcept
{
  return {std::byte{c.in0}, std::byte{c.in1}, std::byte{c.out0}, std::byte{c.out1}};
}

std::array<std::byte, 3> encode_imu_sensor_config(const ImuSensorConfig & c) noexcept
{
  return {
    std::byte{static_cast<std::uint8_t>(c.output_rate)},
    std::byte{static_cast<std::uint8_t>(c.accel_range)},
    std::byte{static_cast<std::uint8_t>(c.gyro_range)}};
}

// ---- decoders --------------------------------------------------------------
namespace
{
template <typename T>
std::expected<T, KeyError> decode_scalar(std::span<const std::byte> v) noexcept
{
  if (v.size() != sizeof(T)) {
    return std::unexpected(KeyError::kWrongLength);
  }
  return read_le<T>(v, 0);
}
}  // namespace

std::expected<std::uint8_t, KeyError> decode_u8(std::span<const std::byte> v) noexcept
{
  return decode_scalar<std::uint8_t>(v);
}
std::expected<std::uint16_t, KeyError> decode_u16(std::span<const std::byte> v) noexcept
{
  return decode_scalar<std::uint16_t>(v);
}
std::expected<std::uint32_t, KeyError> decode_u32(std::span<const std::byte> v) noexcept
{
  return decode_scalar<std::uint32_t>(v);
}
std::expected<std::int32_t, KeyError> decode_i32(std::span<const std::byte> v) noexcept
{
  return decode_scalar<std::int32_t>(v);
}
std::expected<std::uint64_t, KeyError> decode_u64(std::span<const std::byte> v) noexcept
{
  return decode_scalar<std::uint64_t>(v);
}
std::expected<std::int64_t, KeyError> decode_i64(std::span<const std::byte> v) noexcept
{
  return decode_scalar<std::int64_t>(v);
}

std::expected<HostIpConfig, KeyError> decode_host_ip_config(std::span<const std::byte> v) noexcept
{
  if (v.size() != 8) {
    return std::unexpected(KeyError::kWrongLength);
  }
  HostIpConfig c;
  std::memcpy(c.ip.data(), v.data(), 4);
  c.dst_port = read_le<std::uint16_t>(v, 4);
  c.src_port = read_le<std::uint16_t>(v, 6);
  return c;
}

std::expected<LidarIpConfig, KeyError> decode_lidar_ip_config(std::span<const std::byte> v) noexcept
{
  if (v.size() != 12) {
    return std::unexpected(KeyError::kWrongLength);
  }
  LidarIpConfig c;
  std::memcpy(c.ip.data(), v.data(), 4);
  std::memcpy(c.netmask.data(), v.data() + 4, 4);
  std::memcpy(c.gateway.data(), v.data() + 8, 4);
  return c;
}

std::expected<InstallAttitude, KeyError> decode_install_attitude(
  std::span<const std::byte> v) noexcept
{
  if (v.size() != 24) {
    return std::unexpected(KeyError::kWrongLength);
  }
  return InstallAttitude{read_le<float>(v, 0),         read_le<float>(v, 4),
                         read_le<float>(v, 8),         read_le<std::int32_t>(v, 12),
                         read_le<std::int32_t>(v, 16), read_le<std::int32_t>(v, 20)};
}

std::expected<FovConfig, KeyError> decode_fov_config(std::span<const std::byte> v) noexcept
{
  if (v.size() != 20) {
    return std::unexpected(KeyError::kWrongLength);
  }
  return FovConfig{
    read_le<std::int32_t>(v, 0), read_le<std::int32_t>(v, 4), read_le<std::int32_t>(v, 8),
    read_le<std::int32_t>(v, 12), read_le<std::uint32_t>(v, 16)};
}

std::expected<FuncIoConfig, KeyError> decode_func_io_config(std::span<const std::byte> v) noexcept
{
  if (v.size() != 4) {
    return std::unexpected(KeyError::kWrongLength);
  }
  return FuncIoConfig{
    read_le<std::uint8_t>(v, 0), read_le<std::uint8_t>(v, 1), read_le<std::uint8_t>(v, 2),
    read_le<std::uint8_t>(v, 3)};
}

std::expected<ImuSensorConfig, KeyError> decode_imu_sensor_config(
  std::span<const std::byte> v) noexcept
{
  if (v.size() != 3) {
    return std::unexpected(KeyError::kWrongLength);
  }
  const auto r = read_le<std::uint8_t>(v, 0);
  const auto a = read_le<std::uint8_t>(v, 1);
  const auto g = read_le<std::uint8_t>(v, 2);
  if (r > 3 || a > 3 || g > 7) {
    return std::unexpected(KeyError::kOutOfRange);
  }
  return ImuSensorConfig{
    static_cast<ImuOutputRate>(r), static_cast<ImuAccelRange>(a), static_cast<ImuGyroRange>(g)};
}

std::expected<Version, KeyError> decode_version(std::span<const std::byte> v) noexcept
{
  if (v.size() != 4) {
    return std::unexpected(KeyError::kWrongLength);
  }
  Version out;
  std::memcpy(out.v.data(), v.data(), 4);
  return out;
}

std::expected<std::array<std::uint8_t, 6>, KeyError> decode_mac(
  std::span<const std::byte> v) noexcept
{
  if (v.size() != 6) {
    return std::unexpected(KeyError::kWrongLength);
  }
  std::array<std::uint8_t, 6> m{};
  std::memcpy(m.data(), v.data(), 6);
  return m;
}

std::expected<WorkState, KeyError> decode_work_state(std::span<const std::byte> v) noexcept
{
  auto u = decode_u8(v);
  if (!u) {
    return std::unexpected(u.error());
  }
  switch (*u) {
    case 0x01:
    case 0x02:
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x08:
    case 0x09:
      return static_cast<WorkState>(*u);
    default:
      return std::unexpected(KeyError::kOutOfRange);
  }
}

std::expected<DiagStatus, KeyError> decode_diag_status(std::span<const std::byte> v) noexcept
{
  auto u = decode_u16(v);
  if (!u) {
    return std::unexpected(u.error());
  }
  return DiagStatus{
    static_cast<std::uint8_t>(*u & 0xF), static_cast<std::uint8_t>((*u >> 4) & 0xF),
    static_cast<std::uint8_t>((*u >> 8) & 0xF), static_cast<std::uint8_t>((*u >> 12) & 0xF)};
}

std::expected<std::array<std::uint32_t, 8>, KeyError> decode_hms_codes(
  std::span<const std::byte> v) noexcept
{
  if (v.size() != 32) {
    return std::unexpected(KeyError::kWrongLength);
  }
  std::array<std::uint32_t, 8> out{};
  for (std::size_t i = 0; i < 8; ++i) {
    out[i] = read_le<std::uint32_t>(v, 4 * i);
  }
  return out;
}

std::string_view decode_string(std::span<const std::byte> v) noexcept
{
  const auto * p = reinterpret_cast<const char *>(v.data());
  const auto * end = std::find(p, p + v.size(), '\0');
  return {p, static_cast<std::size_t>(end - p)};
}

std::optional<std::span<const std::byte>> find_key(std::span<const KeyValue> kvs, Key key) noexcept
{
  const auto k = static_cast<std::uint16_t>(key);
  for (const auto & kv : kvs) {
    if (kv.key == k) {
      return kv.value;
    }
  }
  return std::nullopt;
}

}  // namespace livox::mid360
