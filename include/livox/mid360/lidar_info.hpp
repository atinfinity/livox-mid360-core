// SPDX-License-Identifier: Apache-2.0
// Aggregated read-back types over the keys.hpp codecs (issues #38 / #41): the identity of a
// LiDAR (serial, product string, firmware versions, MAC), its stored settings and its live
// status. Each type has a key list, a decoder that takes any parsed key-value list (an
// InquireResult or a 0x0102 push) and a one-line to_string with the wire key names, so logs
// stay greppable. The struct to_string()s of the typed key values live here as well.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

#include "livox/mid360/export.hpp"
#include "livox/mid360/hms.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

/// "aa.bb.cc.dd".
[[nodiscard]] std::string to_string(const Version & v);

/// Keys 0x8000-0x8005 (issue #38). Read with Device::identity() or
/// `decode_identity(inquire(kIdentityKeys)->values)`.
struct DeviceIdentity
{
  std::string serial_number;          ///< 0x8000
  std::string product_info;           ///< 0x8001
  Version version_app;                ///< 0x8002
  Version version_loader;             ///< 0x8003
  Version version_hardware;           ///< 0x8004
  std::array<std::uint8_t, 6> mac{};  ///< 0x8005
};

inline constexpr std::array<Key, 6> kIdentityKeys{
  Key::kSn, Key::kProductInfo, Key::kVersionApp, Key::kVersionLoader, Key::kVersionHardware,
  Key::kMac};

/// Missing or undecodable keys leave the field at its default (empty string / zero).
[[nodiscard]] DeviceIdentity decode_identity(std::span<const KeyValue> kvs) noexcept;
/// `sn=... product_info=... version_app=a.b.c.d version_loader=... version_hardware=... mac=..`
[[nodiscard]] std::string to_string(const DeviceIdentity & id);

// --- typed key values -------------------------------------------------------------------
[[nodiscard]] std::string_view to_string(ScanPattern p) noexcept;
[[nodiscard]] std::string_view to_string(DetectMode m) noexcept;
[[nodiscard]] std::string_view to_string(TimeSyncType t) noexcept;
[[nodiscard]] std::string_view to_string(FwType t) noexcept;
[[nodiscard]] std::string_view to_string(ImuOutputRate r) noexcept;
[[nodiscard]] std::string_view to_string(ImuAccelRange r) noexcept;
[[nodiscard]] std::string_view to_string(ImuGyroRange r) noexcept;
/// `ip:dst_port<-src_port`
[[nodiscard]] std::string to_string(const HostIpConfig & c);
/// `ip/netmask/gateway`
[[nodiscard]] std::string to_string(const LidarIpConfig & c);
/// `r<roll>/p<pitch>/y<yaw>/x<mm>/y<mm>/z<mm>`
[[nodiscard]] std::string to_string(const InstallAttitude & a);
/// `yaw<start>-<stop>/pitch<start>-<stop>`
[[nodiscard]] std::string to_string(const FovConfig & f);
/// `fov0:<0|1>,fov1:<0|1>`
[[nodiscard]] std::string to_string(const FovEnable & e);
/// `fov0=<...> fov1=<...> enable=<...>`; absent fields are omitted, all absent → "".
[[nodiscard]] std::string to_string(const FovSettings & s);
/// `in0/in1/out0/out1`
[[nodiscard]] std::string to_string(const FuncIoConfig & c);
/// `<rate>/<accel>/<gyro>`, e.g. `200Hz/4g/2000dps`
[[nodiscard]] std::string to_string(const ImuSensorConfig & c);
/// `sys<n>/scan<n>/rng<n>/comm<n>`
[[nodiscard]] std::string_view to_string(DiagLevel level) noexcept;
/// `sys<n>/scan<n>/rng<n>/comm<n>` with the numeric level of each subsystem.
[[nodiscard]] std::string to_string(const DiagStatus & d);

// --- settings (issue #41) ---------------------------------------------------------------
/// The 16 modelled writable keys. Read with Device::settings() or
/// `decode_settings(inquire(kSettingsKeys)->values)`; a key the LiDAR did not answer, or
/// answered with an undecodable value, leaves its optional empty.
struct LidarSettings
{
  std::optional<DataType> pcl_data_type;              ///< 0x0000
  std::optional<ScanPattern> pattern_mode;            ///< 0x0001
  std::optional<LidarIpConfig> lidar_ipcfg;           ///< 0x0004
  std::optional<HostIpConfig> state_info_host_ipcfg;  ///< 0x0005
  std::optional<HostIpConfig> pointcloud_host_ipcfg;  ///< 0x0006
  std::optional<HostIpConfig> imu_host_ipcfg;         ///< 0x0007
  std::optional<InstallAttitude> install_attitude;    ///< 0x0012
  std::optional<FovConfig> fov_cfg0;                  ///< 0x0015
  std::optional<FovConfig> fov_cfg1;                  ///< 0x0016
  std::optional<FovEnable> fov_cfg_en;                ///< 0x0017
  std::optional<DetectMode> detect_mode;              ///< 0x0018
  std::optional<FuncIoConfig> func_io_cfg;            ///< 0x0019
  std::optional<WorkState> work_tgt_mode;             ///< 0x001A
  std::optional<bool> imu_data_en;                    ///< 0x001C
  std::optional<bool> time_filter;                    ///< 0x0026
  std::optional<ImuSensorConfig> imu_sensor_cfg;      ///< 0x002B
};

inline constexpr std::array<Key, 16> kSettingsKeys{
  Key::kPclDataType,
  Key::kPatternMode,
  Key::kLidarIpCfg,
  Key::kStateInfoHostIpCfg,
  Key::kPointCloudHostIpCfg,
  Key::kImuHostIpCfg,
  Key::kInstallAttitude,
  Key::kFovCfg0,
  Key::kFovCfg1,
  Key::kFovCfgEn,
  Key::kDetectMode,
  Key::kFuncIoCfg,
  Key::kWorkTgtMode,
  Key::kImuDataEn,
  Key::kTimeFilter,
  Key::kImuSensorCfg};

[[nodiscard]] LidarSettings decode_settings(std::span<const KeyValue> kvs) noexcept;
/// One line, `name=value` per present key in kSettingsKeys order, empty optionals omitted.
[[nodiscard]] std::string to_string(const LidarSettings & s);

// --- status (issue #41) -----------------------------------------------------------------
/// Keys 0x8006-0x8011: the live state, also carried by the 0x0102 push. Read with
/// Device::status() (inquire) or Device::pushed_status() (last push).
struct LidarStatus
{
  /// Host time (CLOCK_REALTIME ns) at which the ACK or push behind this snapshot was
  /// received; 0 when decoded from a bare key-value list (issue #56).
  std::uint64_t time_ns = 0;
  std::optional<WorkState> cur_work_state;         ///< 0x8006
  std::optional<std::int32_t> core_temp;           ///< 0x8007, 0.01 degC units
  std::optional<std::uint32_t> powerup_cnt;        ///< 0x8008
  std::optional<std::uint64_t> local_time_now;     ///< 0x8009, ns
  std::optional<std::uint64_t> last_sync_time;     ///< 0x800A, ns
  std::optional<std::int64_t> time_offset;         ///< 0x800B, ns
  std::optional<TimeSyncType> time_sync_type;      ///< 0x800C
  std::optional<DiagStatus> lidar_diag_status;     ///< 0x800E
  std::optional<FwType> fw_type;                   ///< 0x8010
  std::optional<std::array<HmsCode, 8>> hms_code;  ///< 0x8011, decoded slots
};

inline constexpr std::array<Key, 10> kStatusKeys{
  Key::kCurWorkState, Key::kCoreTemp,   Key::kPowerupCnt,   Key::kLocalTimeNow,
  Key::kLastSyncTime, Key::kTimeOffset, Key::kTimeSyncType, Key::kLidarDiagStatus,
  Key::kFwType,       Key::kHmsCode};

[[nodiscard]] LidarStatus decode_status(std::span<const KeyValue> kvs) noexcept;
/// One line, `name=value` per present key; `core_temp` in degC with two decimals, `hms=[..]`
/// lists only the active codes as `0x<raw>:<level>`.
[[nodiscard]] std::string to_string(const LidarStatus & s);

}  // namespace livox::mid360
LIVOX_MID360_API_END
