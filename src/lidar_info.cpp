// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/lidar_info.hpp"

#include <cstdio>
#include <string_view>

#include "livox/mid360/transport.hpp"

namespace livox::mid360
{

namespace
{
template <Key K>
void decode_into(std::span<const KeyValue> kvs, key_value_t<K> & out) noexcept
{
  if (const auto raw = find_key(kvs, K)) {
    if (auto v = key_traits<K>::decode(*raw)) {
      out = std::move(*v);
    }
  }
}

template <Key K>
void decode_opt(std::span<const KeyValue> kvs, std::optional<key_value_t<K>> & out) noexcept
{
  if (const auto raw = find_key(kvs, K)) {
    if (auto v = key_traits<K>::decode(*raw)) {
      out = std::move(*v);
    }
  }
}

template <typename T, typename F>
void append(std::string & out, std::string_view name, const std::optional<T> & v, const F & fmt)
{
  if (!v) {
    return;
  }
  if (!out.empty()) {
    out += ' ';
  }
  out += name;
  out += '=';
  out += fmt(*v);
}

std::string fmt_float(float f)
{
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.1f", static_cast<double>(f));
  return buf;
}

std::string mac_to_string(const std::array<std::uint8_t, 6> & mac)
{
  char buf[18];
  std::snprintf(
    buf, sizeof buf, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4],
    mac[5]);
  return buf;
}
}  // namespace

std::string to_string(const Version & v)
{
  return std::to_string(v.v[0]) + '.' + std::to_string(v.v[1]) + '.' + std::to_string(v.v[2]) +
         '.' + std::to_string(v.v[3]);
}

DeviceIdentity decode_identity(std::span<const KeyValue> kvs) noexcept
{
  DeviceIdentity id;
  decode_into<Key::kSn>(kvs, id.serial_number);
  decode_into<Key::kProductInfo>(kvs, id.product_info);
  decode_into<Key::kVersionApp>(kvs, id.version_app);
  decode_into<Key::kVersionLoader>(kvs, id.version_loader);
  decode_into<Key::kVersionHardware>(kvs, id.version_hardware);
  decode_into<Key::kMac>(kvs, id.mac);
  return id;
}

std::string to_string(const DeviceIdentity & id)
{
  std::string out;
  out += "sn=" + id.serial_number;
  out += " product_info=" + id.product_info;
  out += " version_app=" + to_string(id.version_app);
  out += " version_loader=" + to_string(id.version_loader);
  out += " version_hardware=" + to_string(id.version_hardware);
  out += " mac=" + mac_to_string(id.mac);
  return out;
}

std::string_view to_string(ScanPattern p) noexcept
{
  switch (p) {
    case ScanPattern::kNonRepetitive:
      return "non_repetitive";
    case ScanPattern::kRepetitive:
      return "repetitive";
    case ScanPattern::kLowRateRepetitive:
      return "low_rate_repetitive";
  }
  return "unknown";
}

std::string_view to_string(DetectMode m) noexcept
{
  switch (m) {
    case DetectMode::kNormal:
      return "normal";
    case DetectMode::kSensitive:
      return "sensitive";
  }
  return "unknown";
}

std::string_view to_string(TimeSyncType t) noexcept
{
  switch (t) {
    case TimeSyncType::kNone:
      return "none";
    case TimeSyncType::kPtp:
      return "ptp";
    case TimeSyncType::kGps:
      return "gps";
  }
  return "unknown";
}

std::string_view to_string(FwType t) noexcept
{
  switch (t) {
    case FwType::kLoader:
      return "loader";
    case FwType::kApp:
      return "app";
  }
  return "unknown";
}

std::string_view to_string(ImuOutputRate r) noexcept
{
  switch (r) {
    case ImuOutputRate::k200Hz:
      return "200Hz";
    case ImuOutputRate::k500Hz:
      return "500Hz";
    case ImuOutputRate::k100Hz:
      return "100Hz";
    case ImuOutputRate::k50Hz:
      return "50Hz";
  }
  return "unknown";
}

std::string_view to_string(ImuAccelRange r) noexcept
{
  switch (r) {
    case ImuAccelRange::k4g:
      return "4g";
    case ImuAccelRange::k8g:
      return "8g";
    case ImuAccelRange::k16g:
      return "16g";
    case ImuAccelRange::k32g:
      return "32g";
  }
  return "unknown";
}

std::string_view to_string(ImuGyroRange r) noexcept
{
  switch (r) {
    case ImuGyroRange::k2000dps:
      return "2000dps";
    case ImuGyroRange::k1000dps:
      return "1000dps";
    case ImuGyroRange::k500dps:
      return "500dps";
    case ImuGyroRange::k250dps:
      return "250dps";
    case ImuGyroRange::k125dps:
      return "125dps";
    case ImuGyroRange::k62_5dps:
      return "62.5dps";
    case ImuGyroRange::k31_25dps:
      return "31.25dps";
    case ImuGyroRange::k15_625dps:
      return "15.625dps";
  }
  return "unknown";
}

std::string to_string(const HostIpConfig & c)
{
  return ip_to_string(c.ip) + ':' + std::to_string(c.dst_port) + "<-" + std::to_string(c.src_port);
}

std::string to_string(const LidarIpConfig & c)
{
  return ip_to_string(c.ip) + '/' + ip_to_string(c.netmask) + '/' + ip_to_string(c.gateway);
}

std::string to_string(const InstallAttitude & a)
{
  return 'r' + fmt_float(a.roll_deg) + "/p" + fmt_float(a.pitch_deg) + "/y" + fmt_float(a.yaw_deg) +
         "/x" + std::to_string(a.x_mm) + "/y" + std::to_string(a.y_mm) + "/z" +
         std::to_string(a.z_mm);
}

std::string to_string(const FovConfig & f)
{
  return "yaw" + std::to_string(f.yaw_start_deg) + '-' + std::to_string(f.yaw_stop_deg) + "/pitch" +
         std::to_string(f.pitch_start_deg) + '-' + std::to_string(f.pitch_stop_deg);
}

std::string to_string(const FovEnable & e)
{
  return std::string("fov0:") + (e.fov0 ? '1' : '0') + ",fov1:" + (e.fov1 ? '1' : '0');
}

std::string to_string(const FovSettings & s)
{
  std::string out;
  append(out, "fov0", s.fov0, [](const FovConfig & f) { return to_string(f); });
  append(out, "fov1", s.fov1, [](const FovConfig & f) { return to_string(f); });
  append(out, "enable", s.enable, [](const FovEnable & e) { return to_string(e); });
  return out;
}

std::string_view to_string(FuncIn0 f) noexcept
{
  switch (f) {
    case FuncIn0::kPps:
      return "pps";
  }
  return "unknown";
}

std::string_view to_string(FuncIn1 f) noexcept
{
  switch (f) {
    case FuncIn1::kGps:
      return "gps";
  }
  return "unknown";
}

std::string_view to_string(FuncOut f) noexcept
{
  switch (f) {
    case FuncOut::kNone:
      return "none";
    case FuncOut::kFollowInput:
      return "follow_input";
    case FuncOut::kSafetyZone:
      return "safety_zone";
  }
  return "unknown";
}

std::string to_string(const FuncIoConfig & c)
{
  return std::string(to_string(c.in0)) + '/' + std::string(to_string(c.in1)) + '/' +
         std::string(to_string(c.out0)) + '/' + std::string(to_string(c.out1));
}

std::string to_string(const ImuSensorConfig & c)
{
  return std::string(to_string(c.output_rate)) + '/' + std::string(to_string(c.accel_range)) + '/' +
         std::string(to_string(c.gyro_range));
}

std::string_view to_string(DiagLevel level) noexcept
{
  switch (level) {
    case DiagLevel::kNormal:
      return "normal";
    case DiagLevel::kWarning:
      return "warning";
    case DiagLevel::kError:
      return "error";
    case DiagLevel::kSafetyError:
      return "safety_error";
  }
  return "unknown";
}

std::string to_string(const DiagStatus & d)
{
  const auto n = [](DiagLevel l) { return std::to_string(static_cast<unsigned>(l)); };
  return "sys" + n(d.system) + "/scan" + n(d.scan) + "/rng" + n(d.ranging) + "/comm" +
         n(d.communication);
}

LidarSettings decode_settings(std::span<const KeyValue> kvs) noexcept
{
  LidarSettings s;
  decode_opt<Key::kPclDataType>(kvs, s.pcl_data_type);
  decode_opt<Key::kPatternMode>(kvs, s.pattern_mode);
  decode_opt<Key::kLidarIpCfg>(kvs, s.lidar_ipcfg);
  decode_opt<Key::kStateInfoHostIpCfg>(kvs, s.state_info_host_ipcfg);
  decode_opt<Key::kPointCloudHostIpCfg>(kvs, s.pointcloud_host_ipcfg);
  decode_opt<Key::kImuHostIpCfg>(kvs, s.imu_host_ipcfg);
  decode_opt<Key::kInstallAttitude>(kvs, s.install_attitude);
  decode_opt<Key::kFovCfg0>(kvs, s.fov_cfg0);
  decode_opt<Key::kFovCfg1>(kvs, s.fov_cfg1);
  decode_opt<Key::kFovCfgEn>(kvs, s.fov_cfg_en);
  decode_opt<Key::kDetectMode>(kvs, s.detect_mode);
  decode_opt<Key::kFuncIoCfg>(kvs, s.func_io_cfg);
  decode_opt<Key::kWorkTgtMode>(kvs, s.work_tgt_mode);
  decode_opt<Key::kImuDataEn>(kvs, s.imu_data_en);
  decode_opt<Key::kTimeFilter>(kvs, s.time_filter);
  decode_opt<Key::kImuSensorCfg>(kvs, s.imu_sensor_cfg);
  return s;
}

std::string to_string(const LidarSettings & s)
{
  const auto sv = [](auto v) { return std::string(to_string(v)); };
  const auto str = [](const auto & v) { return to_string(v); };
  const auto boolean = [](bool v) { return std::string(v ? "1" : "0"); };
  std::string out;
  append(out, "pcl_data_type", s.pcl_data_type, sv);
  append(out, "pattern_mode", s.pattern_mode, sv);
  append(out, "lidar_ipcfg", s.lidar_ipcfg, str);
  append(out, "state_info_host_ipcfg", s.state_info_host_ipcfg, str);
  append(out, "pointcloud_host_ipcfg", s.pointcloud_host_ipcfg, str);
  append(out, "imu_host_ipcfg", s.imu_host_ipcfg, str);
  append(out, "install_attitude", s.install_attitude, str);
  append(out, "fov_cfg0", s.fov_cfg0, str);
  append(out, "fov_cfg1", s.fov_cfg1, str);
  append(out, "fov_cfg_en", s.fov_cfg_en, str);
  append(out, "detect_mode", s.detect_mode, sv);
  append(out, "func_io_cfg", s.func_io_cfg, str);
  append(out, "work_tgt_mode", s.work_tgt_mode, sv);
  append(out, "imu_data_en", s.imu_data_en, boolean);
  append(out, "time_filter", s.time_filter, boolean);
  append(out, "imu_sensor_cfg", s.imu_sensor_cfg, str);
  return out;
}

LidarStatus decode_status(std::span<const KeyValue> kvs) noexcept
{
  LidarStatus s;
  decode_opt<Key::kCurWorkState>(kvs, s.cur_work_state);
  decode_opt<Key::kCoreTemp>(kvs, s.core_temp);
  decode_opt<Key::kPowerupCnt>(kvs, s.powerup_cnt);
  decode_opt<Key::kLocalTimeNow>(kvs, s.local_time_now);
  decode_opt<Key::kLastSyncTime>(kvs, s.last_sync_time);
  decode_opt<Key::kTimeOffset>(kvs, s.time_offset);
  decode_opt<Key::kTimeSyncType>(kvs, s.time_sync_type);
  decode_opt<Key::kLidarDiagStatus>(kvs, s.lidar_diag_status);
  decode_opt<Key::kFwType>(kvs, s.fw_type);
  if (const auto raw = find_key(kvs, Key::kHmsCode)) {
    if (const auto codes = decode_hms_codes(*raw)) {
      std::array<HmsCode, 8> hms{};
      for (std::size_t i = 0; i < codes->size(); ++i) {
        hms[i] = decode_hms((*codes)[i]);
      }
      s.hms_code = hms;
    }
  }
  return s;
}

std::string to_string(const LidarStatus & s)
{
  const auto sv = [](auto v) { return std::string(to_string(v)); };
  const auto str = [](const auto & v) { return to_string(v); };
  const auto num = [](auto v) { return std::to_string(v); };
  std::string out;
  append(out, "cur_work_state", s.cur_work_state, sv);
  append(out, "core_temp", s.core_temp, [](std::int32_t t) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.2fC", t / 100.0);
    return std::string(buf);
  });
  append(out, "powerup_cnt", s.powerup_cnt, num);
  append(out, "local_time_now", s.local_time_now, num);
  append(out, "last_sync_time", s.last_sync_time, num);
  append(out, "time_offset", s.time_offset, num);
  append(out, "time_sync_type", s.time_sync_type, sv);
  append(out, "lidar_diag_status", s.lidar_diag_status, str);
  append(out, "fw_type", s.fw_type, sv);
  append(out, "hms", s.hms_code, [](const std::array<HmsCode, 8> & hms) {
    std::string list = "[";
    for (const HmsCode & c : hms) {
      if (!c.active()) {
        continue;
      }
      if (list.size() > 1) {
        list += ',';
      }
      char buf[16];
      std::snprintf(buf, sizeof buf, "0x%08x:", c.raw);
      list += buf;
      list += to_string(c.level);
    }
    return list + ']';
  });
  return out;
}

}  // namespace livox::mid360
