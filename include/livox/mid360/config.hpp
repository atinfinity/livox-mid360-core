// SPDX-License-Identifier: Apache-2.0
// Host setup flow on top of Session (issue #5): point the LiDAR at this host, choose the point
// cloud format, enable the IMU and optionally switch the work mode.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <vector>

#include "livox/mid360/export.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/session.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

/// What the LiDAR needs to know about this host. Sent as one 0x0100 request (keys 0x0005,
/// 0x0006, 0x0007, 0x0000, 0x001C in that order, then `scan_pattern` 0x0001 and the present
/// `fov` keys 0x0015 / 0x0016 / 0x0017 when set) followed, when `work_tgt_mode` is set, by a second one with 0x001A. Keys not listed
/// here (0x0004 lidar ipcfg, attitude, ...) are left to Session::configure.
struct HostSetup
{
  /// Host address the LiDAR sends to. Empty: the session socket's local address, which must
  /// then not be 0.0.0.0 (bind to a specific interface) → kInvalidArgument otherwise.
  std::optional<Ipv4> ip;
  std::uint16_t push_port = kDefaultHostPushPort;         ///< 0x0102 push, key 0x0005
  std::uint16_t point_port = kDefaultHostPointCloudPort;  ///< point cloud, key 0x0006
  std::uint16_t imu_port = kDefaultHostImuPort;           ///< IMU, key 0x0007
  DataType pcl_data_type = DataType::kCartesian32;        ///< key 0x0000
  bool imu_enable = true;                                 ///< key 0x001C
  /// Target work mode (key 0x001A). Only kSampling, kIdle and kReady are accepted; anything
  /// else → kInvalidArgument with error_key 0x001A. Empty: leave the mode unchanged.
  std::optional<WorkState> work_tgt_mode;
  /// Scan pattern (key 0x0001, issue #40). Empty: leave the stored value alone.
  std::optional<ScanPattern> scan_pattern;
  /// FOV windows / enable mask (issue #39) applied in the same request as the host keys, so a
  /// reconnect restores them. Out-of-range window → kInvalidArgument with its key. Empty:
  /// leave the stored FOV alone.
  std::optional<FovSettings> fov;
  /// Detection mode (key 0x0018, issue #46), time filter (key 0x0026, issue #54) and IMU
  /// sensor config (key 0x002B, issue #47). Empty: leave the stored value alone. The IMU
  /// config key is missing on older firmware, so an absent optional is the safe default.
  std::optional<DetectMode> detect_mode;
  std::optional<bool> time_filter;
  std::optional<ImuSensorConfig> imu_sensor_config;
  /// After setting `work_tgt_mode`, poll 0x8006 until it is observed. 0: return right after
  /// the ACK.
  std::chrono::milliseconds wait_timeout{10000};
};

struct HostSetupResult
{
  /// One of the ACKs returned 0x21: the LiDAR applies the change after a reboot.
  bool reboot_required = false;
  /// Set when `work_tgt_mode` was requested and `wait_timeout` was non-zero.
  std::optional<WorkState> final_state;
};

/// The 0x0100 payload of the first request, without the session: `values` are views into
/// `storage`, so the struct must outlive its use. Pure; useful for tests and for callers that
/// drive Session::configure themselves.
struct HostSetupKeyValues
{
  std::vector<std::byte> storage;
  std::vector<KeyValue> values;  ///< views into `storage`

  HostSetupKeyValues() = default;
  HostSetupKeyValues(HostSetupKeyValues &&) noexcept = default;
  HostSetupKeyValues & operator=(HostSetupKeyValues &&) noexcept = default;
  HostSetupKeyValues(const HostSetupKeyValues &) = delete;
  HostSetupKeyValues & operator=(const HostSetupKeyValues &) = delete;
  ~HostSetupKeyValues() = default;
};

[[nodiscard]] HostSetupKeyValues host_setup_key_values(
  const HostSetup & setup, const Ipv4 & host_ip);

/// Apply `setup` through `session`. Errors are the session's: kLidarRejected carries the
/// `error_key` the LiDAR complained about; kTimeout / kUnexpectedState come from the wait.
[[nodiscard]] std::expected<HostSetupResult, SessionError> apply_host_setup(
  Session & session, const HostSetup & setup, std::optional<RequestOptions> opts = std::nullopt);

}  // namespace livox::mid360
LIVOX_MID360_API_END
