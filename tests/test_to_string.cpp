// SPDX-License-Identifier: Apache-2.0
// Name tables behind to_string(): the strings are part of the log / CLI output and the key and
// ret_code names mirror the wire names of the protocol, so every enumerator is pinned here.
#include <catch2/catch_test_macros.hpp>
#include <cerrno>
#include <cstring>
#include <string>
#include <string_view>
#include <utility>

#include "livox/mid360/event.hpp"
#include "livox/mid360/hms.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/lidar_info.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/session.hpp"
#include "livox/mid360/tag.hpp"
#include "livox/mid360/transport.hpp"

using namespace livox::mid360;

namespace
{
template <typename E>
using Names = std::initializer_list<std::pair<E, std::string_view>>;

template <typename E>
void check_names(Names<E> names)
{
  for (const auto & [value, name] : names) {
    CAPTURE(name);
    CHECK(to_string(value) == name);
  }
}
}  // namespace

TEST_CASE("to_string: protocol enums", "[to_string]")
{
  check_names<RetCode>({
    {RetCode::kSuccess, "SUCCESS"},
    {RetCode::kFailure, "FAILURE"},
    {RetCode::kNotPermitNow, "NOT_PERMIT_NOW"},
    {RetCode::kOutOfRange, "OUT_OF_RANGE"},
    {RetCode::kParamNotSupport, "PARAM_NOTSUPPORT"},
    {RetCode::kParamRebootEffect, "PARAM_REBOOT_EFFECT"},
    {RetCode::kParamReadOnly, "PARAM_RD_ONLY"},
    {RetCode::kParamInvalidLen, "PARAM_INVALID_LEN"},
    {RetCode::kParamKeyNumErr, "PARAM_KEY_NUM_ERR"},
    {RetCode::kUpgradePubKeyError, "UPGRADE_PUB_KEY_ERROR"},
    {RetCode::kUpgradeDigestError, "UPGRADE_DIGEST_ERROR"},
    {RetCode::kUpgradeFwTypeError, "UPGRADE_FW_TYPE_ERROR"},
    {RetCode::kUpgradeFwOutOfRange, "UPGRADE_FW_OUT_OF_RANGE"},
    {RetCode::kUpgradeFwErasing, "UPGRADE_FW_ERASING"},
  });
  check_names<WorkState>({
    {WorkState::kSampling, "SAMPLING"},
    {WorkState::kIdle, "IDLE"},
    {WorkState::kError, "ERROR"},
    {WorkState::kSelfCheck, "SELFCHECK"},
    {WorkState::kMotorStartup, "MOTORSTARTUP"},
    {WorkState::kUpgrade, "UPGRADE"},
    {WorkState::kReady, "READY"},
  });
  check_names<DataType>({
    {DataType::kImu, "IMU"},
    {DataType::kCartesian32, "CARTESIAN32"},
    {DataType::kCartesian16, "CARTESIAN16"},
    {DataType::kSpherical, "SPHERICAL"},
  });
  check_names<TimeType>({
    {TimeType::kNoSync, "NO_SYNC"},
    {TimeType::kPtp, "PTP"},
    {TimeType::kGps, "GPS"},
  });
  check_names<ParseError>({
    {ParseError::kTooShort, "too short"},
    {ParseError::kBadSof, "bad SOF"},
    {ParseError::kBadVersion, "bad version"},
    {ParseError::kLengthMismatch, "length mismatch"},
    {ParseError::kBadCrc16, "bad CRC16"},
    {ParseError::kBadCrc32, "bad CRC32"},
    {ParseError::kUnknownDataType, "unknown data_type"},
    {ParseError::kBadDotNum, "bad dot_num"},
    {ParseError::kTruncated, "truncated"},
    {ParseError::kKeyNumMismatch, "key_num mismatch"},
  });
  check_names<CmdId>({
    {CmdId::kDiscovery, "DISCOVERY"},
    {CmdId::kParamConfig, "PARAM_CONFIG"},
    {CmdId::kParamInquire, "PARAM_INQUIRE"},
    {CmdId::kInfoPush, "INFO_PUSH"},
    {CmdId::kReboot, "REBOOT"},
    {CmdId::kFactoryReset, "FACTORY_RESET"},
    {CmdId::kSetGpsTimestamp, "SET_GPS_TIMESTAMP"},
    {CmdId::kPushLog, "PUSH_LOG"},
    {CmdId::kCollectionLog, "COLLECTION_LOG"},
    {CmdId::kDebugDataControl, "DEBUG_DATA_CONTROL"},
  });
}

TEST_CASE("to_string: values outside the enum do not crash", "[to_string]")
{
  // A LiDAR (or a corrupt datagram) may send values the enums do not list.
  // NOLINTBEGIN(clang-analyzer-optin.core.EnumCastOutOfRange): unknown values on purpose
  CHECK(to_string(static_cast<WorkState>(0x03)) == "UNKNOWN");
  CHECK(to_string(static_cast<DataType>(0x04)) == "UNKNOWN");
  CHECK(to_string(static_cast<TimeType>(0x03)) == "UNKNOWN");
  CHECK(to_string(static_cast<CmdId>(0x0302)) == "UNKNOWN");
  CHECK(to_string(static_cast<Key>(0x7FFF)) == "unknown");
  CHECK(to_string(static_cast<HmsLevel>(5)) == "unknown");
  CHECK(to_string(static_cast<ImuGyroRange>(8)) == "unknown");
  CHECK_FALSE(key_value_length(static_cast<Key>(0x7FFF)).has_value());
  // NOLINTEND(clang-analyzer-optin.core.EnumCastOutOfRange)
}

TEST_CASE("to_string: tag confidence and TagInfo", "[to_string]")
{
  check_names<TagConfidence>({
    {TagConfidence::kHigh, "high"},
    {TagConfidence::kMedium, "medium"},
    {TagConfidence::kLow, "low"},
    {TagConfidence::kReserved, "reserved"},
  });
  CHECK(to_string(decode_tag(0b00'10'01'00)) == "glue=high particles=medium other=low");
  // The reserved bits are only printed when set.
  CHECK(
    to_string(decode_tag(0b01'00'00'11)) ==
    "glue=reserved particles=high other=high reserved=medium");
}

TEST_CASE("to_string: hms levels", "[to_string][hms]")
{
  check_names<HmsLevel>({
    {HmsLevel::kNone, "none"},
    {HmsLevel::kInfo, "info"},
    {HmsLevel::kWarning, "warning"},
    {HmsLevel::kError, "error"},
    {HmsLevel::kFatal, "fatal"},
  });
}

TEST_CASE("every key has its wire name and value length", "[to_string][keys]")
{
  struct Row
  {
    Key key;
    std::string_view name;
    std::size_t length;
  };
  // Names and lengths as listed in the Livox Mid-360 protocol wiki.
  const Row rows[] = {
    {Key::kPclDataType, "pcl_data_type", 1},
    {Key::kPatternMode, "pattern_mode", 1},
    {Key::kLidarIpCfg, "lidar_ipcfg", 12},
    {Key::kStateInfoHostIpCfg, "state_info_host_ipcfg", 8},
    {Key::kPointCloudHostIpCfg, "pointcloud_host_ipcfg", 8},
    {Key::kImuHostIpCfg, "imu_host_ipcfg", 8},
    {Key::kLogHostIpCfg, "log_host_ipcfg", 8},
    {Key::kInstallAttitude, "install_attitude", 24},
    {Key::kFovCfg0, "fov_cfg0", 20},
    {Key::kFovCfg1, "fov_cfg1", 20},
    {Key::kFovCfgEn, "fov_cfg_en", 1},
    {Key::kDetectMode, "detect_mode", 1},
    {Key::kFuncIoCfg, "func_io_cfg", 4},
    {Key::kWorkTgtMode, "work_tgt_mode", 1},
    {Key::kImuDataEn, "imu_data_en", 1},
    {Key::kSpeedMode, "speed_mode", 1},
    {Key::kTimeFilter, "time_filter", 1},
    {Key::kPcFreqMod, "pc_freq_mod", 1},
    {Key::kImuSensorCfg, "imu_sensor_cfg", 3},
    {Key::kSn, "sn", 16},
    {Key::kProductInfo, "product_info", 64},
    {Key::kVersionApp, "version_app", 4},
    {Key::kVersionLoader, "version_loader", 4},
    {Key::kVersionHardware, "version_hardware", 4},
    {Key::kMac, "mac", 6},
    {Key::kCurWorkState, "cur_work_state", 1},
    {Key::kCoreTemp, "core_temp", 4},
    {Key::kPowerupCnt, "powerup_cnt", 4},
    {Key::kLocalTimeNow, "local_time_now", 8},
    {Key::kLastSyncTime, "last_sync_time", 8},
    {Key::kTimeOffset, "time_offset", 8},
    {Key::kTimeSyncType, "time_sync_type", 1},
    {Key::kLidarDiagStatus, "lidar_diag_status", 2},
    {Key::kFwType, "FW_TYPE", 1},
    {Key::kHmsCode, "hms_code", 32},
  };
  for (const Row & row : rows) {
    CAPTURE(row.name);
    CHECK(to_string(row.key) == row.name);
    CHECK(key_value_length(row.key) == row.length);
  }
}

TEST_CASE("to_string: typed key value enums", "[to_string][lidar_info]")
{
  check_names<ScanPattern>({
    {ScanPattern::kNonRepetitive, "non_repetitive"},
    {ScanPattern::kRepetitive, "repetitive"},
    {ScanPattern::kLowRateRepetitive, "low_rate_repetitive"},
  });
  check_names<DetectMode>({{DetectMode::kNormal, "normal"}, {DetectMode::kSensitive, "sensitive"}});
  check_names<TimeSyncType>({
    {TimeSyncType::kNone, "none"},
    {TimeSyncType::kPtp, "ptp"},
    {TimeSyncType::kGps, "gps"},
  });
  check_names<FwType>({{FwType::kLoader, "loader"}, {FwType::kApp, "app"}});
  check_names<ImuOutputRate>({
    {ImuOutputRate::k200Hz, "200Hz"},
    {ImuOutputRate::k500Hz, "500Hz"},
    {ImuOutputRate::k100Hz, "100Hz"},
    {ImuOutputRate::k50Hz, "50Hz"},
  });
  check_names<ImuAccelRange>({
    {ImuAccelRange::k4g, "4g"},
    {ImuAccelRange::k8g, "8g"},
    {ImuAccelRange::k16g, "16g"},
    {ImuAccelRange::k32g, "32g"},
  });
  check_names<ImuGyroRange>({
    {ImuGyroRange::k2000dps, "2000dps"},
    {ImuGyroRange::k1000dps, "1000dps"},
    {ImuGyroRange::k500dps, "500dps"},
    {ImuGyroRange::k250dps, "250dps"},
    {ImuGyroRange::k125dps, "125dps"},
    {ImuGyroRange::k62_5dps, "62.5dps"},
    {ImuGyroRange::k31_25dps, "31.25dps"},
    {ImuGyroRange::k15_625dps, "15.625dps"},
  });
  check_names<FuncOut>({
    {FuncOut::kNone, "none"},
    {FuncOut::kFollowInput, "follow_input"},
    {FuncOut::kSafetyZone, "safety_zone"},
  });
  check_names<DiagLevel>({
    {DiagLevel::kNormal, "normal"},
    {DiagLevel::kWarning, "warning"},
    {DiagLevel::kError, "error"},
    {DiagLevel::kSafetyError, "safety_error"},
  });
  CHECK(
    to_string(TimeSyncStatus{
      .local_time_ns = 1000,
      .last_sync_time_ns = 900,
      .offset_ns = -5,
      .type = TimeSyncType::kGps}) == "local_time=1000 last_sync=900 offset=-5 type=gps");
}

TEST_CASE("to_string: transport error codes", "[to_string][transport]")
{
  check_names<TransportErrorCode>({
    {TransportErrorCode::kSocketCreate, "socket_create"},
    {TransportErrorCode::kBind, "bind"},
    {TransportErrorCode::kSetOption, "set_option"},
    {TransportErrorCode::kAddressInUse, "address_in_use"},
    {TransportErrorCode::kNetworkUnreachable, "network_unreachable"},
    {TransportErrorCode::kSendFailed, "send_failed"},
    {TransportErrorCode::kMessageTooLong, "message_too_long"},
    {TransportErrorCode::kWouldBlock, "would_block"},
    {TransportErrorCode::kTimeout, "timeout"},
    {TransportErrorCode::kInterrupted, "interrupted"},
    {TransportErrorCode::kClosed, "closed"},
    {TransportErrorCode::kInvalidArgument, "invalid_argument"},
    {TransportErrorCode::kOther, "other"},
  });
}

TEST_CASE("to_string: SessionError names the command and the cause", "[to_string][session]")
{
  check_names<DeviceType>({
    {DeviceType::kUnknown, "unknown"},
    {DeviceType::kMid360, "mid360"},
  });

  check_names<SessionErrorKind>({
    {SessionErrorKind::kTransport, "transport"},
    {SessionErrorKind::kTimeout, "timeout"},
    {SessionErrorKind::kBadResponse, "bad_response"},
    {SessionErrorKind::kLidarRejected, "lidar_rejected"},
    {SessionErrorKind::kUnexpectedState, "unexpected_state"},
    {SessionErrorKind::kCancelled, "cancelled"},
    {SessionErrorKind::kInvalidArgument, "invalid_argument"},
  });

  SessionError e;
  e.kind = SessionErrorKind::kTimeout;
  CHECK(to_string(e) == "timeout");  // no command in flight: nothing appended
  e.cmd_id = 0x0101;
  e.attempts = 3;
  CHECK(to_string(e) == "timeout cmd 0x0101 after 3 attempt(s)");
  e.attempts = 0;
  e.work_state = WorkState::kMotorStartup;  // wait_for_state() gave up (#221)
  CHECK(to_string(e) == "timeout cmd 0x0101 after 0 attempt(s): MOTORSTARTUP");

  e = {};
  e.kind = SessionErrorKind::kTransport;
  CHECK(to_string(e) == "transport");
  e.transport = TransportError{TransportErrorCode::kMessageTooLong, 0};
  e.cmd_id = 0x0100;
  CHECK(to_string(e) == "transport cmd 0x0100 after 0 attempt(s): message_too_long");

  e = {};
  e.kind = SessionErrorKind::kBadResponse;
  e.cmd_id = 0x0000;
  e.attempts = 1;
  CHECK(to_string(e) == "bad_response cmd 0x0000 after 1 attempt(s)");
  e.parse = ParseError::kBadCrc32;
  CHECK(to_string(e) == "bad_response cmd 0x0000 after 1 attempt(s): bad CRC32");

  e = {};
  e.kind = SessionErrorKind::kLidarRejected;
  e.cmd_id = 0x0100;
  e.attempts = 1;
  e.ret_code = RetCode::kParamReadOnly;
  CHECK(to_string(e) == "lidar_rejected cmd 0x0100 after 1 attempt(s): ret PARAM_RD_ONLY");
  e.error_key = 0x8000;
  CHECK(
    to_string(e) == "lidar_rejected cmd 0x0100 after 1 attempt(s): ret PARAM_RD_ONLY key 0x8000");

  e = {};
  e.kind = SessionErrorKind::kUnexpectedState;
  CHECK(to_string(e) == "unexpected_state");
  e.work_state = WorkState::kError;
  CHECK(to_string(e) == "unexpected_state: ERROR");

  e = {};
  e.kind = SessionErrorKind::kInvalidArgument;
  CHECK(to_string(e) == "invalid_argument");
  e.error_key = 0x001a;
  CHECK(to_string(e) == "invalid_argument: key 0x001a");

  e = {};
  e.kind = SessionErrorKind::kCancelled;
  e.cmd_id = 0x0101;
  CHECK(to_string(e) == "cancelled cmd 0x0101 after 0 attempt(s)");
}

TEST_CASE("to_string: events and device errors", "[to_string][device]")
{
  check_names<Event::Kind>({
    {Event::Kind::kStateChanged, "state_changed"},
    {Event::Kind::kHms, "hms"},
    {Event::Kind::kDiagChanged, "diag_changed"},
    {Event::Kind::kDisconnected, "disconnected"},
    {Event::Kind::kReconnected, "reconnected"},
    {Event::Kind::kStats, "stats"},
    {Event::Kind::kFirmwareLogGap, "firmware_log_gap"},
  });
  check_names<DisconnectReason>({
    {DisconnectReason::kNone, "none"},
    {DisconnectReason::kPushTimeout, "push_timeout"},
    {DisconnectReason::kCommandTimeout, "command_timeout"},
    {DisconnectReason::kRebootRequested, "reboot_requested"},
    {DisconnectReason::kUser, "user"},
  });
  check_names<DeviceError::Kind>({
    {DeviceError::Kind::kSession, "session"},
    {DeviceError::Kind::kInvalidArgument, "invalid_argument"},
    {DeviceError::Kind::kInvalidState, "invalid_state"},
    {DeviceError::Kind::kAlreadyRegistered, "already_registered"},
    {DeviceError::Kind::kNotOpen, "not_open"},
    {DeviceError::Kind::kDisconnected, "disconnected"},
    {DeviceError::Kind::kDecodeFailed, "decode_failed"},
    {DeviceError::Kind::kIo, "io"},
  });

  Event e;
  e.kind = Event::Kind::kStats;
  e.stats.packets = 1;
  e.stats.points = 2;
  e.stats.frames = 3;
  e.stats.imu_samples = 4;
  e.stats.bad_packets = 5;
  e.stats.dropped_packets = 6;
  e.stats.reordered = 7;
  CHECK(to_string(e) == "stats packets=1 points=2 frames=3 imu=4 bad=5 dropped=6 reordered=7");
  e.kind = Event::Kind::kDisconnected;
  e.reason = DisconnectReason::kPushTimeout;
  CHECK(to_string(e) == "disconnected reason=push_timeout");
  e.kind = Event::Kind::kReconnected;
  e.attempts = 4;
  CHECK(to_string(e) == "reconnected attempts=4");
  e.kind = Event::Kind::kFirmwareLogGap;
  e.log_file_index = 2;
  e.log_expected = 10;
  e.log_actual = 12;
  CHECK(to_string(e) == "firmware_log_gap file=2 expected=10 actual=12");

  DeviceError err;
  err.kind = DeviceError::Kind::kDecodeFailed;
  err.key = Key::kFovCfg0;
  CHECK(to_string(err) == "decode_failed: key fov_cfg0");
  err = {};
  err.kind = DeviceError::Kind::kIo;
  CHECK(to_string(err) == "io");
  err.errno_value = ENOENT;
  CHECK(to_string(err) == std::string("io: ") + std::strerror(ENOENT));
  // errno is only meaningful for kIo
  err.kind = DeviceError::Kind::kNotOpen;
  CHECK(to_string(err) == "not_open");
}
