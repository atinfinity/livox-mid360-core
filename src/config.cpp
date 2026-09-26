// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/config.hpp"

#include <array>
#include <cstring>
#include <optional>
#include <vector>

namespace livox::mid360
{

namespace
{
SessionError invalid_argument(std::uint16_t error_key)
{
  SessionError err;
  err.kind = SessionErrorKind::kInvalidArgument;
  err.cmd_id = static_cast<std::uint16_t>(CmdId::kParamConfig);
  err.error_key = error_key;
  return err;
}

/// The key of the first out-of-range window, if any.
std::optional<Key> fov_out_of_range(const std::optional<FovSettings> & fov)
{
  if (!fov) {
    return std::nullopt;
  }
  if (fov->fov0 && !fov_in_range(*fov->fov0)) {
    return Key::kFovCfg0;
  }
  if (fov->fov1 && !fov_in_range(*fov->fov1)) {
    return Key::kFovCfg1;
  }
  return std::nullopt;
}

bool is_requestable(WorkState s)
{
  return s == WorkState::kSampling || s == WorkState::kIdle || s == WorkState::kReady;
}
}  // namespace

HostSetupKeyValues host_setup_key_values(const HostSetup & setup, const Ipv4 & host_ip)
{
  HostSetupKeyValues out;
  out.storage.reserve(8 * 3 + 2 + 20 * 2 + 1);
  std::vector<std::size_t> lengths;
  const auto put = [&](Key key, std::span<const std::byte> bytes) {
    out.storage.insert(out.storage.end(), bytes.begin(), bytes.end());
    out.values.push_back({static_cast<std::uint16_t>(key), {}});
    lengths.push_back(bytes.size());
  };
  put(Key::kStateInfoHostIpCfg, encode_host_ip_config({host_ip, setup.push_port, kPushPort}));
  put(
    Key::kPointCloudHostIpCfg, encode_host_ip_config({host_ip, setup.point_port, kPointCloudPort}));
  put(Key::kImuHostIpCfg, encode_host_ip_config({host_ip, setup.imu_port, kImuPort}));
  put(Key::kPclDataType, encode_u8(static_cast<std::uint8_t>(setup.pcl_data_type)));
  put(Key::kImuDataEn, encode_u8(setup.imu_enable ? 1 : 0));
  if (setup.fov) {
    if (setup.fov->fov0) {
      put(Key::kFovCfg0, encode_fov_config(*setup.fov->fov0));
    }
    if (setup.fov->fov1) {
      put(Key::kFovCfg1, encode_fov_config(*setup.fov->fov1));
    }
    if (setup.fov->enable) {
      put(Key::kFovCfgEn, encode_fov_enable(*setup.fov->enable));
    }
  }
  // Fix the views up once storage has its final size.
  std::size_t off = 0;
  for (std::size_t i = 0; i < out.values.size(); ++i) {
    out.values[i].value = std::span<const std::byte>(out.storage).subspan(off, lengths[i]);
    off += lengths[i];
  }
  return out;
}

std::expected<HostSetupResult, SessionError> apply_host_setup(
  Session & session, const HostSetup & setup, std::optional<RequestOptions> opts)
{
  Ipv4 ip = setup.ip.value_or(session.local_endpoint().ip);
  if (ip == Ipv4{0, 0, 0, 0}) {
    return std::unexpected(invalid_argument(static_cast<std::uint16_t>(Key::kPointCloudHostIpCfg)));
  }
  if (setup.work_tgt_mode && !is_requestable(*setup.work_tgt_mode)) {
    return std::unexpected(invalid_argument(static_cast<std::uint16_t>(Key::kWorkTgtMode)));
  }
  if (const auto bad = fov_out_of_range(setup.fov)) {
    return std::unexpected(invalid_argument(static_cast<std::uint16_t>(*bad)));
  }

  HostSetupResult result;
  const auto kvs = host_setup_key_values(setup, ip);
  const auto ack = session.configure(kvs.values, opts);
  if (!ack) {
    return std::unexpected(ack.error());
  }
  result.reboot_required = ack->ret_code == RetCode::kParamRebootEffect;

  if (setup.work_tgt_mode) {
    const auto mode = encode_u8(static_cast<std::uint8_t>(*setup.work_tgt_mode));
    const KeyValue kv{static_cast<std::uint16_t>(Key::kWorkTgtMode), mode};
    const auto ack2 = session.configure(std::span<const KeyValue>(&kv, 1), opts);
    if (!ack2) {
      return std::unexpected(ack2.error());
    }
    result.reboot_required =
      result.reboot_required || ack2->ret_code == RetCode::kParamRebootEffect;
    if (setup.wait_timeout.count() > 0) {
      const auto w = session.wait_for_state(*setup.work_tgt_mode, setup.wait_timeout);
      if (!w) {
        return std::unexpected(w.error());
      }
      result.final_state = setup.work_tgt_mode;
    }
  }
  return result;
}

}  // namespace livox::mid360
