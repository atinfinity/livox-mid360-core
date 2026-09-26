// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/lidar_info.hpp"

#include <cstdio>

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

}  // namespace livox::mid360
