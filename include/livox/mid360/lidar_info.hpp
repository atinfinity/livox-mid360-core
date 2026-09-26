// SPDX-License-Identifier: Apache-2.0
// Aggregated read-back types over the keys.hpp codecs (issues #38 / #41): the identity of a
// LiDAR (serial, product string, firmware versions, MAC). Each type has a key list, a
// decoder that takes any parsed key-value list (an InquireResult or a 0x0102 push) and a
// one-line to_string with the wire key names, so logs stay greppable.
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>

#include "livox/mid360/export.hpp"
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

}  // namespace livox::mid360
LIVOX_MID360_API_END
