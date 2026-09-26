// SPDX-License-Identifier: Apache-2.0
// Public API (issue #9): the shared receive side. The Mid-360 host ports for push
// (56201), point cloud (56301), IMU (56401) and firmware log (56501, #44) are the same for
// every LiDAR, so one Context owns those four sockets and the single receive thread, and dispatches datagrams to the
// registered Devices by source IP. Data path implemented in #6; push parsing is #7.
#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <string_view>
#include <vector>

#include "livox/mid360/event.hpp"
#include "livox/mid360/export.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/transport.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

struct ContextOptions
{
  Ipv4 bind_address{0, 0, 0, 0};                   ///< interface for the four receive sockets
  std::uint16_t push_port = kDefaultHostPushPort;  ///< 0 = ephemeral (tests)
  std::uint16_t point_port = kDefaultHostPointCloudPort;  ///< 0 = ephemeral
  std::uint16_t imu_port = kDefaultHostImuPort;           ///< 0 = ephemeral
  std::uint16_t log_port = kDefaultHostLogPort;           ///< firmware log (#44); 0 = ephemeral
  std::size_t recv_buffer_bytes = 4u << 20;               ///< SO_RCVBUF request per socket
  std::size_t batch_size = 32;                            ///< datagrams per recvmmsg
};

class Device;

/// Owns the receive sockets and the receive thread. The thread starts in create() and is
/// joined by the destructor. Every Device opened on a Context must be destroyed before it
/// (asserted in debug builds). Non-copyable, non-movable: Devices hold a reference.
class Context
{
public:
  [[nodiscard]] static std::expected<std::unique_ptr<Context>, DeviceError> create(
    const ContextOptions & opts = {});

  ~Context();
  Context(const Context &) = delete;
  Context & operator=(const Context &) = delete;
  Context(Context &&) = delete;
  Context & operator=(Context &&) = delete;

  /// Effective options: ports requested as 0 are replaced by the bound ones.
  [[nodiscard]] const ContextOptions & options() const noexcept;
  [[nodiscard]] ContextStats stats() const;

  // --- multi-device (issue #8): the Devices open on this Context, keyed by serial number.
  // Non-owning: the caller keeps the unique_ptr and destroys it before the Context.
  /// The open Device with this serial number, or nullptr.
  [[nodiscard]] Device * find(std::string_view serial_number) const;
  /// Every open Device, in registration order.
  [[nodiscard]] std::vector<Device *> devices() const;

private:
  friend class Device;
  struct Impl;
  explicit Context(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace livox::mid360
LIVOX_MID360_API_END
