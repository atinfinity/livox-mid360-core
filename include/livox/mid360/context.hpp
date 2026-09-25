// SPDX-License-Identifier: Apache-2.0
// Public API skeleton (issue #9): the shared receive side. The Mid-360 host ports for push
// (56201), point cloud (56301) and IMU (56401) are the same for every LiDAR, so one Context
// owns those three sockets and the single receive thread, and dispatches datagrams to the
// registered Devices by source IP. Implemented in #6 (data) and #7 (push).
#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>

#include "livox/mid360/event.hpp"
#include "livox/mid360/export.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/transport.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360 {

struct ContextOptions {
  Ipv4 bind_address{0, 0, 0, 0};  ///< interface for the three receive sockets
  std::uint16_t push_port = kDefaultHostPushPort;
  std::uint16_t point_port = kDefaultHostPointCloudPort;
  std::uint16_t imu_port = kDefaultHostImuPort;
  std::size_t recv_buffer_bytes = 4u << 20;  ///< SO_RCVBUF request per socket
  std::size_t batch_size = 32;               ///< datagrams per recvmmsg
};

class Device;

/// Owns the receive sockets and the receive thread. The thread starts in create() and is
/// joined by the destructor. Every Device opened on a Context must be destroyed before it
/// (asserted in debug builds). Non-copyable, non-movable: Devices hold a reference.
class Context {
 public:
  [[nodiscard]] static std::expected<std::unique_ptr<Context>, DeviceError> create(
      const ContextOptions& opts = {});

  ~Context();
  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;
  Context(Context&&) = delete;
  Context& operator=(Context&&) = delete;

  [[nodiscard]] const ContextOptions& options() const noexcept;
  [[nodiscard]] ContextStats stats() const;

 private:
  friend class Device;
  struct Impl;
  explicit Context(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace livox::mid360
LIVOX_MID360_API_END
