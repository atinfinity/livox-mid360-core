// SPDX-License-Identifier: Apache-2.0
// Internal (not installed): Context::Impl and the interface a Device exposes to the receive
// thread. Shared by context.cpp and device.cpp only.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "livox/mid360/context.hpp"
#include "livox/mid360/transport.hpp"

namespace livox::mid360 {

namespace detail {

enum class DataPort : std::uint8_t { kPush, kPoint, kImu };

/// What the receive thread calls on a registered Device. Both methods run on the receive
/// thread only.
class Receiver {
 public:
  Receiver() = default;
  Receiver(const Receiver&) = delete;
  Receiver& operator=(const Receiver&) = delete;
  virtual ~Receiver() = default;

  virtual void on_datagram(DataPort port, const Datagram& datagram) = 0;
  /// Timers (idle frame close, stats). Returns the next time it wants to be called.
  virtual std::optional<std::chrono::steady_clock::time_point> tick(
      std::chrono::steady_clock::time_point now) = 0;
};

}  // namespace detail

struct Context::Impl {
  struct Entry {
    Ipv4 ip;
    detail::Receiver* receiver;
  };

  ContextOptions options;
  UdpSocket push_socket;
  UdpSocket point_socket;
  UdpSocket imu_socket;
  Poller poller;

  std::thread thread;
  std::atomic<bool> stop{false};
  std::thread::id thread_id;

  // Registry: written by user threads under `mutex`, snapshotted by the receive thread when
  // `generation` changes. `observed` is the generation the receive thread last snapshotted;
  // once it reaches the caller's generation no callback of a removed entry is in flight.
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<Entry> entries;
  std::atomic<std::uint64_t> generation{1};
  std::uint64_t observed = 0;

  std::atomic<std::uint64_t> datagrams{0};
  std::atomic<std::uint64_t> unknown_source{0};

  /// False when the IP is already registered.
  [[nodiscard]] bool add(const Ipv4& ip, detail::Receiver* receiver);
  /// Removes and blocks until the receive thread has dropped its snapshot of `receiver`.
  void remove(detail::Receiver* receiver);
  [[nodiscard]] bool on_receive_thread() const noexcept {
    return std::this_thread::get_id() == thread_id;
  }

  void run();
};

}  // namespace livox::mid360
