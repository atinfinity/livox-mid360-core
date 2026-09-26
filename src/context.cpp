// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/context.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <vector>

#include "context_impl.hpp"
#include "log_detail.hpp"

namespace livox::mid360
{

namespace
{

using Clock = std::chrono::steady_clock;

constexpr std::uint64_t kPushTag = 1;
constexpr std::uint64_t kPointTag = 2;
constexpr std::uint64_t kImuTag = 3;
constexpr std::uint64_t kLogTag = 4;
constexpr auto kMaxPoll = std::chrono::milliseconds{100};

DeviceError transport_error(const TransportError & err)
{
  SessionError s;
  s.kind = SessionErrorKind::kTransport;
  s.transport = err;
  return DeviceError{.kind = DeviceError::Kind::kSession, .session = s, .key = std::nullopt};
}

}  // namespace

Context::Impl::AddResult Context::Impl::add(
  const Ipv4 & ip, std::string serial, detail::Receiver * receiver)
{
  const std::lock_guard lock(mutex);
  if (std::ranges::any_of(entries, [&](const Entry & e) { return e.ip == ip; })) {
    return AddResult::kDuplicateIp;
  }
  if (std::ranges::any_of(entries, [&](const Entry & e) { return e.serial == serial; })) {
    return AddResult::kDuplicateSerial;
  }
  entries.push_back({ip, receiver, std::move(serial), nullptr});
  generation.fetch_add(1, std::memory_order_release);
  poller.wake();
  return AddResult::kOk;
}

void Context::Impl::bind(detail::Receiver * receiver, Device * device)
{
  const std::lock_guard lock(mutex);
  for (Entry & e : entries) {
    if (e.receiver == receiver) {
      e.device = device;
    }
  }
}

bool Context::Impl::rekey(detail::Receiver * receiver, const Ipv4 & ip)
{
  const std::lock_guard lock(mutex);
  const auto it =
    std::ranges::find_if(entries, [&](const Entry & e) { return e.receiver == receiver; });
  if (it == entries.end()) {
    return false;
  }
  if (it->ip == ip) {
    return true;
  }
  if (std::ranges::any_of(entries, [&](const Entry & e) { return e.ip == ip; })) {
    return false;
  }
  it->ip = ip;
  generation.fetch_add(1, std::memory_order_release);
  poller.wake();
  return true;
}

void Context::Impl::remove(detail::Receiver * receiver)
{
  assert(!on_receive_thread() && "Device destroyed from its own callback");
  std::unique_lock lock(mutex);
  std::erase_if(entries, [&](const Entry & e) { return e.receiver == receiver; });
  const std::uint64_t target = generation.fetch_add(1, std::memory_order_release) + 1;
  poller.wake();
  cv.wait(lock, [&] { return observed >= target || stop.load(std::memory_order_acquire); });
}

void Context::Impl::run()
{
  std::vector<std::array<std::byte, kMaxDatagramSize>> storage(options.batch_size);
  std::vector<Datagram> batch(options.batch_size);
  std::vector<Entry> snapshot;
  std::uint64_t seen = 0;

  while (!stop.load(std::memory_order_acquire)) {
    if (generation.load(std::memory_order_acquire) != seen) {
      const std::lock_guard lock(mutex);
      snapshot = entries;
      seen = generation.load(std::memory_order_acquire);
      observed = seen;
      cv.notify_all();
    }

    // Timers first: they also give the poll timeout.
    Clock::time_point now = Clock::now();
    Clock::time_point deadline = now + kMaxPoll;
    for (const Entry & e : snapshot) {
      if (const auto next = e.receiver->tick(now)) {
        deadline = std::min(deadline, *next);
      }
    }
    now = Clock::now();
    const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::max(deadline, now) - now + std::chrono::microseconds{999});

    const auto ready = poller.wait(timeout);
    if (!ready) {
      if (ready.error().code != TransportErrorCode::kInterrupted) {
        LIVOX_LOG(LogLevel::kError, {}, "poll failed: {}", to_string(ready.error()));
      }
      continue;  // EINTR or similar; the loop re-evaluates `stop`
    }
    for (const ReadyEvent & ev : *ready) {
      const UdpSocket * socket = nullptr;
      detail::DataPort port = detail::DataPort::kPush;
      switch (ev.tag) {
        case kPushTag:
          socket = &push_socket;
          port = detail::DataPort::kPush;
          break;
        case kPointTag:
          socket = &point_socket;
          port = detail::DataPort::kPoint;
          break;
        case kImuTag:
          socket = &imu_socket;
          port = detail::DataPort::kImu;
          break;
        case kLogTag:
          socket = &log_socket;
          port = detail::DataPort::kLog;
          break;
        default:
          continue;
      }
      // Drain the socket; bounded so that one busy socket cannot starve the others.
      for (int round = 0; round < 8; ++round) {
        for (std::size_t i = 0; i < batch.size(); ++i) {
          batch[i].data = storage[i];
        }
        const auto n = socket->recv_batch(batch);
        if (!n) {
          if (n.error().code != TransportErrorCode::kWouldBlock) {
            LIVOX_LOG(LogLevel::kError, {}, "recv failed: {}", to_string(n.error()));
          }
          break;
        }
        datagrams.fetch_add(*n, std::memory_order_relaxed);
        if (port == detail::DataPort::kLog) {
          log_datagrams.fetch_add(*n, std::memory_order_relaxed);
        }
        for (std::size_t i = 0; i < *n; ++i) {
          const Datagram & d = batch[i];
          const auto it =
            std::ranges::find_if(snapshot, [&](const Entry & e) { return e.ip == d.from.ip; });
          if (it == snapshot.end()) {
            unknown_source.fetch_add(1, std::memory_order_relaxed);
            LIVOX_LOG(
              LogLevel::kDebug, {}, "datagram from unknown source {}:{}", ip_to_string(d.from.ip),
              d.from.port);
            continue;
          }
          it->receiver->on_datagram(port, d);
        }
        if (*n < batch.size()) {
          break;
        }
      }
    }
  }
  // Let a remove() that raced with shutdown return.
  const std::lock_guard lock(mutex);
  cv.notify_all();
}

std::expected<std::unique_ptr<Context>, DeviceError> Context::create(const ContextOptions & opts)
{
  if (opts.batch_size == 0) {
    return std::unexpected(DeviceError{
      .kind = DeviceError::Kind::kInvalidArgument, .session = std::nullopt, .key = std::nullopt});
  }
  auto impl = std::make_unique<Impl>();
  impl->options = opts;
  SocketOptions sopts;
  sopts.recv_buffer_bytes = opts.recv_buffer_bytes;
  auto open = [&](std::uint16_t port) {
    return UdpSocket::open(Endpoint{opts.bind_address, port}, sopts);
  };
  auto push = open(opts.push_port);
  if (!push) {
    return std::unexpected(transport_error(push.error()));
  }
  auto point = open(opts.point_port);
  if (!point) {
    return std::unexpected(transport_error(point.error()));
  }
  auto imu = open(opts.imu_port);
  if (!imu) {
    return std::unexpected(transport_error(imu.error()));
  }
  auto log = open(opts.log_port);
  if (!log) {
    return std::unexpected(transport_error(log.error()));
  }
  auto poller = Poller::create();
  if (!poller) {
    return std::unexpected(transport_error(poller.error()));
  }
  impl->push_socket = std::move(*push);
  impl->point_socket = std::move(*point);
  impl->imu_socket = std::move(*imu);
  impl->log_socket = std::move(*log);
  impl->poller = std::move(*poller);
  for (auto [socket, tag] :
       {std::pair{&impl->push_socket, kPushTag}, std::pair{&impl->point_socket, kPointTag},
        std::pair{&impl->imu_socket, kImuTag}, std::pair{&impl->log_socket, kLogTag}}) {
    if (auto r = impl->poller.add(*socket, tag); !r) {
      return std::unexpected(transport_error(r.error()));
    }
  }
  impl->options.push_port = impl->push_socket.local_endpoint().port;
  impl->options.point_port = impl->point_socket.local_endpoint().port;
  impl->options.imu_port = impl->imu_socket.local_endpoint().port;
  impl->options.log_port = impl->log_socket.local_endpoint().port;

  Impl * raw = impl.get();
  impl->thread = std::thread([raw] { raw->run(); });
  impl->thread_id = impl->thread.get_id();
  return std::unique_ptr<Context>(new Context(std::move(impl)));
}

Context::Context(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

Context::~Context()
{
  {
    const std::lock_guard lock(impl_->mutex);
    assert(impl_->entries.empty() && "Context destroyed before its Devices");
  }
  impl_->stop.store(true, std::memory_order_release);
  impl_->poller.wake();
  if (impl_->thread.joinable()) {
    impl_->thread.join();
  }
}

const ContextOptions & Context::options() const noexcept { return impl_->options; }

Device * Context::find(std::string_view serial_number) const
{
  const std::lock_guard lock(impl_->mutex);
  for (const Impl::Entry & e : impl_->entries) {
    if (e.serial == serial_number) {
      return e.device;
    }
  }
  return nullptr;
}

std::vector<Device *> Context::devices() const
{
  const std::lock_guard lock(impl_->mutex);
  std::vector<Device *> out;
  for (const Impl::Entry & e : impl_->entries) {
    if (e.device != nullptr) {
      out.push_back(e.device);
    }
  }
  return out;
}

ContextStats Context::stats() const
{
  return ContextStats{
    .datagrams = impl_->datagrams.load(std::memory_order_relaxed),
    .unknown_source = impl_->unknown_source.load(std::memory_order_relaxed),
    .log_datagrams = impl_->log_datagrams.load(std::memory_order_relaxed),
  };
}

}  // namespace livox::mid360
