// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/device.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

#include "context_impl.hpp"
#include "frame_assembler.hpp"

namespace livox::mid360
{

namespace
{

using Clock = std::chrono::steady_clock;

std::uint64_t realtime_now_ns() noexcept
{
  timespec ts{};
  ::clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ull +
         static_cast<std::uint64_t>(ts.tv_nsec);
}

DeviceError wrap(const SessionError & err)
{
  return DeviceError{.kind = DeviceError::Kind::kSession, .session = err, .key = std::nullopt};
}

DeviceError error(DeviceError::Kind kind)
{
  return DeviceError{.kind = kind, .session = std::nullopt, .key = std::nullopt};
}

}  // namespace

struct Device::Impl : detail::Receiver
{
  Impl(
    Context::Impl & ctx, DiscoveredDevice dev, const DeviceOptions & o, HostSetup setup,
    SessionOptions sopts, std::stop_source stop_source, Session s)
  : context(ctx),
    options(o),
    host_setup(setup),
    session_options(std::move(sopts)),
    stop(std::move(stop_source)),
    session(std::move(s)),
    info_(std::move(dev)),
    assembler(o.frame_policy, o.timestamp_policy),
    idle_window(std::chrono::duration_cast<Clock::duration>(o.frame_policy.window)),
    next_stats(Clock::now() + o.stats_interval)
  {
    last_push_steady_ns.store(Clock::now().time_since_epoch().count());
  }

  // --- fixed after open ---------------------------------------------------
  Context::Impl & context;
  const DeviceOptions options;
  const HostSetup host_setup;            ///< as applied at open(); replayed on reconnect
  const SessionOptions session_options;  ///< with `stop` and verify_serial for reconnects
  std::stop_source stop;                 ///< requested by the destructor

  // --- commands (caller threads, serialised) -------------------------------
  std::mutex cmd_mutex;
  Session session;  ///< replaced by a reconnect under cmd_mutex + conn_mutex
  std::atomic<bool> sampling_requested{false};
  std::atomic<bool> discard_requested{false};
  std::atomic<bool> rebase_requested{false};  ///< reconnect: re-measure the time offset

  // --- connection (issue #8) --------------------------------------------------
  std::atomic<bool> connected{true};
  mutable std::mutex conn_mutex;  ///< info_, pending, worker hand-off, session swap / cancel
  std::condition_variable conn_cv;
  DiscoveredDevice info_;
  std::vector<Event> pending;  ///< raised on the receive thread at the next tick
  std::thread worker;
  bool worker_started = false;
  std::atomic<std::uint32_t> attempts{0};
  std::atomic<std::uint64_t> disconnects{0};
  std::atomic<std::uint64_t> reconnects{0};
  std::atomic<std::int64_t> last_push_steady_ns{0};  ///< steady_clock ticks of the last push

  // --- callbacks: written under cb_mutex, copied by the receive thread on a
  // generation change --------------------------------------------------------
  std::mutex cb_mutex;
  PacketCallback packet_cb;
  FrameCallback frame_cb;
  ImuCallback imu_cb;
  EventCallback event_cb;
  std::atomic<std::uint64_t> cb_generation{1};

  // --- receive-thread state --------------------------------------------------
  std::uint64_t cb_seen = 0;
  PacketCallback rx_packet_cb;
  FrameCallback rx_frame_cb;
  ImuCallback rx_imu_cb;
  EventCallback rx_event_cb;
  detail::FrameAssembler assembler;
  detail::DropCounter imu_drops;
  std::uint64_t imu_dropped = 0;
  std::uint64_t imu_reordered = 0;
  Clock::duration idle_window;
  Clock::time_point last_point_packet;
  Clock::time_point next_stats;

  // --- counters (relaxed; written by the receive thread only) ----------------
  std::atomic<std::uint64_t> packets{0};
  std::atomic<std::uint64_t> points{0};
  std::atomic<std::uint64_t> frames{0};
  std::atomic<std::uint64_t> imu_samples{0};
  std::atomic<std::uint64_t> bad_packets{0};
  std::atomic<std::uint64_t> dropped_packets{0};
  std::atomic<std::uint64_t> reordered{0};
  std::atomic<std::uint64_t> frame_cnt_fallback{0};
  std::atomic<std::uint64_t> last_packet_time_ns{0};
  std::atomic<std::uint64_t> pushes{0};
  std::atomic<std::uint64_t> last_push_time_ns{0};
  std::atomic<std::int64_t> time_offset_ns{0};
  std::atomic<bool> time_offset_valid{false};

  // --- pushed state (written by the receive thread under push_mutex) ------------
  mutable std::mutex push_mutex;
  std::optional<WorkState> pushed_state;
  std::array<HmsCode, 8> pushed_hms{};
  std::array<std::uint32_t, 8> hms_sorted{};  ///< raw codes, sorted, for change detection

  [[nodiscard]] DeviceStats snapshot() const
  {
    constexpr auto kRelaxed = std::memory_order_relaxed;
    return DeviceStats{
      .packets = packets.load(kRelaxed),
      .points = points.load(kRelaxed),
      .frames = frames.load(kRelaxed),
      .imu_samples = imu_samples.load(kRelaxed),
      .bad_packets = bad_packets.load(kRelaxed),
      .dropped_packets = dropped_packets.load(kRelaxed),
      .reordered = reordered.load(kRelaxed),
      .queue_drops = 0,
      .frame_cnt_fallback = frame_cnt_fallback.load(kRelaxed),
      .last_packet_time_ns = last_packet_time_ns.load(kRelaxed),
      .pushes = pushes.load(kRelaxed),
      .last_push_time_ns = last_push_time_ns.load(kRelaxed),
      .disconnects = disconnects.load(kRelaxed),
      .reconnects = reconnects.load(kRelaxed),
      .time_offset_ns = time_offset_ns.load(kRelaxed),
      .time_offset_valid = time_offset_valid.load(kRelaxed),
    };
  }

  void refresh_callbacks()
  {
    const auto gen = cb_generation.load(std::memory_order_acquire);
    if (gen == cb_seen) {
      return;
    }
    const std::lock_guard lock(cb_mutex);
    rx_packet_cb = packet_cb;
    rx_frame_cb = frame_cb;
    rx_imu_cb = imu_cb;
    rx_event_cb = event_cb;
    cb_seen = cb_generation.load(std::memory_order_acquire);
  }

  void publish_assembler_counters()
  {
    constexpr auto kRelaxed = std::memory_order_relaxed;
    const auto & c = assembler.counters();
    points.store(c.points, kRelaxed);
    frames.store(c.frames, kRelaxed);
    dropped_packets.store(c.dropped_packets + imu_dropped, kRelaxed);
    reordered.store(c.reordered + imu_reordered, kRelaxed);
    frame_cnt_fallback.store(c.frame_cnt_fallback, kRelaxed);
    if (const auto off = assembler.time_mapper().offset_ns()) {
      time_offset_ns.store(*off, kRelaxed);
      time_offset_valid.store(true, kRelaxed);
    }
  }

  void deliver(std::optional<Frame> frame)
  {
    publish_assembler_counters();
    if (frame && rx_frame_cb) {
      rx_frame_cb(std::move(*frame));
    }
  }

  // 0x0102: only cur_work_state and hms_code are consumed (issue #7). The first push
  // records the state without a kStateChanged; the HMS baseline is all-zero, so active
  // codes in the first push do raise kHms. Slot order is ignored for change detection.
  void on_push(const Datagram & d)
  {
    const auto frame = parse_command_frame(d.data);
    if (!frame || frame->header.cmd_id != static_cast<std::uint16_t>(CmdId::kInfoPush)) {
      bad_packets.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    const auto push = parse_info_push(frame->data);
    if (!push) {
      bad_packets.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    pushes.fetch_add(1, std::memory_order_relaxed);
    last_push_time_ns.store(d.recv_time_ns, std::memory_order_relaxed);
    last_push_steady_ns.store(Clock::now().time_since_epoch().count(), std::memory_order_relaxed);
    refresh_callbacks();

    std::optional<Event> state_event;
    std::optional<Event> hms_event;
    {
      const std::lock_guard lock(push_mutex);
      if (const auto v = find_key(push->values, Key::kCurWorkState)) {
        if (const auto st = decode_work_state(*v)) {
          if (pushed_state && *pushed_state != *st) {
            Event ev;
            ev.kind = Event::Kind::kStateChanged;
            ev.time_ns = d.recv_time_ns;
            ev.old_state = *pushed_state;
            ev.new_state = *st;
            state_event = ev;
          }
          pushed_state = *st;
        }
      }
      if (const auto v = find_key(push->values, Key::kHmsCode)) {
        if (const auto raw = decode_hms_codes(*v)) {
          for (std::size_t i = 0; i < raw->size(); ++i) {
            pushed_hms[i] = decode_hms((*raw)[i]);
          }
          std::array<std::uint32_t, 8> sorted = *raw;
          std::ranges::sort(sorted);
          if (sorted != hms_sorted) {
            hms_sorted = sorted;
            Event ev;
            ev.kind = Event::Kind::kHms;
            ev.time_ns = d.recv_time_ns;
            ev.hms = pushed_hms;
            for (const HmsCode & c : pushed_hms) {
              if (c.active() && c.level > ev.hms_level) {
                ev.hms_level = c.level;
              }
            }
            hms_event = ev;
          }
        }
      }
    }
    if (rx_event_cb) {
      if (state_event) {
        rx_event_cb(*state_event);
      }
      if (hms_event) {
        rx_event_cb(*hms_event);
      }
    }
  }

  void on_datagram(detail::DataPort port, const Datagram & d) override
  {
    if (port == detail::DataPort::kPush) {
      on_push(d);
      return;
    }
    const auto pkt = parse_data_packet(d.data, options.verify_crc);
    if (!pkt) {
      bad_packets.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    packets.fetch_add(1, std::memory_order_relaxed);
    last_packet_time_ns.store(d.recv_time_ns, std::memory_order_relaxed);
    consume_requests();
    refresh_callbacks();
    if (rx_packet_cb) {
      rx_packet_cb(*pkt, ReceiveInfo{.host_time_ns = d.recv_time_ns, .source = d.from});
    }
    const DataPacketHeader & h = pkt->header;
    if (h.data_type == DataType::kImu) {
      const auto drop = imu_drops.observe(h.udp_cnt, false);
      imu_dropped += drop.dropped;
      imu_reordered += drop.reordered ? 1u : 0u;
      // kHostOffsetOnce measures its offset at the first point-cloud packet; an IMU packet
      // that arrives before it is stamped with the receive time instead.
      detail::TimeMapper & tm = assembler.time_mapper();
      const bool pending_offset = options.timestamp_policy == TimestampPolicy::kHostOffsetOnce &&
                                  !tm.offset_ns() && h.time_type == TimeType::kNoSync;
      const std::uint64_t t0 = pending_offset ? d.recv_time_ns : tm.map(h, d.recv_time_ns);
      for (std::size_t i = 0; i < h.dot_num; ++i) {
        const std::uint64_t t = t0 + (sample_timestamp_ns(h, i) - h.timestamp_ns);
        if (rx_imu_cb) {
          rx_imu_cb(ImuData{.time_ns = t, .sample = decode_imu(*pkt, i)});
        }
      }
      imu_samples.fetch_add(h.dot_num, std::memory_order_relaxed);
      publish_assembler_counters();
      return;
    }
    last_point_packet = Clock::now();
    deliver(assembler.push(*pkt, d.recv_time_ns));
  }

  void consume_requests()
  {
    if (discard_requested.exchange(false, std::memory_order_acq_rel)) {
      assembler.discard();
    }
    if (rebase_requested.exchange(false, std::memory_order_acq_rel)) {
      assembler.time_mapper().reset();
      imu_drops.reset();
    }
  }

  // --- connection state machine (issue #8) ------------------------------------

  [[nodiscard]] Clock::time_point last_push_steady() const noexcept
  {
    return Clock::time_point(Clock::duration(last_push_steady_ns.load(std::memory_order_relaxed)));
  }

  /// connected → disconnected. Any thread. Queues kDisconnected for the receive thread and
  /// wakes the worker. Returns false when already disconnected.
  bool declare_disconnected(DisconnectReason reason)
  {
    bool was = true;
    if (!connected.compare_exchange_strong(was, false, std::memory_order_acq_rel)) {
      return false;
    }
    disconnects.fetch_add(1, std::memory_order_relaxed);
    attempts.store(0, std::memory_order_relaxed);
    discard_requested.store(true, std::memory_order_release);
    Event ev;
    ev.kind = Event::Kind::kDisconnected;
    ev.time_ns = realtime_now_ns();
    ev.reason = reason;
    {
      const std::lock_guard lock(conn_mutex);
      pending.push_back(ev);
      if (options.reconnect.enabled && !worker_started) {
        worker_started = true;
        worker = std::thread([this] { run_worker(); });
      }
    }
    conn_cv.notify_all();
    context.poller.wake();
    return true;
  }

  /// A command timed out: a disconnect only when the push is stale too (one nominal period).
  void note_command_error(const SessionError & err)
  {
    if (err.kind != SessionErrorKind::kTimeout) {
      return;
    }
    if (Clock::now() - last_push_steady() > options.reconnect.push_timeout / 3) {
      declare_disconnected(DisconnectReason::kCommandTimeout);
    }
  }

  /// One recovery attempt; shared by the worker and Device::reconnect(). Holds cmd_mutex.
  std::expected<void, DeviceError> attempt()
  {
    assert(!context.on_receive_thread() && "Device::reconnect() called from a callback");
    const std::lock_guard lock(cmd_mutex);
    if (connected.load(std::memory_order_acquire)) {
      return {};
    }
    const std::uint32_t n = attempts.fetch_add(1, std::memory_order_relaxed) + 1;
    DiscoveredDevice target = snapshot_info();

    // 1. The last known endpoint (a cable pull keeps the address), serial verified.
    auto s = Session::connect(target, session_options);
    if (!s) {
      if (stop.stop_requested()) {
        return std::unexpected(wrap(s.error()));
      }
      // 2. Discovery, filtered by serial (a reboot or DHCP may have moved the LiDAR).
      DiscoveryOptions d;
      d.targets = options.reconnect.discovery_targets;
      d.timeout = options.reconnect.discovery_timeout;
      d.bind_address = session_options.bind_address;
      d.stop = stop.get_token();
      auto found = discover(d);
      if (!found) {
        return std::unexpected(wrap(found.error()));
      }
      const auto it = std::ranges::find_if(*found, [&](const DiscoveredDevice & f) {
        return f.serial_number == target.serial_number;
      });
      if (it == found->end()) {
        return std::unexpected(wrap(s.error()));
      }
      target = *it;
      s = Session::connect(target, session_options);
      if (!s) {
        return std::unexpected(wrap(s.error()));
      }
    }
    if (!context.rekey(this, target.ip)) {
      return std::unexpected(error(DeviceError::Kind::kAlreadyRegistered));
    }
    {
      const std::lock_guard clock(conn_mutex);
      info_ = target;
      session = std::move(*s);
    }
    if (auto r = apply_host_setup(session, host_setup); !r) {
      return std::unexpected(wrap(r.error()));
    }
    if (sampling_requested.load(std::memory_order_acquire)) {
      if (auto r = set_mode_locked(WorkState::kSampling, std::nullopt); !r) {
        return r;
      }
    }
    rebase_requested.store(true, std::memory_order_release);
    last_push_steady_ns.store(Clock::now().time_since_epoch().count(), std::memory_order_relaxed);
    reconnects.fetch_add(1, std::memory_order_relaxed);
    Event ev;
    ev.kind = Event::Kind::kReconnected;
    ev.time_ns = realtime_now_ns();
    ev.attempts = n;
    {
      const std::lock_guard clock(conn_mutex);
      pending.push_back(ev);
    }
    connected.store(true, std::memory_order_release);
    conn_cv.notify_all();
    context.poller.wake();
    return {};
  }

  void run_worker()
  {
    std::unique_lock lock(conn_mutex);
    const auto stopping = [&] { return stop.stop_requested(); };
    while (!stopping()) {
      conn_cv.wait(lock, [&] { return stopping() || !connected.load(std::memory_order_acquire); });
      if (stopping()) {
        break;
      }
      auto backoff = options.reconnect.initial_backoff;
      while (!stopping() && !connected.load(std::memory_order_acquire)) {
        lock.unlock();
        const auto r = attempt();
        lock.lock();
        if (r) {
          break;
        }
        conn_cv.wait_for(
          lock, backoff, [&] { return stopping() || connected.load(std::memory_order_acquire); });
        backoff = std::min(backoff * 2, options.reconnect.max_backoff);
      }
    }
  }

  [[nodiscard]] DiscoveredDevice snapshot_info() const
  {
    const std::lock_guard lock(conn_mutex);
    return info_;
  }

  /// Receive thread: push-timeout detection and delivery of queued connection events.
  std::optional<Clock::time_point> connection_tick(Clock::time_point now)
  {
    std::optional<Clock::time_point> next;
    if (connected.load(std::memory_order_acquire)) {
      const auto deadline = last_push_steady() + options.reconnect.push_timeout;
      if (now >= deadline) {
        declare_disconnected(DisconnectReason::kPushTimeout);
      } else {
        next = deadline;
      }
    }
    std::vector<Event> events;
    {
      const std::lock_guard lock(conn_mutex);
      events.swap(pending);
    }
    if (rx_event_cb) {
      for (const Event & ev : events) {
        rx_event_cb(ev);
      }
    }
    return next;
  }

  std::optional<Clock::time_point> tick(Clock::time_point now) override
  {
    refresh_callbacks();
    std::optional<Clock::time_point> next = connection_tick(now);
    consume_requests();
    if (assembler.has_partial()) {
      const auto deadline = last_point_packet + idle_window;
      if (now >= deadline) {
        deliver(assembler.flush());
      } else {
        next = deadline;
      }
    }
    if (options.stats_interval.count() > 0) {
      if (now >= next_stats) {
        // Skip missed periods rather than bursting.
        while (next_stats <= now) {
          next_stats += options.stats_interval;
        }
        if (rx_event_cb) {
          Event ev;
          ev.kind = Event::Kind::kStats;
          ev.time_ns = realtime_now_ns();
          ev.stats = snapshot();
          rx_event_cb(ev);
        }
      }
      next = next ? std::min(*next, next_stats) : next_stats;
    }
    return next;
  }

  template <class F>
  std::expected<void, DeviceError> set_callback(F & slot, F cb)
  {
    if (sampling_requested.load(std::memory_order_acquire)) {
      return std::unexpected(error(DeviceError::Kind::kInvalidState));
    }
    const std::lock_guard lock(cb_mutex);
    slot = std::move(cb);
    cb_generation.fetch_add(1, std::memory_order_release);
    return {};
  }

  /// Commands refuse to touch the LiDAR while disconnected (fail fast, issue #8).
  [[nodiscard]] std::expected<void, DeviceError> check_connected() const
  {
    if (!connected.load(std::memory_order_acquire)) {
      return std::unexpected(error(DeviceError::Kind::kDisconnected));
    }
    return {};
  }

  std::expected<void, DeviceError> set_mode_locked(
    WorkState target, std::optional<RequestOptions> opts)
  {
    const auto value = encode_u8(static_cast<std::uint8_t>(target));
    const KeyValue kv[] = {{static_cast<std::uint16_t>(Key::kWorkTgtMode), value}};
    if (auto r = session.configure(kv, opts); !r) {
      return std::unexpected(wrap(r.error()));
    }
    // Frozen callbacks from the moment the LiDAR acknowledged SAMPLING; released again
    // only by a successful stop_sampling().
    if (target == WorkState::kSampling) {
      sampling_requested.store(true, std::memory_order_release);
    }
    if (options.host_setup.wait_timeout.count() > 0) {
      if (auto r = session.wait_for_state(target, options.host_setup.wait_timeout); !r) {
        return std::unexpected(wrap(r.error()));
      }
    }
    return {};
  }

  std::expected<void, DeviceError> set_mode(WorkState target, std::optional<RequestOptions> opts)
  {
    assert(!context.on_receive_thread() && "Device command called from a callback");
    if (auto c = check_connected(); !c) {
      return c;
    }
    const std::lock_guard lock(cmd_mutex);
    if (auto c = check_connected(); !c) {
      return c;
    }
    auto r = set_mode_locked(target, opts);
    if (!r) {
      if (const auto & s = r.error().session) {
        note_command_error(*s);
      }
    }
    return r;
  }
};

std::expected<std::unique_ptr<Device>, DeviceError> Device::open(
  Context & context, const DiscoveredDevice & device, const DeviceOptions & opts)
{
  Context::Impl & ctx = *context.impl_;
  const ReconnectOptions & rc = opts.reconnect;
  if (
    opts.frame_policy.window.count() <= 0 || opts.stats_interval.count() < 0 ||
    rc.push_timeout.count() <= 0 || rc.initial_backoff.count() <= 0 ||
    rc.max_backoff < rc.initial_backoff || rc.discovery_timeout.count() <= 0) {
    return std::unexpected(error(DeviceError::Kind::kInvalidArgument));
  }
  std::stop_source stop;
  SessionOptions sopts = opts.session;
  if (sopts.bind_address == Ipv4{0, 0, 0, 0}) {
    sopts.bind_address = ctx.options.bind_address;
  }
  sopts.stop = stop.get_token();
  auto session = Session::connect(device, sopts);
  if (!session) {
    return std::unexpected(wrap(session.error()));
  }
  // Reconnects always verify the serial number: that is what identifies the LiDAR.
  sopts.verify_serial = true;

  HostSetup setup = opts.host_setup;
  setup.push_port = ctx.options.push_port;
  setup.point_port = ctx.options.point_port;
  setup.imu_port = ctx.options.imu_port;
  setup.work_tgt_mode.reset();

  // Register before the host setup so that the first packets are not counted as unknown.
  auto impl = std::make_unique<Impl>(
    ctx, device, opts, setup, std::move(sopts), std::move(stop), std::move(*session));
  if (ctx.add(device.ip, device.serial_number, impl.get()) != Context::Impl::AddResult::kOk) {
    return std::unexpected(error(DeviceError::Kind::kAlreadyRegistered));
  }
  if (auto r = apply_host_setup(impl->session, setup); !r) {
    ctx.remove(impl.get());
    return std::unexpected(wrap(r.error()));
  }
  impl->last_push_steady_ns.store(
    Clock::now().time_since_epoch().count(), std::memory_order_relaxed);
  auto dev = std::unique_ptr<Device>(new Device(std::move(impl)));
  ctx.bind(dev->impl_.get(), dev.get());
  return dev;
}

Device::Device(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

Device::~Device()
{
  // Stop the worker first: it must not touch the Context after remove().
  impl_->stop.request_stop();
  impl_->conn_cv.notify_all();
  {
    std::thread worker;
    {
      const std::lock_guard lock(impl_->conn_mutex);
      worker = std::move(impl_->worker);
    }
    if (worker.joinable()) {
      worker.join();
    }
  }
  impl_->context.remove(impl_.get());
}

std::expected<void, DeviceError> Device::on_packet(PacketCallback cb)
{
  return impl_->set_callback(impl_->packet_cb, std::move(cb));
}
std::expected<void, DeviceError> Device::on_frame(FrameCallback cb)
{
  return impl_->set_callback(impl_->frame_cb, std::move(cb));
}
std::expected<void, DeviceError> Device::on_imu(ImuCallback cb)
{
  return impl_->set_callback(impl_->imu_cb, std::move(cb));
}
std::expected<void, DeviceError> Device::on_event(EventCallback cb)
{
  return impl_->set_callback(impl_->event_cb, std::move(cb));
}

std::expected<void, DeviceError> Device::start_sampling(std::optional<RequestOptions> opts)
{
  return impl_->set_mode(WorkState::kSampling, opts);
}

std::expected<void, DeviceError> Device::stop_sampling(std::optional<RequestOptions> opts)
{
  auto r = impl_->set_mode(WorkState::kIdle, opts);
  if (r) {
    impl_->sampling_requested.store(false, std::memory_order_release);
    impl_->discard_requested.store(true, std::memory_order_release);
  }
  return r;
}

std::expected<ParamConfigAck, DeviceError> Device::configure(
  std::span<const KeyValue> values, std::optional<RequestOptions> opts)
{
  assert(!impl_->context.on_receive_thread() && "Device command called from a callback");
  if (auto c = impl_->check_connected(); !c) {
    return std::unexpected(c.error());
  }
  const std::lock_guard lock(impl_->cmd_mutex);
  if (auto c = impl_->check_connected(); !c) {
    return std::unexpected(c.error());
  }
  auto r = impl_->session.configure(values, opts);
  if (!r) {
    impl_->note_command_error(r.error());
    return std::unexpected(wrap(r.error()));
  }
  return *r;
}

std::expected<InquireResult, DeviceError> Device::inquire(
  std::span<const std::uint16_t> keys, std::optional<RequestOptions> opts)
{
  assert(!impl_->context.on_receive_thread() && "Device command called from a callback");
  if (auto c = impl_->check_connected(); !c) {
    return std::unexpected(c.error());
  }
  const std::lock_guard lock(impl_->cmd_mutex);
  if (auto c = impl_->check_connected(); !c) {
    return std::unexpected(c.error());
  }
  auto r = impl_->session.inquire(keys, opts);
  if (!r) {
    impl_->note_command_error(r.error());
    return std::unexpected(wrap(r.error()));
  }
  return std::move(*r);
}

std::expected<InquireResult, DeviceError> Device::inquire(
  std::span<const Key> keys, std::optional<RequestOptions> opts)
{
  std::vector<std::uint16_t> raw(keys.size());
  std::ranges::transform(keys, raw.begin(), [](Key k) { return static_cast<std::uint16_t>(k); });
  return inquire(raw, opts);
}

std::expected<void, DeviceError> Device::reboot(std::optional<RequestOptions> opts)
{
  assert(!impl_->context.on_receive_thread() && "Device command called from a callback");
  if (auto c = impl_->check_connected(); !c) {
    return c;
  }
  const std::lock_guard lock(impl_->cmd_mutex);
  if (auto c = impl_->check_connected(); !c) {
    return c;
  }
  if (auto r = impl_->session.reboot(100, opts); !r) {
    impl_->note_command_error(r.error());
    return std::unexpected(wrap(r.error()));
  }
  // The outage is known: do not wait for the push timeout to notice it.
  impl_->declare_disconnected(DisconnectReason::kRebootRequested);
  return {};
}

void Device::cancel() noexcept
{
  const std::lock_guard lock(impl_->conn_mutex);
  impl_->session.cancel();
}

bool Device::connected() const noexcept { return impl_->connected.load(std::memory_order_acquire); }

std::expected<void, DeviceError> Device::reconnect() { return impl_->attempt(); }

void Device::disconnect() { impl_->declare_disconnected(DisconnectReason::kUser); }

DiscoveredDevice Device::info() const { return impl_->snapshot_info(); }

std::optional<WorkState> Device::work_state() const
{
  const std::lock_guard lock(impl_->push_mutex);
  return impl_->pushed_state;
}

std::array<HmsCode, 8> Device::hms() const
{
  const std::lock_guard lock(impl_->push_mutex);
  return impl_->pushed_hms;
}

DeviceStats Device::stats() const { return impl_->snapshot(); }

SessionStats Device::session_stats() const
{
  const std::lock_guard lock(impl_->cmd_mutex);
  return impl_->session.stats();
}

}  // namespace livox::mid360
