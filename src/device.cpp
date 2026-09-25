// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/device.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

#include "context_impl.hpp"
#include "frame_assembler.hpp"

namespace livox::mid360 {

namespace {

using Clock = std::chrono::steady_clock;

std::uint64_t realtime_now_ns() noexcept {
  timespec ts{};
  ::clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ull +
         static_cast<std::uint64_t>(ts.tv_nsec);
}

DeviceError wrap(const SessionError& err) {
  return DeviceError{.kind = DeviceError::Kind::kSession, .session = err};
}

DeviceError error(DeviceError::Kind kind) {
  return DeviceError{.kind = kind, .session = std::nullopt};
}

}  // namespace

struct Device::Impl : detail::Receiver {
  Impl(Context::Impl& ctx, DiscoveredDevice dev, const DeviceOptions& o, Session s)
      : context(ctx),
        info(std::move(dev)),
        options(o),
        session(std::move(s)),
        assembler(o.frame_policy, o.timestamp_policy),
        idle_window(std::chrono::duration_cast<Clock::duration>(o.frame_policy.window)),
        next_stats(Clock::now() + o.stats_interval) {}

  // --- fixed after open ---------------------------------------------------
  Context::Impl& context;
  const DiscoveredDevice info;
  const DeviceOptions options;

  // --- commands (caller threads, serialised) -------------------------------
  std::mutex cmd_mutex;
  Session session;
  std::atomic<bool> sampling_requested{false};
  std::atomic<bool> discard_requested{false};

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
  std::atomic<std::int64_t> time_offset_ns{0};
  std::atomic<bool> time_offset_valid{false};

  [[nodiscard]] DeviceStats snapshot() const {
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
        .time_offset_ns = time_offset_ns.load(kRelaxed),
        .time_offset_valid = time_offset_valid.load(kRelaxed),
    };
  }

  void refresh_callbacks() {
    const auto gen = cb_generation.load(std::memory_order_acquire);
    if (gen == cb_seen) return;
    const std::lock_guard lock(cb_mutex);
    rx_packet_cb = packet_cb;
    rx_frame_cb = frame_cb;
    rx_imu_cb = imu_cb;
    rx_event_cb = event_cb;
    cb_seen = cb_generation.load(std::memory_order_acquire);
  }

  void publish_assembler_counters() {
    constexpr auto kRelaxed = std::memory_order_relaxed;
    const auto& c = assembler.counters();
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

  void deliver(std::optional<Frame> frame) {
    publish_assembler_counters();
    if (frame && rx_frame_cb) rx_frame_cb(std::move(*frame));
  }

  void on_datagram(detail::DataPort port, const Datagram& d) override {
    if (port == detail::DataPort::kPush) return;  // parsed in #7
    const auto pkt = parse_data_packet(d.data, options.verify_crc);
    if (!pkt) {
      bad_packets.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    packets.fetch_add(1, std::memory_order_relaxed);
    last_packet_time_ns.store(d.recv_time_ns, std::memory_order_relaxed);
    if (discard_requested.exchange(false, std::memory_order_acq_rel)) assembler.discard();
    refresh_callbacks();
    if (rx_packet_cb) {
      rx_packet_cb(*pkt, ReceiveInfo{.host_time_ns = d.recv_time_ns, .source = d.from});
    }
    const DataPacketHeader& h = pkt->header;
    if (h.data_type == DataType::kImu) {
      const auto drop = imu_drops.observe(h.udp_cnt, false);
      imu_dropped += drop.dropped;
      imu_reordered += drop.reordered ? 1u : 0u;
      // kHostOffsetOnce measures its offset at the first point-cloud packet; an IMU packet
      // that arrives before it is stamped with the receive time instead.
      detail::TimeMapper& tm = assembler.time_mapper();
      const bool pending_offset = options.timestamp_policy == TimestampPolicy::kHostOffsetOnce &&
                                  !tm.offset_ns() && h.time_type == TimeType::kNoSync;
      const std::uint64_t t0 = pending_offset ? d.recv_time_ns : tm.map(h, d.recv_time_ns);
      for (std::size_t i = 0; i < h.dot_num; ++i) {
        const std::uint64_t t = t0 + (sample_timestamp_ns(h, i) - h.timestamp_ns);
        if (rx_imu_cb) rx_imu_cb(ImuData{.time_ns = t, .sample = decode_imu(*pkt, i)});
      }
      imu_samples.fetch_add(h.dot_num, std::memory_order_relaxed);
      publish_assembler_counters();
      return;
    }
    last_point_packet = Clock::now();
    deliver(assembler.push(*pkt, d.recv_time_ns));
  }

  std::optional<Clock::time_point> tick(Clock::time_point now) override {
    refresh_callbacks();
    std::optional<Clock::time_point> next;
    if (discard_requested.exchange(false, std::memory_order_acq_rel)) assembler.discard();
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
        while (next_stats <= now) next_stats += options.stats_interval;
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
  std::expected<void, DeviceError> set_callback(F& slot, F cb) {
    if (sampling_requested.load(std::memory_order_acquire)) {
      return std::unexpected(error(DeviceError::Kind::kInvalidState));
    }
    const std::lock_guard lock(cb_mutex);
    slot = std::move(cb);
    cb_generation.fetch_add(1, std::memory_order_release);
    return {};
  }

  std::expected<void, DeviceError> set_mode(WorkState target, std::optional<RequestOptions> opts) {
    assert(!context.on_receive_thread() && "Device command called from a callback");
    const std::lock_guard lock(cmd_mutex);
    const auto value = encode_u8(static_cast<std::uint8_t>(target));
    const KeyValue kv[] = {{static_cast<std::uint16_t>(Key::kWorkTgtMode), value}};
    if (auto r = session.configure(kv, opts); !r) return std::unexpected(wrap(r.error()));
    // Frozen callbacks from the moment the LiDAR acknowledged SAMPLING; released again
    // only by a successful stop_sampling().
    if (target == WorkState::kSampling) sampling_requested.store(true, std::memory_order_release);
    if (options.host_setup.wait_timeout.count() > 0) {
      if (auto r = session.wait_for_state(target, options.host_setup.wait_timeout); !r) {
        return std::unexpected(wrap(r.error()));
      }
    }
    return {};
  }
};

std::expected<std::unique_ptr<Device>, DeviceError> Device::open(Context& context,
                                                                 const DiscoveredDevice& device,
                                                                 const DeviceOptions& opts) {
  Context::Impl& ctx = *context.impl_;
  if (opts.frame_policy.window.count() <= 0 || opts.stats_interval.count() < 0) {
    return std::unexpected(error(DeviceError::Kind::kInvalidArgument));
  }
  SessionOptions sopts = opts.session;
  if (sopts.bind_address == Ipv4{0, 0, 0, 0}) sopts.bind_address = ctx.options.bind_address;
  auto session = Session::connect(device, sopts);
  if (!session) return std::unexpected(wrap(session.error()));

  HostSetup setup = opts.host_setup;
  setup.push_port = ctx.options.push_port;
  setup.point_port = ctx.options.point_port;
  setup.imu_port = ctx.options.imu_port;
  setup.work_tgt_mode.reset();

  // Register before the host setup so that the first packets are not counted as unknown.
  auto impl = std::make_unique<Impl>(ctx, device, opts, std::move(*session));
  if (!ctx.add(device.ip, impl.get())) {
    return std::unexpected(error(DeviceError::Kind::kAlreadyRegistered));
  }
  if (auto r = apply_host_setup(impl->session, setup); !r) {
    ctx.remove(impl.get());
    return std::unexpected(wrap(r.error()));
  }
  return std::unique_ptr<Device>(new Device(std::move(impl)));
}

Device::Device(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

Device::~Device() {
  impl_->context.remove(impl_.get());
}

std::expected<void, DeviceError> Device::on_packet(PacketCallback cb) {
  return impl_->set_callback(impl_->packet_cb, std::move(cb));
}
std::expected<void, DeviceError> Device::on_frame(FrameCallback cb) {
  return impl_->set_callback(impl_->frame_cb, std::move(cb));
}
std::expected<void, DeviceError> Device::on_imu(ImuCallback cb) {
  return impl_->set_callback(impl_->imu_cb, std::move(cb));
}
std::expected<void, DeviceError> Device::on_event(EventCallback cb) {
  return impl_->set_callback(impl_->event_cb, std::move(cb));
}

std::expected<void, DeviceError> Device::start_sampling(std::optional<RequestOptions> opts) {
  return impl_->set_mode(WorkState::kSampling, opts);
}

std::expected<void, DeviceError> Device::stop_sampling(std::optional<RequestOptions> opts) {
  auto r = impl_->set_mode(WorkState::kIdle, opts);
  if (r) {
    impl_->sampling_requested.store(false, std::memory_order_release);
    impl_->discard_requested.store(true, std::memory_order_release);
  }
  return r;
}

std::expected<ParamConfigAck, DeviceError> Device::configure(std::span<const KeyValue> values,
                                                             std::optional<RequestOptions> opts) {
  assert(!impl_->context.on_receive_thread() && "Device command called from a callback");
  const std::lock_guard lock(impl_->cmd_mutex);
  auto r = impl_->session.configure(values, opts);
  if (!r) return std::unexpected(wrap(r.error()));
  return *r;
}

std::expected<InquireResult, DeviceError> Device::inquire(std::span<const std::uint16_t> keys,
                                                          std::optional<RequestOptions> opts) {
  assert(!impl_->context.on_receive_thread() && "Device command called from a callback");
  const std::lock_guard lock(impl_->cmd_mutex);
  auto r = impl_->session.inquire(keys, opts);
  if (!r) return std::unexpected(wrap(r.error()));
  return std::move(*r);
}

std::expected<void, DeviceError> Device::reboot(std::optional<RequestOptions> opts) {
  assert(!impl_->context.on_receive_thread() && "Device command called from a callback");
  const std::lock_guard lock(impl_->cmd_mutex);
  if (auto r = impl_->session.reboot(100, opts); !r) return std::unexpected(wrap(r.error()));
  return {};
}

void Device::cancel() noexcept {
  impl_->session.cancel();
}

const DiscoveredDevice& Device::info() const noexcept {
  return impl_->info;
}

// Always nullopt until cur_work_state push tracking lands (#7).
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
std::optional<WorkState> Device::work_state() const {
  return std::nullopt;
}

DeviceStats Device::stats() const {
  return impl_->snapshot();
}

SessionStats Device::session_stats() const {
  const std::lock_guard lock(impl_->cmd_mutex);
  return impl_->session.stats();
}

}  // namespace livox::mid360
