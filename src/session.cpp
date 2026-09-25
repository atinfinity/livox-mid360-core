// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/session.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

#include "session_detail.hpp"

namespace livox::mid360 {

namespace {

using Clock = std::chrono::steady_clock;

constexpr std::uint64_t kSocketTag = 1;

SessionError transport_error(TransportError e, std::uint16_t cmd_id = 0,
                             std::uint32_t attempts = 0) {
  SessionError err;
  err.kind = SessionErrorKind::kTransport;
  err.cmd_id = cmd_id;
  err.attempts = attempts;
  err.transport = e;
  return err;
}

SessionError bad_response(ParseError e, std::uint16_t cmd_id, std::uint32_t attempts) {
  SessionError err;
  err.kind = SessionErrorKind::kBadResponse;
  err.cmd_id = cmd_id;
  err.attempts = attempts;
  err.parse = e;
  return err;
}

SessionError rejected(RetCode ret, std::uint16_t error_key, std::uint16_t cmd_id,
                      std::uint32_t attempts) {
  SessionError err;
  err.kind = SessionErrorKind::kLidarRejected;
  err.cmd_id = cmd_id;
  err.attempts = attempts;
  err.ret_code = ret;
  err.error_key = error_key;
  return err;
}

std::string hex16(std::uint16_t v) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string s = "0x";
  for (int shift = 12; shift >= 0; shift -= 4) s += kHex[(v >> shift) & 0xF];
  return s;
}

std::chrono::milliseconds remaining(Clock::time_point deadline) {
  const auto now = Clock::now();
  if (now >= deadline) return std::chrono::milliseconds{0};
  return std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
}

}  // namespace

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

std::string_view to_string(SessionErrorKind kind) noexcept {
  switch (kind) {
    case SessionErrorKind::kTransport:
      return "transport";
    case SessionErrorKind::kTimeout:
      return "timeout";
    case SessionErrorKind::kBadResponse:
      return "bad_response";
    case SessionErrorKind::kLidarRejected:
      return "lidar_rejected";
    case SessionErrorKind::kUnexpectedState:
      return "unexpected_state";
    case SessionErrorKind::kCancelled:
      return "cancelled";
    case SessionErrorKind::kInvalidArgument:
      return "invalid_argument";
  }
  return "unknown";
}

std::string to_string(const SessionError& err) {
  std::string s(to_string(err.kind));
  if (err.cmd_id != 0 || err.attempts != 0) {
    s += " cmd " + hex16(err.cmd_id) + " after " + std::to_string(err.attempts) + " attempt(s)";
  }
  switch (err.kind) {
    case SessionErrorKind::kTransport:
      if (err.transport) s += ": " + to_string(*err.transport);
      break;
    case SessionErrorKind::kBadResponse:
      if (err.parse) s += ": " + std::string(to_string(*err.parse));
      break;
    case SessionErrorKind::kLidarRejected:
      s += ": ret " + std::string(to_string(err.ret_code));
      if (err.error_key != 0) s += " key " + hex16(err.error_key);
      break;
    case SessionErrorKind::kUnexpectedState:
      if (err.work_state) s += ": " + std::string(to_string(*err.work_state));
      break;
    case SessionErrorKind::kInvalidArgument:
      if (err.error_key != 0) s += ": key " + hex16(err.error_key);
      break;
    default:
      break;
  }
  return s;
}

// ---------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------

std::expected<std::vector<DiscoveredDevice>, SessionError> discover(
    const DiscoveryOptions& options) {
  SocketOptions sopts;
  sopts.broadcast = options.targets.empty();
  auto sock = UdpSocket::open(Endpoint{options.bind_address, 0}, sopts);
  if (!sock) return std::unexpected(transport_error(sock.error(), 0));
  auto poller = Poller::create();
  if (!poller) return std::unexpected(transport_error(poller.error(), 0));
  if (auto r = poller->add(*sock, kSocketTag); !r) {
    return std::unexpected(transport_error(r.error(), 0));
  }

  CommandFrameSpec spec;
  spec.seq_num = 1;
  spec.cmd_id = static_cast<std::uint16_t>(CmdId::kDiscovery);
  const auto frame = build_command_frame(spec);
  if (!frame) return std::unexpected(bad_response(ParseError::kTooShort, 0, 0));  // unreachable
  std::vector<Endpoint> targets = options.targets;
  if (targets.empty()) targets.push_back(Endpoint::broadcast(kDiscoveryPort));
  for (const auto& t : targets) {
    if (auto r = sock->send_to(*frame, t); !r) {
      return std::unexpected(transport_error(r.error(), 0, 1));
    }
  }

  std::vector<DiscoveredDevice> found;
  std::vector<bool> answered(targets.size(), false);
  const bool unicast = !options.targets.empty();
  std::array<std::byte, kMaxDatagramSize> buf{};
  const auto deadline = Clock::now() + options.timeout;
  while (true) {
    const auto left = remaining(deadline);
    if (left.count() == 0) break;
    const auto ev = poller->wait(left);
    if (!ev) return std::unexpected(transport_error(ev.error(), 0, 1));
    if (ev->empty()) continue;
    while (true) {
      const auto d = sock->recv_one(buf);
      if (!d) break;
      auto dev = detail::parse_discovered_device(d->data, d->from);
      if (!dev) continue;
      const bool dup = std::any_of(found.begin(), found.end(), [&](const DiscoveredDevice& f) {
        return f.serial_number == dev->serial_number;
      });
      if (!dup) found.push_back(std::move(*dev));
      if (unicast) {
        for (std::size_t i = 0; i < targets.size(); ++i) {
          if (targets[i] == d->from || targets[i].ip == d->from.ip) answered[i] = true;
        }
      }
    }
    if (unicast && std::all_of(answered.begin(), answered.end(), [](bool b) { return b; })) {
      break;
    }
  }
  return found;
}

// ---------------------------------------------------------------------------
// Session
// ---------------------------------------------------------------------------

Session::Session(Session&& o) noexcept
    : socket_(std::move(o.socket_)),
      poller_(std::move(o.poller_)),
      lidar_(o.lidar_),
      options_(o.options_),
      stats_(o.stats_),
      serial_(std::move(o.serial_)),
      next_seq_(o.next_seq_),
      cancel_(o.cancel_.load()) {}

Session& Session::operator=(Session&& o) noexcept {
  if (this != &o) {
    socket_ = std::move(o.socket_);
    poller_ = std::move(o.poller_);
    lidar_ = o.lidar_;
    options_ = o.options_;
    stats_ = o.stats_;
    serial_ = std::move(o.serial_);
    next_seq_ = o.next_seq_;
    cancel_.store(o.cancel_.load());
  }
  return *this;
}

Session::~Session() = default;

std::expected<void, SessionError> Session::open(const SessionOptions& options, Endpoint lidar) {
  options_ = options;
  lidar_ = lidar;
  auto sock = UdpSocket::open(Endpoint{options.bind_address, options.host_command_port});
  if (!sock) return std::unexpected(transport_error(sock.error()));
  auto poller = Poller::create();
  if (!poller) return std::unexpected(transport_error(poller.error()));
  if (auto r = poller->add(*sock, kSocketTag); !r) {
    return std::unexpected(transport_error(r.error()));
  }
  socket_ = std::move(*sock);
  poller_ = std::move(*poller);
  return {};
}

std::expected<void, SessionError> Session::read_serial() {
  const Key keys[] = {Key::kSn};
  auto r = inquire(keys);
  if (!r) return std::unexpected(r.error());
  const auto sn = r->get(Key::kSn);
  if (!sn) return std::unexpected(bad_response(ParseError::kTruncated, 0x0101, 1));
  serial_ = std::string(decode_string(*sn));
  return {};
}

std::expected<Session, SessionError> Session::connect(const DiscoveredDevice& device,
                                                      const SessionOptions& options) {
  Session s;
  if (auto r = s.open(options, Endpoint{device.ip, device.cmd_port}); !r) {
    return std::unexpected(r.error());
  }
  if (options.verify_serial) {
    if (auto r = s.read_serial(); !r) return std::unexpected(r.error());
    if (s.serial_ != device.serial_number) {
      SessionError err;
      err.kind = SessionErrorKind::kBadResponse;
      err.cmd_id = 0x0101;
      err.attempts = 1;
      return std::unexpected(err);
    }
  } else {
    s.serial_ = device.serial_number;
  }
  return s;
}

std::expected<Session, SessionError> Session::connect(Endpoint cmd_endpoint,
                                                      const SessionOptions& options) {
  Session s;
  if (auto r = s.open(options, cmd_endpoint); !r) return std::unexpected(r.error());
  if (auto r = s.read_serial(); !r) return std::unexpected(r.error());
  return s;
}

void Session::cancel() noexcept {
  cancel_.store(true);
  poller_.wake();
}

std::expected<RawAck, SessionError> Session::request(std::uint16_t cmd_id,
                                                     std::span<const std::byte> data,
                                                     std::optional<RequestOptions> opts) {
  const RequestOptions ro = opts.value_or(options_.request);
  const std::uint32_t seq = next_seq_++;
  if (next_seq_ == 0) next_seq_ = 1;
  ++stats_.requests;

  CommandFrameSpec spec;
  spec.seq_num = seq;
  spec.cmd_id = cmd_id;
  spec.data = data;
  const auto frame = build_command_frame(spec);
  if (!frame) {
    SessionError err;
    err.kind = SessionErrorKind::kTransport;
    err.cmd_id = cmd_id;
    err.transport = TransportError{TransportErrorCode::kMessageTooLong, 0};
    return std::unexpected(err);
  }

  std::array<std::byte, kMaxDatagramSize> buf{};
  const std::uint32_t attempts = std::max<std::uint32_t>(ro.attempts, 1);
  for (std::uint32_t attempt = 1; attempt <= attempts; ++attempt) {
    if (cancel_.exchange(false)) {
      SessionError err;
      err.kind = SessionErrorKind::kCancelled;
      err.cmd_id = cmd_id;
      err.attempts = attempt - 1;
      return std::unexpected(err);
    }
    if (attempt > 1) ++stats_.retries;
    if (auto r = socket_.send_to(*frame, lidar_); !r) {
      return std::unexpected(transport_error(r.error(), cmd_id, attempt));
    }
    const auto deadline = Clock::now() + ro.timeout;
    while (true) {
      const auto left = remaining(deadline);
      if (left.count() == 0) break;
      const auto ev = poller_.wait(left);
      if (!ev) return std::unexpected(transport_error(ev.error(), cmd_id, attempt));
      if (cancel_.exchange(false)) {
        SessionError err;
        err.kind = SessionErrorKind::kCancelled;
        err.cmd_id = cmd_id;
        err.attempts = attempt;
        return std::unexpected(err);
      }
      if (ev->empty()) continue;
      while (true) {
        const auto d = socket_.recv_one(buf);
        if (!d) break;
        const auto view = detail::match_ack(d->data, d->from, seq, cmd_id, lidar_.ip);
        if (!view) {
          if (view.error() == detail::AckMismatch::kBadFrame) ++stats_.bad_frames;
          if (view.error() == detail::AckMismatch::kLate) ++stats_.late_acks;
          continue;
        }
        RawAck ack;
        ack.cmd_id = cmd_id;
        ack.seq_num = seq;
        ack.data.assign(view->data.begin(), view->data.end());
        return ack;
      }
    }
  }
  ++stats_.timeouts;
  SessionError err;
  err.kind = SessionErrorKind::kTimeout;
  err.cmd_id = cmd_id;
  err.attempts = attempts;
  return std::unexpected(err);
}

// -- typed -------------------------------------------------------------------

std::expected<DiscoveryAck, SessionError> Session::discovery_ack(
    std::optional<RequestOptions> opts) {
  auto r = request(static_cast<std::uint16_t>(CmdId::kDiscovery), {}, opts);
  if (!r) return std::unexpected(r.error());
  return detail::to_discovery_ack(*r);
}

std::expected<ParamConfigAck, SessionError> Session::configure(std::span<const KeyValue> kvs,
                                                               std::optional<RequestOptions> opts) {
  const auto payload = encode_param_config_request(kvs);
  auto r = request(static_cast<std::uint16_t>(CmdId::kParamConfig), payload, opts);
  if (!r) return std::unexpected(r.error());
  return detail::to_config_ack(*r);
}

std::expected<InquireResult, SessionError> Session::inquire(std::span<const std::uint16_t> keys,
                                                            std::optional<RequestOptions> opts) {
  const auto payload = encode_param_inquire_request(keys);
  auto r = request(static_cast<std::uint16_t>(CmdId::kParamInquire), payload, opts);
  if (!r) return std::unexpected(r.error());
  return detail::to_inquire_result(std::move(*r));
}

std::expected<InquireResult, SessionError> Session::inquire(std::span<const Key> keys,
                                                            std::optional<RequestOptions> opts) {
  std::vector<std::uint16_t> raw(keys.size());
  std::transform(keys.begin(), keys.end(), raw.begin(),
                 [](Key k) { return static_cast<std::uint16_t>(k); });
  return inquire(std::span<const std::uint16_t>(raw), opts);
}

namespace {
std::expected<SimpleAck, SessionError> simple(const std::expected<RawAck, SessionError>& r) {
  if (!r) return std::unexpected(r.error());
  return detail::to_simple_ack(*r);
}
}  // namespace

std::expected<SimpleAck, SessionError> Session::reboot(std::uint16_t timeout_ms,
                                                       std::optional<RequestOptions> opts) {
  return simple(
      request(static_cast<std::uint16_t>(CmdId::kReboot), encode_reboot_request(timeout_ms), opts));
}

std::expected<SimpleAck, SessionError> Session::factory_reset(std::optional<RequestOptions> opts) {
  return simple(request(static_cast<std::uint16_t>(CmdId::kFactoryReset),
                        encode_factory_reset_request(), opts));
}

std::expected<SimpleAck, SessionError> Session::set_gps_time(std::uint64_t pps_time_ns,
                                                             std::optional<RequestOptions> opts) {
  return simple(request(static_cast<std::uint16_t>(CmdId::kSetGpsTimestamp),
                        encode_set_gps_timestamp_request(pps_time_ns), opts));
}

// -- state -------------------------------------------------------------------

std::expected<WorkState, SessionError> Session::work_state(std::optional<RequestOptions> opts) {
  const Key keys[] = {Key::kCurWorkState};
  auto r = inquire(keys, opts);
  if (!r) return std::unexpected(r.error());
  return detail::to_work_state(*r);
}

std::expected<void, SessionError> Session::wait_for_state(WorkState target,
                                                          std::chrono::milliseconds timeout) {
  const auto deadline = Clock::now() + timeout;
  while (true) {
    auto ws = work_state();
    if (!ws) return std::unexpected(ws.error());
    if (*ws == target) return {};
    if (*ws == WorkState::kError || *ws == WorkState::kUpgrade) {
      SessionError err;
      err.kind = SessionErrorKind::kUnexpectedState;
      err.cmd_id = 0x0101;
      err.work_state = *ws;
      return std::unexpected(err);
    }
    const auto left = remaining(deadline);
    if (left.count() == 0) {
      SessionError err;
      err.kind = SessionErrorKind::kTimeout;
      err.cmd_id = 0x0101;
      err.work_state = *ws;
      return std::unexpected(err);
    }
    // Sleep on the poller so that cancel() interrupts the wait.
    const auto ev = poller_.wait(std::min(left, options_.state_poll_interval));
    if (!ev) return std::unexpected(transport_error(ev.error(), 0x0101));
    if (cancel_.exchange(false)) {
      SessionError err;
      err.kind = SessionErrorKind::kCancelled;
      err.cmd_id = 0x0101;
      return std::unexpected(err);
    }
    if (!ev->empty()) {  // drain stray datagrams (late ACKs / pushes)
      std::array<std::byte, kMaxDatagramSize> buf{};
      while (socket_.recv_one(buf)) ++stats_.late_acks;
    }
  }
}

// ---------------------------------------------------------------------------
// detail (see session_detail.hpp)
// ---------------------------------------------------------------------------

namespace detail {

std::expected<CommandFrameView, AckMismatch> match_ack(std::span<const std::byte> datagram,
                                                       const Endpoint& from, std::uint32_t seq,
                                                       std::uint16_t cmd_id,
                                                       const Ipv4& lidar_ip) noexcept {
  const auto view = parse_command_frame(datagram);
  if (!view) return std::unexpected(AckMismatch::kBadFrame);
  if (view->header.cmd_type != CmdType::kAck) return std::unexpected(AckMismatch::kNotAck);
  if (view->header.seq_num != seq || view->header.cmd_id != cmd_id || from.ip != lidar_ip) {
    return std::unexpected(AckMismatch::kLate);
  }
  return *view;
}

std::optional<DiscoveredDevice> parse_discovered_device(std::span<const std::byte> datagram,
                                                        const Endpoint& from) {
  const auto view = parse_command_frame(datagram);
  if (!view || view->header.cmd_id != static_cast<std::uint16_t>(CmdId::kDiscovery) ||
      view->header.cmd_type != CmdType::kAck) {
    return std::nullopt;
  }
  const auto ack = parse_discovery_ack(view->data);
  if (!ack || ack->ret_code != RetCode::kSuccess) return std::nullopt;
  DiscoveredDevice dev;
  dev.serial_number = std::string(ack->serial_number_view());
  dev.ip = ack->lidar_ip;
  dev.cmd_port = ack->cmd_port;
  dev.dev_type = ack->dev_type;
  dev.from = from;
  return dev;
}

std::expected<DiscoveryAck, SessionError> to_discovery_ack(const RawAck& ack) {
  const auto a = parse_discovery_ack(ack.data);
  if (!a) return std::unexpected(bad_response(a.error(), ack.cmd_id, 0));
  if (a->ret_code != RetCode::kSuccess) {
    return std::unexpected(rejected(a->ret_code, 0, ack.cmd_id, 0));
  }
  return *a;
}

std::expected<ParamConfigAck, SessionError> to_config_ack(const RawAck& ack) {
  const auto a = parse_param_config_ack(ack.data);
  if (!a) return std::unexpected(bad_response(a.error(), ack.cmd_id, 0));
  if (a->ret_code != RetCode::kSuccess && a->ret_code != RetCode::kParamRebootEffect) {
    return std::unexpected(rejected(a->ret_code, a->error_key, ack.cmd_id, 0));
  }
  return *a;
}

std::expected<InquireResult, SessionError> to_inquire_result(RawAck ack) {
  InquireResult res;
  res.raw = std::move(ack.data);
  const auto a = parse_param_inquire_ack(res.raw);
  if (!a) return std::unexpected(bad_response(a.error(), ack.cmd_id, 0));
  if (a->ret_code != RetCode::kSuccess) {
    const std::uint16_t key = a->values.empty() ? 0 : a->values.front().key;
    return std::unexpected(rejected(a->ret_code, key, ack.cmd_id, 0));
  }
  res.ret_code = a->ret_code;
  res.values = a->values;  // spans point into res.raw, which the result owns
  return res;
}

std::expected<SimpleAck, SessionError> to_simple_ack(const RawAck& ack) {
  const auto a = parse_simple_ack(ack.data);
  if (!a) return std::unexpected(bad_response(a.error(), ack.cmd_id, 0));
  if (a->ret_code != RetCode::kSuccess) {
    return std::unexpected(rejected(a->ret_code, 0, ack.cmd_id, 0));
  }
  return *a;
}

std::expected<WorkState, SessionError> to_work_state(const InquireResult& result) {
  const auto v = result.get(Key::kCurWorkState);
  if (!v) return std::unexpected(bad_response(ParseError::kTruncated, 0x0101, 0));
  const auto ws = decode_work_state(*v);
  if (!ws) return std::unexpected(bad_response(ParseError::kTruncated, 0x0101, 0));
  return *ws;
}

}  // namespace detail

}  // namespace livox::mid360
