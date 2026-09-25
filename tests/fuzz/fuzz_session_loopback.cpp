// SPDX-License-Identifier: Apache-2.0
// Session request loop end to end over a loopback socket pair (issue #28).
//
// One static Session talks to a peer UdpSocket on 127.0.0.1. The fuzz input is turned into
// reply datagrams that are queued to the session *before* the call, so the request loop sees
// them on its first Poller::wait; no threads, no timeouts longer than a few milliseconds.
//
// Input layout: [op][flags][attempts][cmd_id u16 when op == raw][datagrams...]
//   op % 8     : 0 raw request, 1 discovery_ack, 2 configure, 3 inquire, 4 work_state,
//                5 reboot, 6 factory_reset, 7 set_gps_time
//   flags      : bit0 fixup (patch seq_num + CRC16 of every reply so it can match),
//                bit1 prepend one malformed datagram, bit2 cancel() before the call
//   attempts   : 1 + (byte % 3)
//   datagrams  : up to 4 x [len u8][body len bytes]; the rest of the input goes into the
//                last one. Empty datagrams are not sent.
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <vector>

#include "fuzz_check.hpp"
#include "livox/mid360/crc.hpp"
#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/session.hpp"
#include "livox/mid360/transport.hpp"

namespace {
using namespace livox::mid360;
using namespace std::chrono_literals;

constexpr std::size_t kMaxDatagrams = 4;
constexpr std::uint8_t kFlagFixup = 1;
constexpr std::uint8_t kFlagBadPrefix = 2;
constexpr std::uint8_t kFlagCancel = 4;

struct Harness {
  UdpSocket peer;
  std::optional<Session> session;
  std::uint32_t next_seq = 1;
  std::uint64_t datagrams_sent = 0;  ///< cumulative, replies may outlive one call

  static Harness& instance() {
    static Harness h = make();
    return h;
  }

 private:
  static Harness make() {
    Harness h;
    auto peer = UdpSocket::open(Endpoint::loopback(0));
    fuzz::require(peer.has_value());
    h.peer = std::move(*peer);
    SessionOptions o;
    o.host_command_port = 0;
    o.bind_address = {127, 0, 0, 1};
    o.verify_serial = false;  // the Endpoint overload always reads the serial; this one does not
    DiscoveredDevice dev;
    dev.serial_number = "FUZZ";
    dev.ip = h.peer.local_endpoint().ip;
    dev.cmd_port = h.peer.local_endpoint().port;
    auto s = Session::connect(dev, o);
    fuzz::require(s.has_value());
    h.session.emplace(std::move(*s));
    return h;
  }
};

/// Overwrite seq_num and recompute the header CRC so the frame can match the pending request.
void fixup(std::vector<std::byte>& frame, std::uint32_t seq) {
  if (frame.size() < kCommandHeaderSize) return;
  std::memcpy(frame.data() + 4, &seq, sizeof(seq));
  const auto crc = crc::crc16_ccitt_false(std::span<const std::byte>{frame.data(), 18});
  std::memcpy(frame.data() + 18, &crc, sizeof(crc));
}

bool inside(std::span<const std::byte> view, const std::vector<std::byte>& owner) {
  if (view.empty()) return true;
  const auto* b = owner.data();
  return view.data() >= b && view.data() + view.size() <= b + owner.size();
}

/// Drain the requests the session sent to the peer; each must be a well formed request frame.
std::size_t drain_requests(const UdpSocket& peer, std::uint32_t seq, std::uint16_t cmd_id) {
  std::array<std::byte, kMaxDatagramSize> buf{};
  std::size_t n = 0;
  while (true) {
    const auto d = peer.recv_one(buf);
    if (!d) {
      fuzz::require(d.error().code == TransportErrorCode::kWouldBlock);
      return n;
    }
    const auto v = parse_command_frame(d->data);
    fuzz::require(v.has_value());
    fuzz::require(v->header.seq_num == seq && v->header.cmd_id == cmd_id);
    fuzz::require(v->header.cmd_type == CmdType::kReq &&
                  v->header.sender_type == SenderType::kHost);
    ++n;
  }
}

template <typename T>
std::optional<SessionError> check_result(const std::expected<T, SessionError>& r,
                                         std::uint16_t cmd_id) {
  if (r) return std::nullopt;
  const auto& e = r.error();
  fuzz::require(e.cmd_id == cmd_id);
  fuzz::require(e.kind != SessionErrorKind::kTransport);  // never on loopback
  fuzz::require(e.kind != SessionErrorKind::kUnexpectedState);
  return e;
}
}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  if (size < 3) return 0;
  auto& h = Harness::instance();
  Session& s = *h.session;

  const std::uint8_t op = data[0] % 8;
  const std::uint8_t flags = data[1];
  const std::uint32_t attempts = 1 + (data[2] % 3);
  std::size_t pos = 3;
  std::uint16_t cmd_id = 0;
  switch (op) {
    case 0:
      if (size < pos + 2) return 0;
      std::memcpy(&cmd_id, data + pos, 2);
      pos += 2;
      break;
    case 1:
      cmd_id = static_cast<std::uint16_t>(CmdId::kDiscovery);
      break;
    case 2:
      cmd_id = static_cast<std::uint16_t>(CmdId::kParamConfig);
      break;
    case 3:
    case 4:
      cmd_id = static_cast<std::uint16_t>(CmdId::kParamInquire);
      break;
    case 5:
      cmd_id = static_cast<std::uint16_t>(CmdId::kReboot);
      break;
    case 6:
      cmd_id = static_cast<std::uint16_t>(CmdId::kFactoryReset);
      break;
    default:
      cmd_id = static_cast<std::uint16_t>(CmdId::kSetGpsTimestamp);
      break;
  }

  // -- split the rest into reply datagrams -------------------------------
  std::vector<std::vector<std::byte>> replies;
  if ((flags & kFlagBadPrefix) != 0) {
    replies.push_back({std::byte{0xAA}, std::byte{0x00}, std::byte{0x18}});  // truncated header
  }
  const auto* bytes = reinterpret_cast<const std::byte*>(data);
  for (std::size_t i = 0; i < kMaxDatagrams && pos < size; ++i) {
    std::size_t len = static_cast<std::size_t>(data[pos]);
    ++pos;
    if (i + 1 == kMaxDatagrams || pos + len >= size) len = size - pos;  // remainder
    replies.emplace_back(bytes + pos, bytes + pos + len);
    pos += len;
  }

  const std::uint32_t seq = h.next_seq++;
  if (h.next_seq == 0) h.next_seq = 1;
  const SessionStats before = s.stats();
  const Endpoint to = s.local_endpoint();
  for (auto& r : replies) {
    if (r.empty()) continue;
    if ((flags & kFlagFixup) != 0 && r.size() > 3) fixup(r, seq);
    fuzz::require(h.peer.send_to(r, to).has_value());
    ++h.datagrams_sent;
  }

  // -- the call -----------------------------------------------------------
  const bool cancel = (flags & kFlagCancel) != 0;
  if (cancel) s.cancel();
  RequestOptions ro;
  ro.timeout = 1ms;
  ro.attempts = attempts;

  std::optional<SessionError> err;
  switch (op) {
    case 0: {
      const auto r = s.request(cmd_id, {}, ro);
      err = check_result(r, cmd_id);
      if (r) fuzz::require(r->cmd_id == cmd_id && r->seq_num == seq);
      break;
    }
    case 1: {
      const auto r = s.discovery_ack(ro);
      err = check_result(r, cmd_id);
      if (r) fuzz::require(r->ret_code == RetCode::kSuccess);
      break;
    }
    case 2: {
      const std::array<std::byte, 1> one{std::byte{1}};
      const std::array<KeyValue, 1> kvs{KeyValue{static_cast<std::uint16_t>(Key::kImuDataEn), one}};
      const auto r = s.configure(kvs, ro);
      err = check_result(r, cmd_id);
      if (r) {
        fuzz::require(r->ret_code == RetCode::kSuccess ||
                      r->ret_code == RetCode::kParamRebootEffect);
      }
      break;
    }
    case 3: {
      const std::array<Key, 2> keys{Key::kSn, Key::kCurWorkState};
      auto r = s.inquire(keys, ro);
      err = check_result(r, cmd_id);
      if (r) {
        fuzz::require(r->ret_code == RetCode::kSuccess);
        for (const auto& kv : r->values) fuzz::require(inside(kv.value, r->raw));
        const InquireResult moved = std::move(*r);
        for (const auto& kv : moved.values) fuzz::require(inside(kv.value, moved.raw));
      }
      break;
    }
    case 4: {
      const auto r = s.work_state(ro);
      err = check_result(r, cmd_id);
      break;
    }
    case 5:
      err = check_result(s.reboot(100, ro), cmd_id);
      break;
    case 6:
      err = check_result(s.factory_reset(ro), cmd_id);
      break;
    default:
      err = check_result(s.set_gps_time(1700000000ull * 1000000000ull, ro), cmd_id);
      break;
  }

  // -- invariants on the stats and on what the session sent --------------
  const SessionStats after = s.stats();
  fuzz::require(after.requests == before.requests + 1);
  fuzz::require(after.late_acks + after.bad_frames <= h.datagrams_sent);
  const std::size_t sent = drain_requests(h.peer, seq, cmd_id);
  if (cancel) {
    fuzz::require(err && err->kind == SessionErrorKind::kCancelled && err->attempts == 0);
    fuzz::require(sent == 0 && after.retries == before.retries);
  } else if (err && err->kind == SessionErrorKind::kTimeout) {
    fuzz::require(err->attempts == attempts);
    fuzz::require(after.retries == before.retries + attempts - 1);
    fuzz::require(after.timeouts == before.timeouts + 1 && sent == attempts);
  } else {
    fuzz::require(!err || err->kind == SessionErrorKind::kBadResponse ||
                  err->kind == SessionErrorKind::kLidarRejected);
    fuzz::require(sent >= 1 && sent <= attempts && after.timeouts == before.timeouts);
    fuzz::require(after.retries == before.retries + sent - 1);
  }
  return 0;
}
