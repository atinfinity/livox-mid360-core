// SPDX-License-Identifier: Apache-2.0
// Branches of session.cpp that the simulator does not produce (issue #99). A socket of the
// test stands in for the LiDAR and answers with whatever the test case asks for: duplicate
// and unusable discovery answers, a serial number that does not match, datagrams that are not
// the awaited ACK, ERROR / UPGRADE work states, stray datagrams during wait_for_state() and
// ACKs with a ret_code or a payload the typed calls must refuse.
#include <array>
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "livox/mid360/session.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{

using Bytes = std::vector<std::byte>;

void put_u16(Bytes & out, std::uint16_t v)
{
  out.push_back(static_cast<std::byte>(v & 0xFFu));
  out.push_back(static_cast<std::byte>(v >> 8u));
}

Bytes discovery_payload(std::uint8_t ret, std::string_view serial, std::uint16_t cmd_port)
{
  Bytes out{std::byte{ret}, std::byte{9}};
  for (std::size_t i = 0; i < 16; ++i) {
    out.push_back(i < serial.size() ? static_cast<std::byte>(serial[i]) : std::byte{0});
  }
  for (const std::uint8_t b : Ipv4{127, 0, 0, 1}) {
    out.push_back(static_cast<std::byte>(b));
  }
  put_u16(out, cmd_port);
  return out;
}

struct Value
{
  Key key;
  Bytes value;
};

Bytes inquire_payload(std::uint8_t ret, const std::vector<Value> & values)
{
  Bytes out{std::byte{ret}};
  put_u16(out, static_cast<std::uint16_t>(values.size()));
  for (const Value & v : values) {
    put_u16(out, static_cast<std::uint16_t>(v.key));
    put_u16(out, static_cast<std::uint16_t>(v.value.size()));
    out.insert(out.end(), v.value.begin(), v.value.end());
  }
  return out;
}

Bytes serial_value(std::string_view serial)
{
  Bytes out(16, std::byte{0});
  for (std::size_t i = 0; i < serial.size() && i < out.size(); ++i) {
    out[i] = static_cast<std::byte>(serial[i]);
  }
  return out;
}

Bytes state_payload(WorkState s)
{
  return inquire_payload(0, {{Key::kCurWorkState, {static_cast<std::byte>(s)}}});
}

/// Stands in for the LiDAR on 127.0.0.1: every request goes to the handler of the test case.
class FakeLidar
{
public:
  using Handler = std::function<void(FakeLidar &, const CommandHeader &, const Endpoint &)>;

  FakeLidar()
  {
    auto s = UdpSocket::open(Endpoint{{127, 0, 0, 1}, 0});
    REQUIRE(s.has_value());
    socket_ = std::move(*s);
    thread_ = std::jthread([this](const std::stop_token & st) { run(st); });
  }

  [[nodiscard]] Endpoint endpoint() const { return Endpoint::loopback(port()); }
  [[nodiscard]] std::uint16_t port() const { return socket_.local_endpoint().port; }
  [[nodiscard]] std::uint64_t requests() const { return requests_; }
  [[nodiscard]] bool send_failed() const { return send_failed_; }

  void on_request(Handler h)
  {
    const std::lock_guard lock(mutex_);
    handler_ = std::move(h);
  }

  /// Answer every request with the same payload.
  void always(Bytes payload)
  {
    on_request([p = std::move(payload)](
                 FakeLidar & l, const CommandHeader & h, const Endpoint & to) { l.ack(h, to, p); });
  }

  void ack(const CommandHeader & h, const Endpoint & to, std::span<const std::byte> data)
  {
    frame(h.seq_num, h.cmd_id, CmdType::kAck, to, data);
  }

  void frame(
    std::uint32_t seq, std::uint16_t cmd_id, CmdType type, const Endpoint & to,
    std::span<const std::byte> data)
  {
    CommandFrameSpec spec;
    spec.seq_num = seq;
    spec.cmd_id = cmd_id;
    spec.cmd_type = type;
    spec.sender_type = SenderType::kLidar;
    spec.data = data;
    const auto bytes = build_command_frame(spec);
    REQUIRE(bytes.has_value());
    raw(*bytes, to);
  }

  void raw(std::span<const std::byte> data, const Endpoint & to)
  {
    if (!socket_.send_to(data, to)) {
      send_failed_ = true;
    }
  }

private:
  void run(const std::stop_token & st)
  {
    std::array<std::byte, kMaxDatagramSize> buf{};
    while (!st.stop_requested()) {
      const auto d = socket_.recv_one(buf);
      if (!d) {
        std::this_thread::sleep_for(2ms);
        continue;
      }
      const auto view = parse_command_frame(d->data);
      if (!view || view->header.cmd_type != CmdType::kReq) {
        continue;
      }
      ++requests_;
      Handler h;
      {
        const std::lock_guard lock(mutex_);
        h = handler_;
      }
      if (h) {
        h(*this, view->header, d->from);
      }
    }
  }

  UdpSocket socket_;
  std::mutex mutex_;
  Handler handler_;
  std::atomic<std::uint64_t> requests_{0};
  std::atomic<bool> send_failed_{false};
  std::jthread thread_;  // last: stops before the socket closes
};

const std::array<std::byte, 8> kJunk{std::byte{0x55}, std::byte{1}, std::byte{2}};

SessionOptions session_options()
{
  SessionOptions o;
  o.host_command_port = 0;
  o.bind_address = {127, 0, 0, 1};
  o.verify_serial = false;
  o.request = {.timeout = 150ms, .attempts = 2};
  o.state_poll_interval = 50ms;
  return o;
}

DiscoveredDevice device(const FakeLidar & lidar, std::string serial = "FAKE0001")
{
  return DiscoveredDevice{
    .serial_number = std::move(serial),
    .ip = {127, 0, 0, 1},
    .cmd_port = lidar.port(),
    .dev_type = 9,
    .from = lidar.endpoint()};
}

Session connect(const FakeLidar & lidar)
{
  auto s = Session::connect(device(lidar), session_options());
  REQUIRE(s.has_value());
  return std::move(*s);
}

DiscoveryOptions discovery_options(std::vector<Endpoint> targets, std::chrono::milliseconds t)
{
  DiscoveryOptions o;
  o.targets = std::move(targets);
  o.timeout = t;
  o.bind_address = {127, 0, 0, 1};
  return o;
}

void check_rejected(
  const SessionError & e, CmdId cmd, std::uint8_t ret, std::uint16_t error_key = 0)
{
  CHECK(e.kind == SessionErrorKind::kLidarRejected);
  CHECK(e.cmd_id == static_cast<std::uint16_t>(cmd));
  CHECK(e.ret_code == static_cast<RetCode>(ret));
  CHECK(e.error_key == error_key);
}

void check_bad_response(const SessionError & e, CmdId cmd)
{
  CHECK(e.kind == SessionErrorKind::kBadResponse);
  CHECK(e.cmd_id == static_cast<std::uint16_t>(cmd));
  CHECK(e.parse.has_value());
}

}  // namespace

TEST_CASE("discover: duplicate answers of one device are collapsed", "[session][fake]")
{
  FakeLidar lidar;
  lidar.on_request([](FakeLidar & l, const CommandHeader & h, const Endpoint & to) {
    l.raw(kJunk, to);
    l.ack(h, to, discovery_payload(0, "FAKE0001", l.port()));
    l.ack(h, to, discovery_payload(0, "FAKE0001", 1234));
    l.ack(h, to, discovery_payload(0, "FAKE0001", 5678));
  });
  const auto start = std::chrono::steady_clock::now();
  const auto found = discover(discovery_options({lidar.endpoint()}, 3s));
  REQUIRE(found.has_value());
  REQUIRE(found->size() == 1);
  CHECK(found->front().serial_number == "FAKE0001");
  CHECK(found->front().cmd_port == lidar.port());  // the first answer wins
  CHECK(found->front().from == lidar.endpoint());
  CHECK(std::chrono::steady_clock::now() - start < 1500ms);
  CHECK(lidar.requests() == 1);
  CHECK_FALSE(lidar.send_failed());
}

TEST_CASE("discover: returns once every unicast target has answered", "[session][fake]")
{
  FakeLidar a;
  FakeLidar b;
  a.on_request([](FakeLidar & l, const CommandHeader & h, const Endpoint & to) {
    l.ack(h, to, discovery_payload(0, "FAKE000A", l.port()));
  });
  b.on_request([](FakeLidar & l, const CommandHeader & h, const Endpoint & to) {
    std::this_thread::sleep_for(100ms);
    l.ack(h, to, discovery_payload(0, "FAKE000B", l.port()));
  });
  const auto start = std::chrono::steady_clock::now();
  const auto found = discover(discovery_options({a.endpoint(), b.endpoint()}, 3s));
  REQUIRE(found.has_value());
  CHECK(std::chrono::steady_clock::now() - start < 1500ms);
  CHECK_FALSE(found->empty());
  CHECK(found->front().serial_number == "FAKE000A");
}

TEST_CASE("discover: unusable answers are ignored until the timeout", "[session][fake]")
{
  FakeLidar lidar;
  lidar.on_request([](FakeLidar & l, const CommandHeader & h, const Endpoint & to) {
    l.raw(kJunk, to);
    l.ack(h, to, discovery_payload(1, "FAKE0001", l.port()));  // rejected
    l.ack(h, to, Bytes(10, std::byte{0}));                     // truncated
    l.frame(h.seq_num, 0x0101, CmdType::kAck, to, state_payload(WorkState::kReady));
    l.frame(h.seq_num, h.cmd_id, CmdType::kReq, to, {});  // not an ACK
  });
  const auto start = std::chrono::steady_clock::now();
  const auto found = discover(discovery_options({lidar.endpoint()}, 300ms));
  REQUIRE(found.has_value());
  CHECK(found->empty());
  CHECK(std::chrono::steady_clock::now() - start >= 300ms);
  CHECK(lidar.requests() == 1);
}

TEST_CASE("Session::connect verifies the serial number", "[session][fake]")
{
  FakeLidar lidar;
  SessionOptions o = session_options();
  o.verify_serial = true;

  SECTION("mismatch")
  {
    lidar.always(inquire_payload(0, {{Key::kSn, serial_value("OTHER")}}));
    const auto s = Session::connect(device(lidar), o);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().kind == SessionErrorKind::kBadResponse);
    CHECK(s.error().cmd_id == 0x0101);
    CHECK(s.error().attempts == 1);
    CHECK(lidar.requests() == 1);
  }
  SECTION("match")
  {
    lidar.always(inquire_payload(0, {{Key::kSn, serial_value("FAKE0001")}}));
    const auto s = Session::connect(device(lidar), o);
    REQUIRE(s.has_value());
    CHECK(s->serial_number() == "FAKE0001");
  }
  SECTION("the answer carries no serial number")
  {
    lidar.always(state_payload(WorkState::kReady));
    const auto by_device = Session::connect(device(lidar), o);
    REQUIRE_FALSE(by_device.has_value());
    check_bad_response(by_device.error(), CmdId::kParamInquire);
    const auto by_endpoint = Session::connect(lidar.endpoint(), o);
    REQUIRE_FALSE(by_endpoint.has_value());
    check_bad_response(by_endpoint.error(), CmdId::kParamInquire);
  }
  SECTION("rejected")
  {
    lidar.always(inquire_payload(0x20, {{Key::kSn, {}}}));
    const auto s = Session::connect(device(lidar), o);
    REQUIRE_FALSE(s.has_value());
    check_rejected(s.error(), CmdId::kParamInquire, 0x20, static_cast<std::uint16_t>(Key::kSn));
  }
  SECTION("not verified: nothing is asked")
  {
    o.verify_serial = false;
    const auto s = Session::connect(device(lidar, "TRUSTED"), o);
    REQUIRE(s.has_value());
    CHECK(s->serial_number() == "TRUSTED");
    CHECK(lidar.requests() == 0);
  }
}

TEST_CASE("Session::request skips datagrams that are not the awaited ACK", "[session][fake]")
{
  FakeLidar lidar;
  Session s = connect(lidar);
  const Bytes ok{std::byte{0}};

  SECTION("the ACK follows")
  {
    lidar.on_request([&](FakeLidar & l, const CommandHeader & h, const Endpoint & to) {
      l.raw(kJunk, to);                                            // kBadFrame
      l.frame(h.seq_num + 1000, h.cmd_id, CmdType::kAck, to, ok);  // kLate: other request
      l.frame(h.seq_num, 0x0101, CmdType::kAck, to, ok);           // kLate: other command
      l.frame(h.seq_num, h.cmd_id, CmdType::kReq, to, ok);         // kNotAck
      l.ack(h, to, ok);
    });
    const auto r = s.request(0x0200, {});
    REQUIRE(r.has_value());
    CHECK(r->cmd_id == 0x0200);
    CHECK(r->data == ok);
    CHECK(s.stats().requests == 1);
    CHECK(s.stats().bad_frames == 1);
    CHECK(s.stats().late_acks == 2);
    CHECK(s.stats().retries == 0);
    CHECK(s.stats().timeouts == 0);
  }
  SECTION("no ACK at all")
  {
    lidar.on_request([&](FakeLidar & l, const CommandHeader & h, const Endpoint & to) {
      l.raw(kJunk, to);
      l.frame(h.seq_num + 1, h.cmd_id, CmdType::kAck, to, ok);
    });
    const auto r = s.request(0x0200, {});
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == SessionErrorKind::kTimeout);
    CHECK(r.error().attempts == 2);
    CHECK(s.stats().bad_frames == 2);
    CHECK(s.stats().late_acks == 2);
    CHECK(s.stats().retries == 1);
    CHECK(s.stats().timeouts == 1);
  }
  SECTION("the ACK of the first attempt answers the retry")
  {
    std::atomic<int> seen{0};
    lidar.on_request([&](FakeLidar & l, const CommandHeader & h, const Endpoint & to) {
      if (++seen == 2) {
        l.ack(h, to, ok);  // same seq_num on every attempt
      }
    });
    const auto r = s.request(0x0200, {});
    REQUIRE(r.has_value());
    CHECK(s.stats().retries == 1);
  }
  CHECK_FALSE(lidar.send_failed());
}

TEST_CASE("Session::request numbers the requests from 1", "[session][fake]")
{
  // The wrap-around of the 32 bit counter (0 is skipped) needs 2^32 requests and is not
  // reachable from here.
  FakeLidar lidar;
  std::mutex mutex;
  std::vector<std::uint32_t> seqs;
  lidar.on_request([&](FakeLidar & l, const CommandHeader & h, const Endpoint & to) {
    {
      const std::lock_guard lock(mutex);
      seqs.push_back(h.seq_num);
    }
    const Bytes ok{std::byte{0}};
    l.ack(h, to, ok);
  });
  Session s = connect(lidar);
  for (int i = 0; i < 3; ++i) {
    REQUIRE(s.request(0x0200, {}).has_value());
  }
  const std::lock_guard lock(mutex);
  CHECK(seqs == std::vector<std::uint32_t>{1, 2, 3});
}

TEST_CASE("Session::wait_for_state ends on ERROR and UPGRADE", "[session][fake]")
{
  FakeLidar lidar;
  Session s = connect(lidar);
  for (const WorkState state : {WorkState::kError, WorkState::kUpgrade}) {
    lidar.always(state_payload(state));
    const auto start = std::chrono::steady_clock::now();
    const auto r = s.wait_for_state(WorkState::kReady, 3s);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == SessionErrorKind::kUnexpectedState);
    CHECK(r.error().cmd_id == 0x0101);
    CHECK(r.error().work_state == state);
    CHECK(std::chrono::steady_clock::now() - start < 1s);
    // Unless it is the state that was asked for.
    CHECK(s.wait_for_state(state, 1s).has_value());
  }
}

TEST_CASE("Session::wait_for_state drains stray datagrams between polls", "[session][fake]")
{
  FakeLidar lidar;
  Session s = connect(lidar);
  std::atomic<int> polls{0};
  std::atomic<int> ready_after{1000};
  lidar.on_request([&](FakeLidar & l, const CommandHeader & h, const Endpoint & to) {
    const bool ready = ++polls > ready_after;
    l.ack(h, to, state_payload(ready ? WorkState::kReady : WorkState::kSelfCheck));
    // Arrives while the session sleeps until the next poll.
    std::this_thread::sleep_for(10ms);
    l.ack(h, to, state_payload(WorkState::kReady));
    l.raw(kJunk, to);
  });

  SECTION("timeout")
  {
    const auto r = s.wait_for_state(WorkState::kReady, 300ms);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == SessionErrorKind::kTimeout);
    CHECK(r.error().cmd_id == 0x0101);
    CHECK(r.error().work_state == WorkState::kSelfCheck);
    CHECK(polls >= 2);
    CHECK(s.stats().late_acks >= 2);
    CHECK(s.stats().timeouts == 0);  // every poll was answered
  }
  SECTION("the state is reached")
  {
    ready_after = 3;
    REQUIRE(s.wait_for_state(WorkState::kReady, 3s).has_value());
    CHECK(polls == 4);
    CHECK(s.stats().late_acks >= 3);
  }
  SECTION("a poll without an answer fails the wait")
  {
    lidar.on_request(nullptr);
    const auto r = s.wait_for_state(WorkState::kReady, 3s);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == SessionErrorKind::kTimeout);
    CHECK(r.error().attempts == 2);
    CHECK_FALSE(r.error().work_state.has_value());
  }
}

TEST_CASE("Session: typed calls check the ret_code and the payload", "[session][fake]")
{
  FakeLidar lidar;
  Session s = connect(lidar);
  const std::array<std::byte, 1> one{std::byte{1}};
  const std::vector<KeyValue> kvs{{static_cast<std::uint16_t>(Key::kImuDataEn), one}};
  const std::array<Key, 1> keys{Key::kCurWorkState};

  SECTION("discovery_ack")
  {
    lidar.always(discovery_payload(0, "FAKE0001", 56100));
    const auto ok = s.discovery_ack();
    REQUIRE(ok.has_value());
    CHECK(ok->serial_number_view() == "FAKE0001");
    CHECK(ok->cmd_port == 56100);

    lidar.always(discovery_payload(1, "FAKE0001", 56100));
    const auto rejected = s.discovery_ack();
    REQUIRE_FALSE(rejected.has_value());
    check_rejected(rejected.error(), CmdId::kDiscovery, 1);

    lidar.always(Bytes(23, std::byte{0}));
    const auto bad = s.discovery_ack();
    REQUIRE_FALSE(bad.has_value());
    check_bad_response(bad.error(), CmdId::kDiscovery);
  }
  SECTION("configure")
  {
    lidar.always({std::byte{0x21}, std::byte{0}, std::byte{0}});
    const auto reboot = s.configure(kvs);
    REQUIRE(reboot.has_value());
    CHECK(reboot->ret_code == RetCode::kParamRebootEffect);

    lidar.always({std::byte{0x20}, std::byte{0x1C}, std::byte{0}});
    const auto rejected = s.configure(kvs);
    REQUIRE_FALSE(rejected.has_value());
    check_rejected(rejected.error(), CmdId::kParamConfig, 0x20, 0x001C);

    lidar.always({std::byte{0}, std::byte{0}});
    const auto bad = s.configure(kvs);
    REQUIRE_FALSE(bad.has_value());
    check_bad_response(bad.error(), CmdId::kParamConfig);
  }
  SECTION("inquire")
  {
    lidar.always(inquire_payload(0x20, {{Key::kCurWorkState, {}}}));
    const auto named = s.inquire(keys);
    REQUIRE_FALSE(named.has_value());
    check_rejected(named.error(), CmdId::kParamInquire, 0x20, 0x8006);

    lidar.always(inquire_payload(0x01, {}));
    const auto unnamed = s.inquire(keys);
    REQUIRE_FALSE(unnamed.has_value());
    check_rejected(unnamed.error(), CmdId::kParamInquire, 0x01, 0);

    lidar.always({std::byte{0}, std::byte{1}});
    const auto shorter = s.inquire(keys);
    REQUIRE_FALSE(shorter.has_value());
    check_bad_response(shorter.error(), CmdId::kParamInquire);

    // Two values announced, one present.
    Bytes cut = state_payload(WorkState::kReady);
    cut[1] = std::byte{2};
    lidar.always(cut);
    const auto truncated = s.inquire(keys);
    REQUIRE_FALSE(truncated.has_value());
    check_bad_response(truncated.error(), CmdId::kParamInquire);
  }
  SECTION("work_state")
  {
    lidar.always(inquire_payload(0, {{Key::kSn, serial_value("FAKE0001")}}));
    const auto missing = s.work_state();
    REQUIRE_FALSE(missing.has_value());
    check_bad_response(missing.error(), CmdId::kParamInquire);

    lidar.always(inquire_payload(0, {{Key::kCurWorkState, {}}}));
    const auto empty = s.work_state();
    REQUIRE_FALSE(empty.has_value());
    check_bad_response(empty.error(), CmdId::kParamInquire);
  }
  SECTION("reboot, factory_reset and set_gps_time")
  {
    lidar.always({std::byte{0}});
    CHECK(s.reboot().has_value());
    CHECK(s.factory_reset().has_value());
    CHECK(s.set_gps_time(1).has_value());

    lidar.always({std::byte{2}});
    const auto reboot = s.reboot();
    REQUIRE_FALSE(reboot.has_value());
    check_rejected(reboot.error(), CmdId::kReboot, 2);
    const auto reset = s.factory_reset();
    REQUIRE_FALSE(reset.has_value());
    check_rejected(reset.error(), CmdId::kFactoryReset, 2);
    const auto gps = s.set_gps_time(1);
    REQUIRE_FALSE(gps.has_value());
    check_rejected(gps.error(), CmdId::kSetGpsTimestamp, 2);

    lidar.always({});
    const auto bad = s.reboot();
    REQUIRE_FALSE(bad.has_value());
    check_bad_response(bad.error(), CmdId::kReboot);
    const auto bad_gps = s.set_gps_time(1);
    REQUIRE_FALSE(bad_gps.has_value());
    check_bad_response(bad_gps.error(), CmdId::kSetGpsTimestamp);
  }
  CHECK_FALSE(lidar.send_failed());
}
