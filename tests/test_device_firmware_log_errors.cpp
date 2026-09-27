// SPDX-License-Identifier: Apache-2.0
// Error paths of the firmware log collection in device.cpp (issue #96). The control requests
// of the Device go to tools/livox_mid360_sim.py as usual, but DeviceOptions::lidar_log_port
// points at a socket of the test that answers 0x0301 with whatever the test case asks for.
#include <array>
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "livox/mid360/mid360.hpp"
#include "sim_process.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{

bool wait_until(const std::function<bool()> & pred, std::chrono::milliseconds timeout = 5s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return pred();
}

/// Stands in for the LiDAR's log port (56500) on 127.0.0.1.
class FakeLogPort
{
public:
  enum class Mode : std::uint8_t
  {
    kOk,          ///< ACK with ret_code 0
    kSilent,      ///< no answer
    kEmptyAck,    ///< ACK without the ret_code byte
    kRejected,    ///< ACK with ret_code 0x02
    kWrongSeq,    ///< ACK whose seq_num is off by 1000
    kWrongCmd,    ///< ACK of another command (0x0302) with the right seq_num
    kGarbage,     ///< bytes that are no command frame
    kWrongFirst,  ///< a wrong-seq ACK followed by the right one
  };

  FakeLogPort()
  {
    auto s = UdpSocket::open(Endpoint{{127, 0, 0, 1}, 0});
    REQUIRE(s.has_value());
    socket_ = std::move(*s);
    thread_ = std::jthread([this](const std::stop_token & st) { run(st); });
  }

  [[nodiscard]] std::uint16_t port() const { return socket_.local_endpoint().port; }
  void set(Mode m) { mode_ = m; }
  [[nodiscard]] std::uint64_t requests() const { return requests_; }
  [[nodiscard]] bool send_failed() const { return send_failed_; }

private:
  void answer(const CommandHeader & h, const Endpoint & to, std::span<const std::byte> data) const
  {
    CommandFrameSpec spec;
    spec.seq_num = h.seq_num;
    spec.cmd_id = h.cmd_id;
    spec.cmd_type = CmdType::kAck;
    spec.sender_type = SenderType::kLidar;
    spec.data = data;
    send(spec, to);
  }

  void transmit(std::span<const std::byte> data, const Endpoint & to) const
  {
    if (!socket_.send_to(data, to)) {
      send_failed_ = true;
    }
  }

  void send(const CommandFrameSpec & spec, const Endpoint & to) const
  {
    std::array<std::byte, 64> buf{};
    const auto n = encode_command_frame(buf, spec);
    if (n) {
      transmit(std::span(buf).first(*n), to);
    }
  }

  void handle(const Datagram & d)
  {
    const auto frame = parse_command_frame(d.data);
    if (!frame || frame->header.cmd_type != CmdType::kReq) {
      return;
    }
    ++requests_;
    const CommandHeader & h = frame->header;
    const std::array ok{std::byte{0}};
    const std::array rejected{std::byte{0x02}};
    CommandFrameSpec other;
    other.cmd_type = CmdType::kAck;
    other.sender_type = SenderType::kLidar;
    other.seq_num = h.seq_num + 1000;
    other.cmd_id = h.cmd_id;
    other.data = ok;
    switch (mode_.load()) {
      case Mode::kOk:
        answer(h, d.from, ok);
        break;
      case Mode::kSilent:
        break;
      case Mode::kEmptyAck:
        answer(h, d.from, {});
        break;
      case Mode::kRejected:
        answer(h, d.from, rejected);
        break;
      case Mode::kWrongSeq:
        send(other, d.from);
        break;
      case Mode::kWrongCmd:
        other.seq_num = h.seq_num;
        other.cmd_id = 0x0302;
        send(other, d.from);
        break;
      case Mode::kGarbage: {
        const std::array<std::byte, 8> junk{std::byte{0x55}, std::byte{1}, std::byte{2}};
        transmit(junk, d.from);
        break;
      }
      case Mode::kWrongFirst:
        send(other, d.from);
        answer(h, d.from, ok);
        break;
    }
  }

  void run(const std::stop_token & st)
  {
    std::array<std::byte, kMaxDatagramSize> buf{};
    while (!st.stop_requested()) {
      const auto d = socket_.recv_one(buf);
      if (d) {
        handle(*d);
      } else {
        std::this_thread::sleep_for(2ms);
      }
    }
  }

  UdpSocket socket_;
  std::atomic<Mode> mode_{Mode::kOk};
  std::atomic<std::uint64_t> requests_{0};
  mutable std::atomic<bool> send_failed_{false};
  std::jthread thread_;  // last: stops before the socket closes
};

struct Fixture
{
  FakeLogPort fake;
  std::optional<SimProcess> sim;
  std::unique_ptr<Context> context;
  std::unique_ptr<Device> dev;

  explicit Fixture(RequestOptions request = {.timeout = 150ms, .attempts = 2})
  {
    std::string err;
    sim = SimProcess::start(err, {"--rate-multiplier", "0.05", "--push-rate", "10"});
    if (!sim) {
      SKIP(err);
    }
    ContextOptions co;
    co.bind_address = {127, 0, 0, 1};
    co.push_port = co.point_port = co.imu_port = co.log_port = 0;
    auto c = Context::create(co);
    REQUIRE(c.has_value());
    context = std::move(*c);

    DeviceOptions o;
    o.session.host_command_port = 0;
    o.session.request = request;
    o.lidar_log_port = fake.port();
    const Endpoint cmd{{127, 0, 0, 1}, sim->ports().cmd};
    const DiscoveredDevice found{
      .serial_number = sim->sn(), .ip = cmd.ip, .cmd_port = cmd.port, .dev_type = 9, .from = cmd};
    auto d = Device::open(*context, found, o);
    if (!d) {
      FAIL(to_string(d.error()));
    }
    dev = std::move(*d);
  }

  /// start_firmware_log() in `mode`, expected to fail with a session error of `kind`.
  SessionError fails(FakeLogPort::Mode mode, SessionErrorKind kind)
  {
    fake.set(mode);
    const auto r = dev->start_firmware_log();
    REQUIRE_FALSE(r.has_value());
    REQUIRE(r.error().kind == DeviceError::Kind::kSession);
    REQUIRE(r.error().session.has_value());
    const SessionError e = *r.error().session;
    CHECK(e.kind == kind);
    CHECK(e.cmd_id == static_cast<std::uint16_t>(CmdId::kCollectionLog));
    return e;
  }

  /// The Device still talks to the LiDAR and the next log request succeeds.
  void still_usable()
  {
    CHECK(dev->identity().has_value());
    CHECK_FALSE(fake.send_failed());
    fake.set(FakeLogPort::Mode::kOk);
    CHECK(dev->start_firmware_log().has_value());
    CHECK(dev->stop_firmware_log().has_value());
  }
};

}  // namespace

TEST_CASE("Device: firmware log ACK without a ret_code is a bad response", "[sim][device]")
{
  Fixture f;
  const auto e = f.fails(FakeLogPort::Mode::kEmptyAck, SessionErrorKind::kBadResponse);
  CHECK(e.attempts == 1);  // not retried
  f.still_usable();
}

TEST_CASE("Device: firmware log request rejected by the LiDAR", "[sim][device]")
{
  Fixture f;
  const auto e = f.fails(FakeLogPort::Mode::kRejected, SessionErrorKind::kLidarRejected);
  CHECK(e.attempts == 1);
  CHECK(e.ret_code == static_cast<RetCode>(0x02));
  f.still_usable();
}

TEST_CASE("Device: firmware log ACK with another seq_num or cmd_id is ignored", "[sim][device]")
{
  Fixture f;
  const auto before = f.fake.requests();
  auto e = f.fails(FakeLogPort::Mode::kWrongSeq, SessionErrorKind::kTimeout);
  CHECK(e.attempts == 2);
  CHECK(f.fake.requests() == before + 2);  // the retry went out
  e = f.fails(FakeLogPort::Mode::kWrongCmd, SessionErrorKind::kTimeout);
  CHECK(e.attempts == 2);
  CHECK(f.dev->stats().bad_log_packets == 0);

  // A stale ACK ahead of the right one does not hide it.
  f.fake.set(FakeLogPort::Mode::kWrongFirst);
  CHECK(f.dev->start_firmware_log().has_value());
  f.still_usable();
}

TEST_CASE("Device: bytes that are no frame on the log port are counted", "[sim][device]")
{
  Fixture f;
  (void)f.fails(FakeLogPort::Mode::kGarbage, SessionErrorKind::kTimeout);
  CHECK(wait_until([&] { return f.dev->stats().bad_log_packets == 2; }));
  CHECK(f.dev->stats().log_chunks == 0);
  f.still_usable();
}

TEST_CASE("Device: cancel() ends a firmware log request that waits for its ACK", "[sim][device]")
{
  Fixture f(RequestOptions{.timeout = 5s, .attempts = 3});
  f.fake.set(FakeLogPort::Mode::kSilent);
  const auto before = f.fake.requests();
  std::jthread canceller([&] {
    (void)wait_until([&] { return f.fake.requests() > before; });
    f.dev->cancel();
  });
  const auto t0 = std::chrono::steady_clock::now();
  const auto r = f.dev->start_firmware_log();
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  REQUIRE_FALSE(r.has_value());
  REQUIRE(r.error().session.has_value());
  CHECK(r.error().session->kind == SessionErrorKind::kCancelled);
  CHECK(r.error().session->attempts == 1);
  CHECK(elapsed < 3s);
  canceller.join();
  // The same cancel() left the Session's flag set, which aborts the next command (#114).
  const auto stale = f.dev->identity();
  REQUIRE_FALSE(stale.has_value());
  REQUIRE(stale.error().session.has_value());
  CHECK(stale.error().session->kind == SessionErrorKind::kCancelled);
  f.still_usable();
}
