// SPDX-License-Identifier: Apache-2.0
// Session layer against tools/livox_mid360_sim.py: discovery, connect, typed commands,
// retries, timeouts, work-state waits and cancellation.
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "livox/mid360/session.hpp"
#include "sim_process.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace {

struct Fixture {
  std::optional<SimProcess> sim;
  std::string err;

  Fixture() { sim = SimProcess::start(err, {"--startup-delay", "1"}); }

  DiscoveryOptions discovery_options() const {
    DiscoveryOptions o;
    o.targets = {Endpoint::loopback(sim->ports().discovery)};
    o.timeout = 2s;
    return o;
  }

  SessionOptions session_options() const {
    SessionOptions o;
    o.host_command_port = 0;  // ephemeral: several tests may run in one process
    o.bind_address = {127, 0, 0, 1};
    return o;
  }

  Session connect() const {
    auto s = Session::connect(Endpoint::loopback(sim->ports().cmd), session_options());
    REQUIRE(s.has_value());
    return std::move(*s);
  }

  /// Send a control command and wait until the simulator confirms it.
  void control(std::string_view json) {
    REQUIRE(sim->control(json));
    REQUIRE(sim->wait_event("\"control\"").has_value());
  }
};

}  // namespace

TEST_CASE("Session: unicast discovery and connect", "[sim][session]") {
  Fixture f;
  if (!f.sim) SKIP("simulator unavailable: " << f.err);

  const auto devices = discover(f.discovery_options());
  REQUIRE(devices.has_value());
  REQUIRE(devices->size() == 1);
  const auto& dev = devices->front();
  CHECK(dev.serial_number == f.sim->sn());
  CHECK(dev.cmd_port == f.sim->ports().cmd);
  CHECK(dev.ip == Ipv4{127, 0, 0, 1});
  CHECK(dev.from.port == f.sim->ports().discovery);

  auto session = Session::connect(dev, f.session_options());
  REQUIRE(session.has_value());
  CHECK(session->serial_number() == f.sim->sn());
  CHECK(session->lidar_endpoint() == Endpoint::loopback(f.sim->ports().cmd));
  CHECK(session->local_endpoint().port != 0);
  CHECK(session->stats().requests == 1);

  // Serial mismatch is detected when verify_serial is on.
  DiscoveredDevice wrong = dev;
  wrong.serial_number = "NOPE";
  const auto bad = Session::connect(wrong, f.session_options());
  REQUIRE_FALSE(bad.has_value());
  CHECK(bad.error().kind == SessionErrorKind::kBadResponse);

  // Move keeps the socket alive. (The simulator, like the wiki, only answers 0x0000 on
  // the discovery port, so discovery_ack() is not exercised here.)
  Session moved = std::move(*session);
  CHECK(moved.work_state().has_value());
  CHECK(f.sim->stop() == 0);
}

TEST_CASE("Session: typed commands and rejections", "[sim][session]") {
  Fixture f;
  if (!f.sim) SKIP("simulator unavailable: " << f.err);
  Session s = f.connect();

  // Read-only key -> 0x22 with error_key.
  const std::byte one{1};
  const KeyValue ro{static_cast<std::uint16_t>(Key::kSn), std::span<const std::byte>(&one, 1)};
  const auto rej = s.configure(std::span<const KeyValue>(&ro, 1));
  REQUIRE_FALSE(rej.has_value());
  CHECK(rej.error().kind == SessionErrorKind::kLidarRejected);
  CHECK(rej.error().ret_code == RetCode::kParamReadOnly);
  CHECK(rej.error().error_key == 0x8000);
  CHECK(rej.error().cmd_id == 0x0100);
  CHECK(to_string(rej.error()).find("lidar_rejected") == 0);

  // A valid configure succeeds.
  const auto host = encode_host_ip_config({.ip = {127, 0, 0, 1}, .dst_port = 1, .src_port = 2});
  const KeyValue kv{static_cast<std::uint16_t>(Key::kPointCloudHostIpCfg), host};
  const auto ok = s.configure(std::span<const KeyValue>(&kv, 1));
  REQUIRE(ok.has_value());
  CHECK(ok->ret_code == RetCode::kSuccess);

  // Inquire returns owned values that survive a move.
  const Key keys[] = {Key::kSn, Key::kPointCloudHostIpCfg};
  auto inq = s.inquire(keys);
  REQUIRE(inq.has_value());
  InquireResult moved = std::move(*inq);
  REQUIRE(moved.get(Key::kSn).has_value());
  CHECK(decode_string(*moved.get(Key::kSn)) == f.sim->sn());
  REQUIRE(moved.get(Key::kPointCloudHostIpCfg).has_value());
  const auto got = *moved.get(Key::kPointCloudHostIpCfg);
  CHECK(std::equal(got.begin(), got.end(), host.begin(), host.end()));

  // Unknown key on inquire -> 0x20.
  const std::uint16_t unknown[] = {0x7FFF};
  const auto bad = s.inquire(unknown);
  REQUIRE_FALSE(bad.has_value());
  CHECK(bad.error().kind == SessionErrorKind::kLidarRejected);
  CHECK(bad.error().ret_code == RetCode::kParamNotSupport);

  CHECK(s.set_gps_time(123456789).has_value());
  CHECK(f.sim->stop() == 0);
}

TEST_CASE("Session: retry on dropped ACK", "[sim][session]") {
  Fixture f;
  if (!f.sim) SKIP("simulator unavailable: " << f.err);
  Session s = f.connect();

  f.control(R"({"cmd":"drop_ack","count":1})");
  RequestOptions ro;
  ro.timeout = 200ms;
  ro.attempts = 3;
  const auto ws = s.work_state(ro);
  REQUIRE(ws.has_value());
  CHECK(s.stats().retries == 1);
  CHECK(s.stats().timeouts == 0);
  CHECK(f.sim->stop() == 0);
}

TEST_CASE("Session: timeout after all attempts", "[sim][session]") {
  Fixture f;
  if (!f.sim) SKIP("simulator unavailable: " << f.err);
  Session s = f.connect();

  f.control(R"({"cmd":"silence","seconds":3})");
  RequestOptions ro;
  ro.timeout = 100ms;
  ro.attempts = 3;
  const auto r = s.work_state(ro);
  REQUIRE_FALSE(r.has_value());
  CHECK(r.error().kind == SessionErrorKind::kTimeout);
  CHECK(r.error().attempts == 3);
  CHECK(r.error().cmd_id == 0x0101);
  CHECK(s.stats().retries == 2);
  CHECK(s.stats().timeouts == 1);
  CHECK(to_string(r.error()) == "timeout cmd 0x0101 after 3 attempt(s)");
  CHECK(f.sim->stop() == 0);
}

TEST_CASE("Session: wait_for_state", "[sim][session]") {
  Fixture f;
  if (!f.sim) SKIP("simulator unavailable: " << f.err);
  Session s = f.connect();

  // The simulator spends --startup-delay in MOTORSTARTUP before SAMPLING.
  const auto ok = s.wait_for_state(WorkState::kSampling, 5s);
  REQUIRE(ok.has_value());
  CHECK(s.work_state().value() == WorkState::kSampling);

  const auto to = s.wait_for_state(WorkState::kIdle, 300ms);
  REQUIRE_FALSE(to.has_value());
  CHECK(to.error().kind == SessionErrorKind::kTimeout);
  CHECK(to.error().work_state == WorkState::kSampling);

  f.control(R"({"cmd":"set_state","state":4})");
  const auto bad = s.wait_for_state(WorkState::kIdle, 2s);
  REQUIRE_FALSE(bad.has_value());
  CHECK(bad.error().kind == SessionErrorKind::kUnexpectedState);
  CHECK(bad.error().work_state == WorkState::kError);
  CHECK(f.sim->stop() == 0);
}

TEST_CASE("Session: cancel from another thread", "[sim][session]") {
  Fixture f;
  if (!f.sim) SKIP("simulator unavailable: " << f.err);
  Session s = f.connect();

  f.control(R"({"cmd":"silence","seconds":5})");
  std::thread canceller([&s] {
    std::this_thread::sleep_for(150ms);
    s.cancel();
  });
  RequestOptions ro;
  ro.timeout = 2s;
  ro.attempts = 3;
  const auto start = std::chrono::steady_clock::now();
  const auto r = s.work_state(ro);
  canceller.join();
  REQUIRE_FALSE(r.has_value());
  CHECK(r.error().kind == SessionErrorKind::kCancelled);
  CHECK(std::chrono::steady_clock::now() - start < 1500ms);
  CHECK(f.sim->stop() == 0);
}

TEST_CASE("Session: broadcast discovery", "[sim][session][.broadcast]") {
  // The simulator binds 127.0.0.1, so 255.255.255.255 does not reach it on most hosts.
  // Kept as a hidden test for manual runs on a LAN with a real Mid-360.
  DiscoveryOptions o;
  o.timeout = 1s;
  const auto devices = discover(o);
  if (!devices) WARN("broadcast discovery failed: " << to_string(devices.error()));
  else if (devices->empty()) WARN("broadcast discovery found no devices");
}
