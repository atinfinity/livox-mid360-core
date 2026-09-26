// SPDX-License-Identifier: Apache-2.0
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include "livox/mid360/transport.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{

std::vector<std::byte> bytes_of(const char * text)
{
  std::vector<std::byte> out(std::strlen(text));
  std::memcpy(out.data(), text, out.size());
  return out;
}

UdpSocket open_loopback(const SocketOptions & opts = {})
{
  auto s = UdpSocket::open(Endpoint::loopback(0), opts);
  REQUIRE(s.has_value());
  REQUIRE(s->is_open());
  REQUIRE(s->local_endpoint().port != 0);
  return std::move(*s);
}

/// Wait until `sock` becomes readable (bounded) so that loopback delivery latency does
/// not make the tests flaky.
bool wait_readable(const UdpSocket & sock, std::chrono::milliseconds timeout = 2000ms)
{
  auto p = Poller::create();
  REQUIRE(p.has_value());
  REQUIRE(p->add(sock, 1).has_value());
  auto ev = p->wait(timeout);
  REQUIRE(ev.has_value());
  return !ev->empty() && (*ev)[0].readable;
}

}  // namespace

TEST_CASE("Endpoint parse/format round trip", "[transport][endpoint]")
{
  const auto ep = parse_endpoint("192.168.1.50:56101");
  REQUIRE(ep.has_value());
  CHECK(ep->ip == std::array<std::uint8_t, 4>{192, 168, 1, 50});
  CHECK(ep->port == 56101);
  CHECK(to_string(*ep) == "192.168.1.50:56101");

  const auto no_port = parse_endpoint("10.0.0.1");
  REQUIRE(no_port.has_value());
  CHECK(no_port->port == 0);
  CHECK(ip_to_string(no_port->ip) == "10.0.0.1");

  CHECK_FALSE(parse_endpoint("").has_value());
  CHECK_FALSE(parse_endpoint("1.2.3").has_value());
  CHECK_FALSE(parse_endpoint("1.2.3.4.5").has_value());
  CHECK_FALSE(parse_endpoint("256.0.0.1").has_value());
  CHECK_FALSE(parse_endpoint("1.2.3.4:").has_value());
  CHECK_FALSE(parse_endpoint("1.2.3.4:70000").has_value());
  CHECK_FALSE(parse_endpoint("a.b.c.d").has_value());
  CHECK(Endpoint::broadcast(56000) == Endpoint{{255, 255, 255, 255}, 56000});
}

TEST_CASE("TransportError to_string", "[transport][error]")
{
  CHECK(to_string(TransportErrorCode::kAddressInUse) == "address_in_use");
  const TransportError e{TransportErrorCode::kTimeout, 0};
  CHECK(to_string(e) == "timeout");
  const TransportError with_errno{TransportErrorCode::kBind, EADDRINUSE};
  CHECK(to_string(with_errno).starts_with("bind ("));
}

TEST_CASE("Loopback send and receive one datagram", "[transport][socket]")
{
  const UdpSocket a = open_loopback();
  const UdpSocket b = open_loopback();

  const auto payload = bytes_of("hello mid360");
  const auto sent = a.send_to(payload, b.local_endpoint());
  REQUIRE(sent.has_value());
  CHECK(*sent == payload.size());

  REQUIRE(wait_readable(b));
  std::array<std::byte, kMaxDatagramSize> buf{};
  const auto d = b.recv_one(buf);
  REQUIRE(d.has_value());
  CHECK(std::vector<std::byte>(d->data.begin(), d->data.end()) == payload);
  CHECK(d->from == a.local_endpoint());
  CHECK(d->recv_time_ns > 1'600'000'000ull * 1'000'000'000ull);  // after 2020

  // Nothing else pending.
  const auto empty = b.recv_one(buf);
  REQUIRE_FALSE(empty.has_value());
  CHECK(empty.error().code == TransportErrorCode::kWouldBlock);
}

TEST_CASE("recv_batch drains several datagrams", "[transport][socket]")
{
  const UdpSocket a = open_loopback();
  const UdpSocket b = open_loopback();

  constexpr int kCount = 5;
  for (int i = 0; i < kCount; ++i) {
    const std::array<std::byte, 1> p{static_cast<std::byte>(i)};
    REQUIRE(a.send_to(p, b.local_endpoint()).has_value());
  }

  std::array<std::array<std::byte, 64>, 8> storage{};
  std::array<Datagram, 8> dg{};
  std::size_t total = 0;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  std::vector<int> seen;
  while (total < kCount && std::chrono::steady_clock::now() < deadline) {
    REQUIRE(wait_readable(b));
    for (std::size_t i = 0; i < dg.size(); ++i) {
      dg[i].data = storage[i];
    }
    const auto n = b.recv_batch(dg);
    REQUIRE(n.has_value());
    for (std::size_t i = 0; i < *n; ++i) {
      REQUIRE(dg[i].data.size() == 1);
      seen.push_back(static_cast<int>(dg[i].data[0]));
      CHECK(dg[i].from == a.local_endpoint());
    }
    total += *n;
  }
  CHECK(total == kCount);
  CHECK(seen.size() == kCount);
}

TEST_CASE("Datagram larger than buffer is truncated to buffer size", "[transport][socket]")
{
  const UdpSocket a = open_loopback();
  const UdpSocket b = open_loopback();
  const std::vector<std::byte> big(200, std::byte{0xAB});
  REQUIRE(a.send_to(big, b.local_endpoint()).has_value());
  REQUIRE(wait_readable(b));
  std::array<std::byte, 16> small{};
  const auto d = b.recv_one(small);
  REQUIRE(d.has_value());
  CHECK(d->data.size() == 16);
}

TEST_CASE("Binding the same port twice reports kAddressInUse", "[transport][socket]")
{
  SocketOptions opts;
  opts.reuse_address = false;
  const UdpSocket a = open_loopback(opts);
  const auto b = UdpSocket::open(a.local_endpoint(), opts);
  REQUIRE_FALSE(b.has_value());
  CHECK(b.error().code == TransportErrorCode::kAddressInUse);
  CHECK(b.error().errno_value == EADDRINUSE);
}

TEST_CASE("Reserved options are rejected", "[transport][socket]")
{
  SocketOptions opts;
  opts.bind_to_device = "eth0";
  const auto s = UdpSocket::open(Endpoint::loopback(0), opts);
  REQUIRE_FALSE(s.has_value());
  CHECK(s.error().code == TransportErrorCode::kInvalidArgument);
}

TEST_CASE("Receive buffer size can be requested and queried", "[transport][socket]")
{
  SocketOptions opts;
  opts.recv_buffer_bytes = std::size_t{512} * 1024;
  const UdpSocket s = open_loopback(opts);
  const auto eff = s.recv_buffer_bytes();
  REQUIRE(eff.has_value());
  CHECK(*eff > 0);  // kernel may clamp or double; only require sanity
}

TEST_CASE("Closed socket reports kClosed", "[transport][socket]")
{
  UdpSocket s = open_loopback();
  s.close();
  CHECK_FALSE(s.is_open());
  std::array<std::byte, 4> buf{};
  CHECK(s.send_to(buf, Endpoint::loopback(9)).error().code == TransportErrorCode::kClosed);
  CHECK(s.recv_one(buf).error().code == TransportErrorCode::kClosed);
}

TEST_CASE("Move transfers ownership", "[transport][socket]")
{
  UdpSocket a = open_loopback();
  const int fd = a.native_handle();
  const Endpoint ep = a.local_endpoint();
  UdpSocket b(std::move(a));
  CHECK_FALSE(a.is_open());  // NOLINT(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
  CHECK(b.native_handle() == fd);
  CHECK(b.local_endpoint() == ep);
}

TEST_CASE("Broadcast send with SO_BROADCAST", "[transport][socket][broadcast]")
{
  SocketOptions opts;
  opts.broadcast = true;
  const auto sock = UdpSocket::open(Endpoint::any(0), opts);
  REQUIRE(sock.has_value());
  const auto payload = bytes_of("discover");
  const auto r = sock->send_to(payload, Endpoint::broadcast(56000));
  if (!r.has_value()) {
    // Sandboxed CI runners may not have a broadcast-capable interface.
    WARN("broadcast send failed: " << to_string(r.error()) << " (skipped)");
    CHECK(r.error().code == TransportErrorCode::kNetworkUnreachable);
  } else {
    CHECK(*r == payload.size());
  }
}

TEST_CASE("Poller times out with no events", "[transport][poller]")
{
  const UdpSocket s = open_loopback();
  auto p = Poller::create();
  REQUIRE(p.has_value());
  REQUIRE(p->add(s, 7).has_value());
  const auto t0 = std::chrono::steady_clock::now();
  const auto ev = p->wait(50ms);
  REQUIRE(ev.has_value());
  CHECK(ev->empty());
  CHECK_FALSE(p->woken());
  CHECK(std::chrono::steady_clock::now() - t0 >= 40ms);
}

TEST_CASE("Poller reports the readable socket by tag", "[transport][poller]")
{
  const UdpSocket a = open_loopback();
  const UdpSocket b = open_loopback();
  const UdpSocket c = open_loopback();
  auto p = Poller::create();
  REQUIRE(p.has_value());
  REQUIRE(p->add(b, 20).has_value());
  REQUIRE(p->add(c, 30).has_value());
  CHECK(p->size() == 2);
  const auto dup = p->add(c, 30);
  REQUIRE_FALSE(dup.has_value());
  CHECK(dup.error().code == TransportErrorCode::kInvalidArgument);

  const auto payload = bytes_of("x");
  REQUIRE(a.send_to(payload, c.local_endpoint()).has_value());
  const auto ev = p->wait(2000ms);
  REQUIRE(ev.has_value());
  REQUIRE(ev->size() == 1);
  CHECK((*ev)[0].tag == 30);
  CHECK((*ev)[0].readable);
  CHECK_FALSE((*ev)[0].error);

  p->remove(30);
  CHECK(p->size() == 1);
}

TEST_CASE("Poller wake interrupts wait from another thread", "[transport][poller]")
{
  const UdpSocket s = open_loopback();
  auto p = Poller::create();
  REQUIRE(p.has_value());
  REQUIRE(p->add(s, 1).has_value());

  std::thread waker([&p] {
    std::this_thread::sleep_for(30ms);
    p->wake();
  });
  const auto t0 = std::chrono::steady_clock::now();
  const auto ev = p->wait(5000ms);
  waker.join();
  REQUIRE(ev.has_value());
  CHECK(ev->empty());
  CHECK(p->woken());
  CHECK(std::chrono::steady_clock::now() - t0 < 4000ms);

  // A wake issued before wait() is not lost.
  p->wake();
  const auto ev2 = p->wait(1000ms);
  REQUIRE(ev2.has_value());
  CHECK(p->woken());
}
