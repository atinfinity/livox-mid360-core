# UDP transport layer

`include/livox/mid360/transport.hpp`, `src/transport.cpp`. Design decisions are recorded in
[issue #2](https://github.com/atinfinity/livox-mid360-core/issues/2); this page describes the
result and how to use it.

## Position in the stack

```
④ public API           (phase 2, #9)
③ device / session     discovery, parameter config, state machine, retries   (#4, #5, #7, #8)
② transport            UdpSocket, Poller                                      ← this page
① protocol             CRC, frames, packets, key-value (pure functions)
```

The transport layer knows nothing about the Mid-360 protocol. It moves datagrams and reports
readiness; parsing happens in layer ① and threading policy in layer ③.

## Principles

- **No dependencies beyond POSIX sockets.** Linux (Ubuntu 24.04) is the target; macOS keeps
  building for development through `#ifdef __linux__` fallbacks.
- **No exceptions.** Every fallible call returns `std::expected<T, TransportError>`.
- **Caller-owned buffers.** `recv_batch()` fills spans that the caller provides, exactly like
  the protocol parsers return non-owning views. The library never allocates per datagram.
- **No threads.** `UdpSocket` and `Poller` are plain objects; a receive thread that owns a
  `Poller` is created by the session layer.
- **Non-blocking sockets only.** Waiting is done exclusively through `Poller::wait()`.

## Types

### `Endpoint`

IPv4 address as `std::array<std::uint8_t, 4>` plus a port, the same representation used by
`HostIpConfig` in `keys.hpp`. Helpers: `Endpoint::any(port)`, `Endpoint::loopback(port)`,
`Endpoint::broadcast(port)`, `parse_endpoint("a.b.c.d[:port]")`, `to_string(ep)`,
`ip_to_string(ip)`. IPv6 is not supported; the Mid-360 is IPv4 only.

### `TransportError`

```cpp
struct TransportError
{
  TransportErrorCode code;
  int errno_value;
};
```

`code` is a normalised category so that callers and tests never depend on errno values;
`errno_value` keeps the original for diagnostics. `to_string()` renders both.

| Code | Typical errno | Meaning |
|---|---|---|
| `kSocketCreate` | `EMFILE`, `ENOBUFS` | `socket()` / `eventfd()` / `pipe()` failed |
| `kBind` | | `bind()` failed for a reason other than address in use |
| `kSetOption` | | `setsockopt()` / `fcntl()` failed |
| `kAddressInUse` | `EADDRINUSE` | port already bound |
| `kNetworkUnreachable` | `ENETUNREACH`, `EHOSTUNREACH`, `EADDRNOTAVAIL`, `EACCES` | no route, or broadcast not permitted |
| `kSendFailed` | other | `sendto()` failed |
| `kMessageTooLong` | `EMSGSIZE` | datagram exceeds the interface MTU |
| `kWouldBlock` | `EAGAIN` | nothing pending on a non-blocking receive |
| `kTimeout` | | reserved for upper layers |
| `kInterrupted` | `EINTR` | signal during a syscall |
| `kClosed` | `EBADF` | socket or poller already closed |
| `kInvalidArgument` | `EINVAL`, `EEXIST`, `ENOTSUP` | bad argument, duplicate tag, reserved option |
| `kOther` | | anything else; the errno is preserved |

### `Datagram`

```cpp
struct Datagram
{
  std::span<std::byte> data;
  Endpoint from;
  std::uint64_t recv_time_ns;
};
```

On entry to `recv_batch()`, `data` must point at a caller-owned buffer of at least
`kMaxDatagramSize` (1500) bytes. On return it is trimmed to the received length. A datagram
larger than the buffer is **truncated**, as usual for UDP; size buffers with the MTU.

`recv_time_ns` is the host receive time in nanoseconds since the Unix epoch on
`CLOCK_REALTIME`, the same unit and epoch as the LiDAR's packet `timestamp`, so the two can be
subtracted directly. On Linux it is the kernel timestamp taken at packet arrival
(`SO_TIMESTAMPNS`); elsewhere it is sampled in user space right after the receive call.

### `SocketOptions`

| Field | Default | Effect |
|---|---|---|
| `reuse_address` | `true` | `SO_REUSEADDR`, only when binding to an explicit port. A bind to port 0 never sets it: on Linux the ephemeral-port search would otherwise treat a port held by another `SO_REUSEADDR` socket of the same user as free, and the newer socket would silently take over its unicast traffic |
| `broadcast` | `false` | `SO_BROADCAST`; required to send to 255.255.255.255 (discovery) |
| `recv_buffer_bytes` | `0` | `SO_RCVBUF` request; `0` leaves the OS default. The kernel may clamp the value (and Linux doubles it); read back with `recv_buffer_bytes()` |
| `multicast_group` | empty | **reserved**, rejected with `kInvalidArgument` |
| `bind_to_device` | empty | **reserved**, rejected with `kInvalidArgument` |

The transport layer sets no policy. Choosing a receive buffer size for the point-cloud socket
is the receive pipeline's job (#6).

### `UdpSocket`

```cpp
auto sock =
  UdpSocket::open(Endpoint::any(kDefaultHostPointCloudPort), {.recv_buffer_bytes = 4 << 20});
if (!sock) {
  // sock.error()
}
sock->send_to(bytes, Endpoint{{192, 168, 1, 100}, kCommandPort});
sock->local_endpoint();  // bound address; port resolved when 0 was requested
sock->native_handle();   // raw fd for Poller / tests, -1 when closed
```

Move-only; the descriptor is closed in the destructor or by `close()`.

Receiving:

```cpp
std::array<std::array<std::byte, kMaxDatagramSize>, 64> storage;
std::array<Datagram, 64> batch;
for (std::size_t i = 0; i < batch.size(); ++i) {
  batch[i].data = storage[i];
}

auto n = sock->recv_batch(batch);  // never blocks
if (n) {
  for (std::size_t i = 0; i < *n; ++i) {
    handle(batch[i]);
  }
}
```

`recv_batch()` returns the number of datagrams received, `0` when nothing is pending, or an
error. On Linux it uses `recvmmsg()` in chunks of 64 and loops until the queue is drained or
the output span is full. Elsewhere it loops over `recvfrom()`. `recv_one(buffer)` is a
single-datagram convenience that returns `kWouldBlock` when nothing is pending.

`send_to()` returns the number of bytes sent. It is never partial for UDP; a datagram larger
than the MTU fails with `kMessageTooLong`.

### `Poller`

```cpp
auto poller = Poller::create();
poller->add(cmd_sock, kCmdTag);
poller->add(pcl_sock, kPclTag);

while (running) {
  auto events = poller->wait(std::chrono::milliseconds{100});  // negative = block forever
  if (!events) {
    break;
  }
  if (poller->woken()) {
    // someone called wake()
  }
  for (const ReadyEvent & ev : *events) {
    if (ev.error) {
      // POLLERR / POLLHUP / POLLNVAL on ev.tag
    }
    if (ev.readable) {
      drain(sockets[ev.tag]);
    }
  }
}
```

- `add()` registers a socket under a caller-chosen `tag`; tags must be unique. `remove(tag)`
  unregisters.
- `wait()` blocks until at least one socket is readable, `wake()` is called, or the timeout
  elapses. It returns a span into poller-owned storage that is valid until the next `wait()`.
  Timeout and wake-up both return an empty span; distinguish them with `woken()`.
- `wake()` is thread-safe and may be called before `wait()`; the wake-up is not lost. It is
  implemented with `eventfd` on Linux and a non-blocking `pipe` elsewhere.
- Readiness is level-triggered: if a socket is not drained, the next `wait()` returns
  immediately. Drain with `recv_batch()` until it returns `0`.

One `Poller` per receive thread. The implementation uses `poll(2)`; the API deliberately hides
the mechanism so that `epoll` can replace it without changes to callers.

## Typical wiring for one Mid-360

| Socket | Bind | Options | Purpose |
|---|---|---|---|
| discovery | `any(0)` | `broadcast = true` | send 0x0000 to 255.255.255.255:56000, receive the ACK |
| command | `any(56101)` | | 0x01xx / 0x02xx requests and ACKs to `lidar_ip:56100` |
| push | `any(56201)` | | 0x0102 pushes |
| point cloud | `any(56301)` | `recv_buffer_bytes` large | data type 1–3 packets, ~2000 packets/s |
| IMU | `any(56401)` | | data type 0 packets, 200 packets/s |

The host ports must match what is configured through keys `0x0006`–`0x0008` (`HostIpConfig`).

## Platform notes

| Feature | Linux | macOS (development only) |
|---|---|---|
| batched receive | `recvmmsg()` | `recvfrom()` loop |
| receive timestamp | kernel, `SO_TIMESTAMPNS` | user space, `clock_gettime(CLOCK_REALTIME)` |
| poller wake-up | `eventfd` | `pipe` |
| broadcast from a loopback-bound socket | fails (`EACCES`/`EINVAL`) | fails (`EADDRNOTAVAIL`), both mapped to `kNetworkUnreachable` |

`nfds_t` is 64-bit on Linux and 32-bit on macOS; the cast in `Poller::wait()` goes through
`std::uint32_t` so that both `-Wuseless-cast` and `-Wshorten-64-to-32` stay quiet.

## Tests

`tests/test_transport.cpp`, all on loopback: send/receive, `recv_batch` draining several
datagrams, truncation, `kAddressInUse`, reserved options, `SO_RCVBUF` query, closed socket,
move semantics, poller timeout, per-tag reporting, cross-thread wake-up, and broadcast send.
The broadcast test only asserts the error category when the runner has no broadcast route.

## Not in scope (yet)

- Multicast membership (`IP_ADD_MEMBERSHIP`) and `SO_BINDTODEVICE`: names reserved.
- `epoll`, `SO_REUSEPORT` multi-thread receive, `io_uring`: revisit if the single-thread
  `recvmmsg` path proves insufficient in the long-run test (#13).
- Send batching (`sendmmsg`): command traffic is too light to need it.
