// SPDX-License-Identifier: Apache-2.0
/// @file
/// UDP transport layer: a thin, dependency-free wrapper over POSIX sockets.
///
/// Design (see issue #2):
///   * Non-blocking sockets multiplexed with a Poller (poll(2) today, epoll-ready API).
///   * Caller-owned buffers: recv_batch() fills spans the caller provides, mirroring the
///     protocol layer's non-owning views.
///   * Errors are std::expected<T, TransportError>; no exceptions.
///   * Thread creation is the responsibility of the upper layers.
///   * Ubuntu 24.04 is the target; macOS keeps building for development via fallbacks.
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "livox/mid360/export.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

// ---------------------------------------------------------------------------
// Endpoint
// ---------------------------------------------------------------------------

/// IPv4 address + UDP port. Mid-360 is IPv4 only.
struct Endpoint
{
  std::array<std::uint8_t, 4> ip{};  ///< address bytes in network order (a.b.c.d)
  std::uint16_t port = 0;            ///< UDP port, 0 = any / ephemeral

  /// 0.0.0.0:`p` (all interfaces).
  [[nodiscard]] static constexpr Endpoint any(std::uint16_t p) { return {{0, 0, 0, 0}, p}; }
  /// 127.0.0.1:`p`.
  [[nodiscard]] static constexpr Endpoint loopback(std::uint16_t p) { return {{127, 0, 0, 1}, p}; }
  /// 255.255.255.255:`p`.
  [[nodiscard]] static constexpr Endpoint broadcast(std::uint16_t p)
  {
    return {{255, 255, 255, 255}, p};
  }

  /// Equal when address and port match.
  friend constexpr bool operator==(const Endpoint &, const Endpoint &) = default;
};

/// Parse "a.b.c.d" (port = 0) or "a.b.c.d:port". Returns nullopt on malformed input.
[[nodiscard]] std::optional<Endpoint> parse_endpoint(std::string_view text);
/// Format as "a.b.c.d:port".
[[nodiscard]] std::string to_string(const Endpoint & ep);
/// Format only the address part, "a.b.c.d".
[[nodiscard]] std::string ip_to_string(const std::array<std::uint8_t, 4> & ip);

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Category of a TransportError. Known errno values map to a specific code; the rest keep the
/// code of the failing step.
enum class TransportErrorCode : std::uint8_t
{
  kSocketCreate,        ///< socket(), eventfd() or pipe() failed
  kBind,                ///< bind() or getsockname() failed
  kSetOption,           ///< a socket or descriptor option could not be set
  kAddressInUse,        ///< EADDRINUSE
  kNetworkUnreachable,  ///< ENETUNREACH, EHOSTUNREACH, EADDRNOTAVAIL, EACCES
  kSendFailed,          ///< sendto() failed with another errno
  kMessageTooLong,      ///< EMSGSIZE
  kWouldBlock,          ///< nothing pending on a non-blocking socket (EAGAIN)
  kTimeout,             ///< timed out
  kInterrupted,         ///< EINTR
  kClosed,              ///< the socket or Poller is closed (EBADF, ENOTSOCK)
  kInvalidArgument,     ///< EINVAL, EAFNOSUPPORT, or a rejected argument
  kOther,               ///< any other failure
};

/// Short snake_case name, e.g. "would_block".
[[nodiscard]] std::string_view to_string(TransportErrorCode code);

/// Error of a transport call: a category plus the errno it came from.
struct TransportError
{
  TransportErrorCode code = TransportErrorCode::kOther;  ///< category
  int errno_value = 0;                                   ///< Original errno, 0 if not applicable.

  /// Equal when code and errno match.
  friend constexpr bool operator==(const TransportError &, const TransportError &) = default;
};

/// Human readable "code (strerror)" text.
[[nodiscard]] std::string to_string(const TransportError & err);

// ---------------------------------------------------------------------------
// Datagram
// ---------------------------------------------------------------------------

/// One received UDP datagram. `data` points into the caller-provided buffer and is
/// trimmed to the received length.
struct Datagram
{
  std::span<std::byte> data;  ///< received bytes
  Endpoint from;              ///< sender address
  /// Host receive time in nanoseconds since the Unix epoch (CLOCK_REALTIME). On Linux
  /// this is the kernel timestamp (SO_TIMESTAMPNS); elsewhere it is sampled in user
  /// space right after the receive call.
  std::uint64_t recv_time_ns = 0;
};

/// Maximum datagram size worth allocating for: Ethernet MTU. Command frames are at
/// most 1400 bytes and point-cloud packets 36 + 96 * 14 = 1380 bytes.
inline constexpr std::size_t kMaxDatagramSize = 1500;

// ---------------------------------------------------------------------------
// UdpSocket
// ---------------------------------------------------------------------------

/// Options of UdpSocket::open().
struct SocketOptions
{
  /// SO_REUSEADDR, applied only when binding to an explicit port: an ephemeral port
  /// (port 0) is always exclusive, so that the kernel cannot hand out one that is in use.
  bool reuse_address = true;
  bool broadcast = false;  ///< SO_BROADCAST, needed to send to 255.255.255.255
  /// Requested SO_RCVBUF in bytes; 0 leaves the OS default untouched. The kernel may
  /// clamp the value; see UdpSocket::recv_buffer_bytes().
  std::size_t recv_buffer_bytes = 0;
  /// Reserved (#2 design): not implemented, must stay at default. Multicast reception is
  /// not a v1 goal (#112); bind_to_device is reserved in the same way.
  std::optional<std::array<std::uint8_t, 4>> multicast_group;
  std::string bind_to_device;  ///< reserved, see `multicast_group`
};

/// Non-blocking IPv4 UDP socket. Move-only; closes the descriptor on destruction.
class UdpSocket
{
public:
  /// Create a non-blocking socket and bind it to `bind_to` (port 0 = ephemeral). Setting
  /// the reserved `multicast_group` or `bind_to_device` fails with kInvalidArgument.
  [[nodiscard]] static std::expected<UdpSocket, TransportError> open(
    Endpoint bind_to, const SocketOptions & options = {});

  UdpSocket() = default;
  /// Takes over the descriptor; `other` is left closed.
  UdpSocket(UdpSocket && other) noexcept;
  /// Closes the current descriptor and takes over the one of `other`.
  UdpSocket & operator=(UdpSocket && other) noexcept;
  UdpSocket(const UdpSocket &) = delete;
  UdpSocket & operator=(const UdpSocket &) = delete;
  ~UdpSocket();

  /// True while the socket holds a descriptor.
  [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }
  /// Raw descriptor (for Poller and tests). -1 when closed.
  [[nodiscard]] int native_handle() const noexcept { return fd_; }
  /// Address actually bound (port resolved when 0 was requested).
  [[nodiscard]] Endpoint local_endpoint() const noexcept { return local_; }
  /// Effective SO_RCVBUF as reported by the kernel.
  [[nodiscard]] std::expected<std::size_t, TransportError> recv_buffer_bytes() const;

  /// Send one datagram. Returns bytes sent (always data.size() for UDP on success).
  [[nodiscard]] std::expected<std::size_t, TransportError> send_to(
    std::span<const std::byte> data, const Endpoint & to) const;

  /// Receive up to `out.size()` datagrams without blocking. Each out[i].data must point
  /// to a caller-owned buffer on entry; on return it is trimmed to the received size and
  /// `from` / `recv_time_ns` are filled. Returns the number of datagrams received, or
  /// kWouldBlock when nothing is pending.
  [[nodiscard]] std::expected<std::size_t, TransportError> recv_batch(
    std::span<Datagram> out) const;

  /// Convenience: receive a single datagram into `buffer`.
  [[nodiscard]] std::expected<Datagram, TransportError> recv_one(std::span<std::byte> buffer) const;

  /// Close the descriptor. Safe to call on a closed socket.
  void close() noexcept;

private:
  int fd_ = -1;
  Endpoint local_{};
};

// ---------------------------------------------------------------------------
// Poller
// ---------------------------------------------------------------------------

/// One socket reported by Poller::wait().
struct ReadyEvent
{
  std::uint64_t tag = 0;  ///< tag given to Poller::add()
  bool readable = false;  ///< data is pending (POLLIN)
  bool error = false;     ///< POLLERR, POLLHUP or POLLNVAL
};

/// Waits for readability on a set of sockets. One Poller per receive thread. wake()
/// may be called from any thread to interrupt wait().
class Poller
{
public:
  /// Create a Poller with its wake descriptor (eventfd on Linux, a pipe elsewhere). A
  /// default-constructed Poller has none and its wait() fails with kClosed.
  [[nodiscard]] static std::expected<Poller, TransportError> create();

  Poller() = default;
  /// Takes over the registrations and the wake descriptor of `other`.
  Poller(Poller && other) noexcept;
  /// Closes the current wake descriptor and takes over the state of `other`.
  Poller & operator=(Poller && other) noexcept;
  Poller(const Poller &) = delete;
  Poller & operator=(const Poller &) = delete;
  ~Poller();

  /// Register a socket. `tag` is returned in ReadyEvent; tags must be unique.
  [[nodiscard]] std::expected<void, TransportError> add(
    const UdpSocket & socket, std::uint64_t tag);
  /// Unregister by tag. Unknown tags are ignored.
  void remove(std::uint64_t tag);
  /// Number of registered sockets.
  [[nodiscard]] std::size_t size() const noexcept { return entries_.size(); }

  /// Block until at least one socket is readable, wake() is called, or `timeout`
  /// elapses. Returns the ready events (empty on timeout or wake). A negative timeout
  /// blocks indefinitely.
  [[nodiscard]] std::expected<std::span<const ReadyEvent>, TransportError> wait(
    std::chrono::milliseconds timeout);

  /// Interrupt a pending or future wait(). Thread-safe.
  void wake() const noexcept;
  /// True if the last wait() returned because of wake().
  [[nodiscard]] bool woken() const noexcept { return woken_; }

private:
  struct Entry
  {
    int fd;
    std::uint64_t tag;
  };
  void close_wake_fds() noexcept;

  std::vector<Entry> entries_;
  std::vector<ReadyEvent> ready_;
  int wake_read_fd_ = -1;
  int wake_write_fd_ = -1;  ///< == wake_read_fd_ when eventfd is used.
  bool woken_ = false;
};

}  // namespace livox::mid360
LIVOX_MID360_API_END
