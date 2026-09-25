// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/transport.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <cstring>
#include <ctime>
#include <limits>
#include <utility>

#if defined(__linux__)
#include <sys/eventfd.h>
#define LIVOX_MID360_HAVE_RECVMMSG 1
#define LIVOX_MID360_HAVE_EVENTFD 1
#else
#define LIVOX_MID360_HAVE_RECVMMSG 0
#define LIVOX_MID360_HAVE_EVENTFD 0
#endif

namespace livox::mid360 {

namespace {

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

TransportError from_errno(TransportErrorCode fallback, int err) {
  switch (err) {
    case EADDRINUSE:
      return {TransportErrorCode::kAddressInUse, err};
    case ENETUNREACH:
    case EHOSTUNREACH:
    case EADDRNOTAVAIL:  // e.g. broadcast from a loopback-bound socket
    case EACCES:         // broadcast without SO_BROADCAST, or forbidden destination
      return {TransportErrorCode::kNetworkUnreachable, err};
    case EMSGSIZE:
      return {TransportErrorCode::kMessageTooLong, err};
    case EAGAIN:
#if EWOULDBLOCK != EAGAIN
    case EWOULDBLOCK:
#endif
      return {TransportErrorCode::kWouldBlock, err};
    case EINTR:
      return {TransportErrorCode::kInterrupted, err};
    case EBADF:
    case ENOTSOCK:
      return {TransportErrorCode::kClosed, err};
    case EINVAL:
    case EAFNOSUPPORT:
      return {TransportErrorCode::kInvalidArgument, err};
    default:
      return {fallback, err};
  }
}

sockaddr_in to_sockaddr(const Endpoint& ep) {
  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(ep.port);
  std::memcpy(&sa.sin_addr.s_addr, ep.ip.data(), ep.ip.size());
  return sa;
}

Endpoint from_sockaddr(const sockaddr_in& sa) {
  Endpoint ep;
  std::memcpy(ep.ip.data(), &sa.sin_addr.s_addr, ep.ip.size());
  ep.port = ntohs(sa.sin_port);
  return ep;
}

std::uint64_t now_realtime_ns() {
  timespec ts{};
  ::clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ull +
         static_cast<std::uint64_t>(ts.tv_nsec);
}

bool set_nonblocking(int fd) {
  const int flags = ::fcntl(fd, F_GETFL, 0);
  return flags >= 0 && ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

template <typename T>
bool set_option(int fd, int level, int name, const T& value) {
  return ::setsockopt(fd, level, name, &value, static_cast<socklen_t>(sizeof(T))) == 0;
}

#if LIVOX_MID360_HAVE_RECVMMSG
/// Extract a kernel receive timestamp (SO_TIMESTAMPNS) from control data, 0 if absent.
std::uint64_t timestamp_from_cmsg(msghdr& msg) {
  for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c)) {
    if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMPNS) {
      timespec ts{};
      std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
      return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ull +
             static_cast<std::uint64_t>(ts.tv_nsec);
    }
  }
  return 0;
}
#endif

}  // namespace

// ---------------------------------------------------------------------------
// Endpoint
// ---------------------------------------------------------------------------

std::optional<Endpoint> parse_endpoint(std::string_view text) {
  Endpoint ep;
  std::string_view addr = text;
  std::string_view port_text;
  if (const auto colon = text.find(':'); colon != std::string_view::npos) {
    addr = text.substr(0, colon);
    port_text = text.substr(colon + 1);
  }

  std::size_t octet = 0;
  std::size_t pos = 0;
  while (true) {
    const auto dot = addr.find('.', pos);
    const auto part =
        addr.substr(pos, dot == std::string_view::npos ? std::string_view::npos : dot - pos);
    if (octet == 4 || part.empty() || part.size() > 3) return std::nullopt;
    unsigned value = 0;
    const auto [ptr, ec] = std::from_chars(part.data(), part.data() + part.size(), value);
    if (ec != std::errc{} || ptr != part.data() + part.size() || value > 255) return std::nullopt;
    ep.ip[octet++] = static_cast<std::uint8_t>(value);
    if (dot == std::string_view::npos) break;
    pos = dot + 1;
  }
  if (octet != 4) return std::nullopt;

  if (!port_text.empty()) {
    unsigned port = 0;
    const auto [ptr, ec] =
        std::from_chars(port_text.data(), port_text.data() + port_text.size(), port);
    if (ec != std::errc{} || ptr != port_text.data() + port_text.size() || port > 65535) {
      return std::nullopt;
    }
    ep.port = static_cast<std::uint16_t>(port);
  } else if (text.find(':') != std::string_view::npos) {
    return std::nullopt;  // trailing ':' with no port
  }
  return ep;
}

std::string ip_to_string(const std::array<std::uint8_t, 4>& ip) {
  std::string out;
  for (std::size_t i = 0; i < ip.size(); ++i) {
    if (i != 0) out += '.';
    out += std::to_string(static_cast<unsigned>(ip[i]));
  }
  return out;
}

std::string to_string(const Endpoint& ep) {
  return ip_to_string(ep.ip) + ':' + std::to_string(ep.port);
}

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

std::string_view to_string(TransportErrorCode code) {
  switch (code) {
    case TransportErrorCode::kSocketCreate:
      return "socket_create";
    case TransportErrorCode::kBind:
      return "bind";
    case TransportErrorCode::kSetOption:
      return "set_option";
    case TransportErrorCode::kAddressInUse:
      return "address_in_use";
    case TransportErrorCode::kNetworkUnreachable:
      return "network_unreachable";
    case TransportErrorCode::kSendFailed:
      return "send_failed";
    case TransportErrorCode::kMessageTooLong:
      return "message_too_long";
    case TransportErrorCode::kWouldBlock:
      return "would_block";
    case TransportErrorCode::kTimeout:
      return "timeout";
    case TransportErrorCode::kInterrupted:
      return "interrupted";
    case TransportErrorCode::kClosed:
      return "closed";
    case TransportErrorCode::kInvalidArgument:
      return "invalid_argument";
    case TransportErrorCode::kOther:
      return "other";
  }
  return "unknown";
}

std::string to_string(const TransportError& err) {
  std::string out{to_string(err.code)};
  if (err.errno_value != 0) {
    out += " (";
    out += std::strerror(err.errno_value);
    out += ')';
  }
  return out;
}

// ---------------------------------------------------------------------------
// UdpSocket
// ---------------------------------------------------------------------------

std::expected<UdpSocket, TransportError> UdpSocket::open(Endpoint bind_to,
                                                         const SocketOptions& options) {
  if (options.multicast_group.has_value() || !options.bind_to_device.empty()) {
    return std::unexpected(TransportError{TransportErrorCode::kInvalidArgument, ENOTSUP});
  }

  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return std::unexpected(from_errno(TransportErrorCode::kSocketCreate, errno));

  UdpSocket sock;
  sock.fd_ = fd;  // owns from here on

  if (!set_nonblocking(fd)) {
    return std::unexpected(from_errno(TransportErrorCode::kSetOption, errno));
  }
  if (options.reuse_address && !set_option(fd, SOL_SOCKET, SO_REUSEADDR, 1)) {
    return std::unexpected(from_errno(TransportErrorCode::kSetOption, errno));
  }
  if (options.broadcast && !set_option(fd, SOL_SOCKET, SO_BROADCAST, 1)) {
    return std::unexpected(from_errno(TransportErrorCode::kSetOption, errno));
  }
  if (options.recv_buffer_bytes != 0 &&
      !set_option(fd, SOL_SOCKET, SO_RCVBUF, static_cast<int>(options.recv_buffer_bytes))) {
    return std::unexpected(from_errno(TransportErrorCode::kSetOption, errno));
  }
#if defined(__linux__)
  // Best effort: kernel receive timestamps. Failure falls back to user-space time.
  (void)set_option(fd, SOL_SOCKET, SO_TIMESTAMPNS, 1);
#endif

  const sockaddr_in sa = to_sockaddr(bind_to);
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&sa), sizeof(sa)) != 0) {
    return std::unexpected(from_errno(TransportErrorCode::kBind, errno));
  }

  sockaddr_in bound{};
  socklen_t len = sizeof(bound);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &len) != 0) {
    return std::unexpected(from_errno(TransportErrorCode::kBind, errno));
  }
  sock.local_ = from_sockaddr(bound);
  return sock;
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept
    : fd_(std::exchange(other.fd_, -1)), local_(std::exchange(other.local_, Endpoint{})) {}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept {
  if (this != &other) {
    close();
    fd_ = std::exchange(other.fd_, -1);
    local_ = std::exchange(other.local_, Endpoint{});
  }
  return *this;
}

UdpSocket::~UdpSocket() {
  close();
}

void UdpSocket::close() noexcept {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::expected<std::size_t, TransportError> UdpSocket::recv_buffer_bytes() const {
  if (fd_ < 0) return std::unexpected(TransportError{TransportErrorCode::kClosed, EBADF});
  int value = 0;
  socklen_t len = sizeof(value);
  if (::getsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &value, &len) != 0) {
    return std::unexpected(from_errno(TransportErrorCode::kSetOption, errno));
  }
  return static_cast<std::size_t>(value);
}

std::expected<std::size_t, TransportError> UdpSocket::send_to(std::span<const std::byte> data,
                                                              const Endpoint& to) const {
  if (fd_ < 0) return std::unexpected(TransportError{TransportErrorCode::kClosed, EBADF});
  const sockaddr_in sa = to_sockaddr(to);
  const ssize_t n = ::sendto(fd_, data.data(), data.size(), 0,
                             reinterpret_cast<const sockaddr*>(&sa), sizeof(sa));
  if (n < 0) return std::unexpected(from_errno(TransportErrorCode::kSendFailed, errno));
  return static_cast<std::size_t>(n);
}

std::expected<std::size_t, TransportError> UdpSocket::recv_batch(std::span<Datagram> out) const {
  if (fd_ < 0) return std::unexpected(TransportError{TransportErrorCode::kClosed, EBADF});
  if (out.empty()) return 0;

#if LIVOX_MID360_HAVE_RECVMMSG
  constexpr std::size_t kMaxBatch = 64;
  std::size_t received_total = 0;
  while (received_total < out.size()) {
    const std::size_t n = std::min(out.size() - received_total, kMaxBatch);
    std::array<mmsghdr, kMaxBatch> msgs{};
    std::array<iovec, kMaxBatch> iovs{};
    std::array<sockaddr_in, kMaxBatch> addrs{};
    alignas(cmsghdr) std::array<std::array<char, CMSG_SPACE(sizeof(timespec))>, kMaxBatch> ctrl{};
    for (std::size_t i = 0; i < n; ++i) {
      Datagram& d = out[received_total + i];
      iovs[i] = {d.data.data(), d.data.size()};
      msghdr& h = msgs[i].msg_hdr;
      h.msg_name = &addrs[i];
      h.msg_namelen = sizeof(sockaddr_in);
      h.msg_iov = &iovs[i];
      h.msg_iovlen = 1;
      h.msg_control = ctrl[i].data();
      h.msg_controllen = ctrl[i].size();
    }
    const int got = ::recvmmsg(fd_, msgs.data(), static_cast<unsigned>(n), MSG_DONTWAIT, nullptr);
    if (got < 0) {
      if (received_total > 0) break;
      return std::unexpected(from_errno(TransportErrorCode::kOther, errno));
    }
    const std::uint64_t fallback_ts = now_realtime_ns();
    for (int i = 0; i < got; ++i) {
      Datagram& d = out[received_total + static_cast<std::size_t>(i)];
      const auto idx = static_cast<std::size_t>(i);
      d.data = d.data.first(msgs[idx].msg_len);
      d.from = from_sockaddr(addrs[idx]);
      const std::uint64_t ts = timestamp_from_cmsg(msgs[idx].msg_hdr);
      d.recv_time_ns = ts != 0 ? ts : fallback_ts;
    }
    received_total += static_cast<std::size_t>(got);
    if (static_cast<std::size_t>(got) < n) break;  // queue drained
  }
  return received_total;
#else
  std::size_t received = 0;
  for (Datagram& d : out) {
    sockaddr_in from{};
    socklen_t len = sizeof(from);
    const ssize_t n =
        ::recvfrom(fd_, d.data.data(), d.data.size(), 0, reinterpret_cast<sockaddr*>(&from), &len);
    if (n < 0) {
      if (received > 0) break;
      return std::unexpected(from_errno(TransportErrorCode::kOther, errno));
    }
    d.data = d.data.first(static_cast<std::size_t>(n));
    d.from = from_sockaddr(from);
    d.recv_time_ns = now_realtime_ns();
    ++received;
  }
  return received;
#endif
}

std::expected<Datagram, TransportError> UdpSocket::recv_one(std::span<std::byte> buffer) const {
  Datagram d{buffer, {}, 0};
  auto r = recv_batch(std::span<Datagram>(&d, 1));
  if (!r) return std::unexpected(r.error());
  if (*r == 0) return std::unexpected(TransportError{TransportErrorCode::kWouldBlock, EAGAIN});
  return d;
}

// ---------------------------------------------------------------------------
// Poller
// ---------------------------------------------------------------------------

std::expected<Poller, TransportError> Poller::create() {
  Poller p;
#if LIVOX_MID360_HAVE_EVENTFD
  const int efd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (efd < 0) return std::unexpected(from_errno(TransportErrorCode::kSocketCreate, errno));
  p.wake_read_fd_ = efd;
  p.wake_write_fd_ = efd;
#else
  int fds[2] = {-1, -1};
  if (::pipe(fds) != 0)
    return std::unexpected(from_errno(TransportErrorCode::kSocketCreate, errno));
  p.wake_read_fd_ = fds[0];
  p.wake_write_fd_ = fds[1];
  if (!set_nonblocking(fds[0]) || !set_nonblocking(fds[1])) {
    return std::unexpected(from_errno(TransportErrorCode::kSetOption, errno));
  }
#endif
  return p;
}

Poller::Poller(Poller&& other) noexcept
    : entries_(std::move(other.entries_)),
      ready_(std::move(other.ready_)),
      wake_read_fd_(std::exchange(other.wake_read_fd_, -1)),
      wake_write_fd_(std::exchange(other.wake_write_fd_, -1)),
      woken_(other.woken_) {}

Poller& Poller::operator=(Poller&& other) noexcept {
  if (this != &other) {
    this->~Poller();
    entries_ = std::move(other.entries_);
    ready_ = std::move(other.ready_);
    wake_read_fd_ = std::exchange(other.wake_read_fd_, -1);
    wake_write_fd_ = std::exchange(other.wake_write_fd_, -1);
    woken_ = other.woken_;
  }
  return *this;
}

Poller::~Poller() {
  if (wake_write_fd_ >= 0 && wake_write_fd_ != wake_read_fd_) ::close(wake_write_fd_);
  if (wake_read_fd_ >= 0) ::close(wake_read_fd_);
  wake_read_fd_ = wake_write_fd_ = -1;
}

std::expected<void, TransportError> Poller::add(const UdpSocket& socket, std::uint64_t tag) {
  if (!socket.is_open()) {
    return std::unexpected(TransportError{TransportErrorCode::kClosed, EBADF});
  }
  for (const Entry& e : entries_) {
    if (e.tag == tag) {
      return std::unexpected(TransportError{TransportErrorCode::kInvalidArgument, EEXIST});
    }
  }
  entries_.push_back({socket.native_handle(), tag});
  return {};
}

void Poller::remove(std::uint64_t tag) {
  std::erase_if(entries_, [tag](const Entry& e) { return e.tag == tag; });
}

std::expected<std::span<const ReadyEvent>, TransportError> Poller::wait(
    std::chrono::milliseconds timeout) {
  ready_.clear();
  woken_ = false;
  if (wake_read_fd_ < 0) {
    return std::unexpected(TransportError{TransportErrorCode::kClosed, EBADF});
  }

  std::vector<pollfd> fds;
  fds.reserve(entries_.size() + 1);
  fds.push_back({wake_read_fd_, POLLIN, 0});
  for (const Entry& e : entries_) fds.push_back({e.fd, POLLIN, 0});

  const int timeout_ms = timeout.count() < 0
                             ? -1
                             : static_cast<int>(std::min<std::chrono::milliseconds::rep>(
                                   timeout.count(), std::numeric_limits<int>::max()));
  // nfds_t is 64-bit on Linux and 32-bit on macOS; go through a fixed-width type so that
  // neither -Wuseless-cast (GCC) nor -Wshorten-64-to-32 (Clang) fires.
  const auto nfds = static_cast<nfds_t>(static_cast<std::uint32_t>(fds.size()));
  const int n = ::poll(fds.data(), nfds, timeout_ms);
  if (n < 0) return std::unexpected(from_errno(TransportErrorCode::kOther, errno));

  if ((fds[0].revents & POLLIN) != 0) {
    woken_ = true;
#if LIVOX_MID360_HAVE_EVENTFD
    std::uint64_t counter = 0;
    (void)!::read(wake_read_fd_, &counter, sizeof(counter));
#else
    char drain[64];
    while (::read(wake_read_fd_, drain, sizeof(drain)) > 0) {
    }
#endif
  }
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const short rev = fds[i + 1].revents;
    if (rev == 0) continue;
    ready_.push_back(
        {entries_[i].tag, (rev & POLLIN) != 0, (rev & (POLLERR | POLLHUP | POLLNVAL)) != 0});
  }
  return std::span<const ReadyEvent>(ready_);
}

void Poller::wake() noexcept {
  if (wake_write_fd_ < 0) return;
#if LIVOX_MID360_HAVE_EVENTFD
  const std::uint64_t one = 1;
  (void)!::write(wake_write_fd_, &one, sizeof(one));
#else
  const char one = 1;
  (void)!::write(wake_write_fd_, &one, 1);
#endif
}

}  // namespace livox::mid360
