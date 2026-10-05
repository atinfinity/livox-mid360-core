// SPDX-License-Identifier: Apache-2.0
// `debug-data`: Device::start_debug_data -> on_debug_data -> file (issue #93). Two formats
// (docs/debug_data.md): `raw`, provisional and private to this CLI, and `sdk2`, the
// .LivoxDebugPointCloudData file Livox-SDK2 writes (#107). tools/livox_mid360_debug_data.py
// reads both.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>

#include "cli.hpp"
#include "livox/mid360/crc.hpp"
#include "livox/mid360/mid360.hpp"

namespace cli
{
namespace
{
using namespace livox::mid360;
using namespace std::chrono_literals;

constexpr std::array<char, 8> kMagic{'L', 'M', 'D', 'B', 'G', 'R', 'A', 'W'};
constexpr std::uint32_t kFormatVersion = 0;  // provisional
constexpr std::size_t kSerialSize = 16;
constexpr std::size_t kFileHeaderSize = 36;
constexpr std::size_t kRecordHeaderSize = 14;
constexpr std::uint64_t kDefaultMaxSize = 4ULL * 1024 * 1024 * 1024;

// Livox-SDK2 LivoxLidarDebugPointCloudFileHeader (sdk_core/comm/define.h): file_ver u8,
// dev_type u8, data_type u8, sn u8[16], rsvd u8[107], crc16 u16 (CRC-16/CCITT-FALSE of the
// 126 bytes before it). The datagrams follow as received, without framing.
constexpr std::size_t kSdk2HeaderSize = 128;
constexpr std::size_t kSdk2CrcOffset = 126;
constexpr std::uint8_t kSdk2FileVersion = 1;
constexpr std::uint8_t kSdk2DataType = 1;

enum class Format : std::uint8_t
{
  kRaw,   ///< provisional format of this CLI: one record header per datagram
  kSdk2,  ///< .LivoxDebugPointCloudData as Livox-SDK2 writes it
};

constexpr std::size_t header_size(Format f) noexcept
{
  return f == Format::kSdk2 ? kSdk2HeaderSize : kFileHeaderSize;
}

constexpr std::size_t record_overhead(Format f) noexcept
{
  return f == Format::kSdk2 ? 0 : kRecordHeaderSize;
}

struct Args
{
  std::filesystem::path out;
  std::optional<Ipv4> lidar_ip;
  Ipv4 host_ip{0, 0, 0, 0};
  std::optional<std::string> sn;
  int duration = 0;  // seconds, 0 = until SIGINT
  std::uint16_t port = kDefaultHostDebugDataPort;
  std::uint64_t max_size = kDefaultMaxSize;
  Format format = Format::kRaw;
  bool start_sampling = false;
};

std::optional<Args> parse_args(int argc, char ** argv)
{
  Args a;
  for (int i = 0; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--start-sampling") {
      a.start_sampling = true;
      continue;
    }
    if (i + 1 >= argc) {
      return std::nullopt;
    }
    const std::string val = argv[++i];
    if (key == "--out") {
      a.out = val;
    } else if (key == "--lidar-ip" || key == "--host-ip") {
      const auto ip = parse_ip(val);
      if (!ip) {
        return std::nullopt;
      }
      if (key == "--lidar-ip") {
        a.lidar_ip = *ip;
      } else {
        a.host_ip = *ip;
      }
    } else if (key == "--sn") {
      a.sn = val;
    } else if (key == "--duration") {
      a.duration = std::stoi(val);
    } else if (key == "--port") {
      const int port = std::stoi(val);
      if (port < 0 || port > 65535) {
        return std::nullopt;
      }
      a.port = static_cast<std::uint16_t>(port);
    } else if (key == "--format") {
      if (val == "raw") {
        a.format = Format::kRaw;
      } else if (val == "sdk2") {
        a.format = Format::kSdk2;
      } else {
        return std::nullopt;
      }
    } else if (key == "--max-size") {
      if (val.empty() || val.find_first_not_of("0123456789") != std::string::npos) {
        return std::nullopt;
      }
      a.max_size = std::stoull(val);
    } else {
      return std::nullopt;
    }
  }
  if (a.out.empty() || a.duration < 0 || a.max_size < header_size(a.format)) {
    return std::nullopt;
  }
  return a;
}

template <std::size_t N>
void put_le(std::array<char, N> & buf, std::size_t at, std::uint64_t value, std::size_t bytes)
{
  for (std::size_t i = 0; i < bytes; ++i) {
    buf.at(at + i) = static_cast<char>((value >> (8U * i)) & 0xFFU);
  }
}

/// The output file. Not thread-safe; the caller locks.
class Sink
{
public:
  bool open(
    const std::filesystem::path & path, Format format, const DiscoveredDevice & lidar,
    std::uint64_t max_size)
  {
    format_ = format;
    max_size_ = max_size;
    out_.open(path, std::ios::binary | std::ios::trunc);
    if (!out_) {
      return false;
    }
    const std::string & serial = lidar.serial_number;
    if (format == Format::kSdk2) {
      std::array<char, kSdk2HeaderSize> h{};
      h[0] = static_cast<char>(kSdk2FileVersion);
      h[1] = static_cast<char>(lidar.dev_type);
      h[2] = static_cast<char>(kSdk2DataType);
      std::memcpy(&h.at(3), serial.data(), std::min(serial.size(), kSerialSize));
      const auto crc = crc::crc16_ccitt_false(std::as_bytes(std::span(h.data(), kSdk2CrcOffset)));
      put_le(h, kSdk2CrcOffset, crc, 2);
      out_.write(h.data(), static_cast<std::streamsize>(h.size()));
      bytes_ = h.size();
      return static_cast<bool>(out_);
    }
    std::array<char, kFileHeaderSize> h{};
    std::memcpy(h.data(), kMagic.data(), kMagic.size());
    put_le(h, 8, kFormatVersion, 4);
    std::memcpy(&h.at(12), serial.data(), std::min(serial.size(), kSerialSize));
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    put_le(
      h, 28,
      static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()),
      8);
    out_.write(h.data(), static_cast<std::streamsize>(h.size()));
    bytes_ = h.size();
    return static_cast<bool>(out_);
  }

  /// False once the file is full or a write failed; nothing is written then.
  bool write(const DebugDataPacket & p)
  {
    if (full_ || failed_) {
      return false;
    }
    const std::size_t overhead = record_overhead(format_);
    if (bytes_ + overhead + p.data.size() > max_size_) {
      full_ = true;
      return false;
    }
    if (format_ == Format::kRaw) {
      std::array<char, kRecordHeaderSize> h{};
      put_le(h, 0, p.host_receive_time_ns, 8);
      put_le(h, 8, p.from.port, 2);
      put_le(h, 10, p.data.size(), 4);
      out_.write(h.data(), static_cast<std::streamsize>(h.size()));
    }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): bytes to the char stream
    out_.write(
      reinterpret_cast<const char *>(p.data.data()), static_cast<std::streamsize>(p.data.size()));
    if (!out_) {
      failed_ = true;
      return false;
    }
    bytes_ += overhead + p.data.size();
    ++packets_;
    return true;
  }

  bool close()
  {
    out_.close();
    return !failed_ && !out_.fail();
  }

  [[nodiscard]] std::uint64_t packets() const { return packets_; }
  [[nodiscard]] std::uint64_t bytes() const { return bytes_; }
  [[nodiscard]] bool full() const { return full_; }
  [[nodiscard]] bool failed() const { return failed_; }

private:
  std::ofstream out_;
  Format format_ = Format::kRaw;
  std::uint64_t max_size_ = kDefaultMaxSize;
  std::uint64_t bytes_ = 0;
  std::uint64_t packets_ = 0;
  bool full_ = false;
  bool failed_ = false;
};
}  // namespace

int run_debug_data(int argc, char ** argv)
{
  const auto args = parse_args(argc, argv);
  if (!args) {
    std::cerr << "usage: livox-mid360-cli debug-data --out FILE [--lidar-ip A.B.C.D] "
                 "[--host-ip A.B.C.D] [--sn SN] [--duration SECONDS] [--port N] "
                 "[--start-sampling] [--max-size BYTES] [--format raw|sdk2]\n";
    return 1;
  }
  install_sigint();

  ContextOptions ctx_opts;
  ctx_opts.bind_address = args->host_ip;
  ctx_opts.debug_data_port = args->port;
  auto ctx = Context::create(ctx_opts);
  if (!ctx) {
    std::cerr << "context: " << to_string(ctx.error()) << "\n";
    return 2;
  }
  DiscoveryOptions disc;
  disc.bind_address = args->host_ip;
  if (args->lidar_ip) {
    disc.targets.push_back(Endpoint{*args->lidar_ip, kDiscoveryPort});
  }
  auto found = discover(disc);
  if (!found || found->empty()) {
    std::cerr << "discovery: " << (found ? "no LiDAR answered" : to_string(found.error())) << "\n";
    return 2;
  }
  const DiscoveredDevice * target = nullptr;
  for (const auto & d : *found) {
    if (!args->sn || d.serial_number == *args->sn) {
      target = &d;
      break;
    }
  }
  if (target == nullptr) {
    std::cerr << "discovery: no LiDAR with SN " << *args->sn << "\n";
    return 2;
  }
  std::cerr << "found " << target->serial_number << " at " << to_string(target->from) << "\n";

  DeviceOptions opts;
  opts.session.bind_address = args->host_ip;
  auto dev = Device::open(**ctx, *target, opts);
  if (!dev) {
    std::cerr << "open: " << to_string(dev.error()) << "\n";
    return 2;
  }

  Sink sink;
  if (!sink.open(args->out, args->format, *target, args->max_size)) {
    std::cerr << "open " << args->out.string() << " failed\n";
    return 2;
  }
  std::mutex mu;  // sink vs. the progress reader
  auto r = (*dev)->on_debug_data([&](const DebugDataPacket & p) {
    const std::lock_guard lock(mu);
    if (!sink.write(p)) {
      g_stop = true;
    }
  });
  if (!r) {
    std::cerr << "on_debug_data: " << to_string(r.error()) << "\n";
    return 2;
  }
  if (args->start_sampling) {
    if (auto s = (*dev)->start_sampling(); !s) {
      std::cerr << "start_sampling: " << to_string(s.error()) << "\n";
      return 2;
    }
  }
  int status = 0;
  if (auto s = (*dev)->start_debug_data(); !s) {
    std::cerr << "start_debug_data: " << to_string(s.error()) << "\n";
    status = 2;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(args->duration);
  while (status == 0 && !g_stop &&
         (args->duration == 0 || std::chrono::steady_clock::now() < deadline)) {
    std::this_thread::sleep_for(200ms);
    const std::lock_guard lock(mu);
    std::cerr << std::format("packets={} bytes={}\n", sink.packets(), sink.bytes());
  }
  // Always sent, also after a failed start: the LiDAR must not keep streaming.
  if (auto s = (*dev)->stop_debug_data(); !s) {
    std::cerr << "stop_debug_data: " << to_string(s.error()) << "\n";
  }
  if (args->start_sampling) {
    if (auto s = (*dev)->stop_sampling(); !s) {
      std::cerr << "stop_sampling: " << to_string(s.error()) << "\n";
    }
  }
  dev->reset();  // no more callbacks after this
  const std::lock_guard lock(mu);
  const bool closed = sink.close();
  if (status != 0) {
    return status;
  }
  if (!closed) {
    std::cerr << "write " << args->out.string() << " failed\n";
    return 2;
  }
  if (sink.full()) {
    std::cerr << "stopped: --max-size reached\n";
  }
  std::cerr << std::format(
    "wrote {}: packets={} bytes={}\n", args->out.string(), sink.packets(), sink.bytes());
  return sink.packets() == 0 ? 3 : 0;
}
}  // namespace cli
