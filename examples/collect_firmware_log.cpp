// SPDX-License-Identifier: Apache-2.0
// Firmware log collection sample (issue #44): Context -> discover -> Device::open ->
// on_firmware_log -> start_firmware_log, one file per firmware log file, a progress line per
// second, gap events on stderr, stop_firmware_log on exit.
//
//   collect_firmware_log --lidar-ip 192.168.1.10 --host-ip 192.168.1.5 --out logs [--duration 30]
//
// Files: <out>/<SN>_<UTC start, ISO basic>_<type>_<file_index>.log; a new file starts on every
// packet flagged "file begin". Exit codes: 0 ok, 1 usage, 2 setup failure, 3 no chunk received
// (the empty file is kept so that the run is visible).
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>

#include "livox/mid360/mid360.hpp"

namespace
{
using namespace livox::mid360;
using namespace std::chrono_literals;

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables): set by the SIGINT handler
std::atomic<bool> g_stop{false};
void on_sigint(int /*signal*/) { g_stop = true; }

struct Args
{
  std::optional<Ipv4> lidar_ip;
  Ipv4 host_ip{0, 0, 0, 0};
  ContextOptions ctx;
  std::filesystem::path out = ".";
  int duration = 0;  // seconds, 0 = until SIGINT
  FirmwareLogType type = FirmwareLogType::kRealTime;
  bool start_sampling = false;
};

std::optional<Ipv4> parse_ip(const std::string & s)
{
  Ipv4 ip{};
  std::size_t pos = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    const auto dot = (i < 3) ? s.find('.', pos) : s.size();
    if (dot == std::string::npos || dot == pos) {
      return std::nullopt;
    }
    const std::string part = s.substr(pos, dot - pos);
    if (part.find_first_not_of("0123456789") != std::string::npos || std::stoi(part) > 255) {
      return std::nullopt;
    }
    ip[i] = static_cast<std::uint8_t>(std::stoi(part));
    pos = dot + 1;
  }
  return ip;
}

std::optional<Args> parse_args(int argc, char ** argv)
{
  Args a;
  int i = 1;
  while (i < argc) {
    const std::string key = argv[i];
    if (key == "--start-sampling") {
      a.start_sampling = true;
      ++i;
      continue;
    }
    if (i + 1 >= argc) {
      return std::nullopt;
    }
    const std::string val = argv[i + 1];
    i += 2;
    if (key == "--lidar-ip" || key == "--host-ip") {
      const auto ip = parse_ip(val);
      if (!ip) {
        return std::nullopt;
      }
      if (key == "--lidar-ip") {
        a.lidar_ip = *ip;
      } else {
        a.host_ip = a.ctx.bind_address = *ip;
      }
    } else if (key == "--out") {
      a.out = val;
    } else if (key == "--duration") {
      a.duration = std::stoi(val);
    } else if (key == "--log-port") {
      a.ctx.log_port = static_cast<std::uint16_t>(std::stoi(val));
    } else if (key == "--type") {
      if (val == "realtime") {
        a.type = FirmwareLogType::kRealTime;
      } else if (val == "exception") {
        a.type = FirmwareLogType::kException;
      } else {
        return std::nullopt;
      }
    } else {
      return std::nullopt;
    }
  }
  return a;
}

std::string utc_stamp()
{
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  ::gmtime_r(&now, &tm);
  return std::format(
    "{:04}{:02}{:02}T{:02}{:02}{:02}Z", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
    tm.tm_min, tm.tm_sec);
}

/// Writes chunks to one file per firmware log file. Called on the receive thread; the main
/// thread reads the counters for the progress line.
class LogWriter
{
public:
  LogWriter(std::filesystem::path dir, std::string prefix)
  : dir_(std::move(dir)), prefix_(std::move(prefix))
  {
  }

  void on_chunk(const FirmwareLogChunk & c)
  {
    const std::lock_guard lock(mutex_);
    if (c.header.flags.file_begin() || !file_.is_open() || c.header.file_index != file_index_) {
      open_file(c.header);
    }
    file_.write(
      reinterpret_cast<const char *>(c.data.data()), static_cast<std::streamsize>(c.data.size()));
    ++chunks_;
    bytes_ += c.data.size();
    if (c.header.flags.file_end()) {
      file_.flush();
    }
  }

  /// Opens the first (possibly empty) file so that a run with no chunk still leaves a trace.
  void ensure_open(FirmwareLogType type)
  {
    const std::lock_guard lock(mutex_);
    if (!file_.is_open()) {
      FirmwareLogPushHeader h{};
      h.log_type = type;
      h.file_index = 0;
      open_file(h);
    }
  }

  struct Progress
  {
    std::uint8_t file_index;
    std::uint64_t chunks;
    std::uint64_t bytes;
    std::uint64_t files;
  };
  [[nodiscard]] Progress progress()
  {
    const std::lock_guard lock(mutex_);
    return {file_index_, chunks_, bytes_, files_};
  }

private:
  void open_file(const FirmwareLogPushHeader & h)
  {
    if (file_.is_open()) {
      file_.close();
    }
    file_index_ = h.file_index;
    const auto name = std::format(
      "{}_{}_{}.log", prefix_, to_string(h.log_type), static_cast<unsigned>(h.file_index));
    const auto path = dir_ / name;
    file_.open(path, std::ios::binary | std::ios::trunc);
    if (!file_) {
      std::cerr << "cannot open " << path.string() << "\n";
    } else {
      ++files_;
      std::cerr << "writing " << path.string() << "\n";
    }
  }

  std::mutex mutex_;
  std::filesystem::path dir_;
  std::string prefix_;
  std::ofstream file_;
  std::uint8_t file_index_ = 0;
  std::uint64_t chunks_ = 0;
  std::uint64_t bytes_ = 0;
  std::uint64_t files_ = 0;
};

void must(const std::expected<void, DeviceError> & r, const char * what)
{
  if (!r) {
    std::cerr << what << ": " << to_string(r.error()) << "\n";
    std::exit(2);
  }
}

int run(int argc, char ** argv)
{
  const auto args = parse_args(argc, argv);
  if (!args) {
    std::cerr << "usage: collect_firmware_log [--lidar-ip A.B.C.D] [--host-ip A.B.C.D] [--out DIR] "
                 "[--duration N] [--log-port N] [--type realtime|exception] [--start-sampling]\n";
    return 1;
  }
  (void)std::signal(SIGINT, on_sigint);

  std::error_code ec;
  std::filesystem::create_directories(args->out, ec);
  if (ec) {
    std::cerr << "cannot create " << args->out.string() << ": " << ec.message() << "\n";
    return 2;
  }

  auto ctx = Context::create(args->ctx);
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
  const std::string serial = found->front().serial_number;
  std::cerr << "found " << serial << " at " << to_string(found->front().from) << "\n";

  DeviceOptions opts;
  opts.session.bind_address = args->host_ip;
  auto dev = Device::open(**ctx, found->front(), opts);
  if (!dev) {
    std::cerr << "open: " << to_string(dev.error()) << "\n";
    return 2;
  }

  LogWriter writer(args->out, std::format("{}_{}", serial, utc_stamp()));
  std::atomic<std::uint64_t> gaps{0};
  must(
    (*dev)->on_firmware_log([&](const FirmwareLogChunk & c) { writer.on_chunk(c); }),
    "on_firmware_log");
  must(
    (*dev)->on_event([&](const Event & e) {
      if (e.kind == Event::Kind::kFirmwareLogGap) {
        ++gaps;
      }
      std::cerr << to_string(e) << "\n";
    }),
    "on_event");
  if (args->start_sampling) {
    must((*dev)->start_sampling(), "start_sampling");
  }
  must((*dev)->start_firmware_log(args->type), "start_firmware_log");
  writer.ensure_open(args->type);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(args->duration);
  while (!g_stop && (args->duration == 0 || std::chrono::steady_clock::now() < deadline)) {
    std::this_thread::sleep_for(1s);
    const auto p = writer.progress();
    std::cout << std::format(
      "file {}: {} bytes, {} chunks, {} gaps\n", static_cast<unsigned>(p.file_index), p.bytes,
      p.chunks, gaps.load());
  }

  if (auto r = (*dev)->stop_firmware_log(args->type); !r) {
    std::cerr << "stop_firmware_log: " << to_string(r.error()) << "\n";
  }
  std::this_thread::sleep_for(200ms);  // let the "file end" packet arrive
  const DeviceStats s = (*dev)->stats();
  dev->reset();  // Devices before the Context
  const auto p = writer.progress();
  std::cout << std::format(
    "collected {} chunks, {} bytes in {} file(s), {} gap(s), {} ack(s) sent, {} bad packet(s)\n",
    p.chunks, p.bytes, p.files, s.log_gaps, s.log_acks_sent, s.bad_log_packets);
  return p.chunks == 0 ? 3 : 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << "fatal: " << e.what() << "\n";
    return 2;
  }
}
