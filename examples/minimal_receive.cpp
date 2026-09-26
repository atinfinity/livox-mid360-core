// Minimal end-to-end sample (issue #43): Context -> discover -> Device::open -> callbacks ->
// start_sampling, one line per frame, IMU at a throttled rate, clean shutdown on SIGINT.
//
//   minimal_receive --lidar-ip 192.168.1.10 --host-ip 192.168.1.5 [--seconds 10]
//
// Exit codes: 0 ok, 1 usage, 2 setup failure, 3 no frame received.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <expected>
#include <format>
#include <iostream>
#include <optional>
#include <string>
#include <utility>

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
  int seconds = 0;  // 0 = until SIGINT
  int imu_every = 200;
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
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string val = argv[i + 1];
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
    } else if (key == "--push-port") {
      a.ctx.push_port = static_cast<std::uint16_t>(std::stoi(val));
    } else if (key == "--point-port") {
      a.ctx.point_port = static_cast<std::uint16_t>(std::stoi(val));
    } else if (key == "--imu-port") {
      a.ctx.imu_port = static_cast<std::uint16_t>(std::stoi(val));
    } else if (key == "--seconds") {
      a.seconds = std::stoi(val);
    } else if (key == "--imu-every") {
      a.imu_every = std::stoi(val);
    } else {
      return std::nullopt;
    }
  }
  return (argc % 2 == 1) ? std::optional<Args>(a) : std::nullopt;
}

/// Prints the error and exits with 2 when a setup step fails.
void must(const std::expected<void, DeviceError> & r, const char * what)
{
  if (!r) {
    std::cerr << what << ": " << to_string(r.error()) << "\n";
    std::exit(2);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  const auto args = parse_args(argc, argv);
  if (!args) {
    std::cerr << "usage: minimal_receive [--lidar-ip A.B.C.D] [--host-ip A.B.C.D] "
                 "[--push-port N] [--point-port N] [--imu-port N] [--seconds N] [--imu-every N]\n";
    return 1;
  }
  (void)std::signal(SIGINT, on_sigint);

  // One receive thread for all LiDARs on this interface.
  auto ctx = Context::create(args->ctx);
  if (!ctx) {
    std::cerr << "context: " << to_string(ctx.error()) << "\n";
    return 2;
  }

  // Unicast discovery when the LiDAR is known, broadcast otherwise.
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
  std::cerr << "found " << found->front().serial_number << " at " << to_string(found->front().from)
            << "\n";

  // Point the LiDAR at this host; the work mode is untouched until start_sampling().
  DeviceOptions opts;
  opts.session.bind_address = args->host_ip;
  auto dev = Device::open(**ctx, found->front(), opts);
  if (!dev) {
    std::cerr << "open: " << to_string(dev.error()) << "\n";
    return 2;
  }

  // Frames go to the main thread through a bounded queue; IMU and events print in place.
  BoundedQueue<Frame> frames;
  std::atomic<std::uint64_t> imu_count{0};
  must((*dev)->on_frame([&](Frame && f) { frames.push(std::move(f)); }), "on_frame");
  must(
    (*dev)->on_imu([&](const ImuData & imu) {
      if (imu_count++ % static_cast<std::uint64_t>(args->imu_every) == 0) {
        const auto & s = imu.sample;
        std::cout << std::format(
          "imu t={} gyro=({:.3f},{:.3f},{:.3f}) acc=({:.3f},{:.3f},{:.3f})\n", imu.time_ns,
          s.gyro_x, s.gyro_y, s.gyro_z, s.acc_x, s.acc_y, s.acc_z);
      }
    }),
    "on_imu");
  must((*dev)->on_event([](const Event & e) { std::cerr << to_string(e) << "\n"; }), "on_event");
  must((*dev)->start_sampling(), "start_sampling");  // work_tgt_mode = SAMPLING + wait

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(args->seconds);
  std::uint64_t frame_count = 0;
  while (!g_stop && (args->seconds == 0 || std::chrono::steady_clock::now() < deadline)) {
    const auto f = frames.pop(200ms);
    if (!f) {
      continue;
    }
    ++frame_count;
    const Point first = f->points.empty() ? Point{} : f->points.front();
    std::cout << std::format(
      "frame {} points={} first=({:.3f},{:.3f},{:.3f}) t={} dropped={}\n", f->index,
      f->points.size(), first.x, first.y, first.z, f->base_time_ns, f->dropped_packets);
  }

  if (auto r = (*dev)->stop_sampling(); !r) {
    std::cerr << "stop_sampling: " << to_string(r.error()) << "\n";
  }
  dev->reset();  // Devices before the Context
  std::cout << std::format("received {} frames, {} imu packets\n", frame_count, imu_count.load());
  return frame_count == 0 ? 3 : 0;
}
