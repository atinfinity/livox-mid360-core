// SPDX-License-Identifier: Apache-2.0
// `live`: discover -> Device::open -> on_frame -> BoundedQueue -> Rerun (issue #147).
#include <chrono>
#include <format>
#include <iostream>
#include <optional>
#include <string>
#include <utility>

#include "livox/mid360/mid360.hpp"
#include "viewer.hpp"

namespace viewer
{
namespace
{
using namespace livox::mid360;
using namespace std::chrono_literals;

struct Args
{
  std::optional<Ipv4> lidar_ip;
  Ipv4 host_ip{0, 0, 0, 0};
  ContextOptions ctx;
  std::optional<std::string> sn;
  int duration = 0;  // seconds, 0 = until SIGINT
  Options view;
};

std::optional<Ipv4> parse_ip(const std::string & s)
{
  const auto ep = parse_endpoint(s);
  if (!ep || s.find(':') != std::string::npos) {
    return std::nullopt;
  }
  return ep->ip;
}

std::optional<Args> parse_args(int argc, char ** argv)
{
  Args a;
  for (int i = 0; i < argc; ++i) {
    const std::string key = argv[i];
    if (const auto f = parse_flag(key, a.view); f != Parsed::kUnknown) {
      if (f == Parsed::kInvalid) {
        return std::nullopt;
      }
      continue;
    }
    if (i + 1 >= argc) {
      return std::nullopt;
    }
    const std::string val = argv[++i];
    if (const auto o = parse_option(key, val, a.view); o != Parsed::kUnknown) {
      if (o == Parsed::kInvalid) {
        return std::nullopt;
      }
    } else if (key == "--lidar-ip" || key == "--host-ip") {
      const auto ip = parse_ip(val);
      if (!ip) {
        return std::nullopt;
      }
      if (key == "--lidar-ip") {
        a.lidar_ip = *ip;
      } else {
        a.host_ip = a.ctx.bind_address = *ip;
      }
    } else if (key == "--sn") {
      a.sn = val;
    } else if (key == "--duration") {
      a.duration = std::stoi(val);
    } else if (key == "--push-port") {
      a.ctx.push_port = static_cast<std::uint16_t>(std::stoi(val));
    } else if (key == "--point-port") {
      a.ctx.point_port = static_cast<std::uint16_t>(std::stoi(val));
    } else if (key == "--imu-port") {
      a.ctx.imu_port = static_cast<std::uint16_t>(std::stoi(val));
    } else {
      return std::nullopt;
    }
  }
  if (a.duration < 0) {
    return std::nullopt;
  }
  return a;
}
}  // namespace

int run_live(int argc, char ** argv)
{
  const auto args = parse_args(argc, argv);
  if (!args) {
    std::cerr << "usage: livox-mid360-rerun live [--lidar-ip A.B.C.D] [--host-ip A.B.C.D] "
                 "[--sn SN] [--duration SECONDS]\n"
              << kCommonUsage;
    return 1;
  }
  install_sigint();

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
  const DiscoveredDevice * target = nullptr;
  for (const auto & d : *found) {
    if (!args->sn || d.serial_number == *args->sn) {
      target = &d;
      break;
    }
  }
  if (target == nullptr) {
    std::cerr << "discovery: no LiDAR with SN " << args->sn.value_or("") << "\n";
    return 2;
  }
  std::cerr << "found " << target->serial_number << " at " << to_string(target->from) << "\n";

  DeviceOptions opts;
  opts.frame_policy = args->view.frame_policy;
  opts.session.bind_address = args->host_ip;
  auto dev = Device::open(**ctx, *target, opts);
  if (!dev) {
    std::cerr << "open: " << to_string(dev.error()) << "\n";
    return 2;
  }
  std::optional<Extrinsic> extrinsic;
  if (args->view.extrinsic) {
    auto att = (*dev)->install_attitude();
    if (!att) {
      std::cerr << "install attitude: " << to_string(att.error()) << "\n";
      return 2;
    }
    extrinsic = extrinsic_from(*att);
  }

  // The viewer's default timestamp policy gives host time, hence a timestamp timeline.
  Viewer viewer(args->view, Viewer::SensorTime::kTimestamp);
  if (!viewer.open()) {
    return 2;
  }
  // Frames are logged on this thread, not the receive thread; a slow sink drops the oldest.
  BoundedQueue<Frame> queue(4);
  if (auto r = (*dev)->on_frame([&](Frame && f) { queue.push(std::move(f)); }); !r) {
    std::cerr << "on_frame: " << to_string(r.error()) << "\n";
    return 2;
  }
  if (auto s = (*dev)->start_sampling(); !s) {
    std::cerr << "start_sampling: " << to_string(s.error()) << "\n";
    return 2;
  }

  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::seconds(args->duration);
  auto next_report = start + 1s;
  while (!g_stop && (args->duration == 0 || std::chrono::steady_clock::now() < deadline)) {
    if (auto f = queue.pop(100ms)) {
      viewer.log(*f, extrinsic);
    }
    if (const auto now = std::chrono::steady_clock::now(); now >= next_report) {
      next_report = now + 1s;
      std::cerr << std::format(
        "frames={} points={} queue_dropped={}\n", viewer.frames(), viewer.points(),
        queue.dropped());
    }
  }
  if (auto s = (*dev)->stop_sampling(); !s) {
    std::cerr << "stop_sampling: " << to_string(s.error()) << "\n";
  }
  dev->reset();  // no more callbacks after this
  viewer.flush();
  std::cerr << std::format(
    "logged frames={} points={} queue_dropped={}\n", viewer.frames(), viewer.points(),
    queue.dropped());
  return viewer.frames() == 0 ? 3 : 0;
}
}  // namespace viewer
