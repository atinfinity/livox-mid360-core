// SPDX-License-Identifier: Apache-2.0
// `record`: discover -> Device::open -> on_packet -> Lvx2Writer (issue #35).
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <expected>
#include <filesystem>
#include <format>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "cli.hpp"
#include "livox/mid360/mid360.hpp"

namespace cli
{
namespace
{
using namespace livox::mid360;
using namespace std::chrono_literals;

struct Args
{
  std::filesystem::path out;
  std::optional<Ipv4> lidar_ip;
  Ipv4 host_ip{0, 0, 0, 0};
  ContextOptions ctx;
  std::optional<std::string> sn;
  int duration = 0;  // seconds, 0 = until SIGINT
  bool force = false;
};

std::optional<Args> parse_args(int argc, char ** argv)
{
  Args a;
  for (int i = 0; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--force") {
      a.force = true;
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
  if (a.out.empty() || a.duration < 0) {
    return std::nullopt;
  }
  return a;
}

/// SDK2 "handle" of a LiDAR: its IP bytes as one u32 in network order (docs/lvx2.md).
std::uint32_t lidar_id_of(const Ipv4 & ip)
{
  std::uint32_t id = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    id |= static_cast<std::uint32_t>(ip[i]) << (8U * static_cast<unsigned>(i));
  }
  return id;
}
}  // namespace

int run_record(int argc, char ** argv)
{
  const auto args = parse_args(argc, argv);
  if (!args) {
    std::cerr << "usage: livox-mid360-cli record --out FILE.lvx2 [--lidar-ip A.B.C.D] "
                 "[--host-ip A.B.C.D] [--sn SN] [--duration SECONDS] [--force]\n";
    return 1;
  }
  if (!args->force && std::filesystem::exists(args->out)) {
    std::cerr << args->out.string() << " exists (use --force to overwrite)\n";
    return 2;
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

  Lvx2DeviceInfo info;
  info.lidar_sn = target->serial_number;
  info.lidar_id = lidar_id_of(target->ip);
  if (auto att = (*dev)->install_attitude(); att) {
    info.extrinsic_enable = true;
    info.roll_deg = att->roll_deg;
    info.pitch_deg = att->pitch_deg;
    info.yaw_deg = att->yaw_deg;
    info.x_m = static_cast<float>(att->x_mm) * 0.001F;
    info.y_m = static_cast<float>(att->y_mm) * 0.001F;
    info.z_m = static_cast<float>(att->z_mm) * 0.001F;
  } else {
    std::cerr << "install attitude not readable (" << to_string(att.error())
              << "); extrinsic disabled\n";
  }

  Lvx2Writer writer;
  if (auto r = writer.open(args->out, std::span(&info, 1)); !r) {
    std::cerr << "open " << args->out.string() << ": " << to_string(r.error()) << "\n";
    return 2;
  }
  std::mutex mu;  // writer vs. the progress reader
  std::optional<Lvx2Error> write_error;
  auto r = (*dev)->on_packet([&](const DataPacketView & p, const ReceiveInfo &) {
    const std::lock_guard lock(mu);
    if (write_error) {
      return;
    }
    if (auto w = writer.write(0, p); !w) {
      write_error = w.error();
      g_stop = true;
    }
  });
  if (!r) {
    std::cerr << "on_packet: " << to_string(r.error()) << "\n";
    return 2;
  }
  if (auto s = (*dev)->start_sampling(); !s) {
    std::cerr << "start_sampling: " << to_string(s.error()) << "\n";
    return 2;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(args->duration);
  while (!g_stop && (args->duration == 0 || std::chrono::steady_clock::now() < deadline)) {
    std::this_thread::sleep_for(1s);
    const std::lock_guard lock(mu);
    const auto s = writer.stats();
    std::cerr << std::format("packets={} frames={} bytes={}\n", s.packets, s.frames, s.bytes);
  }
  if (auto s = (*dev)->stop_sampling(); !s) {
    std::cerr << "stop_sampling: " << to_string(s.error()) << "\n";
  }
  dev->reset();  // no more callbacks after this
  Lvx2Writer::Stats stats;
  {
    const std::lock_guard lock(mu);
    if (auto c = writer.close(); !c && !write_error) {
      write_error = c.error();
    }
    stats = writer.stats();
  }
  if (write_error) {
    std::cerr << "write " << args->out.string() << ": " << to_string(*write_error) << "\n";
    return 2;
  }
  std::cerr << std::format(
    "wrote {}: packets={} frames={} bytes={} imu_ignored={}\n", args->out.string(), stats.packets,
    stats.frames, stats.bytes, stats.ignored);
  return stats.packets == 0 ? 3 : 0;
}
}  // namespace cli
