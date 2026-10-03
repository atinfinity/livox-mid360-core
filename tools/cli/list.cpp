// SPDX-License-Identifier: Apache-2.0
// `list`: one discovery round, one line per LiDAR that answered. Broadcast by default; with
// --lidar-ip (repeatable) the listed addresses are asked by unicast instead.
#include <chrono>
#include <format>
#include <iostream>
#include <string>
#include <vector>

#include "cli.hpp"
#include "livox/mid360/mid360.hpp"

namespace cli
{
std::string describe(const livox::mid360::DiscoveredDevice & d)
{
  using livox::mid360::ip_to_string;
  return std::format(
    "sn={} ip={} cmd_port={} dev_type={} from={}", d.serial_number, ip_to_string(d.ip), d.cmd_port,
    d.dev_type, to_string(d.from));
}

namespace
{
using namespace livox::mid360;

struct Args
{
  std::vector<Ipv4> lidar_ips;
  Ipv4 host_ip{0, 0, 0, 0};
  int timeout_ms = 1000;
};

std::optional<Args> parse_args(int argc, char ** argv)
{
  if (argc % 2 != 0) {
    return std::nullopt;
  }
  Args a;
  for (int i = 0; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string val = argv[i + 1];
    if (key == "--lidar-ip" || key == "--host-ip") {
      const auto ip = parse_ip(val);
      if (!ip) {
        return std::nullopt;
      }
      if (key == "--lidar-ip") {
        a.lidar_ips.push_back(*ip);
      } else {
        a.host_ip = *ip;
      }
    } else if (key == "--timeout-ms") {
      try {
        a.timeout_ms = std::stoi(val);
      } catch (const std::exception &) {
        return std::nullopt;
      }
      if (a.timeout_ms <= 0) {
        return std::nullopt;
      }
    } else {
      return std::nullopt;
    }
  }
  return a;
}
}  // namespace

int run_list(int argc, char ** argv)
{
  const auto args = parse_args(argc, argv);
  if (!args) {
    std::cerr << "usage: livox-mid360-cli list [--host-ip A.B.C.D] [--lidar-ip A.B.C.D]... "
                 "[--timeout-ms N]\n";
    return 1;
  }
  DiscoveryOptions disc;
  disc.bind_address = args->host_ip;
  disc.timeout = std::chrono::milliseconds{args->timeout_ms};
  for (const auto & ip : args->lidar_ips) {
    disc.targets.push_back(Endpoint{ip, kDiscoveryPort});
  }
  auto found = discover(disc);
  if (!found) {
    std::cerr << "discovery: " << to_string(found.error()) << "\n";
    return 2;
  }
  for (const auto & d : *found) {
    std::cout << describe(d) << "\n";
    if (d.ip != d.from.ip) {
      // Commands go to `ip`, so a LiDAR in this state cannot be opened from here.
      std::cerr << "warning: " << d.serial_number << " reports ip=" << ip_to_string(d.ip)
                << " but answered from " << to_string(d.from) << "\n";
    }
  }
  std::cerr << "found " << found->size() << " LiDAR(s)\n";
  return found->empty() ? 2 : 0;
}
}  // namespace cli
