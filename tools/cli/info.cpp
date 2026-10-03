// SPDX-License-Identifier: Apache-2.0
// `info`: discovery -> Session -> 0x0101 inquire of the identity keys. Read-only: unlike
// Device::open it writes no host setup, so the LiDAR's configuration is left as it is.
// Prints what the hardware checks of issue #110 / #11 record (dev_type, cmd_port, firmware).
#include <format>
#include <iostream>
#include <optional>
#include <string>

#include "cli.hpp"
#include "livox/mid360/mid360.hpp"

namespace cli
{
namespace
{
using namespace livox::mid360;

struct Args
{
  std::optional<Ipv4> lidar_ip;
  Ipv4 host_ip{0, 0, 0, 0};
  std::optional<std::string> sn;
};

std::optional<Args> parse_args(int argc, char ** argv)
{
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
        a.lidar_ip = *ip;
      } else {
        a.host_ip = *ip;
      }
    } else if (key == "--sn") {
      a.sn = val;
    } else {
      return std::nullopt;
    }
  }
  if (argc % 2 != 0) {
    return std::nullopt;
  }
  return a;
}
}  // namespace

int run_info(int argc, char ** argv)
{
  const auto args = parse_args(argc, argv);
  if (!args) {
    std::cerr
      << "usage: livox-mid360-cli info [--lidar-ip A.B.C.D] [--host-ip A.B.C.D] [--sn SN]\n";
    return 1;
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
  int status = 2;  // until one LiDAR has been printed
  for (const auto & d : *found) {
    if (args->sn && d.serial_number != *args->sn) {
      continue;
    }
    std::cout << std::format(
      "discovery: sn={} ip={} cmd_port={} dev_type={} from={}\n", d.serial_number,
      ip_to_string(d.ip), d.cmd_port, d.dev_type, to_string(d.from));
    SessionOptions opts;
    opts.bind_address = args->host_ip;
    auto session = Session::connect(d, opts);
    if (!session) {
      std::cerr << "connect " << d.serial_number << ": " << to_string(session.error()) << "\n";
      continue;
    }
    auto r = session->inquire(kIdentityKeys);
    if (!r) {
      std::cerr << "inquire " << d.serial_number << ": " << to_string(r.error()) << "\n";
      continue;
    }
    std::cout << "identity: " << to_string(decode_identity(r->values)) << "\n";
    if (status == 2) {
      status = 0;
    }
  }
  if (args->sn && status == 2) {
    std::cerr << "no LiDAR with SN " << *args->sn << " answered\n";
  }
  return status;
}
}  // namespace cli
