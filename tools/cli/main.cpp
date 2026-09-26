// SPDX-License-Identifier: Apache-2.0
// livox-mid360-cli: `record` writes the packet stream of one LiDAR to an lvx2 file, `replay`
// plays such a file back through the frame assembler (issue #35, docs/lvx2.md).
#include <csignal>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>

#include "cli.hpp"
#include "livox/mid360/version.hpp"

namespace cli
{
std::atomic<bool> g_stop{false};
namespace
{
void on_sigint(int /*signal*/) { g_stop = true; }
}  // namespace
void install_sigint() { (void)std::signal(SIGINT, on_sigint); }

std::optional<livox::mid360::Ipv4> parse_ip(const std::string & s)
{
  livox::mid360::Ipv4 ip{};
  std::size_t pos = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    const auto dot = (i < 3) ? s.find('.', pos) : s.size();
    if (dot == std::string::npos || dot == pos) {
      return std::nullopt;
    }
    const std::string part = s.substr(pos, dot - pos);
    if (
      part.size() > 3 || part.find_first_not_of("0123456789") != std::string::npos ||
      std::stoi(part) > 255) {
      return std::nullopt;
    }
    ip[i] = static_cast<std::uint8_t>(std::stoi(part));
    pos = dot + 1;
  }
  return ip;
}
}  // namespace cli

namespace
{
void usage()
{
  std::cerr << "usage: livox-mid360-cli <command> [options]\n"
               "  record --out FILE.lvx2 [--lidar-ip A.B.C.D] [--host-ip A.B.C.D] [--sn SN]\n"
               "         [--duration SECONDS] [--force]\n"
               "  replay FILE.lvx2 [--rate X] [--loop] [--frame-mode counter|window]\n"
               "         [--window-ms N] [--quiet]\n"
               "  --version | --help\n";
}

int run(int argc, char ** argv)
{
  if (argc < 2) {
    usage();
    return 1;
  }
  const std::string cmd = argv[1];
  if (cmd == "--version" || cmd == "version") {
    std::cout << "livox-mid360-cli " << livox::mid360::kVersionString << "\n";
    return 0;
  }
  if (cmd == "--help" || cmd == "-h" || cmd == "help") {
    usage();
    return 0;
  }
  if (cmd == "record") {
    return cli::run_record(argc - 2, argv + 2);
  }
  if (cmd == "replay") {
    return cli::run_replay(argc - 2, argv + 2);
  }
  std::cerr << "unknown command: " << cmd << "\n";
  usage();
  return 1;
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
