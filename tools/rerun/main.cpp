// SPDX-License-Identifier: Apache-2.0
// livox-mid360-rerun: shows the point cloud of a live Mid-360 (`live`) or of an lvx2 recording
// (`play`) in the Rerun viewer (issue #147, docs/rerun.md).
#include <exception>
#include <iostream>
#include <string>

#include "livox/mid360/version.hpp"
#include "viewer.hpp"

namespace
{
void usage()
{
  std::cerr << "usage: livox-mid360-rerun <command> [options]\n"
               "  live [--lidar-ip A.B.C.D] [--host-ip A.B.C.D] [--sn SN] [--duration SECONDS]\n"
               "  play FILE.lvx2 [--rate X] [--loop] [--lidar-id N]\n"
               "common options:\n"
            << viewer::kCommonUsage << "  --version | --help\n";
}

int run(int argc, char ** argv)
{
  if (argc < 2) {
    usage();
    return 1;
  }
  const std::string cmd = argv[1];
  if (cmd == "--version" || cmd == "version") {
    std::cout << "livox-mid360-rerun " << livox::mid360::kVersionString << "\n";
    return 0;
  }
  if (cmd == "--help" || cmd == "-h" || cmd == "help") {
    usage();
    return 0;
  }
  if (cmd == "live") {
    return viewer::run_live(argc - 2, argv + 2);
  }
  if (cmd == "play") {
    return viewer::run_play(argc - 2, argv + 2);
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
